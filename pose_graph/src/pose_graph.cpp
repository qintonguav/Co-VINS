#include "pose_graph.h"

PoseGraph::PoseGraph()
{
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);
	t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    earliest_loop_index = -1;
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
}

PoseGraph::~PoseGraph()
{
	t_optimization.join();
}

void PoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 1000);
    pub_base_path = n.advertise<nav_msgs::Path>("base_path", 1000);
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);
    for (int i = 0; i < 7; i++)
        pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 1000);
}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addAgentFrame(KeyFrame* cur_kf)
{
    cur_kf->index = global_index;
    int sequence = cur_kf->sequence;
    if (!sequence_align_world.count(sequence))
    {
        if (sequence_align_world.empty())
        {
            sequence_align_world[sequence] = 1;
            first_sequence = sequence;
            printf("first sequence %d \n", sequence);
        }
        else
        {
            sequence_align_world[sequence] = 0;
            printf("new sequence %d \n", sequence);
        }
        sequence_t_drift_map[sequence] = Eigen::Vector3d(0, 0, 0);
        sequence_w_t_s_map[sequence] = Eigen::Vector3d(0, 0, 0);
        sequence_r_drift_map[sequence] = Eigen::Matrix3d::Identity();
        sequence_w_r_s_map[sequence] = Eigen::Matrix3d::Identity();
    }
    global_index++;

    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = sequence_w_r_s_map[sequence] * vio_P_cur + sequence_w_t_s_map[sequence];
    vio_R_cur = sequence_w_r_s_map[sequence] *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);


    int loop_index = -1;
    //TicToc t_detectloop;
    loop_index = detectLoop(cur_kf, cur_kf->index);
    //printf("detect loop time %f\n", t_detectloop.toc());
    bool find_connection = false;
    bool need_update_path = false;
    if (loop_index != -1)
    {
        //printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        //TicToc t_findconnection;
        find_connection = cur_kf->findConnection(old_kf);
        if (find_connection)
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            ROS_WARN("find overlap connection %d --- %d", sequence, old_kf->sequence);
            // shift vio pose of whole sequence to the world frame
            if (old_kf->sequence != cur_kf->sequence && sequence_align_world[sequence] == 0 && sequence_align_world[old_kf->sequence] == 1)
            {  
                ROS_WARN("align %d sequence and %d sequence to world frame", sequence, old_kf->sequence);
                sequence_align_world[sequence] = 1;
                Vector3d w_P_old, w_P_cur, vio_P_cur;
                Matrix3d w_R_old, w_R_cur, vio_R_cur;
                old_kf->getVioPose(w_P_old, w_R_old);
                cur_kf->getVioPose(vio_P_cur, vio_R_cur);

                Vector3d relative_t;
                Matrix3d relative_q;
                relative_t = cur_kf->getLoopRelativeT();
                relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
                w_P_cur = w_R_old * relative_t + w_P_old;
                w_R_cur = w_R_old * relative_q;
                double shift_yaw;
                Matrix3d shift_r;
                Vector3d shift_t; 
                shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
                shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
                shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; 

                sequence_w_r_s_map[sequence] = shift_r;
                sequence_w_t_s_map[sequence] = shift_t;
                vio_P_cur = sequence_w_r_s_map[sequence] * vio_P_cur + sequence_w_t_s_map[sequence];
                vio_R_cur = sequence_w_r_s_map[sequence] *  vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                for (size_t i = 0; i < keyframe_vec.size(); i++)
                {
                    KeyFrame* it = keyframe_vec[i];
                    if (it->sequence == sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = sequence_w_r_s_map[sequence] * vio_P_cur + sequence_w_t_s_map[sequence];
                        vio_R_cur = sequence_w_r_s_map[sequence] *  vio_R_cur;
                        (it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                    need_update_path = true;
                }
            }
            if (old_kf->sequence != cur_kf->sequence && sequence_align_world[old_kf->sequence] == 0 && sequence_align_world[sequence] == 1)
            {  
                ROS_WARN("align %d sequence and %d sequence to world frame", old_kf->sequence, sequence);
                sequence_align_world[old_kf->sequence] = 1;
                Vector3d w_P_old, w_P_cur, vio_P_old;
                Matrix3d w_R_old, w_R_cur, vio_R_old;
                old_kf->getVioPose(vio_P_old, vio_R_old);
                cur_kf->getVioPose(w_P_cur, w_R_cur);

                Vector3d relative_t;
                Matrix3d relative_q;
                relative_t = cur_kf->getLoopRelativeT();
                relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
                w_P_old = -w_R_cur * relative_q.transpose() * relative_t + w_P_cur;
                w_R_old = w_R_cur * relative_q.transpose();
                double shift_yaw;
                Matrix3d shift_r;
                Vector3d shift_t; 
                shift_yaw = Utility::R2ypr(w_R_old).x() - Utility::R2ypr(vio_R_old).x();
                shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
                shift_t = w_P_old - w_R_old * vio_R_old.transpose() * vio_P_old; 

                sequence_w_r_s_map[old_kf->sequence] = shift_r;
                sequence_w_t_s_map[old_kf->sequence] = shift_t;
                for (size_t i = 0; i < keyframe_vec.size(); i++)
                {
                    KeyFrame* it = keyframe_vec[i];
                    if (it->sequence == old_kf->sequence)
                    {
                        Vector3d vio_P;
                        Matrix3d vio_R;
                        (it)->getVioPose(vio_P, vio_R);
                        vio_P = sequence_w_r_s_map[old_kf->sequence] * vio_P + sequence_w_t_s_map[old_kf->sequence];
                        vio_R = sequence_w_r_s_map[old_kf->sequence] *  vio_R;
                        (it)->updateVioPose(vio_P, vio_R);
                        need_update_path = true;
                    }
                }
            }
        }
        //printf("find connection time %f\n", t_findconnection.toc());
    }
    m_keyframelist.lock();

    Vector3d P; 
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    P = sequence_r_drift_map[sequence] * P + sequence_t_drift_map[sequence];
    R = sequence_r_drift_map[sequence] * R;
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence].poses.push_back(pose_stamped);
    path[sequence].header = pose_stamped.header;
    
    if (SAVE_LOOP_PATH)
    {
        std::string pose_graph_path = VINS_RESULT_PATH + "/pose_graph_path.csv";
        ofstream loop_path_file(pose_graph_path, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(0);
        loop_path_file << cur_kf->time_stamp * 1e9 << ",";
        loop_path_file.precision(5);
        loop_path_file  << P.x() << ","
              << P.y() << ","
              << P.z() << ","
              << Q.w() << ","
              << Q.x() << ","
              << Q.y() << ","
              << Q.z() << ","
              << endl;
        loop_path_file.close();

        std::string pose_graph_sequence_path = VINS_RESULT_PATH + "/pose_graph_path_" + to_string(sequence) + ".csv";
        ofstream sequence_path_file(pose_graph_sequence_path, ios::app);
        sequence_path_file.setf(ios::fixed, ios::floatfield);
        sequence_path_file.precision(0);
        sequence_path_file << cur_kf->time_stamp * 1e9 << ",";
        sequence_path_file.precision(5);
        sequence_path_file  << P.x() << ","
              << P.y() << ","
              << P.z() << ","
              << Q.w() << ","
              << Q.x() << ","
              << Q.y() << ","
              << Q.z() << ","
              << endl;
        sequence_path_file.close();
    }
    
    //draw local connection
    /*
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    */
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop && cur_kf->sequence != 0)
        {
            //printf("has loop \n");
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if (cur_kf->sequence == connected_KF->sequence)
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 0);
            else
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 1);

            
        }
    }
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);
    keyframe_vec.push_back(cur_kf);
    m_keyframelist.unlock();
    if (need_update_path)
    {
        updatePath();
    }
    publish();
    if (find_connection)
    {
        m_optimize_buf.lock();
        optimize_buf.push(cur_kf->index);
        m_optimize_buf.unlock();
    }
}

void PoseGraph::loadKeyFrame(KeyFrame* cur_kf)
{
    cur_kf->index = global_index;
    int sequence = cur_kf->sequence;
    if (!sequence_align_world.count(sequence))
    {
        if (sequence_align_world.empty())
        {
            sequence_align_world[sequence] = 1;
            first_sequence = sequence;
            printf("first sequence %d \n", sequence);
        }
        else
        {
            sequence_align_world[sequence] = 0;
            printf("new sequence %d \n", sequence);
        }
        sequence_t_drift_map[sequence] = Eigen::Vector3d(0, 0, 0);
        sequence_w_t_s_map[sequence] = Eigen::Vector3d(0, 0, 0);
        sequence_r_drift_map[sequence] = Eigen::Matrix3d::Identity();
        sequence_w_r_s_map[sequence] = Eigen::Matrix3d::Identity();
    }
    global_index++;

    addKeyFrameIntoVoc(cur_kf);

    Vector3d P; 
    Matrix3d R;
    cur_kf->getVioPose(P, R);

    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence].poses.push_back(pose_stamped);
    path[sequence].header = pose_stamped.header;

    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop && cur_kf->sequence != 0)
        {
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if (cur_kf->sequence == connected_KF->sequence)
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 0);
            else
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 1);
            
        }
    }
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);
    m_keyframelist.lock();
    keyframe_vec.push_back(cur_kf);
    m_keyframelist.unlock();
}



KeyFrame* PoseGraph::getKeyFrame(int index)
{
    if (index >= (int)keyframe_vec.size())
        return NULL;
    else
        return keyframe_vec[index];
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)
{
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->feature_des, ret, 4, frame_index - 30);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->feature_des);
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;
    cv::Mat loop_result;

    // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
                find_loop = true;
        }
/*
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
    if (find_loop && frame_index > 10)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    db.add(keyframe->feature_des);
}

void PoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            //printf("optimize pose graph %d --- %d \n", first_looped_index, cur_index);
            TicToc tmp_t;
            m_keyframelist.lock();
            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();

            map<int,int> last_sequence_index;
            int i = 0;
            bool fix_first = false;
            for (size_t k = first_looped_index; k < keyframe_vec.size(); k++)
            {
                KeyFrame* it = keyframe_vec[k];
                last_sequence_index[it->sequence] = k;
                (it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if (!fix_first && (it)->sequence == first_sequence)
                {   
                    fix_first = true;
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                int prev_index = i - 1;
                for (int n = 0; n < 5; n++)
                {
                    while(prev_index >= first_looped_index)
                    {
                        if(keyframe_vec[prev_index]->sequence == it->sequence)
                        {
                            int local_prev_index = keyframe_vec[prev_index]->local_index;
                            Vector3d euler_conncected = Utility::R2ypr(q_array[local_prev_index].toRotationMatrix());
                            Vector3d relative_t(t_array[i][0] - t_array[local_prev_index][0], t_array[i][1] - t_array[local_prev_index][1], t_array[i][2] - t_array[local_prev_index][2]);
                            relative_t = q_array[local_prev_index].inverse() * relative_t;
                            double relative_yaw = euler_array[i][0] - euler_array[local_prev_index][0];
                            ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                           relative_yaw, euler_conncected.y(), euler_conncected.z());
                            problem.AddResidualBlock(cost_function, NULL, euler_array[local_prev_index], 
                                                    t_array[local_prev_index], 
                                                    euler_array[i], 
                                                    t_array[i]);

                            prev_index--;
                            break;
                        }
                        prev_index--;
                    }
                }

                //add loop edge
                if((it)->has_loop && sequence_align_world[(it)->sequence] == 1 && 
                    sequence_align_world[getKeyFrame((it)->loop_index)->sequence] == 1)
                {
                    //printf("add loop edge %d---%d\n",(it)->index, (it)->loop_index);
                    assert((it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((it)->loop_index)->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (it)->getLoopRelativeT();
                    double relative_yaw = (it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
                                                                  t_array[connected_index], 
                                                                  euler_array[i], 
                                                                  t_array[i]);
                    
                }
                
                if ((it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
            
            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            size_t k = 0;
            for (; k < keyframe_vec.size(); k++)
            {
                KeyFrame* it = keyframe_vec[k];
                if ((it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (it)->updatePose(tmp_t, tmp_r);

                if ((it)->index == cur_index)
                    break;
                i++;
            }


            //caculate drift
            map<int, int>::iterator iter;
            for(iter = last_sequence_index.begin(); iter != last_sequence_index.end(); iter++)
            {
                int sequence = iter->first;
                int last_index = iter->second;
                KeyFrame* kf = keyframe_vec[last_index];
                Vector3d cur_t, vio_t;
                Matrix3d cur_r, vio_r;
                kf->getPose(cur_t, cur_r);
                kf->getVioPose(vio_t, vio_r);
                m_drift.lock();
                double yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
                sequence_r_drift_map[sequence] = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
                sequence_t_drift_map[sequence] = cur_t - sequence_r_drift_map[sequence] * vio_t;
                m_drift.unlock();
                //printf("sequence %d t drift %f %f %f", sequence, sequence_t_drift_map[sequence].x(), sequence_t_drift_map[sequence].y(), sequence_t_drift_map[sequence].z());
            }


            // update new poses
            k++;
            for (; k < keyframe_vec.size(); k++)
            {
                KeyFrame* it = keyframe_vec[k];
                int sequence = it->sequence;
                Vector3d P;
                Matrix3d R;
                (it)->getVioPose(P, R);
                P = sequence_r_drift_map[sequence] * P + sequence_t_drift_map[sequence];
                R = sequence_r_drift_map[sequence] * R;
                (it)->updatePose(P, R);
            }
            updatePath();
            m_keyframelist.unlock();
        }

        std::chrono::milliseconds dura(5000);
        std::this_thread::sleep_for(dura);
    }
}

void PoseGraph::updatePath()
{
    map<int, bool>::iterator iter;
    for (iter = sequence_align_world.begin(); iter != sequence_align_world.end(); iter++)
        path[iter->first].poses.clear();
    base_path.poses.clear();
    posegraph_visualization->reset();

    
    if (SAVE_LOOP_PATH)
    {
        std::string pose_graph_path = VINS_RESULT_PATH + "/pose_graph_path.csv";
        ofstream loop_path_file_tmp(pose_graph_path, ios::out);
        loop_path_file_tmp.close();
        map<int, bool>::iterator iter;
        for (iter = sequence_align_world.begin(); iter != sequence_align_world.end(); iter++)
        {
            std::string pose_graph_sequence_path = VINS_RESULT_PATH + "/pose_graph_path_" + to_string(iter->first) + ".csv";
            ofstream loop_path_file_tmp(pose_graph_sequence_path, ios::out);
            loop_path_file_tmp.close();
        }
    }
    

    for (size_t k = 0; k < keyframe_vec.size(); k++)
    {
        KeyFrame* it = keyframe_vec[k];
        Vector3d P;
        Matrix3d R;
        (it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
//        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((it)->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        if((it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        path[(it)->sequence].poses.push_back(pose_stamped);
        path[(it)->sequence].header = pose_stamped.header;

        
        if (SAVE_LOOP_PATH)
        {
            std::string pose_graph_path = VINS_RESULT_PATH + "/pose_graph_path.csv";
            ofstream loop_path_file(pose_graph_path, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(0);
            loop_path_file << (it)->time_stamp * 1e9 << ",";
            loop_path_file.precision(5);
            loop_path_file  << P.x() << ","
                  << P.y() << ","
                  << P.z() << ","
                  << Q.w() << ","
                  << Q.x() << ","
                  << Q.y() << ","
                  << Q.z() << ","
                  << endl;
            loop_path_file.close();

            std::string pose_graph_sequence_path = VINS_RESULT_PATH + "/pose_graph_path_" + to_string((it)->sequence) + ".csv";
            ofstream sequence_path_file(pose_graph_sequence_path, ios::app);
            sequence_path_file.setf(ios::fixed, ios::floatfield);
            sequence_path_file.precision(0);
            sequence_path_file << (it)->time_stamp * 1e9 << ",";
            sequence_path_file.precision(5);
            sequence_path_file  << P.x() << ","
                  << P.y() << ","
                  << P.z() << ","
                  << Q.w() << ","
                  << Q.x() << ","
                  << Q.y() << ","
                  << Q.z() << ","
                  << endl;
            sequence_path_file.close();
        }
        
        //draw local connection
        /*
        if (SHOW_S_EDGE)
        {
            list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame*>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)  
            {  
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;
                    }
                    break;
                }
            } 
        }
        */
        if (SHOW_L_EDGE)
        {
            if ((it)->has_loop && (it)->sequence != 0)
            {
                
                KeyFrame* connected_KF = getKeyFrame((it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (it)->getPose(P, R);
                if (it->sequence == connected_KF->sequence)
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 0);
                else
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), 1);
            }
        }

    }
    publish();
}

void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile;
    printf("pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen (file_path.c_str(),"w");
    
    for (size_t k = 0; k < keyframe_vec.size(); k++)
    {
        KeyFrame* it = keyframe_vec[k];
        std::string descriptor_path, feature_2d_path;
        Quaterniond VIO_tmp_Q{(it)->vio_R_w_i};
        Quaterniond ric{(it)->ric};
        Vector3d VIO_tmp_T = (it)->vio_T_w_i;
        Vector3d tic = (it)->tic;

        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n",
                                    (it)->index, (it)->time_stamp,
                                    VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(), 
                                    VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), 
                                    tic.x(), tic.y(), tic.z(), 
                                    ric.w(), ric.x(), ric.y(), ric.z(), 
                                    (it)->loop_index, 
                                    (it)->loop_info(0), (it)->loop_info(1), (it)->loop_info(2), (it)->loop_info(3),
                                    (it)->loop_info(4), (it)->loop_info(5), (it)->loop_info(6), (it)->loop_info(7),
                                    (int)(it)->feature_2d.size());

        assert((it)->feature_2d.size() == (it)->feature_des.size());
        descriptor_path = POSE_GRAPH_SAVE_PATH + to_string((it)->index) + "_des.csv";
        feature_2d_path = POSE_GRAPH_SAVE_PATH + to_string((it)->index) + "_feature_2d.csv";
        FILE *descriptor_file;
        FILE *feature_2d_file;
        descriptor_file = fopen(descriptor_path.c_str(), "w");
        feature_2d_file = fopen(feature_2d_path.c_str(), "w");
        for (size_t i = 0; i < (it)->feature_2d.size(); i++)
        {
            for (int m = 0; m < 4; m++)
            {
                unsigned long long int tmp_int = 0;
                for (int j = 255 - 64 * m; j > 255 - 64 * m - 64; j--)
                {
                    tmp_int <<= 1;
                    tmp_int += (it)->feature_des[i][j];
                }
                fprintf(descriptor_file, "%lld ", tmp_int);
            }
            //fprintf(descriptor_file, "\n");
            fprintf(feature_2d_file, "%f %f", (it)->feature_2d[i].x, (it)->feature_2d[i].y);
        }
        fclose(descriptor_file);
        fclose(feature_2d_file);
    }

    fclose(pFile);

    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}


void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double tic_x, tic_y, tic_z;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double ric_w, ric_x, ric_y, ric_z;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", 
                                    &index, &time_stamp,
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz, 
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz, 
                                    &tic_x, &tic_y, &tic_z, 
                                    &ric_w, &ric_x, &ric_y, &ric_z, 
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3, 
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF) 
    {
        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d tic(tic_x, tic_y, tic_z);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond qic;
        qic.w() = ric_w;
        qic.x() = ric_x;
        qic.y() = ric_y;
        qic.z() = ric_z;
        Matrix3d VIO_R, ric;
        VIO_R = VIO_Q.toRotationMatrix();
        ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }
 
        string descriptor_path = POSE_GRAPH_SAVE_PATH +  to_string(index) + "_des.csv";
        string feature_2d_path = POSE_GRAPH_SAVE_PATH +  to_string(index) + "_feature_2d.csv";
        FILE *descriptor_file;
        FILE *feature_2d_file;
        descriptor_file = fopen(descriptor_path.c_str(), "r");
        feature_2d_file = fopen(feature_2d_path.c_str(), "r");
        vector<cv::Point2f> feature_2d;
        vector<BRIEF::bitset> feature_des;
        for (int i = 0; i < keypoints_num; i++)
        {
            double p_x, p_y;
            if(!fscanf(feature_2d_file,"%lf %lf", &p_x, &p_y))
                printf(" fail to load pose graph \n");
            feature_2d.push_back(cv::Point2f(p_x, p_y));

            boost::dynamic_bitset<> tmp_brief(256);
            for (int k = 0; k < 4; k++)
            {
                unsigned long long int tmp_int;
                if(!fscanf(descriptor_file,"%lld", &tmp_int))
                    printf(" fail to load pose graph \n");
                for (int j = 0; j < 64; ++j, tmp_int >>= 1)
                {
                    tmp_brief[256 - 64 * (k + 1) + j] = (tmp_int & 1);
                }
            } 
            feature_des.push_back(tmp_brief);
        }
        fclose(feature_2d_file);
        fclose(descriptor_file);

        vector<cv::Point3f> point_3d;
        vector<BRIEF::bitset> point_des;
        KeyFrame* keyframe = new KeyFrame(0, time_stamp, VIO_T, VIO_R, tic, ric, point_3d, feature_2d, 
                                          point_des, feature_des); 
        if (loop_index != -1)
        {
            keyframe->setLoop(loop_index, loop_info);
            //printf("frame %d connect loop with %d\n",index, loop_index);
        }

        loadKeyFrame(keyframe);

        if (cnt % 20 == 0)
            publish();
        cnt++;
    }
    m_optimize_buf.lock();
    optimize_buf.push(index);
    m_optimize_buf.unlock();
    fclose (pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
}


void PoseGraph::publish()
{
    map<int, bool>::iterator iter;
    for (iter = sequence_align_world.begin(); iter != sequence_align_world.end(); iter++)
    {
        int sequence = iter->first; 
        pub_pg_path.publish(path[sequence]);
        pub_path[sequence].publish(path[sequence]);
        posegraph_visualization->publish_by(pub_pose_graph, path[sequence].header);
    }
    pub_base_path.publish(base_path);
    //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
    publishTF();
}

void PoseGraph::publishTF()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    
    map<int, bool>::iterator iter;
    for (iter = sequence_align_world.begin(); iter != sequence_align_world.end(); iter++)
    {
        if (iter->second)
        {
            int sequence = iter->first;

            Vector3d TF_t;
            Quaterniond TF_q;
            TF_q = sequence_r_drift_map[sequence] * sequence_w_r_s_map[sequence];
            TF_t = sequence_r_drift_map[sequence] * sequence_w_t_s_map[sequence] + sequence_t_drift_map[sequence];

            transform.setOrigin(tf::Vector3(TF_t(0), TF_t(1), TF_t(2)));
            q.setW(TF_q.w());
            q.setX(TF_q.x());
            q.setY(TF_q.y());
            q.setZ(TF_q.z());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/global", "/drone_" + to_string(sequence)));
            //printf("%d tf t %f %f %f \n", sequence, TF_t.x(), TF_t.y(),TF_t.z());
            //printf("%d tf q %f %f %f %f \n", sequence, TF_q.w(),TF_q.x(), TF_q.y(), TF_q.z());
        }
    }
}



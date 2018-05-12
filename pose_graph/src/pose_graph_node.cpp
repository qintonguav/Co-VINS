#include <vector>
#include <ros/ros.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "parameters.h"
#include <agent_msg/AgentMsg.h>
#include "ThirdParty/DVision/DVision.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

using namespace std;

std::mutex m_buf;
std::mutex m_process;
int sequence = 1;
PoseGraph posegraph;
bool load_flag = 0;
bool start_flag = 1;
double SKIP_DIS = 0;
int frame_cnt = 0;
double t_agent = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;

queue<agent_msg::AgentMsgConstPtr> agent_msg_buf;
std::mutex m_agent_msg_buf;

ros::Publisher meshPub;
std::string mesh_resource;
visualization_msgs::Marker meshROS;
tf::TransformListener* listener;


void agent_callback(const agent_msg::AgentMsgConstPtr &agent_msg)
{
    if(start_flag)
    {
        m_agent_msg_buf.lock();
        agent_msg_buf.push(agent_msg);
        m_agent_msg_buf.unlock();
    }
}

void agent_process()
{
    while (true)
    {
        agent_msg::AgentMsgConstPtr agent_msg = NULL;
        m_agent_msg_buf.lock();
        //if ((int)agent_msg_buf.size() > 10)
        //    printf("agent_msg buf size %d\n", agent_msg_buf.size());
        if (!agent_msg_buf.empty())
        {
            agent_msg = agent_msg_buf.front();
            agent_msg_buf.pop();
        }
        m_agent_msg_buf.unlock();

        if(agent_msg != NULL)
        {
            // build keyframe
            TicToc t_addframe;
            double time_stamp = agent_msg->header.stamp.toSec();
            int sequence = agent_msg->seq;
            Vector3d T = Vector3d(agent_msg->position_imu.x,
                                  agent_msg->position_imu.y,
                                  agent_msg->position_imu.z);
            Matrix3d R = Quaterniond(agent_msg->orientation_imu.w,
                                     agent_msg->orientation_imu.x,
                                     agent_msg->orientation_imu.y,
                                     agent_msg->orientation_imu.z).toRotationMatrix();

            Vector3d tic = Vector3d(agent_msg->tic.x,
                                    agent_msg->tic.y,
                                    agent_msg->tic.z);
            Matrix3d ric = Quaterniond(agent_msg->ric.w,
                                       agent_msg->ric.x,
                                       agent_msg->ric.y,
                                       agent_msg->ric.z).toRotationMatrix();

            vector<cv::Point3f> point_3d;  
            vector<cv::Point2f> feature_2d;
            vector<BRIEF::bitset> feature_descriptors, point_descriptors;

            
            for (unsigned int i = 0; i < agent_msg->point_3d.size(); i++)
            {
                cv::Point3f p_3d;
                p_3d.x = agent_msg->point_3d[i].x;
                p_3d.y = agent_msg->point_3d[i].y;
                p_3d.z = agent_msg->point_3d[i].z;
                point_3d.push_back(p_3d);
            }
            for (unsigned int i = 0; i < agent_msg->feature_2d.size(); i++)
            {
                cv::Point2f p_2d;
                p_2d.x = agent_msg->feature_2d[i].x;
                p_2d.y = agent_msg->feature_2d[i].y;
                feature_2d.push_back(p_2d);
            }

            for (unsigned int i = 0; i < agent_msg->point_des.size(); i = i + 4)
            {
                boost::dynamic_bitset<> tmp_brief(256);
                for (int k = 0; k < 4; k++)
                {
                    unsigned long long int tmp_int = agent_msg->point_des[i + k];
                    for (int j = 0; j < 64; ++j, tmp_int >>= 1)
                    {
                        tmp_brief[256 - 64 * (k + 1) + j] = (tmp_int & 1);
                    }
                } 
                point_descriptors.push_back(tmp_brief);
            } 

            for (unsigned int i = 0; i < agent_msg->feature_des.size(); i = i + 4)
            {
                boost::dynamic_bitset<> tmp_brief(256);
                for (int k = 0; k < 4; k++)
                {
                    unsigned long long int tmp_int = agent_msg->feature_des[i + k];
                    for (int j = 0; j < 64; ++j, tmp_int >>= 1)
                    {
                        tmp_brief[256 - 64 * (k + 1) + j] = (tmp_int & 1);
                    }
                } 
                feature_descriptors.push_back(tmp_brief);
                //cout << i / 4 << "  "<< tmp_brief << endl;
            } 

            KeyFrame* keyframe = new KeyFrame(sequence, time_stamp, T, R, tic, ric, point_3d, feature_2d, 
                                             point_descriptors, feature_descriptors);             
            m_process.lock();
            posegraph.addAgentFrame(keyframe);
            t_agent += t_addframe.toc();
            frame_cnt++;
            //printf("add agent frame time %f\n",t_agent / frame_cnt);
            m_process.unlock();
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\n you can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            //printf("program shutting down...\n");
            //ros::shutdown();
        }
        if(c  == 'l')
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
            m_process.unlock();
            printf("load pose graph finish\n");
        }
        if(c  == 'b')
        {
            printf("begin receive agent msg\n");
            start_flag = 1;
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Mesh model           
    //ROS_INFO("odometry callback");       
    int sequence = std::stoi(msg->child_frame_id);
    Quaterniond q(msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z);
    Vector3d t(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    tf::StampedTransform transform;
    try{
        listener->lookupTransform("/global", "/drone_" + to_string(sequence),
                                ros::Time(0), transform);

        tf::Vector3 tf_t = transform.getOrigin();
        tf::Quaternion tf_q = transform.getRotation();

        Vector3d w_T_local = Vector3d(tf_t.x(), tf_t.y(), tf_t.z());
        geometry_msgs::Quaternion g_Q;
        tf::quaternionTFToMsg(tf_q, g_Q);   
        Quaterniond w_Q_local(g_Q.w, g_Q.x, g_Q.y, g_Q.z);

        q = w_Q_local * q;
        t = w_Q_local * t + w_T_local;
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("no %d transform yet", sequence);
        //return;
    }
    //ROS_WARN("read transform success!");



    Vector3d ypr = Utility::R2ypr(q.toRotationMatrix());
    ypr(0)    += 90.0*3.14159/180.0;
    q          = Utility::ypr2R(ypr); 
        
    meshROS.header.frame_id = string("/world");
    meshROS.header.stamp = msg->header.stamp; 
    meshROS.ns = "mesh";
    meshROS.id = sequence;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = t.x();
    meshROS.pose.position.y = t.y();
    meshROS.pose.position.z = t.z();
    meshROS.pose.orientation.w = 1;
    meshROS.pose.orientation.x = 0;
    meshROS.pose.orientation.y = 0;
    meshROS.pose.orientation.z = 0;
    meshROS.scale.x = 1;
    meshROS.scale.y = 1;
    meshROS.scale.z = 1;

    meshROS.color.a = 1.0;
    if (sequence == 1)
    {
        meshROS.color.r = 0.0;
        meshROS.color.g = 1.0;
        meshROS.color.b = 0.0;
    }
    else if(sequence == 2)
    {
        meshROS.color.r = 1.0;
        meshROS.color.g = 0.5;
        meshROS.color.b = 0.0;
    }
    else if(sequence == 3)
    {
        meshROS.color.r = 1.0;
        meshROS.color.g = 0.0;
        meshROS.color.b = 0.0;
    }
    else if(sequence == 4)
    {
        meshROS.color.r = 1.0;
        meshROS.color.g = 1.0;
        meshROS.color.b = 1.0;
    }
    else if(sequence == 5)
    {
        meshROS.color.r = 0.0;
        meshROS.color.g = 0.5;
        meshROS.color.b = 1.0;
    }

    meshROS.mesh_resource = mesh_resource;
    meshPub.publish(meshROS);  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_graph");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);

    // read param
    n.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
    n.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    n.getParam("skip_dis", SKIP_DIS);
    n.getParam("pose_graph_save_path", POSE_GRAPH_SAVE_PATH);
    n.param("mesh_resource", mesh_resource, std::string("package://pose_graph/meshes/hummingbird.mesh"));

    std::string pkg_path = ros::package::getPath("pose_graph");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    n.getParam("pose_graph_result_path", VINS_RESULT_PATH);
    std::string pose_graph_path = VINS_RESULT_PATH + "/pose_graph_path.csv";
    ofstream loop_path_file_tmp(pose_graph_path, ios::out);
    loop_path_file_tmp.close();

    ros::Subscriber sub_agent_msg = n.subscribe("/agent_frame", 2000, agent_callback);
    ros::Subscriber sub_odom1 = n.subscribe("/vins_1/vins_estimator/odometry",  100,  odom_callback);
    ros::Subscriber sub_odom2 = n.subscribe("/vins_2/vins_estimator/odometry",  100,  odom_callback);
    ros::Subscriber sub_odom3 = n.subscribe("/vins_3/vins_estimator/odometry",  100,  odom_callback);
    ros::Subscriber sub_odom4 = n.subscribe("/vins_4/vins_estimator/odometry",  100,  odom_callback);

    meshPub = n.advertise<visualization_msgs::Marker>("robot", 100, true); 

    std::thread agent_frame_thread;
    agent_frame_thread = std::thread(agent_process);
    std::thread keyboard_command_process;
    keyboard_command_process = std::thread(command);

    listener = new tf::TransformListener(n);
    ros::spin();
    delete listener;
    return 0;
}

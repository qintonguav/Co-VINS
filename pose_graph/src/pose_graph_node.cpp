#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
} // namespace backward

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

using namespace std;

std::mutex m_buf;
std::mutex m_process;
int sequence = 1;
PoseGraph posegraph;
bool load_flag = 0;
bool start_flag = 0;
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


void agent_callback(const agent_msg::AgentMsgConstPtr &agent_msg)
{
    m_agent_msg_buf.lock();
    agent_msg_buf.push(agent_msg);
    m_agent_msg_buf.unlock();
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

            KeyFrame* keyframe = new KeyFrame(sequence, T, R, tic, ric, point_3d, feature_2d, 
                                             point_descriptors, feature_descriptors);             
            m_process.lock();
            start_flag = 1;
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
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
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

    std::string pkg_path = ros::package::getPath("pose_graph");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    /*
    if (0)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("no previous pose graph\n");
        load_flag = 1;
    }
    */

    ros::Subscriber sub_agent_msg = n.subscribe("/vins_estimator/agent_frame",2000, agent_callback);

    std::thread agent_frame_thread;
    agent_frame_thread = std::thread(agent_process);
    std::thread keyboard_command_process;
    keyboard_command_process = std::thread(command);


    ros::spin();

    return 0;
}

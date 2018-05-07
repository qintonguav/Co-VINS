#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "parameters.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"

#define MIN_LOOP_NUM 25

using namespace Eigen;
using namespace std;
using namespace DVision;

class KeyFrame
{
public:
	/*
	KeyFrame(double _time_stamp, int _index, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_T_w_i, Matrix3d &_R_w_i,
			 cv::Mat &_image, int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info,
			 vector<cv::KeyPoint> &_keypoints, vector<cv::Point2f> &_feature_2d, vector<BRIEF::bitset> &_feature_des);
	*/
	KeyFrame(int _seq, double _time_stamp, Vector3d &_vio_T_w_i, Matrix3d &_vio_R_w_i, Vector3d &_tic, Matrix3d &_ric,
			 vector<cv::Point3f> &_point_3d, vector<cv::Point2f> &_feature_2d, 
			 vector<BRIEF::bitset> &_point_des, vector<BRIEF::bitset> &_feature_des);
	bool findConnection(KeyFrame* old_kf);
	int HammingDis(const BRIEF::bitset &a, const BRIEF::bitset &b);
	bool searchInAera(const BRIEF::bitset window_descriptor,
	                  const std::vector<BRIEF::bitset> &descriptors_old,
	                  const std::vector<cv::Point2f> &feature_2d_old,
	                  cv::Point2f &best_match_norm);
	void searchByBRIEFDes(std::vector<cv::Point2f> &matched_2d_old,
                          std::vector<uchar> &status,
                          const std::vector<BRIEF::bitset> &descriptors_old,
                          const std::vector<cv::Point2f> &feature_2d_old);
	void PnPRANSAC(const vector<cv::Point2f> &matched_2d_old,
	               const std::vector<cv::Point3f> &matched_3d,
	               std::vector<uchar> &status,
	               Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old);
	void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
	void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
	void updateLoop(Eigen::Matrix<double, 8, 1 > &_loop_info);
	void setLoop(int _loop_index, Eigen::Matrix<double, 8, 1 > &_loop_info);

	Eigen::Vector3d getLoopRelativeT();
	double getLoopRelativeYaw();
	Eigen::Quaterniond getLoopRelativeQ();



	double time_stamp; 
	int index;
	int local_index;
	Eigen::Vector3d vio_T_w_i; 
	Eigen::Matrix3d vio_R_w_i; 
	Eigen::Vector3d T_w_i;
	Eigen::Matrix3d R_w_i;
	Eigen::Vector3d origin_vio_T;		
	Eigen::Matrix3d origin_vio_R;
	cv::Mat image;
	cv::Mat thumbnail;
	vector<cv::Point3f> point_3d; 
	vector<cv::Point2f> point_uv;
	vector<cv::Point2f> point_2d;
	vector<double> point_id;
	vector<cv::Point2f> feature_2d;
	vector<cv::KeyPoint> window_keypoints;
	vector<BRIEF::bitset> feature_des;
	vector<BRIEF::bitset> point_des;
	bool has_fast_point;
	int sequence;

	bool has_loop;
	int loop_index;
	Eigen::Matrix<double, 8, 1 > loop_info;
	Vector3d tic;
	Matrix3d ric;

};


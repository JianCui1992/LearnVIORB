#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

const double TIME_SHIFT = 1526010000.0;


namespace ORBVIO
{

extern double IMAGE_DELAY_TIME;
extern std::string VOC_PATH;
extern std::string IMAGE_PATH;
extern std::string IMU_PATH;
// extern std::vector<Eigen::Matrix3d> RIC;
// extern std::vector<Eigen::Vector3d> TIC;

// extern cv::Mat cv_K;
// extern cv::Mat cv_D;

extern void read_parameters(const std::string &filename);

struct image_with_timestamp
{
	cv::Mat image;
	double timestamp;
};

struct imu_with_timestamp
{
	double timestamp;

	double linear_acceleration_x;
	double linear_acceleration_y;
	double linear_acceleration_z;

	double angular_velocity_x;
	double angular_velocity_y;
	double angular_velocity_z;
};

}
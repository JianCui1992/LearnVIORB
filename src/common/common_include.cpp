#include "common_include.h"

namespace ORBVIO
{

double IMAGE_DELAY_TIME;
std::string VOC_PATH;
std::string IMAGE_PATH;
std::string IMU_PATH;
// std::vector<Eigen::Matrix3d> RIC;
// std::vector<Eigen::Vector3d> TIC;
// cv::Mat cv_K;
// cv::Mat cv_D;

void read_parameters(const std::string &filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "ERROR: Wrong path to setting" << std::endl;
    }
    IMAGE_PATH = (std::string)fs["image_path"];
    IMU_PATH = (std::string)fs["imu_path"];
    VOC_PATH = (std::string)fs["voc_path"];
    IMAGE_DELAY_TIME = fs["td"];

    // cv::Mat cv_R, cv_T;
    // fs["extrinsicRotation"] >> cv_R;
    // fs["extrinsicTranslation"] >> cv_T;
    // Eigen::Matrix3d eigen_R;
    // Eigen::Vector3d eigen_T;
    // cv::cv2eigen(cv_R, eigen_R);
    // cv::cv2eigen(cv_T, eigen_T);
    // Eigen::Quaterniond Q(eigen_R);
    // eigen_R = Q.normalized();
    // RIC.push_back(eigen_R);
    // TIC.push_back(eigen_T);

    // fs["CAMERA.intrinsics.K"] >> cv_K;
    // fs["CAMERA.intrinsics.D"] >> cv_D;
}
}
#include <thread>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>


#include <opencv2/core/core.hpp>

#include "System.h"

#include "common_include.h"
#include "data.h"

#include "IMU/configparam.h"
#include "IMU/imudata.h"

using namespace std;


int main(int argc, char **argv)
{

    ORBVIO::read_parameters(argv[1]);

    thread *img_callback_thread;
    thread *imu_callback_thread;

    ORB_SLAM2::System SLAM(ORBVIO::VOC_PATH, argv[1],
                           ORB_SLAM2::System::MONOCULAR, true);

    ORB_SLAM2::ConfigParam config(argv[1]);

    /**
     * @brief added data sync
     */

    ORBVIO::data_base *mdata_base = new ORBVIO::data_base();

    ORBVIO::image_with_timestamp image_message;
    std::vector<ORBVIO::imu_with_timestamp> v_imu_message;

    img_callback_thread =
        new thread(&ORBVIO::data_base::load_img_from_file, mdata_base);

    imu_callback_thread =
        new thread(&ORBVIO::data_base::load_imu_from_file, mdata_base);

    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();

    while (1)
    {

        std::unique_lock<std::mutex> lk(mdata_base->MutexMeasurements);
        mdata_base->con.wait(lk, [&]{ return (mdata_base->get_recent_messages(image_message, v_imu_message)); });
        lk.unlock();

        ORB_SLAM2::IMUData::vector_t vimuData;
        // ROS_INFO("image time: %.3f",imageMsg->header.stamp.toSec());
        for (unsigned int i = 0; i < v_imu_message.size(); i++)
        {
            ORBVIO::imu_with_timestamp imu_msg = v_imu_message[i];
            double ax = imu_msg.linear_acceleration_x;
            double ay = imu_msg.linear_acceleration_y;
            double az = imu_msg.linear_acceleration_z;
            if (bAccMultiply98)
            {
                ax *= g3dm;
                ay *= g3dm;
                az *= g3dm;
            }
            ORB_SLAM2::IMUData imudata(
                imu_msg.angular_velocity_x, imu_msg.angular_velocity_y,
                imu_msg.angular_velocity_z, ax, ay, az, imu_msg.timestamp);
            vimuData.push_back(imudata);
        }

        cv::Mat im = image_message.image.clone();
        {
            // To test relocalization
            static double startT = -1;
            if (startT < 0)
                startT = image_message.timestamp;
            // Below to test relocalizaiton
            // if(imageMsg->header.stamp.toSec() > startT+25 &&
            // imageMsg->header.stamp.toSec() < startT+25.3)
            if (image_message.timestamp <
                startT + config._testDiscardTime)
                im = cv::Mat::zeros(im.rows, im.cols, im.type());
        }
        SLAM.TrackMonoVI(im, vimuData,
                         image_message.timestamp - ORBVIO::IMAGE_DELAY_TIME);

        // Wait local mapping end.
        bool bstop = false;
        while (!SLAM.bLocalMapAcceptKF())
        {
            if(mdata_base->end_flags)
                bstop = true;
        };
        if (bstop)
            break;
        
    }

    
    SLAM.SaveKeyFrameTrajectoryNavState(config._tmpFilePath +
                                        "KeyFrameNavStateTrajectory.txt");

    cout << endl << endl << "press any key to shutdown" << endl;
    getchar();

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

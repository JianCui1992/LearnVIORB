#pragma once

#include "common_include.h"
#include "tic_toc.h"
#include <mutex>
#include <queue>
#include <condition_variable>

using namespace std;

namespace ORBVIO
{
class data_base
{
  public:
    enum Status
    {
        NOTINIT = 0,
        INIT,
        NORMAL
    };

    data_base();
    ~data_base();

    // add messages in callbacks
    void add_image_message(const image_with_timestamp &image_message);
    void add_imu_message(const imu_with_timestamp &imu_message);

    void load_imu_from_file();
    void load_img_from_file();

    // loop in main function to handle all messages
    bool get_recent_messages(image_with_timestamp &image_message,
                             std::vector<imu_with_timestamp> &v_imu_message);

    void ClearBuffer(void);

    inline Status getStatus(void) { return status; }

    bool end_flags = false;
    std::mutex MutexMeasurements;
    std::condition_variable con;

  private:
    Status status;
    int data_unsync_count = 0;




    std::queue<image_with_timestamp> image_buf;
    std::queue<imu_with_timestamp> imu_buf;

    bool is_first_imu = true;
    bool is_first_img = true;

    double last_imu_time = 0.0;
    double last_img_time = 0.0;
    double imu_start_timestamp;
};
}

#include "data.h"
#include "IMU/configparam.h"

namespace ORBVIO
{

data_base::data_base() : status(NOTINIT)
{
    printf("image delay set as %.1fms\n", IMAGE_DELAY_TIME * 1000);
}

data_base::~data_base() {}

bool data_base::get_recent_messages(
    image_with_timestamp &image_message,
    std::vector<imu_with_timestamp> &v_imu_message)
{
    if (status == NOTINIT || status == INIT)
    {
        return false;
    }
    if (image_buf.empty())
    {
        printf("no image stored in queue currently.\n");
        return false;
    }
    if (imu_buf.empty())
    {
        printf("no imu message stored, shouldn't. \n");
        return false;
    }

    {
        image_with_timestamp image_msg;
        imu_with_timestamp imu_msg;
        //
        image_msg = image_buf.back();
        imu_msg = imu_buf.front();

        // Check dis-continuity, tolerance 3 seconds
        if (image_msg.timestamp - IMAGE_DELAY_TIME + 3.0 < imu_msg.timestamp)
        {
            printf(
                "IMAGE Data dis-continuity, > 3 seconds. Buffer cleared. \n");
            ClearBuffer();
            return false;
        }

        //
        image_msg = image_buf.front();
        imu_msg = imu_buf.back();

        // Wait imu messages in case communication block
        if (image_msg.timestamp - IMAGE_DELAY_TIME > imu_msg.timestamp)
        {
            return false;
        }

        // Check dis-continuity, tolerance 3 seconds
        if (image_msg.timestamp - IMAGE_DELAY_TIME > imu_msg.timestamp + 3.0)
        {
            printf("IMU Data dis-continuity, > 3 seconds. Buffer cleared. \n");
            ClearBuffer();
            return false;
        }

        // Wait until the imu packages totaly come
        if (image_buf.size() < 10 && imu_buf.size() < 15 &&
            image_msg.timestamp - IMAGE_DELAY_TIME > imu_msg.timestamp)
        {
            return false;
        }
    }

    // get image message
    image_message = image_buf.front();
    image_buf.pop();

    // clear imu message vector, and push all imu messages whose timestamp is
    // earlier than image message
    v_imu_message.clear();
    while (true)
    {
        // if no more imu messages, stop loop
        if (imu_buf.empty())
            break;

        // consider delay between image and imu serial
        imu_with_timestamp temp_imu_msg = imu_buf.front();
        if (temp_imu_msg.timestamp < image_message.timestamp - IMAGE_DELAY_TIME)
        {
            // add to imu message vector
            v_imu_message.push_back(temp_imu_msg);
            {
                imu_buf.pop();
            }

            data_unsync_count = 0;
        } else
        {
            if (data_unsync_count++ > 10)
            {
                data_unsync_count = 0;
                ClearBuffer();
                printf("data unsynced many times, reset sync \n");
                return false;
            }
            // stop loop
            break;
        }
    }

    // the camera fps 20Hz, imu message 100Hz. so there should be not more than
    // 5 imu messages between images
    if (v_imu_message.size() > 10)
        printf("WARNING: %lu imu messages between images, note \n",
               v_imu_message.size());
    if (v_imu_message.size() == 0)
        printf("WARNING: no imu message between images! \n");

    return true;
}

void data_base::add_imu_message(const imu_with_timestamp &imu_message)
{
    if (IMAGE_DELAY_TIME >= 0)
    {
        imu_buf.push(imu_message);
        if (status == NOTINIT)
        {
            imu_start_timestamp = imu_message.timestamp;
            status = INIT;
        }
    } else
    {
        // if there's no image messages, don't add image
        if (status == NOTINIT)
            return;
        else if (status == INIT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if (imu_message.timestamp + IMAGE_DELAY_TIME > imu_start_timestamp)
            {
                imu_buf.push(imu_message);
                status = NORMAL;
            }
        } else
        {
            // push message into queue
            imu_buf.push(imu_message);
        }
    }
}

void data_base::load_imu_from_file()
{
    std::ifstream fimu(IMU_PATH);
    if (fimu.is_open())
    {
        double var0, var1, var2, var3, var4, var5, var6;
        imu_with_timestamp imu_;
        while (fimu >> var0 >> var1 >> var2 >> var3 >> var4 >> var5 >> var6)
        {

            TicToc t_imu;
            imu_.angular_velocity_x = var0;
            imu_.angular_velocity_y = var1;
            imu_.angular_velocity_z = var2;
            imu_.linear_acceleration_x = var3;
            imu_.linear_acceleration_y = var4;
            imu_.linear_acceleration_z = var5;
            imu_.timestamp = var6 - TIME_SHIFT;
            MutexMeasurements.lock();
            add_imu_message(imu_);
            MutexMeasurements.unlock();
            con.notify_one();

            double dt;
            if (is_first_imu)
            {
                dt = 0.01;
                is_first_imu = false;
            } else
            {
                dt = imu_.timestamp - last_imu_time - t_imu.toc() / 1000;
            }
            last_imu_time = imu_.timestamp;

            // //std::cout<<"dt: "<<dt<<std::endl;
            if (dt > 0)
            {
                usleep(dt * 1000000);
            }
        }
    }
}

void data_base::add_image_message(const image_with_timestamp &image_message)
{
    if (IMAGE_DELAY_TIME >= 0)
    {
        // if there's no imu messages, don't add image
        if (status == NOTINIT)
            return;
        else if (status == INIT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if (image_message.timestamp - IMAGE_DELAY_TIME >
                imu_start_timestamp)
            {
                image_buf.push(image_message);
                status = NORMAL;
            }
        } else
        {
            // push message into queue
            image_buf.push(image_message);
        }
    } else
    { // start by image message
        if (status == NOTINIT)
        {
            imu_start_timestamp = image_message.timestamp;
            status = INIT;
        } else
        { // no image data if there's no imu message
            image_buf.push(image_message);
        }
    }

    if (ORB_SLAM2::ConfigParam::GetRealTimeFlag())
    {
        // Ignore earlier frames
        if (image_buf.size() > 2)
            image_buf.pop();
    }
}

void data_base::load_img_from_file()
{
    std::ifstream fimage;
    std::string strPathImageFile = IMAGE_PATH + "image_name.txt";
    std::vector<std::string> v_image_name;
    std::vector<double> v_time_stamp;

    fimage.open(strPathImageFile.c_str());
    std::string strPrefixLeft = IMAGE_PATH;

    while (!fimage.eof())
    {
        std::string s;
        getline(fimage, s);
        if (!s.empty())
        {
            v_image_name.push_back(strPrefixLeft + s);

            std::stringstream stime(s.substr(0, 17));
            double time;
            stime >> time;
            v_time_stamp.push_back(time - TIME_SHIFT);
        }
    }
    printf("The image load over, total %ld .\n", v_image_name.size());
    for (unsigned int i = 0; i < v_image_name.size(); i++)
    {
        TicToc t_img;
        image_with_timestamp image_msg;

        image_msg.image =
            cv::imread(v_image_name[i], CV_LOAD_IMAGE_UNCHANGED).clone();
        image_msg.timestamp = v_time_stamp[i];
        MutexMeasurements.lock();
        add_image_message(image_msg);
        MutexMeasurements.unlock();
        con.notify_one();
        double dt;
        if (is_first_img)
        {
            dt = 0.01;
            is_first_img = false;
        } else
        {
            dt = v_time_stamp[i] - last_img_time - t_img.toc() / 1000;
        }

        last_img_time = v_time_stamp[i];
        if (dt > 0)
        {
            usleep(dt * 1000000);
        }
    }
    end_flags = true;
}

void data_base::ClearBuffer(void)
{
    image_buf = std::queue<image_with_timestamp>();
    imu_buf = std::queue<imu_with_timestamp>();
}

}

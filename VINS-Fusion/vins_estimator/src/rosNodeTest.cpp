/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

// 实例化 Estimator 对象
Estimator estimator;

// 声明 全局变量 imu_buf feature_buf img0_buf img1_buf， 使多个线程可以访问。用数据结构queue存储，先进先出
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;

// 线程互斥锁 作用：由于 ROS（Robot Operating System）通常采用多线程回调机制，
// 可能会有多个回调函数并发访问 sensor_buf，从而引发数据竞争
std::mutex m_buf;

// 订阅传感器话题后，执行img0的回调函数，
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // 在push新消息前加锁，确保只有一个线程可以push,由于 queue 不是线程安全的数据结构，
    // 多个线程同时修改 img0_buf 可能会导致数据损坏或未定义行为
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

// 订阅传感器话题后，执行img1的回调函数，
void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


// 将img_msg转成cv::Mat
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    // 如果传感器驱动发布的图像消息是8UC1则转成mono8，最后转成指向CvImage的指针
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        // 将sensor_msgs::Image 转成CvImage的指针
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
// 同步线程（可以理解成执行前端featuretracker的线程）
void sync_process()
{
    // while（1)说明这个线程一直在运行
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            // 此时已经订阅到相机话题了，img0_buf不为空
            if(!img0_buf.empty())
            {
                // img0_buf非空， 取队列中最老的一帧的时间戳
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                // 将图像存到cv::mat中，丢掉第一帧
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                // 要是cv::mat非空，将double类型的时间戳（秒制）和 cv::mat传给estimator的inputImage成员函数
                estimator.inputImage(time, image);
        }

        // 当前线程暂停 2ms （给前端featuretracker预留足够时间？？？）
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


// 订阅 imu 传感器后，imu消息的回调函数
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 只取时间戳和线加速度以及角速度
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    // 传入estimator的inputIMU成员函数accBuf、gyrBuf
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    // 根据命令行参数初始化ros vins_estimator 节点
    ros::init(argc, argv, "vins_estimator");
    // 定义主节点句柄
    ros::NodeHandle n("~");
    // 这行代码的作用是设置 ROS 日志的默认级别为 Info，即调整 ROS 日志的输出级别，使 INFO 级别及以上的日志消息可见
    // 这行使得ROS_DEBUG级别的log不可见
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::debug);

    // 命令行参数不为2，打印提示log， argc是传入参数的个数，在这里：argv[0] = "vins_node" , 
    // argv[1] = ""~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml
    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    // 将配置文件的path从命令行传入
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    // readParameters 在parameters.cpp中实现，将配置文件变量读到全局变量
    // 类似这种 extern double ACC_N, ACC_W;
    readParameters(config_file);
    // 将config文件中的参数传给estimator实例，用于初始化一些变量
    estimator.setParameter();

    // #ifdef是条件编译指令，用于检查 EIGEN_DONT_PARALLELIZE 是否被预定义
    // 未宏定义，表示没有禁止Eigen的并行化

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    // 定义ros对象用于订阅 imu 消息
    ros::Subscriber sub_imu;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }

    // 没有节点发布这个话题，什么也没有订阅到，可以注释掉，
    // vins根本没有发布这个话题, 方便换新前端直接发布这个话题，在这里订阅，因此提供了一个接口
    // ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;
    if(STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }
    // 订阅外部状态话题
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    // 创建一个新线程并执行，用于进行 estimator的图像数据 输入
    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}

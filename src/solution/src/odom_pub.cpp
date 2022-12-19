#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <fstream>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>
ros::Publisher odom_pub;
std::mutex mutex_lock;
double scan_period=1;
// int indexds=0;
double m_to_mm=1;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

void handle(const sensor_msgs::PointCloud2ConstPtr &cloud_ros) {
    mutex_lock.lock();
    pointCloudBuf.push(cloud_ros);
    mutex_lock.unlock();
}
void return_odom() {
    // double x = 0.0;
    // double y = 0.0;
    // double th = 0.0;
    // double vx = 0.1;
    // double vy = 0.0;
    // double vth = 0.0;
    // 
    std::ifstream ifs;
    ifs.open("/home/zsm/Downloads/SE3Data.txt",std::ios::in);
            if (!ifs.is_open()) {
                ROS_INFO("!read fail!!!!!!!!!!!!!!!!");
                return;
            }
    while (1)
    {
        if (!pointCloudBuf.empty())
        {
            
            double r11,r12,r13,r21,r22,r23,r31,r32,r33,x,y,z;
            ifs>>r11>>r12>>r13>>r21>>r22>>r23>>r31>>r32>>r33>>x>>y>>z;
            // if (indexds<402) {
            //     indexds++;
            //     continue;
            // }
            // r11*=m_to_mm,r12*=m_to_mm,r13*=m_to_mm,
            // r21*=m_to_mm,r22*=m_to_mm,r23*=m_to_mm,
            // r31*=m_to_mm,r32*=m_to_mm,r33*=m_to_mm,
            x*=m_to_mm,y*=m_to_mm,z*=m_to_mm;
            // ROS_INFO("%lf,%lf,%lf",r11,r12,r13);
            // ROS_INFO("IS: %lf %lf %lf",x,y,z);
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(0,0,0) );
            Eigen::Matrix3d rota_mat;
            rota_mat<<  r11,r12,r13,
                        r21,r22,r23,
                        r31,r32,r33;
            Eigen::AngleAxisd rotation_vector(rota_mat);
            Eigen::Quaterniond qq(rotation_vector);
            tf::Quaternion q(qq.x(),qq.y(),qq.z(),qq.w());
            // ROS_INFO("%lf,%lf,%lf,%lf",qq.x(),qq.y(),qq.z(),qq.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
            nav_msgs::Odometry laserOdometry;
            ros::Time pointcloud_time=ros::Time::now();
            // laserOdometry.header.frame_id = "map";
            // laserOdometry.child_frame_id ="base_link";
            laserOdometry.header.frame_id ="base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = qq.x();
            laserOdometry.pose.pose.orientation.y = qq.y();
            laserOdometry.pose.pose.orientation.z = qq.z();
            laserOdometry.pose.pose.orientation.w = qq.w();
            laserOdometry.pose.pose.position.x = x*m_to_mm*1000;
            laserOdometry.pose.pose.position.y = y*m_to_mm*1000;
            laserOdometry.pose.pose.position.z = z*m_to_mm;
            odom_pub.publish(laserOdometry);
            // ROS_INFO("ODOM");
        }
        std::chrono::milliseconds dura(3);
        std::this_thread::sleep_for(dura);
    }
    ifs.close();
}
int main(int argc,char** argv) {
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/d3", 100, handle);
    ROS_INFO("ODOM running");
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread handler{return_odom};
    ros::spin();
}
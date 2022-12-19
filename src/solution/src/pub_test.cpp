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
#include <queue>
ros::Publisher odom_pub;
std::mutex mutex_lock;
double scan_period=0.1;
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
    double l=0.000001;
    while (l)
    {
        
        if (!pointCloudBuf.empty())
        {
        
            mutex_lock.lock();
            ros::Time pointcloud_time =pointCloudBuf.front()->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(0, 0, 0) );
            tf::Quaternion q(0,0,0,1);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map"; 
            laserOdometry.child_frame_id = "base_link"; 
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = 0;
            laserOdometry.pose.pose.orientation.y = 0;
            laserOdometry.pose.pose.orientation.z = 0;
            laserOdometry.pose.pose.orientation.w = 1;
            laserOdometry.pose.pose.position.x = l;
            laserOdometry.pose.pose.position.y = 0;
            laserOdometry.pose.pose.position.z = 0;
            odom_pub.publish(laserOdometry);
        }
        l+=0.1;
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    
}
int main(int argc,char** argv) {
    ros::init(argc, argv, "pub_test");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/d3", 100, handle);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread handler{return_odom};
    ros::spin();
}
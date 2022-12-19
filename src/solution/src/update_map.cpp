#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
double mm_to_m= 1
//0.001
;
std::mutex mutex_lock;
ros::Publisher map_pub;
double scan_period=1;
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_map(new pcl::PointCloud<pcl::PointXYZ>);
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();

void handle(const sensor_msgs::PointCloud2ConstPtr &cloud_ros) {
    mutex_lock.lock();
    pointCloudBuf.push(cloud_ros);
    mutex_lock.unlock();
}
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}
void update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,Eigen::Isometry3d pose) {
    for (size_t i = 0; i < cloud_in->size(); i++)
    {
        pcl::PointXYZ point_temp=cloud_in->points[i];
        Eigen::Vector3d temp(point_temp.x,point_temp.y,point_temp.z);
        temp=pose*temp;
        point_temp.x=temp[0];
        point_temp.y=temp[1];
        point_temp.z=temp[2];
        lidar_map->push_back(point_temp);
    }
    
}
void mapping() {
    while (1)
    {
        if (!odometryBuf.empty() &&!pointCloudBuf.empty()) {
            mutex_lock.lock();
            // ROS_INFO("1");
            // if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*scan_period){
            //     ROS_WARN("mapping::time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
            //     pointCloudBuf.pop();
            //     mutex_lock.unlock();
            //     continue;              
            // }
            // if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()-0.5*scan_period){
            //     odometryBuf.pop();
            //     ROS_INFO("mapping::time stamp unaligned with path final, pls check your data --> laser mapping node");
            //     mutex_lock.unlock();
            //     continue;  
            // }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *cloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();

            Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
            double displacement = delta_transform.translation().squaredNorm();
            double angular_change = delta_transform.linear().eulerAngles(2,1,0)[0]* 180 / M_PI;

            if(angular_change>90) {
                // ROS_INFO("2:%lf",angular_change);

                angular_change = fabs(180 - angular_change);
            }
            ROS_INFO("%f,%f",displacement,angular_change);
            if(displacement> 5*mm_to_m || angular_change>3){
                mutex_lock.lock();
                // ROS_INFO("update map %f,%f",displacement,angular_change);
                last_pose = current_pose;
                update(cloud_in,current_pose);
                sensor_msgs::PointCloud2 PointsMsg;
                mutex_lock.unlock();
                
                pcl::toROSMsg(*lidar_map, PointsMsg);
                PointsMsg.header.stamp = pointcloud_time;
                PointsMsg.header.frame_id = "base_link";
                map_pub.publish(PointsMsg); 
                ROS_INFO("MAP");
            }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    
}
int main(int argc,char** argv) {
    ros::init(argc, argv, "update_map");
    ros::NodeHandle nh;
    ROS_INFO("update_map running");
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/cyc_pc", 100, handle);
    // ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/d3", 100, handle);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    std::thread handler{mapping};
    ros::spin();
}
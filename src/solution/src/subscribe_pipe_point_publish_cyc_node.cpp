#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
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
std::mutex mutex_lock;
ros::Publisher cyc_points;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();

int update_count = 0;
int frame_id=0;
double r=0.08;

void mk_cyc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Eigen::Isometry3d pose) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->size(); i++)
    {
        Eigen::Vector3d temp(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
        temp=pose*temp;
        pcl::PointXYZ pl(temp[0],temp[1],temp[2]);
        cloud_temp->push_back(pl);
    }
    cloud->clear();
    std::sort(cloud_temp->begin(),cloud_temp->end(),[](pcl::PointXYZ pt1,pcl::PointXYZ pt2) {
        if (fabs(pt1.x-pt2.x)<0.00001)
        {
            return pt1.y<pt2.y;
        }
        return pt1.x<pt2.x;
    });
    
}

void handle(sensor_msgs::PointCloud2ConstPtr &cloud_ros) {
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMap() {

}
void cyc_pub() {
    while (1)
    {
        if (!odometryBuf.empty() &&!pointCloudBuf.empty())
        {
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*1){
                ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()-0.5*1){
                odometryBuf.pop();
                ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node");
                mutex_lock.unlock();
                continue;  
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            
            ++update_count;
            mk_cyc(pointcloud_in,current_pose);
            Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
            double displacement = delta_transform.translation().squaredNorm();
            double angular_change = delta_transform.linear().eulerAngles(2,1,0)[0]* 180 / M_PI;

            if(angular_change>90) angular_change = fabs(180 - angular_change);
            
            if(displacement>0.3 || angular_change>20){
                //ROS_INFO("update map %f,%f",displacement,angular_change);
                last_pose = current_pose;
                // laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);

                // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_map = getMap();
                // sensor_msgs::PointCloud2 PointsMsg;
                
                
                // pcl::toROSMsg(*pc_map, PointsMsg);
                // PointsMsg.header.stamp = pointcloud_time;
                // PointsMsg.header.frame_id = "map";
                // cyc_points.publish(PointsMsg); 
            }
        }
        
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    
}
int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    
    ros::init(argc,argv,"subscribe_pipe_point_publish_cyc");
    ros::NodeHandle nh;
    ros::Subscriber sub_pipe = nh.subscribe("/pipe_point",1000,handle);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    cyc_points = nh.advertise<sensor_msgs::PointCloud2Ptr>("/cyc_points",100);
    std::thread cyc{cyc_pub};
    ros::spin();
    return 0;
}
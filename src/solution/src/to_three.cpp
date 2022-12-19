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
const double mm_to_m=1
//0.001
;
std::mutex mutex_lock;
ros::Publisher d3_pub;
std::queue<sensor_msgs::PointCloudConstPtr> pointCloudBuf;
void handle(const sensor_msgs::PointCloudConstPtr &cloud_ros) {
    mutex_lock.lock();
    sensor_msgs::PointCloud temp=*cloud_ros;
    for (size_t i = 0; i < temp.points.size(); i++)
    {
        temp.points[i].x*=mm_to_m;
        temp.points[i].y*=mm_to_m;
    }
    
    pointCloudBuf.push(cloud_ros);
    mutex_lock.unlock();
}
void up_to_three() {
    ROS_INFO("to_three running...");
    while (1)
    {
        if (!pointCloudBuf.empty())
        {
        mutex_lock.lock();
        sensor_msgs::PointCloudConstPtr d2= pointCloudBuf.front();
        ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
        pointCloudBuf.pop();
        mutex_lock.unlock();
        sensor_msgs::PointCloud2Ptr d3(new sensor_msgs::PointCloud2);
        // for (size_t i = 0; i < d2->points.size(); i++)
        // {
        //     d2->points[i].x*=mm_to_m;
        //     d2->points[i].y*=mm_to_m;
        //     d2->points[i].z*=mm_to_m;
        // }
        sensor_msgs::convertPointCloudToPointCloud2(*d2,*d3);

        sensor_msgs::PointCloud2 d3pc=*d3;
        (d3pc).header.stamp = pointcloud_time;
        (d3pc).header.frame_id = "base_link";
        
        d3_pub.publish(d3pc);
        }
        
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    

    
}
int main(int argc,char** argv) {
    ros::init(argc, argv, "to_three");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud>("/point_cloud", 100, handle);
    d3_pub = nh.advertise<sensor_msgs::PointCloud2>("/d3", 100);
    std::thread two_to_three{up_to_three};
    ros::spin();
}
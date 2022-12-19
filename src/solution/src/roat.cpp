#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
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
#include <opencv2/opencv.hpp>
// typedef std::pair<double,double> Point;
double mm_to_m= 1
//0.001
;
std::mutex mutex_lock;
ros::Publisher roat_pub;
double scan_period=5;
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

std::pair<double,double> polyfit(std::vector<cv::Point>& in_point, int n)
{
	int size = in_point.size();
	//所求未知数个数
	int x_num = n + 1;
	//构造矩阵U和Y
	cv::Mat mat_u(size, x_num, CV_64F);
	cv::Mat mat_y(size, 1, CV_64F);
 
	for (int i = 0; i < mat_u.rows; ++i)
		for (int j = 0; j < mat_u.cols; ++j)
		{
			mat_u.at<double>(i, j) = pow(in_point[i].x, j);
		}
 
	for (int i = 0; i < mat_y.rows; ++i)
	{
		mat_y.at<double>(i, 0) = in_point[i].y;
	}
 
	//矩阵运算，获得系数矩阵K
	cv::Mat mat_k(x_num, 1, CV_64F);
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	// std::cout << mat_k << std::endl;
    double b = mat_k.at<double>(0, 0);
    double k = mat_k.at<double>(1,0);
	return std::make_pair(k,b);
}

void return_roat() {
    std::vector<double> pub(3);
    while (1)
    {
        if (!odometryBuf.empty() &&!pointCloudBuf.empty()) {
            // ROS_INFO("OK");
            mutex_lock.lock();
            // if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*scan_period){
            //     ROS_WARN("ROAT::time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
            //     pointCloudBuf.pop();
            //     mutex_lock.unlock();
            //     continue;              
            // }
            // if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()-0.5*scan_period){
            //     odometryBuf.pop();
            //     ROS_INFO("ROAT::time stamp unaligned with path final, pls check your data --> laser mapping node");
            //     mutex_lock.unlock();
            //     continue;  
            // }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *cloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,0));
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            current_pose(2,1)=0,current_pose(2,0)=0;
            current_pose(2,2)=1,current_pose(1,2)=0;
            current_pose(0,2)=0;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (size_t i = 0; i < cloud_in->size(); i++)
            {
                pcl::PointXYZ temp=cloud_in->points[i];
                Eigen::Vector3d t(temp.x,temp.y,temp.z);
                t=current_pose.inverse()*t;
                temp.x=t[0],temp.y=t[1],temp.z=t[2];
                if (temp.y>0)//!!!!!!
                {
                    continue;
                }
                
                cloud->push_back(temp);
            }
            std::sort(cloud->begin(),cloud->end(),[](pcl::PointXYZ pt1,pcl::PointXYZ pt2) {
                return pt1.x<pt2.x;
            });
            std::vector<cv::Point> in_point;
            if(cloud->size()<10) continue;
            for (size_t i = 0; i < cloud->size()/4; i++)
            {
                in_point.push_back(cv::Point(cloud->points[i].x,cloud->points[i].y));
            }
            std::pair<double,double> xs4 =polyfit(in_point,1);
            for (size_t i = cloud->size()/4+1; i < cloud->size()/2; i++)
            {
                in_point.push_back(cv::Point(cloud->points[i].x,cloud->points[i].y));
            }
            std::pair<double,double> xs2 =polyfit(in_point,1);
            for (size_t i = cloud->size()/2+1; i < cloud->size(); i++)
            {
                in_point.push_back(cv::Point(cloud->points[i].x,cloud->points[i].y));
            }
            std::pair<double,double> xs1 =polyfit(in_point,1);
            // double k=0,x2=0,y2=0,x1,y1;
            // int step=2;
            double ang=0;
            double k4=xs4.first,b4=xs4.second;
            double k2=xs2.first,b2=xs2.second;
            double k1=xs1.first,b1=xs1.second;
            if (fabs(k1-k2)>1||fabs(k2-k4)>1||fabs(k1-k4)>1) {
                std::vector<double> re;
                std_msgs::Float64MultiArray p;
                re.push_back(ang);
                re.push_back(pointcloud_time.toSec());
                p.data=re;
                roat_pub.publish(p);
                continue;
            }
            
            double xx,yy,a,bb,c;
            double x=1,y=b2,x1=0,y1=b2;
            yy=k2*x+b2;
            xx=x;
            a=sqrt((y-yy)*(y-yy)+(x-xx)*(x-xx));
            bb=sqrt((yy-y1)*(yy-y1)+(xx-x1)*(xx-x1));
            c=sqrt((y-y1)*(y-y1)+(x-x1)*(x-x1));
            ang= atan2(a,bb);
            if (fabs(k2)>0.5)
            {
                // ROS_INFO("%lf,%lf",k2,90-ang*180/M_PI);
            }
                
                    std::vector<double> re;
                    std_msgs::Float64MultiArray p;
                    re.push_back(ang);
                    re.push_back(pointcloud_time.toSec());
                    p.data=re;
                    roat_pub.publish(p);
            
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc,char** argv) {
    ros::init(argc, argv, "roat");
    ROS_INFO("roat running");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/d3", 100, handle);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    roat_pub = nh.advertise<std_msgs::Float64MultiArray>("/roat", 100);
    std::thread handler{return_roat};
    ros::spin();
}
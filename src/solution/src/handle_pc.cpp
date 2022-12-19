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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
double mm_to_m= 1
//0.001
;
double lastx=0,lasty=0;
double scan_period=1;
pcl::VoxelGrid<pcl::PointXYZ> voxel;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
std::mutex mutex_lock;
ros::Publisher cyc_pub;
double r=140*mm_to_m;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
std::queue<std_msgs::Float64MultiArrayConstPtr> roatBuf;
void handle(const sensor_msgs::PointCloud2ConstPtr &cloud_ros) {
    mutex_lock.lock();
    pointCloudBuf.push(cloud_ros);
    mutex_lock.unlock();
}
void handle_roat(const std_msgs::Float64MultiArrayConstPtr &roat_ros) {
    mutex_lock.lock();
    roatBuf.push(roat_ros);
    mutex_lock.unlock();
}
void filt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,int neigb) {

    // 设置输入点云
	voxel.setInputCloud(cloud_in);//给滤波对象设置需过滤的点云
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);//设置滤波时创建的体素大小为1cm*1cm*1cm的立方体
    voxel.filter(*cloud_out);
    // 设置体素网格的大小
	
	sor.setInputCloud(cloud_out);//设置待滤波的点云
	sor.setMeanK(neigb);//设置在进行统计时考虑查询点邻居点数
	sor.setStddevMulThresh(1.0);//设置判断是否为离群点的阈值
	sor.filter(*cloud_out);//将滤波结果保存在cloud_filtered中
}
void pc_handler() {
    ROS_INFO("handle_pc running...");
    while (1)
    {
        if (!pointCloudBuf.empty()&&!roatBuf.empty())
        {
            // if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<((roatBuf.front()->data[1])-0.5*scan_period)){
            //     ROS_WARN("handle::time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
            //     pointCloudBuf.pop();
            //     mutex_lock.unlock();
            //     continue;              
            // }
            // if(!roatBuf.empty() && roatBuf.front()->data[1] < pointCloudBuf.front()->header.stamp.toSec()-0.5*scan_period){
            //     roatBuf.pop();
            //     ROS_INFO("handle::time stamp unaligned with path final, pls check your data --> laser mapping node");
            //     mutex_lock.unlock();
            //     continue;  
            // }
            
            mutex_lock.lock();
            double ang=roatBuf.front()->data[0];
            roatBuf.pop();
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
            // pcl::fromROSMsg(*pointCloudBuf.front(),*raw_cloud);
            pcl::fromROSMsg(*pointCloudBuf.front(),*raw);
            pointCloudBuf.pop();
            mutex_lock.unlock();
            // if (raw_cloud->empty())
            // {
            //     continue;
            // }
            
            // filt(raw_cloud,raw,5);
            
            std::sort(raw->begin(),raw->end(),[](pcl::PointXYZ pt1,pcl::PointXYZ pt2) {
                if (fabs(pt1.x-pt2.x)<2) {
                    return pt1.y<pt2.y;
                }
                return pt1.x<pt2.x;
            });
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < raw->size()-2; i+=2)
            {
                //ROS_INFO("1:%lf 2:%lf\n",raw->points[i].x,raw->points[i+1].x);
                if (fabs(raw->points[i].x-raw->points[i+1].x)<2*mm_to_m && raw->points[i+1].y-raw->points[i].y>55*mm_to_m) {
                    std::pair<double,double> cyc_mid=std::make_pair((raw->points[i].x+raw->points[i+1].x)/2,(raw->points[i].y+raw->points[i+1].y)/2);
                    lastx=std::max(lastx,cyc_mid.first),lasty=std::max(lasty,cyc_mid.second);
                    for (double theta = 0; theta <= 2*M_PI; theta+=18*M_PI/180)
                    {
                        double x,y,z;
                        x=cyc_mid.first;
                        y=r*cos(theta);
                        z=r*sin(theta);
                        pcl::PointXYZ p(x,y,z);
                        //ROS_INFO("x:%lf y:%lf z:%lf\n",x,y,z);
                        cloud_out->push_back(p);
                    }
                    
                }
                
                if(!roatBuf.empty()&&fabs(pointcloud_time.toSec()-roatBuf.front()->data[1])<0.5*scan_period) {
                    
                    Eigen::AngleAxisd temp(ang, Eigen::Vector3d(0, 0, -1));
                    for (size_t i = 0; i < 3; i++)
                    {
                        for (double theta = 0; theta <= 2*M_PI; theta+=18*M_PI/180)
                        {
                            double x,y,z;
                            x=lastx+i;
                            y=r*cos(theta);
                            z=r*sin(theta);
                            Eigen::Vector3d tp(x,y,z);
                            tp=temp*tp;
                            pcl::PointXYZ p(tp[0],tp[1],tp[2]);
                            //ROS_INFO("x:%lf y:%lf z:%lf\n",x,y,z);
                            cloud_out->push_back(p);
                        }
                    }
                }
                
                
            }
            sensor_msgs::PointCloud2 PointsMsg;
            pcl::toROSMsg(*cloud_out, PointsMsg);
            PointsMsg.header.stamp = pointcloud_time;
            PointsMsg.header.frame_id = "base_link";
            cyc_pub.publish(PointsMsg); 
            // ROS_INFO("a cyc");

        }
        
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
    
}
int main(int argc,char** argv) {
    ros::init(argc, argv, "handle_pc");
    ros::NodeHandle nh;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/d3", 100, handle);
    ros::Subscriber subroat = nh.subscribe<std_msgs::Float64MultiArray>("/roat", 100, handle_roat);
    cyc_pub = nh.advertise<sensor_msgs::PointCloud2>("/cyc_pc_old", 100);
    std::thread handler{pc_handler};
    ros::spin();
}
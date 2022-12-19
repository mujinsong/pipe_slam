#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT1;
int pcd_view(std::string x) {
  
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile(x, *cloud);
  pcl::visualization::PCLVisualizer viewer;
    // 设置背景颜色 (0,0,0)小黑 (255,255,255)大白
    viewer.setBackgroundColor(255, 255, 255);
    //添加坐标系（即红绿蓝三色轴，放置在原点）
    viewer.addCoordinateSystem(3.0); //3.0指轴的长度
    //viewer.addCoordinateSystem (3.0,1,2,3);一个重载函数，3.0指轴的长度，放置在（1，2，3）位置
    //初始化默认相机参数
    viewer.initCameraParameters();
  pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb(cloud, 255, 0, 0);
    // 将点云加入到viewer
    viewer.addPointCloud<PointT>(cloud, "sample cloud33");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
int normal_view(pcl::PointCloud<PointT1>::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr normals) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("fx Viewer")); //创建视窗对象，定义标题栏名称“3D Viewer”
	viewer->addPointCloud<PointT1>(cloud_in, "original_cloud");	//将点云添加到视窗对象中，并定义一个唯一的ID“original_cloud”
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "original_cloud"); //点云附色，三个字段，每个字段范围0-1
	viewer->addPointCloudNormals<PointT1, pcl::Normal>(cloud_in, normals, 3, 0.5, "normals");	//每十个点显示一个法线，长度为0.05
  viewer->addCoordinateSystem(2.0);
  while (!viewer->wasStopped())
  {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
int cloud_view(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    viewer.initCameraParameters();
    viewer.addCoordinateSystem(1.0);
    viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0.5, "original_cloud"); //点云附色，三个字段，每个字段范围0-1
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud_MaxCurvature, 255, 0, 0);
    // viewer.addPointCloud<pcl::PointXYZ>(cloud_MaxCurvature, color, "cloud_MaxCurvature");
    // for (size_t i = 0; i < cloud->size(); ++i)
    // std::cout << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
    while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}



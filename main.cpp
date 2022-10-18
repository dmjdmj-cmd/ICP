/******************************************************************************** 
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： taify
** @date： 2021/01/13
** @desc： MyICP调用demo
** @Ver : V1.0.0
*********************************************************************************/

#include "myicp.h"
#include "myicp_helpers.h"
#include "string"
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
using namespace std;
using namespace pcl;
int main(int argc, char** argv)
{
	
	for(int i=269;i<574;i=i+2){
		ofstream myout("GT-shuchu-error.txt", ios::in | ios::out | ios::app);
		
		cout << i << endl;
		//加载点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		pcl::io::loadPCDFile("D:\\Microsoft\\pcd\\pcd\\" + to_string(i) + ".pcd", *source_cloud);
		int j = i + 2;
		if (i == 573)j = 1;
		pcl::io::loadPCDFile("D:\\Microsoft\\pcd\\pcd\\" + to_string(j) + ".pcd", *target_cloud);

		myout << i << " " << j << " ";
		myout.close();
		
		//运行算法
		MyICP icp;
		icp.setSourceCloud(source_cloud);
		icp.setTargetCloud(target_cloud);
		icp.setLeafSize(0.01);
		icp.downsample();
		icp.setMinError(0.001);
		icp.setMaxIters(40);
		icp.registration();
		//icp.saveICPCloud("icp_cloud.pcd");
		icp.getTransformationMatrix();
		//icp.getScore();
		//icp.visualize();
		
	}

	system("pause");
	
	return 0;
}
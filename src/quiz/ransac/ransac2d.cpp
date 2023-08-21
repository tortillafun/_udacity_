/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
			while(inliers.size()<2)
			{
				inliers.insert(std::rand()%(cloud->points.size()));

			}
	
		auto itr = inliers.begin();
		auto x1 = cloud->points[*itr].x;
		auto y1 = cloud->points[*itr].y;
		itr++;
		auto x2 = cloud->points[*itr].x;
		auto y2 = cloud->points[*itr].y;
	
		auto a = y1-y2;	
		auto b = x2-x1;
		auto c = (x1*y2-x2*y1);

		for(int i = 0 ; i < cloud->points.size();i++)
		{
			if(inliers.count(i)>0)
				continue;
		pcl::PointXYZ myPoint = cloud->points[i];
		auto y3 = myPoint.y;
		auto x3 = myPoint.x;

		auto distance = std::fabs(a*x3+b*y3+c)/std::sqrt(a*a+b*b);

		if(distance < distanceTol)
			inliers.insert(i);

		}

		if(inliers.size()>inliersResult.size())
			inliersResult = inliers;


	
}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::cout << "ransac plane is called/n";
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	while(maxIterations--)
	{
		std::unordered_set<int> inliers;
			while(inliers.size()<3)
			{
				inliers.insert(std::rand()%(cloud->points.size()));

			}
	
		auto itr = inliers.begin();
		auto x1 = cloud->points[*itr].x;
		auto y1 = cloud->points[*itr].y;
		auto z1 = cloud->points[*itr].z;
		itr++;
		auto x2 = cloud->points[*itr].x;
		auto y2 = cloud->points[*itr].y;
		auto z2 = cloud->points[*itr].z;
		itr++;
		auto x3 = cloud->points[*itr].x;
		auto y3 = cloud->points[*itr].y;
		auto z3 = cloud->points[*itr].z;

		auto a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);	
		auto b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);	
		auto c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		auto d = -1*(a*x1+b*y1+c*z1);

		for(int i = 0 ; i < cloud->points.size();i++)
		{
			if(inliers.count(i)>0)
				continue;
		pcl::PointXYZ myPoint = cloud->points[i];
		auto y4 = myPoint.y;
		auto x4 = myPoint.x;
		auto z4 = myPoint.z;

		auto distance = std::fabs(a*x4+b*y4+c*z4+d)/std::sqrt(a*a+b*b+c*c);

		if(distance < distanceTol)
			inliers.insert(i);

		}

		if(inliers.size()>inliersResult.size())
			inliersResult = inliers;


	
}



	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}
int main ()
{
	//duzeltmelereden sonraki yeni version 20/08/2023
	std::cout << "yeni program basladi\n";
	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane2(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

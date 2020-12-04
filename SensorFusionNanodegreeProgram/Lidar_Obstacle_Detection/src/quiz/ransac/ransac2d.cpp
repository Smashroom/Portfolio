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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for(auto iter=0;iter<maxIterations;iter++){
		// Randomly sample subset and fit line
		auto cand1 = rand()%cloud->size();
		auto cand2 = rand()%cloud->size();
		
		// Temp set to keep the values
		std::unordered_set<int> tempInliers;

		
		auto A = cloud->points[cand1].y - cloud->points[cand2].y;
		auto B = cloud->points[cand2].x - cloud->points[cand1].x;
		auto C = ((cloud->points[cand1].x)*(cloud->points[cand2].y) -
						  (cloud->points[cand2].x)*(cloud->points[cand1].y));
						  
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(auto idx=0;idx<cloud->size();idx++){
			auto cost = abs(A*(cloud->points[idx].x) + B*(cloud->points[idx].y)+C)/
							sqrt(pow(A,2) + pow(B,2));
			
			if(cost < distanceTol)
				tempInliers.insert(idx);
		}
		// Return indicies of inliers from fitted line with most inliers
	    if (tempInliers.size()>inliersResult.size())
			inliersResult = tempInliers;
	}
	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	for(auto iter=0;iter<maxIterations;iter++){
		// Randomly sample subset and fit line
		auto cand1 = rand()%cloud->size();
		auto cand2 = rand()%cloud->size();
		auto cand3 = rand()%cloud->size();
		
		// Temp set to keep the values
		std::unordered_set<int> tempInliers;
		
		auto A = (cloud->points[cand2].y-cloud->points[cand1].y)*(cloud->points[cand3].z-cloud->points[cand1].z) 
							 - (cloud->points[cand2].z-cloud->points[cand1].z)*(cloud->points[cand3].y-cloud->points[cand1].y);
		auto B = (cloud->points[cand2].z-cloud->points[cand1].z)*(cloud->points[cand3].x-cloud->points[cand1].x) 
							 - (cloud->points[cand2].x-cloud->points[cand1].x)*(cloud->points[cand3].z-cloud->points[cand1].z);
		auto C = (cloud->points[cand2].x-cloud->points[cand1].x)*(cloud->points[cand3].y-cloud->points[cand1].y) 
							 - (cloud->points[cand2].y-cloud->points[cand1].y)*(cloud->points[cand3].x-cloud->points[cand1].x);
		auto D = -(A*(cloud->points[cand1].x) + 
				   B*(cloud->points[cand1].y) +
				   C*(cloud->points[cand1].z));
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for(auto idx=0;idx<cloud->size();idx++){
			auto cost = abs(A*(cloud->points[idx].x) + B*(cloud->points[idx].y) + C*(cloud->points[idx].z) + D)/
							sqrt(pow(A,2) + pow(B,2) + pow(C,2));
			
			if(cost < distanceTol)
				tempInliers.insert(idx);
		}
		
		// Return indicies of inliers from fitted line with most inliers
	    if (tempInliers.size()>inliersResult.size())
			inliersResult = tempInliers;
	}
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

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

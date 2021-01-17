/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"

int main(int argc, char** argv)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;
	
	// nis stats harvesters
	std::vector<double>nis_lidar_stats;
	std::vector<double>nis_radar_stats;
	
	double nis_radar_k;
	double nis_lidar_k;
	
	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);

		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer, nis_radar_k, nis_lidar_k);
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;
		// Populate calculated nis
		nis_lidar_stats.push_back(nis_lidar_k);
		nis_radar_stats.push_back(nis_lidar_k);
	}
	
	// Process nis stats
			  
	// Chi-squared distribution stats
	double thresh5_lidar = 5.991; /* %5 distribution for 2D measurement*/
	double thresh5_radar = 7.815; /* %5 distribution for 3D measurement*/
	
	int numbOfinliers = 0;
	for(auto nis_lidar_stat: nis_lidar_stats){
		if(nis_lidar_stat > thresh5_lidar){
			numbOfinliers++;
		}
	}
	
	
	std::cout << "Nis stats for lidar: \n" 
			  << "Number of stats:" <<  nis_lidar_stats.size() << "\n"
			  << "Number of inliers:" << numbOfinliers << "\n";
			  
	numbOfinliers = 0;
	for(auto nis_radar_stat: nis_radar_stats){
		if(nis_radar_stat > thresh5_radar){
			numbOfinliers++;
		}
	}
	
	
	std::cout << "Nis stats for radar: \n" 
			  << "Number of stats:" <<  nis_radar_stats.size() << "\n"
			  << "Number of inliers:" << numbOfinliers << "\n";
}
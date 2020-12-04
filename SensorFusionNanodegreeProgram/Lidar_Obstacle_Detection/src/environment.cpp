/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    auto setGroundSlope = 0.0;
    // TODO:: Create lidar sensor 
    auto LidarSensor = new Lidar(cars,setGroundSlope);
    // Scan the environment
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud = LidarSensor->scan();
    
    // Create point processer and Render Rays
    //renderRays(viewer, LidarSensor->position, scan_lidar);
    //Color color(1.0,0.0,0.0);
    //renderPointCloud(viewer, LidarSensor->cloud,"pcl_cloud", color);
    
    // Initialise Point Processor
    ProcessPointClouds<pcl::PointXYZ> point_processor;
    // Segment the plane
    auto maxIterations = 100;
    auto distanceThreshold = 0.2;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, 
              pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = point_processor.SegmentPlane(laser_cloud,maxIterations,distanceThreshold);
//    renderPointCloud(viewer, segmentCloud.first,"obstacles", Color(1,0,0));
//    renderPointCloud(viewer, segmentCloud.second,"road", Color(0,1,0));   
    // Cluster the segmented obstacles
    auto clusterTolerance = 1.5;
    auto minSizeOfCluster = 3;
    auto maxSizeOfCluster = 40;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor.Clustering(segmentCloud.first, clusterTolerance, minSizeOfCluster, maxSizeOfCluster);
    
    int clusterId = 0;
    std::vector<Color> color_vect = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
      std::cout << "cluster size ";
      point_processor.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),color_vect[clusterId]);
      BoxQ boxq = point_processor.BoundingBoxQ(cluster);
      renderBox(viewer,boxq,clusterId);
      ++clusterId;
    }
    
    
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // PART 1: Filter the pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
  // Voxel leaf size
  float voxel_leaf_size = 0.25f;
  // Region of interest borders
  Eigen::Vector4f minROI(-10,-5,-2,1);
  Eigen::Vector4f maxROI(30,5,2,1);
  
  filterCloud = pointProcessorI->FilterCloud(inputCloud, voxel_leaf_size, minROI, maxROI); 
  
  // PART 2: Segment the pointcloud
  auto maxIterations = 100;
  auto distanceThreshold = 0.3;
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud,maxIterations,distanceThreshold);
  // segmentCloud: {obstacles, road}
  renderPointCloud(viewer, segmentCloud.second, "road", Color(1.0,0.0,0.0));

  // PART 3: Cluster the segmented obstacles pointcloud
  auto clusterTolerance = 0.3;
  auto minSizeOfCluster = 20;
  auto maxSizeOfCluster = 450;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, clusterTolerance, minSizeOfCluster, maxSizeOfCluster);
  
  // PART 4: Randomly color each cluster 
  srand((unsigned int)time(NULL));
  int clusterId = 0;
  
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
      float r = (float) rand()/RAND_MAX;
      float g = (float) rand()/RAND_MAX;
      float b = (float) rand()/RAND_MAX;
      Color color(r,g,b);
      std::cout << "r: " << r << "g: " << g << "b: " << b << "\n";
      std::cout << "cluster size ";
      pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),color);
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer,box,clusterId,color);
      ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // Initialise point processor and start to stream the pcd
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
        
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        
        viewer->spinOnce ();
    } 
}

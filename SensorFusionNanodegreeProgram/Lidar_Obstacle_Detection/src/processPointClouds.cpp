// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_av_filtered (new pcl::PointCloud<PointT>());

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    std::cout << "Size of cloud: " << cloud->size() << "\n";
    
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Apply the voxel grid filter
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_voxel_filtered);
   
    // Apply Region of interest
    pcl::CropBox<PointT> roi;
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloud_voxel_filtered);
    roi.filter(*cloud_filtered);
    
    // Crop out the AVs roof 
    std::vector<int> indices;    
    pcl::CropBox<PointT> roof(true);
    // AVs dimensions
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, 0.4, 1));
    roof.setInputCloud(cloud_filtered);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // Move indices to inliers object
    for (int point : indices)
    {
     inliers->indices.push_back(point);
    }
    
    // Extract roof inliers    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    
    std::cout << "Size of filtered cloud: " << cloud_filtered->size() << "\n";
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr road (new pcl::PointCloud<PointT>());
    
    for(auto index:inliers->indices){
        road->points.push_back(cloud->points[index]);
    }
    
    std::cout << "PointCloud representing the planar component: " << road->width * road->height << " data points." << std::endl;

    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacles);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
        
    // Initialise the variables
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  
    
    // Apply RANSAC on the given cloud
    std::unordered_set<int> inliers_set = Ransac3D(cloud, maxIterations, distanceThreshold);
    
    // Convert generated index set to vector
    std::vector<int> inliers_vect;
    inliers_vect.assign(inliers_set.begin(),inliers_set.end());
    
    inliers->indices = inliers_vect;
    
    if(inliers->indices.size () == 0){
        std::cerr << "Could not find a consistent planar object\n";
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::BuildKDtree(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree){
    // Insert the pointcloud to the tree
    for(auto idx=0;idx<cloud->points.size();idx++){
        tree->insert(cloud->points[idx],idx);
    }
}

template<typename PointT>
void ProcessPointClouds<PointT>::ProximityScan(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>* cluster, std::vector<bool>* points_visited, KdTree<PointT>* tree, int point_idx, float distanceTol, int maxSize)
{
	// Mark point is as visited
	points_visited->at(point_idx) = true;
	cluster->push_back(point_idx);
    
    if(cluster->size() < maxSize){
        // Check if cluster is still smaller than given Max Size of cluster 
    	std::vector<int> nearby = tree->search(cloud->points[point_idx],distanceTol);
    	for(int idx: nearby){
    		if(points_visited->at(idx) == false ){
    			ProximityScan(cloud, cluster, points_visited, tree, idx, distanceTol, maxSize);	
    		}
    	}
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
    // Initialise boolean vector to keep track of whether points are visited or not
	std::vector <bool> points_visited(cloud->size(),false);
	std::vector<std::vector<int>> clusters_idx;
	
    for(auto idx=0;idx< cloud->size();idx++){
		if(points_visited[idx] == false ){
			std::vector<int> points_indeces;	
			ProximityScan(cloud, &points_indeces, &points_visited, tree, idx, distanceTol, maxSize);
            // Check if cluster is bigger than given Min size
            if(points_indeces.size() > minSize)
                clusters_idx.push_back(points_indeces);
		}
	}
	return clusters_idx;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> result_cluster;

    // ######
    // Build the KD-Tree
    KdTree<PointT>* tree = new KdTree<PointT>;
    BuildKDtree(cloud, tree);
    
    std::vector<std::vector<int>> clusters = EuclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);
    
    for(std::vector<int> cluster : clusters)
  	{
        typename pcl::PointCloud<PointT>::Ptr cluster_pc(new pcl::PointCloud<PointT>());      
        for (int idx: cluster){
            cluster_pc->points.push_back(PointT(cloud->points[idx].x,cloud->points[idx].y,cloud->points[idx].z));    
        }
        result_cluster.push_back(cluster_pc);
    }
    /*
    // ### Code written with pcl built-in functions
    
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    
    // Initialise the EC and index parameter to store the output
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    // Set the configuration of EC
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    // Extract the indices
    ec.extract (cluster_indices);
      
    // Divide the indices and store them on the clusters
    
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        typename pcl::PointCloud<PointT>::Ptr tempCluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            tempCluster->push_back((*cloud)[*pit]);
        tempCluster->width = tempCluster->size();
        tempCluster->height = 1;
        tempCluster->is_dense = true;
        
        clusters.push_back(tempCluster);
    }
    */
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return result_cluster;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster){
    
   // PART 1: Compute Principal directions to gather Eigenvectors of given cloud
   Eigen::Vector4f pcaCentroid;
   // Compute centroid of the given cluster
   pcl::compute3DCentroid(*cluster, pcaCentroid); 
   Eigen::Matrix3f covariance;
   // Normalized: every entry has been divided by the number of points in the point cloud
   pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
   // Time to gather eigenvectors 
   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
   Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
   eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
   
   // PART 2: Move the points to the Reference Frame which is origin
   Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
   projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
   projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
   typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
   pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
   
   // PART 3: Get the minimum and maximum points of the transformed cloud 
   PointT minPoint, maxPoint;
   pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
   const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
   
   // PART 4: Rotate the given bounding box
   //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
   const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); 
   const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
   
   BoxQ boxq;
   
   boxq.bboxQuaternion = bboxQuaternion;
   boxq.bboxTransform = bboxTransform;
   
   boxq.cube_length = maxPoint.x - minPoint.x;
   boxq.cube_width =  maxPoint.y - minPoint.y;
   boxq.cube_height = maxPoint.z - minPoint.z; 
  
   return boxq;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

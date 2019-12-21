// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include "quiz/cluster/kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(const std::vector<std::vector<float>>& points, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize);

    void clusterHelp(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol, typename pcl::PointCloud<PointT>::Ptr cloud_e);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */

// PCL lib Functions for processing point clouds 



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

    typename pcl::PointCloud<PointT>::Ptr filterCloud (new pcl::PointCloud<PointT>); //for Voxel
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>); //for CropBox

    pcl::CropBox<PointT> region(true); //true because points are inside the cropbox
     

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filterCloud);

    //CropBox
    region.setInputCloud (filterCloud);
    region.setMin(minPoint);
    region.setMax(maxPoint);

    region.filter (*cloudRegion);

    //crop rooftop
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;


    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    /* //pcl verion:

    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // TODO:: Fill in the function to segment cloud into two parts, the drivable plane and obstacles

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset" << std::endl;
    }
    */
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    srand(time(NULL));
    int max_inliers = 0;
    int max_A = 0;
    int max_B = 0;
    int max_C = 0;
    int max_D = 0;
    // TODO: Fill in this function
    if (cloud->points.size () > 3)
    {

    // For max iterations
        for (int index = 0; index < maxIterations; index++)
        {
        // Randomly sample subset and fit line
        int num_inliers = 0;
        std::unordered_set<int> inlier_set;
        while (inlier_set.size() < 3)
            inlier_set.insert(rand() % cloud->points.size());

        float x1, x2, x3, y1, y2, y3, z1, z2, z3;

        auto itr = inlier_set.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
        float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
        float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
        float A = i;
        float B = j;
        float C = k;
        float D = -(i*x1 + j*y1 + k*z1);
        for (int ctr = 0; ctr < cloud->points.size (); ctr++)
        {
            float x = cloud->points[ctr].x;
            float y = cloud->points[ctr].y;
            float z = cloud->points[ctr].z;
            // Measure distance between every point and fitted line
            float d = fabs(A*x + B*y + C*z + D)/sqrt(pow(A,2) + pow(B,2) + pow(C,2));
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceThreshold)
            {
                num_inliers++;
            }
        
        }
        if (num_inliers > max_inliers)
        {
            max_inliers = num_inliers; 
            max_A = A;
            max_B = B;
            max_C = C;
            max_D = D;
        }
      
        }
        for (int ctr = 0; ctr < cloud->points.size (); ctr++)
        {
            // Measure distance between every point and fitted line
            float d = fabs(max_A*cloud->points[ctr].x + max_B*cloud->points[ctr].y + max_C*cloud->points[ctr].z + max_D)/sqrt(pow(max_A,2) + pow(max_B,2) + pow(max_C,2));
            // If distance is smaller than threshold count it as inlier
            if (d <= distanceThreshold)
            {
                inliers->indices.push_back(ctr);
            }
        }
    }

    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    //OWN
    KdTree* tree = new KdTree;
    
    std::vector<std::vector<float>> points;
    for(PointT &it : cloud->points){
        points.push_back({it.x, it.y, it.z});
    }
    for (int i = 0; i < points.size(); i++){
        tree->insert(points[i], i);
    }

    clusters = euclideanCluster(points, cloud, tree, clusterTolerance, minSize, maxSize);
    //From PCL
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    /*//Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);

    }*/


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float clusterTolerance, int minSize, int maxSize)
{
    // TODO: Fill out this function to return list of indices for each cluster
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<int>> clusters_v;

    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while( i < points.size())
    {
        if(processed[i])
        {
            i++;
            continue;
        }

        std::vector<int> cluster;

        clusterHelp(i, points, cluster, processed, tree, clusterTolerance, cloud); //proximity
        if ((cluster.size() >= minSize) && (cluster.size() <= maxSize))
        {
            clusters_v.push_back(cluster);
        }
        ;
        i++;

    }

    for(std::vector<int> it : clusters_v)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_point(new typename pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator itr = it.begin() ; itr != it.end(); ++itr)
        {
            cluster_point->points.push_back(cloud->points[*itr]);
        }

        cluster_point->width = cluster_point->points.size();
        cluster_point->height = 1;
        clusters.push_back(cluster_point);
    }
 
    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelp(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol, typename pcl::PointCloud<PointT>::Ptr cloud)
{

  processed[indice] = true;
  cluster.push_back(indice);

  std::vector<int> nearest = tree->search(points[indice], distanceTol);

  for(int id : nearest)
  {
    if(!processed[id])
        clusterHelp(id, points, cluster, processed, tree, distanceTol, cloud);
  }
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

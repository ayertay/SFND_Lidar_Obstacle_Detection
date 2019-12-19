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
      std::unordered_set<int> inliers;
      while (inliers.size() < 3)
          inliers.insert(rand() % cloud->points.size());

      float x1, x2, x3, y1, y2, y3, z1, z2, z3;

      auto itr = inliers.begin();
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
        if (d <= distanceTol)
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
        if (d <= distanceTol)
        {
          inliersResult.insert(ctr);
        }
      }
  /*My version:
  int max_inliers = 0;
  int max_A = 0;
  int max_B = 0;
  int max_C = 0;
  // TODO: Fill in this function
  if (cloud->points.size () > 2)
  {

  // For max iterations
    for (int i = 0; i < maxIterations; i++)
    {
      // Randomly sample subset and fit line
      int num_inliers = 0;
      int rnd_pt1 = rand() % cloud->points.size ();
      int x1 = cloud->points[rnd_pt1].x;
      int y1 = cloud->points[rnd_pt1].y;

      int rnd_pt2 = rand() % cloud->points.size ();
      while (rnd_pt2 == rnd_pt1)
        {
          rnd_pt2 = rand() % cloud->points.size ();
        }
      int x2 = cloud->points[rnd_pt2].x;
      int y2 = cloud->points[rnd_pt2].y;

      int A = y1 - y2;
      int B = x2 - x1;
      int C = x1*x2 - y1*y2;
      for (int ctr = 0; ctr < cloud->points.size (); ctr++)
      {
        // Measure distance between every point and fitted line
        float d = fabs(A*cloud->points[ctr].x + B*cloud->points[ctr].y + C)/sqrt(pow(A,2) + pow(B,2));
        // If distance is smaller than threshold count it as inlier
        if (d <= distanceTol)
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
      }
      
    }
    for (int ctr = 0; ctr < cloud->points.size (); ctr++)
      {
        // Measure distance between every point and fitted line
        float d = fabs(max_A*cloud->points[ctr].x + max_B*cloud->points[ctr].y + max_C)/sqrt(pow(max_A,2) + pow(max_B,2));
        // If distance is smaller than threshold count it as inlier
        if (d <= distanceTol)
        {
          inliersResult.insert(ctr);
        }
      } */	

/* Alternative code from lesson:
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

  while(maxIterations--)
  {
    //Randomly pick two points

    std::unordered_set<int> inliers; //set contains unique numbers, so it avoids same numbers to be inserted
    while (inliers.size() < 2)
        inliers.insert(rand()%(cloud->points.size()));

    float x1, x2, y1, y2;

    auto itr = inliers.begin();
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    itr++;
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;

    float a = (y1 - y2);
    float b = (x2 - x1);
    float c = (x1*y2 - x2*y1);

    for(int index = 0; index < cloud->points.size(); index++)
    {
      if(inliers.count(index)>0) //if already part of the line
          continue;

      pcl::PointXYZ point = cloud->points[index];
      //2D scenario so z is 0
      float x3 = point.x;
      float y3 = point.y;

      float d = fabs(a*x3 + b*y3 + c)/sqrt(a*a + b*b);

      if(d <= disntaceTol)
          inliers.insert(index);
    }

    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }*/

  }
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(); 2D data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); //3D data

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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

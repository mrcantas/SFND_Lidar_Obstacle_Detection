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
	std::unordered_set<int> inliers;
	int range=cloud->points.size();	
	
	// For max iterations 
	for(int i=0 ;i<maxIterations; i++){
	// Randomly sample subset and fit line
	while (inliers.size()<2)
		inliers.insert(rand() %range);
	auto in_ptr=inliers.begin();

	float x1, x2, y1, y2, d=0;
	x1 = cloud->points[*in_ptr].x;
	y1 = cloud->points[*in_ptr].y;
	in_ptr++;
	x2 = cloud->points[*in_ptr].x;
	y2 = cloud->points[*in_ptr].y;
	float A = (y1-y2);
	float B = (x2-x1);
	float C = x1*y2-x2*y1;
	float sq= sqrt(A*A+B*B);
	for(int j=0; j<range;j++)
	{
		d = fabs(A*cloud->points[j].x+ B*cloud->points[j].y+C)/ sq;
		if (d<= distanceTol)
			inliers.insert(j);
	}
	
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	if (inliers.size()> inliersResult.size())
		inliersResult=inliers;
	else 
		inliers.clear();
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int range=cloud->points.size();	
	
	// For max iterations 
	for(int i=0 ;i<maxIterations; i++){
	std::unordered_set<int> inliers;
	// Randomly sample subset and fit line
	while (inliers.size()<3)
		inliers.insert(rand() %range);
	auto in_ptr=inliers.begin();

	float x1, x2, x3, y1, y2, y3, z1, z2, z3,d=0;
	x1 = cloud->points[*in_ptr].x;
	y1 = cloud->points[*in_ptr].y;
	z1 = cloud->points[*in_ptr].z;
	in_ptr++;
	x2 = cloud->points[*in_ptr].x;
	y2 = cloud->points[*in_ptr].y;
	z2 = cloud->points[*in_ptr].z;
	in_ptr++;
	x3 = cloud->points[*in_ptr].x;
	y3 = cloud->points[*in_ptr].y;
	z3 = cloud->points[*in_ptr].z;
	float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
	float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
	float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
	float D = -(A*x1+B*y1+C*z1);
	float sq= sqrt(A*A+B*B+C*C);
	for(int j=0; j<range;j++)
	{
		d = fabs(A*cloud->points[j].x+ B*cloud->points[j].y+C*cloud->points[j].z+D)/ sq;
		if (d<= distanceTol)
			inliers.insert(j);
	}
	
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	if (inliers.size()> inliersResult.size())
		inliersResult=inliers;
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.4);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac Plane took " << elapsedTime.count() << " milliseconds" << std::endl;


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

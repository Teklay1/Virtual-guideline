#include<ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/common/projection_matrix.h>

ros::Publisher pub;
ros::Publisher bub;
ros::Publisher gug;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::PCLPointCloud2 Blue_points;
    pcl::PCLPointCloud2 Green_points;
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    /*pcl::fromPCLPointCloud2( cloud_filtered, point_cloud);
    pcl::copyPointCloud(point_cloud, *point_cloudPtr);  */  
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (point_cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1,1);
    //pass.filter (point_cloudPtr);
    
    //Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.02, 0.02, 0.02);
    sor.filter (cloud_filtered);
    
    /*pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);*/
    pcl::fromPCLPointCloud2( cloud_filtered, point_cloud);
    pcl::copyPointCloud(point_cloud, *point_cloudPtr);
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(point_cloudPtr);
    
    std::vector<pcl::PointIndices>cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.03);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(99000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(point_cloudPtr);
    ec.extract(cluster_indices);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_Blue(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_Green(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    /* for(int i = 0; i < point_cloudPtr->size(); i++)
    {
        point_cloudPtr->points[i].r = 255;
        point_cloudPtr->points[i].g = 0;
        point_cloudPtr->points[i].b = 0;
     } */
    int j = 0;
        
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
          {
          for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                 {
                  pcl::PointXYZRGB point;
                  point.x = point_cloudPtr->points[*pit].x;
                  point.y = point_cloudPtr->points[*pit].y;
                  point.z = point_cloudPtr->points[*pit].z;
                  pcl::PointXYZRGB pointG;
                  
                  pcl::PointXYZRGB pointB;
                  
                  
                  if (j == 0) //Red	#FF0000	(255,0,0)
                       {
                        pointB.x = point_cloudPtr->points[*pit].x;
                        pointB.y = 0;
                        pointB.z = point_cloudPtr->points[*pit].z;
                        pointB.r = 0;
                        pointB.g = 0;
                        pointB.b = 255;
                       }
                  else if (j == 1) //Lime	#00FF00	(0,255,0)
                       {
                        pointG.x = point_cloudPtr->points[*pit].x;
                        pointG.y = 0;
                        pointG.z = point_cloudPtr->points[*pit].z;
                        pointG.r = 0;
                        pointG.g = 255;
                        pointG.b = 0;
                       }
                  else if (j == 2) //Blue	#0000FF	(0,0,255)
                       {
                       point.r = 255;
                       point.g = 0;
                       point.b = 0;
                       }
                  else if (j == 3) //Yellow	#FFFF00	(255,255,0)
                       {
                       point.r = 255;
                       point.g = 255;
                       point.b = 0;
                       }
                  else if (j == 4) //Cyan	#00FFFF	(0,255,255)
                       {
                       point.r = 0;
                       point.g = 255;
                       point.b = 255;
                       }
                  else if (j == 5) //Magenta	#FF00FF	(255,0,255)
                       {
                       point.r = 255;
                       point.g = 0;
                       point.b = 255;
                       }
                  else if (j == 6) //Olive	#808000	(128,128,0)
                       {
                       point.r = 128;
                       point.g = 128;
                       point.b = 0;
                       }
                  else if (j == 7) //Teal	#008080	(0,128,128)
                       {
                       point.r = 0;
                       point.g = 128;
                       point.b = 128;
                       }
                  else if (j == 8) //Purple	#800080	(128,0,128)
                       {
                       point.r = 128;
                       point.g = 0;
                       point.b = 128;
                       }
                  else
                       {
                       if (j % 2 == 0)
                             {
                             point.r = 255 * j /(cluster_indices.size());
                             point.g = 128;
                             point.b = 50;
                             }
                       else
                             {
                             point.r = 0;
                             point.g = 255 * j /(cluster_indices.size());;
                             point.b = 128;
                             }
                        }
                   point_cloud_segmented->push_back(point);
                   point_Blue->push_back(pointB);
                   point_Green->push_back(pointG);
                   }
          j++;
       }
    //std::cerr<< "segmented:  " << (int)point_cloud_segmented->size() << "\n";
    std::cerr<< "segmented:  " << (int)point_Blue->size() << "\n";
    std::cerr<< "origin:     " << (int)point_cloudPtr->size() << "\n";
    //Convert to ROS data type
    point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;
    point_Blue->header.frame_id = point_cloudPtr->header.frame_id;
    point_Green->header.frame_id = point_cloudPtr->header.frame_id;
    if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
    //else pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
    
    if(point_Blue->size()) pcl::toPCLPointCloud2(*point_Blue, Blue_points);
    if(point_Green->size()) pcl::toPCLPointCloud2(*point_Green, Green_points);
    //else pcl::toPCLPointCloud2(*point_cloudPtr, Blue_points);
    if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
    //else pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
    
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 BP;
    sensor_msgs::PointCloud2 GP;
    
    pcl_conversions::fromPCL(cloud_filtered, output);
    pcl_conversions::fromPCL(Blue_points, BP);
    pcl_conversions::fromPCL(Green_points, GP);
    
    
    //Publish the data
    pub.publish (output);
    bub.publish (BP);
    gug.publish (GP);
    }
    
    int
    main (int argc, char** argv)
    {
    //Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::NodeHandle bp;
    ros::NodeHandle gp;
    
    
    //Create a ROS subscriber for the input point cloud  /extract_indices/output
    ros::Subscriber sub = nh.subscribe ("/extract_indices/output", 1, cloud_cb);
    
    //Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("other", 1);
    bub = bp.advertise<sensor_msgs::PointCloud2> ("blue", 1);
    gug = bp.advertise<sensor_msgs::PointCloud2> ("green", 1);
    
    // Spin
    ros::spin ();
    }
    
    
    
    
    

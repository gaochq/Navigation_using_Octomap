/*
 * dynamic_occupancy_grid.cpp
 * 
 * Copyright 2015 Shibata-Lab <shibata-lab@shibatalab-X500H>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <octomap/ColorOcTree.h>
#include <std_msgs/ColorRGBA.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <deque>
#include <string>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#define SS 3500 // sample size for random sampling
const float res = 0.02; // resolution for the octomap
int count =1; 	// counter for the file names
ros::Publisher octomap_pub ;
octomap::OcTreeStamped tree(res); // global map
typedef octomap::OcTreeStamped::leaf_iterator it_t; // global leaf iterator



// This function comes from the octomap_server pkg
std_msgs::ColorRGBA getColorByHeight(double h) {
  double range = 5.05;
  h = 1.0 - std::min(std::max(h/range, 0.0), 1.0);
  h *= 0.8;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}


void publishMapAsMarkers(octomap::OcTreeStamped octree_msg) {
	visualization_msgs::MarkerArray msg;
	msg.markers.resize(octree_msg.getTreeDepth()+1);
	
	it_t it = octree_msg.begin_leafs();
	it_t end = octree_msg.end_leafs();
	// For leaf in leaves
	for (; it != end; ++it) {
		// If occupied
		if (octree_msg.isNodeOccupied(*it)) {
			// Get some info about the leaf
			double x = it.getX();
			double y = it.getY();
			double z = it.getZ();
			//std::cout<<x<<" "<<y<<" "<<z<<std::endl;
			
			size_t depth = it.getDepth();
			// Insert a point for the leaf's cube
			geometry_msgs::Point leaf_origin;
			leaf_origin.x = x;
			leaf_origin.y = y;
			leaf_origin.z = z;
			msg.markers[depth].points.push_back(leaf_origin);
			// Determine and set the leaf's color by height
			//std::cout<<"determining color by height"<<std::endl;
			msg.markers[depth].colors.push_back(getColorByHeight(leaf_origin.z));
			
		}
	}
	std::cout<<" Finish the marker array setup"<<std::endl;
	std_msgs::ColorRGBA color;
	color.a = 1.0;
	for (size_t i = 0; i < msg.markers.size(); ++i) {
		double size = octree_msg.getNodeSize(i);
		msg.markers[i].header.frame_id = "/map";
		msg.markers[i].header.stamp = ros::Time::now();
		msg.markers[i].ns = "map";
		msg.markers[i].id = i;
		msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		msg.markers[i].scale.x = size;
		msg.markers[i].scale.y = size;
		msg.markers[i].scale.z = size;
		msg.markers[i].action = visualization_msgs::Marker::ADD;
		msg.markers[i].color = color;
	}
	 std::cout<<"Publish the marker array"<<std::endl;
	
    octomap_pub.publish(msg) ;
}

pcl::PointCloud<pcl::PointXYZ> randomSample(pcl::PointCloud<pcl::PointXYZ> in_cld){
	// Randomly sample 1000 pts from the cloud to calculate 2d rigit transform
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cld_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*in_cld_ptr = in_cld;
	pcl::RandomSample<pcl::PointXYZ> sample(true);
	sample.setInputCloud(in_cld_ptr);
	sample.setSample(SS);  // 1000 pts
	std::vector<int> out_idx;
	sample.filter(out_idx);
	pcl::PointCloud<pcl::PointXYZ> out_cld;
	sample.filter(out_cld);
	std::cout<<out_cld.size()<<std::endl;
	return out_cld;
}

/*
void updateOctomap(octomap::PointCloud& pc_msg){
	it_t it = tree.begin_leafs();
	it_t end = tree.end_leafs();
	
	}
*/
// Ros msg callback for sensor_msgs
void processCloud(const sensor_msgs::PointCloud2 msg)
{
	ros::Time start_time = ros::Time::now();
	//********* Retirive and process raw pointcloud************
	// Initialize octomap with given resolution. 
	//octomap::OcTreeStamped tree(res);
	octomap::Pointcloud oct_pc;
	octomap::point3d origin(0.0f,0.0f,0.0f);
	
	// Retrieve cloud msg and convert it to usable cloud for ROS.
	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(msg,cloud);
	pcl::PointCloud<pcl::PointXYZ> pcl_pc;
	pcl::fromPCLPointCloud2(cloud,pcl_pc);
	std::vector<int> nan_indices;
	pcl::removeNaNFromPointCloud(pcl_pc,pcl_pc,nan_indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_0_ptr = pcl_pc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	
	
	
	pcl::PassThrough<pcl::PointXYZ> pass;
  	pass.setInputCloud (cloud_0_ptr);
  	pass.setFilterFieldName ("z");
  	pass.setFilterLimits (0.0, 2.5);
  	//pass.setFilterLimitsNegative (true);
  	pass.filter (*cloud_filtered);
	// filter point cloud 
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> dy_sor;
	dy_sor.setInputCloud (cloud_filtered);
	dy_sor.setMeanK (20);
	dy_sor.setStddevMulThresh (0.5);
	dy_sor.filter (*cloud_filtered);
	
	//*cloud_filtered = randomSample(*cloud_filtered);
	// Add the given point cloud to octomap Pointcloud data structure. 
	for(int i = 0;i<cloud_filtered->points.size();i++){
		oct_pc.push_back((float) cloud_filtered->points[i].x,(float) cloud_filtered->points[i].y,(float) cloud_filtered->points[i].z);
	}
	
	// Insert complete octomap point cloud to the Octree structure.
	tree.insertPointCloud(oct_pc,origin,-1,false,false);
	
	/*
	//******************Traverse the tree ********************
	for(octomap::OcTree::tree_iterator it =tree.begin_tree(), end = tree.end_tree();it!= end;it++){
		 //manipulate node, e.g.:
		std::cout << "_____________________________________"<<std::endl;
		std::cout << "Node center: " << it.getCoordinate() << std::endl;
		std::cout << "Node size: " << it.getSize() << std::endl;
		std::cout << "Node depth: "<<it.getDepth() << std::endl;
		std::cout << "Is Leaf : "<< it.isLeaf()<< std::endl;
		std::cout << "_____________________________________"<<std::endl;
		
		}
	//**********************************************************	
	*/
	
	
	//publish the octomap 
	std::cout<<"publishing octomap"<<std::endl;
	publishMapAsMarkers(tree);
		
	std::cout<<"published"<<std::endl;	
	ros::Duration delta_t = ros::Time::now() - start_time;
	std::cout<< "Time for update :"<< delta_t<<std::endl;
	
	}


int main(int argc, char **argv){	

	// Initializing ros parameters and subscribing to the topic.
	ros::init(argc,argv, "processCloud");
	ros::NodeHandle n;
	octomap_pub = n.advertise<visualization_msgs::MarkerArray>("/map_vis", 1);
	std::cout<<"subscribing to topic point_cloud"<<std::endl;
	uint32_t queue_size = 1;	
	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/pclPoint_out",queue_size,processCloud);
	ros::spin(); 
	return 0;
}
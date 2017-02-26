#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/ColorRGBA.h>

#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <sstream>

using octomap::OcTree;

ros::Publisher map_pub;

// This function comes from the octomap_server pkg
std_msgs::ColorRGBA getColorByHeight(double h) 
{
		double range = 2.05;
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

		switch (i) 
		{
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

void publishMapAsMarkers(OcTree& octree) 
{
	visualization_msgs::MarkerArray msg;
	msg.markers.resize(octree.getTreeDepth()+1);

	typedef OcTree::iterator it_t;
	it_t it = octree.begin();
	it_t end = octree.end();
	// For leaf in leaves
	for (; it != end; ++it) 
	{
	// If occupied
		if (octree.isNodeOccupied(*it)) 
		{
			// Get some info about the leaf
			double x = it.getX();
			double y = it.getY();
			double z = it.getZ();
			if (z > 0.1 && z < 2.0) 
			{
				size_t depth = it.getDepth();
				// Insert a point for the leaf's cube
				geometry_msgs::Point leaf_origin;
				leaf_origin.x = x;
				leaf_origin.y = y;
				leaf_origin.z = z;
				msg.markers[depth].points.push_back(leaf_origin);
				// Determine and set the leaf's color by height
				msg.markers[depth].colors.push_back(getColorByHeight(leaf_origin.z));
			}
		}
	}
  // Finish the marker array setup
	std_msgs::ColorRGBA color;
	color.a = 1.0;
	for (size_t i = 0; i < msg.markers.size(); ++i) 
	{
		double size = octree.getNodeSize(i);

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
  // Publish the marker array
	map_pub.publish(msg);
}

void binary_map_callback(const octomap_msgs::OctomapBinary::ConstPtr& msg) 
{
	OcTree octomap(0.025);
	std::stringstream datastream;
	assert(msg->data.size() > 0);
	datastream.write((const char*) &msg->data[0], msg->data.size());
	octomap.readBinary(datastream);
	publishMapAsMarkers(octomap);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_repub");

	ros::NodeHandle n;

	map_pub = n.advertise<visualization_msgs::MarkerArray>("/map_vis", 1);

	ros::Subscriber sub = n.subscribe("/octomap_out", 1, binary_map_callback);

	ros::spin();

	return 0;
}


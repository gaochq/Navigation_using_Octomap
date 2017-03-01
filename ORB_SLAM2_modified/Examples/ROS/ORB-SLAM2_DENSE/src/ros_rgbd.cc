/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <System.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include"../ORB_SLAM2_modified/include/System.h"
#include"../ORB_SLAM2_modified/include/pointcloudmapping.h"
#include <tf/transform_broadcaster.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud;
ros::Publisher pclPoint_pub;
ros::Publisher CamPose_Pub;
sensor_msgs::PointCloud2 pcl_point;
cv::Mat Camerpose;
tf::Transform orb_slam;
std::vector<float> Pose_quat(4);
std::vector<float> Pose_trans(3);
tf::TransformBroadcaster * orb_slam_broadcaster;
geometry_msgs::PoseStamped Cam_Pose;

void Pub_CamPose(cv::Mat &pose);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);  

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
	pclPoint_pub = nh.advertise<sensor_msgs::PointCloud2>("/pclPoint_out",10);
	CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/CamPose",1);
	
    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Camerpose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
	
	mpSLAM->ReturnPcl(pcl_cloud);
	pcl::toROSMsg(pcl_cloud, pcl_point);
	
	//利用cvbridge将Mat转为sensor_sensor_msgs
	sensor_msgs::ImagePtr Cam_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Camerpose).toImageMsg();
	pclPoint_pub.publish(pcl_point);	
	Pub_CamPose(Camerpose);
}

void Pub_CamPose(cv::Mat &pose)
{
	cv::Mat Rwc(3,3,CV_32F);
	cv::Mat twc(3,1,CV_32F);
	Eigen::Matrix<double,3,3> rotationMat;
	orb_slam_broadcaster = new tf::TransformBroadcaster;
	
	Rwc = pose.rowRange(0,3).colRange(0,3).t();
	twc = -Rwc*pose.rowRange(0,3).col(3);
	
	rotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
				   Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
				   Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
	Eigen::Quaterniond Q(rotationMat);
	Pose_quat[0] = Q.x(); Pose_quat[1] = Q.y();
	Pose_quat[2] = Q.z(); Pose_quat[3] = Q.w();
	
	Pose_trans[0] = twc.at<float>(0);
	Pose_trans[1] = twc.at<float>(1);
	Pose_trans[2] = twc.at<float>(2);
	
	orb_slam.setOrigin(tf::Vector3(Pose_trans[2], -Pose_trans[0], -Pose_trans[1]));
	orb_slam.setRotation(tf::Quaternion(Q.z(), -Q.x(), -Q.y(), Q.w()));
	orb_slam_broadcaster->sendTransform(tf::StampedTransform(orb_slam, ros::Time::now(), "world", "orb_slam"));
	
	Cam_Pose.header.stamp = ros::Time::now();
	Cam_Pose.header.frame_id = "orb_slam";
	tf::pointTFToMsg(orb_slam.getOrigin(), Cam_Pose.pose.position);
	tf::quaternionTFToMsg(orb_slam.getRotation(), Cam_Pose.pose.orientation);
	CamPose_Pub.publish(Cam_Pose);
}


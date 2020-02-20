#include <ros/ros.h>
#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <ar_recog/Tag.h>
#include <ar_recog/Tags.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>

#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/CameraInfo.h"

#include <math.h>

#include <iostream>

cv::Mat cameraIntrinsic;
cv::Mat cameraDistortion;

sensor_msgs::CameraInfo cam_info_;
double tag_width = 0.07;
int frameNo = 0;
ros::Publisher marker_pub;

//YAML::Emitter out;

void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& cam_info)
{
     cam_info_ = *cam_info;
     std::cout << "camera info has been received" << std::endl;
    if (cam_info->D.size() == 5) {
    	cameraDistortion = cv::Mat(cam_info->D);
    	cameraDistortion.reshape(1, 1);
    }
    if (cam_info->K.size() == 9) {
    	double* cameraIntrinsicPtr = (double*) cameraIntrinsic.data;
    	for (int i = 0; i < 9; i++, cameraIntrinsicPtr++) *cameraIntrinsicPtr = cam_info->K[i];
    }
    std::cout << "Camera Intrinsic Matrix: " << std::endl << cameraIntrinsic << std::endl;
}


inline void rotMat2quat(const cv::Mat rotMat,
    double& w, double& x, double& y, double& z) {
  assert(rotMat.rows == 3 && rotMat.cols == 3 && rotMat.isContinuous() &&
      rotMat.elemSize() == sizeof(double));

  double tr = rotMat.at<double>(0, 0) + rotMat.at<double>(1, 1) + rotMat.at<double>(2, 2);
  double s;

  if (tr > 0) {
    s = 0.5/sqrt(tr + 1.0);
    w = 0.25/s;
    x = (rotMat.at<double>(2, 1) - rotMat.at<double>(1, 2))*s;
    y = (rotMat.at<double>(0, 2) - rotMat.at<double>(2, 0))*s;
    z = (rotMat.at<double>(1, 0) - rotMat.at<double>(0, 1))*s;
  } else if (rotMat.at<double>(0, 0) > rotMat.at<double>(1, 1) &&
      rotMat.at<double>(0, 0) > rotMat.at<double>(2, 2)) {
    s = 2.0*sqrt(1.0 + 2*rotMat.at<double>(0, 0) - tr);
    w = (rotMat.at<double>(2, 1) - rotMat.at<double>(1, 2))/s;
    x = 0.25*s;
    y = (rotMat.at<double>(0, 1) + rotMat.at<double>(1, 0))/s;
    z = (rotMat.at<double>(0, 2) + rotMat.at<double>(2, 0))/s;
  } else if (rotMat.at<double>(1, 1) > rotMat.at<double>(2, 2)) {
    s = 2.0*sqrt(1.0 + 2*rotMat.at<double>(1, 1) - tr);
    w = (rotMat.at<double>(0, 2) - rotMat.at<double>(2, 0))/s;
    x = (rotMat.at<double>(0, 1) + rotMat.at<double>(1, 0))/s;
    y = 0.25*s;
    z = (rotMat.at<double>(1, 2) + rotMat.at<double>(2, 1))/s;
  } else {
    s = 2.0*sqrt(1.0 + 2*rotMat.at<double>(2, 2) - tr);
    w = (rotMat.at<double>(1, 0) - rotMat.at<double>(0, 1))/s;
    x = (rotMat.at<double>(0, 2) + rotMat.at<double>(2, 0))/s;
    y = (rotMat.at<double>(1, 2) + rotMat.at<double>(2, 1))/s;
    z = 0.25*s;
  }
};


void tag_callback(const ar_recog::Tags::ConstPtr &msg) {
	frameNo++;
	std::vector <ar_recog::Tag> tags = msg->tags;
	for ( const ar_recog::Tag tag: tags )
	{
		std::ostringstream oss;
		oss << "tag";//_" << tag.id ;

		std::vector<cv::Point2f> cornersPx;
		cv::Point2f p;
		p.x = tag.cwCorners[0];
		p.y = tag.cwCorners[1];
		cornersPx.push_back(p);
		p.x = tag.cwCorners[2];
		p.y = tag.cwCorners[3];
		cornersPx.push_back(p);
		p.x = tag.cwCorners[4];
		p.y = tag.cwCorners[5];
		cornersPx.push_back(p);
		p.x = tag.cwCorners[6];
		p.y = tag.cwCorners[7];
		cornersPx.push_back(p);

		std::vector<cv::Point3d> spatialPoints;
		cv::Mat transVec, rotVec, rotMat;

		double tagSizeHalved = tag_width / 2;
		spatialPoints.push_back(cv::Point3d(-tagSizeHalved, -tagSizeHalved, 0.0));
		spatialPoints.push_back(cv::Point3d( tagSizeHalved, -tagSizeHalved, 0.0));
		spatialPoints.push_back(cv::Point3d( tagSizeHalved,  tagSizeHalved, 0.0));
		spatialPoints.push_back(cv::Point3d(-tagSizeHalved,  tagSizeHalved, 0.0));

		cv::solvePnP(spatialPoints, cornersPx, cameraIntrinsic, cameraDistortion, rotVec, transVec);
		cv::Rodrigues(rotVec, rotMat);
		double rw, rx, ry, rz;
		rotMat2quat(rotMat, rw, rx, ry, rz);

		double tx = transVec.at<double>(0);
		double ty = transVec.at<double>(1);
		double tz = transVec.at<double>(2);

		tf::Quaternion rMat(rx,ry,rz,rw);
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3( tx, ty, tz) );
		transform.setRotation( rMat );
		br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), cam_info_.header.frame_id, oss.str() ) );
		std::cout << "ID: " << tag.id << "( tx, ty, tz ) = ( " << tx << ", " << ty << ", " << tz << " )" << std::endl;
		std::cout << "ID: " << tag.id << "( rx, ry, rz ) = ( " << rotVec.at<double>(0) << ", " << rotVec.at<double>(1) << ", " << rotVec.at<double>(2) << " )" << std::endl;

		visualization_msgs::Marker marker;

		marker.header.stamp = ros::Time::now();
                marker.header.frame_id = cam_info_.header.frame_id; 

		marker.pose.position.x = tx;
		marker.pose.position.y = ty;
		marker.pose.position.z = tz;

		marker.pose.orientation.x = rx;
		marker.pose.orientation.y = ry;
		marker.pose.orientation.z = rz;
		marker.pose.orientation.w = rw;

		std::ostringstream tag_id_str;
		tag_id_str << tag.id;
		marker.id = frameNo;
		marker.text = tag_id_str.str();

		marker_pub.publish(marker);

//		if ( tag.id == 1 )
//		{
//			std::ostringstream oss;
//			oss << frameNo;
//
//			out << YAML::BeginMap;
//			//              out << YAML::Key << ros::Time::now().nsec;
//			out << YAML::Key << oss.str();
//			out << YAML::Value;
//			out << YAML::BeginMap;
//			out << YAML::Key << "Frame No.";
//			out << YAML::Value << frameNo;
//			out << YAML::Key << "Position";
//			out << YAML::Value
//					<< YAML::BeginSeq
//					<< marker.pose.position.x << marker.pose.position.y << marker.pose.position.z
//					<< YAML::EndSeq;
//			out << YAML::Key << "Orientation";
//			out << YAML::Value
//					<< YAML::BeginSeq
//					<< marker.pose.orientation.x << marker.pose.orientation.y
//					<< marker.pose.orientation.z << marker.pose.orientation.w
//					<< YAML::EndSeq ;
//			out << YAML::EndMap;
//			out << YAML::EndMap;
//		}else if ( tag.id == 8 )
//		{
//			out << YAML::EndSeq;
//			std::ofstream fout("/home/dacocp/Dropbox/catkin_ws/trajectory.yaml");
//			fout << out.c_str();
//			std::cout << "Here's the output YAML:\n" << out.c_str();
//		}

	}
}


int main(int argc, char **argv)
{
//	out << YAML::BeginSeq;

	std::cout << "INITIALIZING" << std::endl;
	cameraIntrinsic = cv::Mat::zeros(3, 3, CV_64FC1); // prepare buffer for camera_info
	cameraDistortion = cv::Mat::zeros(1, 5, CV_64FC1); // prepare buffer for camera_info

	ros::init(argc, argv, "ar_compute_pose");

	std::cout << "AFTER INITIALIZING" << std::endl;

	ros::NodeHandle n("~");


	n.getParam("tag_width", tag_width);
	std::cout << "Tag width = " << tag_width << std::endl;

	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber sb_left_camera_info = n.subscribe("/camera/camera_info", 1, camera_info_callback);
	ros::Subscriber tag_sub = n.subscribe("/aqua/tags", 1, tag_callback);

	std::cout << "BEFORE SPINNING" << std::endl;
	ros::spin();
	std::cout << "AFTER SPINNING" << std::endl;
	return 0;
}

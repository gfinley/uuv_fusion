#ifndef CAVR_UTILS_HPP_
#define CAVR_UTILS_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace cavr {

	// Convert quaternion to euler angles (fixed axes)
	void quaternion2fixedRPY(double &roll, double& pitch, double& yaw, double x, double y, double z, double w);
	void quaternion2fixedRPY(double &roll, double& pitch, double& yaw, geometry_msgs::Quaternion q);
	void quaternion2fixedRPY(double &roll, double& pitch, double& yaw, tf2::Quaternion q);

	// convert euler angles to quaternion (fixed axes)
	void fixedRPY2Quaternion(double &x, double &y, double &z, double &w, double roll, double pitch, double yaw);
	void fixedRPY2Quaternion(geometry_msgs::Quaternion &q, double roll, double pitch, double yaw);
	void fixedRPY2Quaternion(tf2::Quaternion &q, double roll, double pitch, double yaw);

	// convert euler angles to quaternion (rotating axes)
	void eulerYPR2Quaternion(double &x, double &y, double &z, double &w, double yaw, double pitch, double roll);
	void eulerYPR2Quaternion(geometry_msgs::Quaternion &q, double yaw, double pitch, double roll);
	void eulerYPR2Quaternion(tf2::Quaternion &q, double yaw, double pitch, double roll);

	// Convert quaternion to euler angles (rotating axes)
	void quaternion2EulerYPR(double &yaw, double& pitch, double& roll, double x, double y, double z, double w);
	void quaternion2EulerYPR(double &yaw, double& pitch, double& roll, geometry_msgs::Quaternion q);
	void quaternion2EulerYPR(double &yaw, double& pitch, double& roll, tf2::Quaternion q);

	// convert rotation matrix to euler or quaternion
	void rotation2EulerYPR(double &yaw, double& pitch, double& roll, double r00, double r01, double r02, double r10, double r11, double r12, double r20, double r21, double r22);
	void rotation2Quaternion(double &x, double& y, double& z, double& w, double r00, double r01, double r02, double r10, double r11, double r12, double r20, double r21, double r22);

	// calc vel magnitude from components
	double velFromComp(double x, double y, double z);

	// radians to degrees
	double rad2Deg(double rad);

	// degrees to radians
	double deg2Rad(double deg);

	// latitude degrees to meters (i.e., degrees as distance measure)
	double decDegToMeter(double deg);
     
	// convert angle to within the range minus pi to plus pi
	double angleInRangeNPitoPi(double angle);

	// convert angle to within the range 0 to 2*pi
	double angleInRange0to2Pi(double angle);

	// calculate the distance between Odometry messages
	double calcDistance(nav_msgs::Odometry s1, nav_msgs::Odometry s2, int dim);
	double calcDistance(geometry_msgs::Transform s1, geometry_msgs::Pose s2, int dim);
	double calcDistance(geometry_msgs::Transform s1, geometry_msgs::Transform s2, int dim);

	// calculate the relative motion (translation and rotation) from pose 1 to pose 2 (in coordinate system of pose 1)
	geometry_msgs::Pose calcRelMotion(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
	geometry_msgs::Pose calcRelMotion(geometry_msgs::Transform p1, geometry_msgs::Transform p2);
	geometry_msgs::Pose calcRelMotion(geometry_msgs::Transform p1, geometry_msgs::Pose p2);

	// transform vector (pb) from body to world frame, with world to body transform given by tf
	geometry_msgs::Point transformVectorB2W(geometry_msgs::Point pb, geometry_msgs::Pose tf);
	// transform vector (pw) from world to body frame, with world to body transform given by tf
	geometry_msgs::Point transformVectorW2B(geometry_msgs::Point pw, geometry_msgs::Pose tf);

	// calculate the body velocities
	geometry_msgs::Vector3 calcBodyVelocities(geometry_msgs::Vector3 vw, geometry_msgs::Pose tf);

	// transform msg classes
	geometry_msgs::TransformStamped convert(geometry_msgs::PoseStamped ps, std::string child_frame_id);

}

#endif // CAVR_UTILS_HPP_

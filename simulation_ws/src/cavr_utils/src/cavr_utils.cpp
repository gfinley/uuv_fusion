#include <cavr_utils/cavr_utils.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <math.h>

#define _USE_MATH_DEFINES

namespace cavr {

// calc vel magnitude from components
double velFromComp(double x, double y, double z) {
	return sqrt(x*x + y*y + z*z);
}

// radians to degrees
double rad2Deg(double rad){
	return rad/M_PI*180.0;
}

// degrees to radians
double deg2Rad(double deg){
	return deg*M_PI/180.0;
}

// latitude degrees to meters (i.e., degrees as distance measure)
double decDegToMeter(double deg){
	return deg*3600.0*1852.0;
}

// Convert quaternion to euler angles (fixed axes)
void quaternion2fixedRPY(double &roll, double& pitch, double& yaw, double x, double y, double z, double w) {
	tf2::Quaternion q(x,y,z,w);
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return;
}

// Convert quaternion to euler angles (fixed axes)
void quaternion2fixedRPY(double &roll, double& pitch, double& yaw, geometry_msgs::Quaternion q) {
	tf2::Quaternion q2(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(q2).getRPY(roll, pitch, yaw);
	return;
}

// Convert quaternion to euler angles (fixed axes)
void quaternion2fixedRPY(double &roll, double& pitch, double& yaw, tf2::Quaternion q) {
	tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
	return;
}

// Convert quaternion to euler angles (rotating axes)
void quaternion2EulerYPR(double &yaw, double& pitch, double& roll, double x, double y, double z, double w) {
	tf2::Quaternion q(x,y,z,w);
	tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
	return;
}

// Convert quaternion to euler angles (rotating axes)
void quaternion2EulerYPR(double &yaw, double& pitch, double& roll, geometry_msgs::Quaternion q) {
	tf2::Quaternion q2(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(q2).getEulerYPR(yaw, pitch, roll);
	return;
}

// Convert quaternion to euler angles (rotating axes)
void quaternion2EulerYPR(double &yaw, double& pitch, double& roll, tf2::Quaternion q) {
	tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
	return;
}

void rotation2EulerYPR(double &yaw, double& pitch, double& roll, double r00, double r01, double r02, double r10, double r11, double r12, double r20, double r21, double r22){
	tf2::Matrix3x3(r00, r01, r02, r10, r11, r12, r20, r21, r22).getEulerYPR(yaw, pitch, roll);
	return;
}

void rotation2Quaternion(double &x, double& y, double& z, double& w, double r00, double r01, double r02, double r10, double r11, double r12, double r20, double r21, double r22){
	double yaw, pitch, roll;
	tf2::Matrix3x3(r00, r01, r02, r10, r11, r12, r20, r21, r22).getEulerYPR(yaw, pitch, roll);
	tf2::Quaternion q;
	q.setEulerZYX(yaw, pitch, roll);
	x = q.getX();
	y = q.getY();
	z = q.getZ();
	w = q.getW();
	return;
}

// convert euler angles to quaternion (fixed axes)
void fixedRPY2Quaternion(double &x, double &y, double &z, double &w, double roll, double pitch, double yaw) {
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	x = q.getX();
	y = q.getY();
	z = q.getZ();
	w = q.getW();
	return;
}

// convert euler angles to quaternion (fixed axes)
void fixedRPY2Quaternion(geometry_msgs::Quaternion &q, double roll, double pitch, double yaw) {
	tf2::Quaternion q2;
	q2.setRPY(roll, pitch, yaw);
	q.x = q2.getX();
	q.y = q2.getY();
	q.z = q2.getZ();
	q.w = q2.getW();
	return;
}

// convert euler angles to quaternion (fixed axes)
void fixedRPY2Quaternion(tf2::Quaternion &q, double roll, double pitch, double yaw) {
	q.setRPY(roll, pitch, yaw);
	return;
}

// convert euler angles to quaternion (rotating axes)
void eulerYPR2Quaternion(double &x, double &y, double &z, double &w, double yaw, double pitch, double roll) {
	tf2::Quaternion q;
	q.setEulerZYX(yaw, pitch, roll);
	x = q.getX();
	y = q.getY();
	z = q.getZ();
	w = q.getW();
	return;
}

// convert euler angles to quaternion (rotating axes)
void eulerYPR2Quaternion(geometry_msgs::Quaternion &q, double yaw, double pitch, double roll) {
	tf2::Quaternion q2;
	q2.setEulerZYX(yaw, pitch, roll);
	q.x = q2.getX();
	q.y = q2.getY();
	q.z = q2.getZ();
	q.w = q2.getW();
	return;
}

// convert euler angles to quaternion (rotating axes)
void eulerYPR2Quaternion(tf2::Quaternion &q, double yaw, double pitch, double roll) {
	q.setEulerZYX(yaw, pitch, roll);
	return;
}

//Convert an angle to minus pi to plus pi
double angleInRangeNPitoPi(double angle){
	if (angle > M_PI){
		while (angle > M_PI){
			angle = angle - 2*M_PI;
		}
		return angle;
	}
	else if (angle <= -M_PI){
		while (angle <= -M_PI){
			angle = angle + 2*M_PI;
		}
		return angle;
	}
	else { return angle; }
}

//Convert angle to within the range 0 to 2PI
double angleInRange0to2Pi(double angle){
	if (angle >= 2*M_PI){
		while (angle >= 2*M_PI){
			angle = angle - 2*M_PI;
		}
		return angle;
	}
	else if (angle < 0 ){
		while(angle < 0) {
			angle = angle + 2*M_PI;
		}
		return angle;
	}
	else {return angle;}
}

// calculate the distance between 2 Odometry messages
double calcDistance(nav_msgs::Odometry s1, nav_msgs::Odometry s2, int dim = 2) {
	if (dim==2)
		return sqrt(pow(s2.pose.pose.position.x-s1.pose.pose.position.x,2) +
					pow(s2.pose.pose.position.y-s1.pose.pose.position.y,2));
	else if (dim==3)
		return sqrt(pow(s2.pose.pose.position.x-s1.pose.pose.position.x,2) +
					pow(s2.pose.pose.position.y-s1.pose.pose.position.y,2) +
					pow(s2.pose.pose.position.z-s1.pose.pose.position.z,2));
}

// calculate the distance between transform and pose
double calcDistance(geometry_msgs::Transform s1, geometry_msgs::Pose s2, int dim = 2) {
	if (dim==2)
		return sqrt(pow(s2.position.x-s1.translation.x,2) +
					pow(s2.position.y-s1.translation.y,2));
	else if (dim==3)
		return sqrt(pow(s2.position.x-s1.translation.x,2) +
					pow(s2.position.y-s1.translation.y,2) +
					pow(s2.position.z-s1.translation.z,2));
	return 0;
}

// calculate the distance between transform and transform
double calcDistance(geometry_msgs::Transform s1, geometry_msgs::Transform s2, int dim = 2) {
	if (dim==2)
		return sqrt(pow(s2.translation.x-s1.translation.x,2) +
					pow(s2.translation.y-s1.translation.y,2));
	else if (dim==3)
		return sqrt(pow(s2.translation.x-s1.translation.x,2) +
					pow(s2.translation.y-s1.translation.y,2) +
					pow(s2.translation.z-s1.translation.z,2));
	return 0;
}

// calculate the relative motion (translation and rotation) between 2 poses
geometry_msgs::Pose calcRelMotion(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
	// Assume that the poses are given in the world frame (W)

	// Step 1: calculate the rotations from the world frame to frames 1 and 2
	tf2::Quaternion q_w1(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w);
	tf2::Quaternion q_w2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);
	tf2::Matrix3x3 R_w1(q_w1);
	tf2::Matrix3x3 R_w2(q_w2);

	// Step 2: the rotations from frame 1 to 2 is given by (R_w1)'*R_w2
	tf2::Matrix3x3 R_12 = R_w1.transposeTimes(R_w2);
	tf2::Quaternion q_12;
	R_12.getRotation(q_12);

	// Step 3: calculate the relative translation (in the world frame)
	tf2::Vector3 d_12_w(p2.position.x-p1.position.x, p2.position.y-p1.position.y, p2.position.z-p1.position.z);

	// Step 4: convert this translation to frame 1
	tf2::Vector3 d_12 = R_w1.transpose()*d_12_w;

	// Step 5: repackage into pose
	geometry_msgs::Pose p;
	p.position.x = d_12.getX();
	p.position.y = d_12.getY();
	p.position.z = d_12.getZ();
	p.orientation.x = q_12.getX();
	p.orientation.y = q_12.getY();
	p.orientation.z = q_12.getZ();
	p.orientation.w = q_12.getW();
	return p;
}

// calculate the relative motion (translation and rotation) between 2 transforms
geometry_msgs::Pose calcRelMotion(geometry_msgs::Transform p1, geometry_msgs::Transform p2) {
	// Assume that the poses are given in the world frame (W)

	// Step 1: calculate the rotations from the world frame to frames 1 and 2
	tf2::Quaternion q_w1(p1.rotation.x, p1.rotation.y, p1.rotation.z, p1.rotation.w);
	tf2::Quaternion q_w2(p2.rotation.x, p2.rotation.y, p2.rotation.z, p2.rotation.w);
	tf2::Matrix3x3 R_w1(q_w1);
	tf2::Matrix3x3 R_w2(q_w2);

	// Step 2: the rotations from frame 1 to 2 is given by (R_w1)'*R_w2
	tf2::Matrix3x3 R_12 = R_w1.transposeTimes(R_w2);
	tf2::Quaternion q_12;
	R_12.getRotation(q_12);

	// Step 3: calculate the relative translation (in the world frame)
	tf2::Vector3 d_12_w(p2.translation.x-p1.translation.x, p2.translation.y-p1.translation.y, p2.translation.z-p1.translation.z);

	// Step 4: convert this translation to frame 1
	tf2::Vector3 d_12 = R_w1.transpose()*d_12_w;

	// Step 5: repackage into pose
	geometry_msgs::Pose p;
	p.position.x = d_12.getX();
	p.position.y = d_12.getY();
	p.position.z = d_12.getZ();
	p.orientation.x = q_12.getX();
	p.orientation.y = q_12.getY();
	p.orientation.z = q_12.getZ();
	p.orientation.w = q_12.getW();
	return p;
}

// calculate the relative motion (translation and rotation) between 2 poses
geometry_msgs::Pose calcRelMotion(geometry_msgs::Transform p1, geometry_msgs::Pose p2) {
	// Assume that the poses are given in the world frame (W)

	// Step 1: calculate the rotations from the world frame to frames 1 and 2
	tf2::Quaternion q_w1(p1.rotation.x, p1.rotation.y, p1.rotation.z, p1.rotation.w);
	tf2::Quaternion q_w2(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w);
	tf2::Matrix3x3 R_w1(q_w1);
	tf2::Matrix3x3 R_w2(q_w2);

	// Step 2: the rotations from frame 1 to 2 is given by (R_w1)'*R_w2
	tf2::Matrix3x3 R_12 = R_w1.transposeTimes(R_w2);
	tf2::Quaternion q_12;
	R_12.getRotation(q_12);

	// Step 3: calculate the relative translation (in the world frame)
	tf2::Vector3 d_12_w(p2.position.x-p1.translation.x, p2.position.y-p1.translation.y, p2.position.z-p1.translation.z);

	// Step 4: convert this translation to frame 1
	tf2::Vector3 d_12 = R_w1.transpose()*d_12_w;

	// Step 5: repackage into pose
	geometry_msgs::Pose p;
	p.position.x = d_12.getX();
	p.position.y = d_12.getY();
	p.position.z = d_12.getZ();
	p.orientation.x = q_12.getX();
	p.orientation.y = q_12.getY();
	p.orientation.z = q_12.getZ();
	p.orientation.w = q_12.getW();
	return p;
}

// transform vector (pb) from body to world frame, with world to body transform given by tf
geometry_msgs::Point transformVectorB2W(geometry_msgs::Point pb, geometry_msgs::Pose tf) {
	// Step 1: calculate the rotation from the world frame to frame 1
	tf2::Quaternion q_w1(tf.orientation.x, tf.orientation.y, tf.orientation.z, tf.orientation.w);
	tf2::Matrix3x3 R_w1(q_w1);

	// Step 2: get the translation to frame 1
	tf2::Vector3 d_w1(tf.position.x, tf.position.y, tf.position.z);

	// Step 3: repackage vector
	tf2::Vector3 pb_(pb.x, pb.y, pb.z);

	// Step 4: convert this translation to world frame
	tf2::Vector3 pw_ = d_w1+R_w1*pb_;

	// Step 5: repackage into pose
	geometry_msgs::Point pw;
	pw.x = pw_.getX();
	pw.y = pw_.getY();
	pw.z = pw_.getZ();
	return pw;
}

// transform vector (pw) from world to body frame, with world to body transform given by tf
geometry_msgs::Point transformVectorW2B(geometry_msgs::Point pw, geometry_msgs::Pose tf) {
	// Step 1: calculate the rotation from the world frame to frame 1
	tf2::Quaternion q_w1(tf.orientation.x, tf.orientation.y, tf.orientation.z, tf.orientation.w);
	tf2::Matrix3x3 R_w1(q_w1);

	// Step 2: get the translation to frame 1
	tf2::Vector3 d_w1(tf.position.x, tf.position.y, tf.position.z);

	// Step 3: repackage vector
	tf2::Vector3 pw_(pw.x, pw.y, pw.z);

	// Step 4: convert this translation to body frame
	tf2::Vector3 pb_ = R_w1.transpose()*pw_ - R_w1.transpose()*d_w1;

	// Step 5: repackage into pose
	geometry_msgs::Point pb;
	pb.x = pb_.getX();
	pb.y = pb_.getY();
	pb.z = pb_.getZ();
	return pb;
}

// calculate the body velocities
geometry_msgs::Vector3 calcBodyVelocities(geometry_msgs::Vector3 vw, geometry_msgs::Pose tf) {
	// Step 1: calculate the rotation from the world frame to frame 1
	tf2::Quaternion q_w1(tf.orientation.x, tf.orientation.y, tf.orientation.z, tf.orientation.w);
	tf2::Matrix3x3 R_w1(q_w1);

	// Step 2: repackage vector
	tf2::Vector3 vw_(vw.x, vw.y, vw.z);

	// Step 3: convert this translation to body frame
	tf2::Vector3 vb_ = R_w1.transpose()*vw_;

	// Step 4: repackage into pose
	geometry_msgs::Vector3 vb;
	vb.x = vb_.getX();
	vb.y = vb_.getY();
	vb.z = vb_.getZ();

	return vb;
}

geometry_msgs::TransformStamped convert(geometry_msgs::PoseStamped ps, std::string child_frame_id) {
	geometry_msgs::TransformStamped ts;
	ts.header = ps.header;
	ts.transform.translation.x = ps.pose.position.x;
	ts.transform.translation.y = ps.pose.position.y;
	ts.transform.translation.z = ps.pose.position.z;
	ts.transform.rotation = ps.pose.orientation;
	ts.child_frame_id = child_frame_id;
	return ts;
}

}


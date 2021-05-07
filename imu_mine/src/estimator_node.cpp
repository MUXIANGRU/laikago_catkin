#include <ros/ros.h>
#include <fstream>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>


double latest_time;
Eigen::Vector3d tmp_P=Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d tmp_V=Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d tmp_Ba;  // acc bias
Eigen::Vector3d tmp_Bg;  // gyro bias
Eigen::Vector3d acc_0=Eigen::Vector3d(0.047, -0.014, 9.799);   //initial acc
Eigen::Vector3d gyr_0=Eigen::Vector3d(0, 0, 0);   //initial gyro
bool init_imu = 1;       //is init imu or not
Eigen::Vector3d g{0,0,9.799};
double last_imu_t = 0;
Eigen::Vector3d acc_complete=Eigen::Vector3d(0.047, -0.014, 0);

ros::Publisher pubdatadisplay;
ros::Publisher euler_pub;
geometry_msgs::Vector3 euler_angle;
Eigen::Vector3d QuaterniondToRPY(Eigen::Quaterniond& imu_rq){

    double imu_0_yaw, imu_0_pitch, imu_0_roll;
    Eigen::Vector3d rpy_vec_;
    imu_rq = imu_rq.normalized();
    tf::Matrix3x3 mat_be(tf::Quaternion(imu_rq.x(),imu_rq.y(),imu_rq.z(),imu_rq.w()));
    mat_be.getEulerYPR(imu_0_yaw,imu_0_pitch,imu_0_roll);
    rpy_vec_ << imu_0_roll,imu_0_pitch,imu_0_yaw;
//    cout << "GetFootPoseInBase---YPR: " << endl << "yaw: "<< rpy_vec_(0) << " pitch: " << rpy_vec_(1) << " roll: " <<rpy_vec_(2) << endl;
    return rpy_vec_;

}

void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
{

    //todo: the time stamp is accurate?
    //todo: how to deal with tmp_ba,tmp_bg
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    double wx = imu_msg->orientation.x;
    double wy = imu_msg->orientation.y;
    double wz = imu_msg->orientation.z;
    double ww = imu_msg->orientation.w;

    Eigen::Quaterniond tmp_Q(wx,wy,wz,ww);
    //前一帧下的加速度,转换到世界坐标系下
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - acc_complete - tmp_Ba) - g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;


    //get orientation by lie group
    tmp_Q.normalized();
    Sophus::SO3 SO3_q(tmp_Q);
    Eigen::Vector3d update = un_gyr * dt;
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update)*SO3_q;
    Eigen::Quaterniond RPY_bw_q;
    RPY_bw_q = Eigen::Quaterniond(SO3_updated.matrix());
    Eigen::Vector3d euler = QuaterniondToRPY(RPY_bw_q);
    euler_angle.x = euler.z();
    euler_angle.y = euler.y();
    euler_angle.z = euler.x();

    tmp_Q.w() = RPY_bw_q.w();
    tmp_Q.x() = RPY_bw_q.x();
    tmp_Q.y() = RPY_bw_q.y();
    tmp_Q.z() = RPY_bw_q.z();

    //tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
    //当前帧下的加速度,转换到世界坐标系下
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - acc_complete - tmp_Ba) - g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    //设前一帧位姿的平移表示为，假设对机器人进行了一次平移，可以表示为如下：
    tmp_V = tmp_V + dt * un_acc;
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;


    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;

    //Eigen::AngleAxisd rollAngle(roll, tmp_Q::UnitX());
    //Eigen::AngleAxisd pitchAngle(pitch, tmp_Q::UnitY());
    //Eigen::AngleAxisd yawAngle(yaw, tmp_Q::UnitZ());
    //Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    geometry_msgs::PoseWithCovarianceStamped display_imu;
    display_imu.header.stamp = ros::Time::now();
    display_imu.pose.pose.position.z = tmp_P[2];
    display_imu.pose.pose.position.y = tmp_P[1];
    display_imu.pose.pose.position.x = tmp_P[0];
    display_imu.pose.pose.orientation.x = tmp_Q.x();
    display_imu.pose.pose.orientation.y = tmp_Q.y();
    display_imu.pose.pose.orientation.z = tmp_Q.z();
    display_imu.pose.pose.orientation.w = tmp_Q.w();
    pubdatadisplay.publish(display_imu);
    euler_pub.publish(euler_angle);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_estimator");
    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu/data", 50, imuHandler);
    pubdatadisplay = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/imu_integral_mine", 100);
    euler_pub = nh.advertise<geometry_msgs::Vector3>("/imu_orientation",1);
    ros::spin();

    return 0;
}

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <string>

using namespace std;

static int frame_count = 0;
ofstream destFile;  //("/home/belight/Dataset/test20191231/predict_odom_test3.txt", ios::out);

// Z-Y-X Euler Angles
Eigen::Vector3d Quat2Euler(Eigen::Quaterniond q){
  Eigen::Vector3d euler;  
  
  Eigen::Matrix3d rot_mat = q.toRotationMatrix().transpose();  //这里必须转置一下，不转置的旋转矩阵为body到world的转换矩阵
  double R11 = q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z();
  double R12 = 2 * (q.x()*q.y() + q.w()*q.z());
  double R13 = 2 * (q.x()*q.z() - q.w()*q.y());
  double R23 = 2 * (q.y()*q.z() + q.w()*q.x());
  double R33 = q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();

  double cy = sqrt(R11*R11 + R12*R12);  // cy = |cos(pitch)|

  // euler.x() = atan2(R23,R33);
  // euler.y() = asin(-R13);
  // euler.z() = atan2(R12,R11);

  euler.x() = atan2(rot_mat(1,2),rot_mat(2,2));  // roll
  // euler.y() = asin(-rot_mat(0,2));
  euler.y() = atan2(-rot_mat(0,2),sqrt(rot_mat(0,0)*rot_mat(0,0) + rot_mat(0,1)*rot_mat(0,1)));  // pitch
  euler.z() = atan2(rot_mat(0,1),rot_mat(0,0));  // yaw

  // euler2.x() = atan2(2 * (q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());  // roll
  // euler2.y() = asin(-2 * (q.x()*q.z() - q.w()*q.y()));  // pitch
  // euler2.z() = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());  // yaw

  return euler;
}

void OdomHandler(const nav_msgs::Odometry::ConstPtr &odom_msg) {

  Eigen::Quaterniond q_odom_curr;
  Eigen::Vector3d t_odom_curr;
  q_odom_curr.x() = odom_msg->pose.pose.orientation.x;
  q_odom_curr.y() = odom_msg->pose.pose.orientation.y;
  q_odom_curr.z() = odom_msg->pose.pose.orientation.z;
  q_odom_curr.w() = odom_msg->pose.pose.orientation.w;
  t_odom_curr.x() = odom_msg->pose.pose.position.x;
  t_odom_curr.y() = odom_msg->pose.pose.position.y;
  t_odom_curr.z() = odom_msg->pose.pose.position.z;
  double odom_time = odom_msg->header.stamp.toSec();
//   cout << setprecision(10) << odom_time << endl;

  // Eigen::Vector3d euler_odom = q_odom_curr.matrix().eulerAngles(2,1,0)*57.3;
  Eigen::Vector3d euler_odom = Quat2Euler(q_odom_curr)*180/M_PI;
//   cout << "predict position: " << t_odom_curr.transpose() << endl;

//   destFile.fill('0');
  destFile <<setw(10)<< frame_count <<setw(15)<< odom_time <<setw(15)<< t_odom_curr.x() <<setw(15)<< t_odom_curr.y() <<setw(15)<< t_odom_curr.z() <<setw(15)\
           << euler_odom.x() <<setw(15)<< euler_odom.y() <<setw(15)<< euler_odom.z() << endl;


  frame_count++;
}

int main(int argc, char** argv)
{
    destFile.setf(ios::left);  //设置对齐方式（左对齐）
    // destFile.fill('0');
    destFile.setf(ios::fixed, ios::floatfield);  //设定为fixed模式，以小数点表示浮点数
    destFile.precision(4);  //设置小数点位数（精度）


    ros::init(argc, argv, "save_predict_odom");
    ros::NodeHandle nh("~");
    
    string output_file;
    nh.getParam("output_file",output_file);
    destFile.open(output_file,ios::out);

    ros::Subscriber sub_predict_odom_ = nh.subscribe<nav_msgs::Odometry>("/predict_odom", 100, OdomHandler); 

    ros::spin();

    destFile.close();

    return 0;
}
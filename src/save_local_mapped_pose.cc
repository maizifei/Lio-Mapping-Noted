#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <string>

using namespace std;

static int frame_count1 = 0;
static int frame_count2 = 0;
// ofstream destFile1;  //("/home/belight/Dataset/test20191231/predict_odom_test3.txt", ios::out);
// ofstream destFile2;
ofstream destFile;

ros::Time time_local_odometry_;
ros::Time time_mapped_odometry_;
bool new_local_odometry_; 
bool new_mapped_odometry_; 

Eigen::Vector3d t_odom_curr;
Eigen::Vector3d t_map_curr;
Eigen::Vector3d euler_odom;
Eigen::Vector3d euler_map;

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

void LocalOdomHandler(const nav_msgs::Odometry::ConstPtr &local_odom_msg) {

  time_local_odometry_ = local_odom_msg->header.stamp;

  Eigen::Quaterniond q_odom_curr;
//   Eigen::Vector3d t_odom_curr;
  q_odom_curr.x() = local_odom_msg->pose.pose.orientation.x;
  q_odom_curr.y() = local_odom_msg->pose.pose.orientation.y;
  q_odom_curr.z() = local_odom_msg->pose.pose.orientation.z;
  q_odom_curr.w() = local_odom_msg->pose.pose.orientation.w;
  t_odom_curr.x() = local_odom_msg->pose.pose.position.x;
  t_odom_curr.y() = local_odom_msg->pose.pose.position.y;
  t_odom_curr.z() = local_odom_msg->pose.pose.position.z;
  double odom_time = local_odom_msg->header.stamp.toSec();
//   cout << setprecision(10) << odom_time << endl;

  // Eigen::Vector3d euler_odom = q_odom_curr.matrix().eulerAngles(2,1,0)*57.3;
  euler_odom = Quat2Euler(q_odom_curr)*180/M_PI;
//   cout << "predict position: " << t_odom_curr.transpose() << endl;

//   destFile1.fill('0');
//   destFile1 <<setw(10)<< frame_count1 <<setw(15)<< odom_time <<setw(15)<< t_odom_curr.x() <<setw(15)<< t_odom_curr.y() <<setw(15)<< t_odom_curr.z() <<setw(15)\
//            << euler_odom.x() <<setw(15)<< euler_odom.y() <<setw(15)<< euler_odom.z() << endl;


  frame_count1++;
  new_local_odometry_ = true;
}

void MapOdomHandler(const nav_msgs::Odometry::ConstPtr &mapped_odom_msg) {

  time_mapped_odometry_ = mapped_odom_msg->header.stamp;
  
  Eigen::Quaterniond q_map_curr;
//   Eigen::Vector3d t_map_curr;
  q_map_curr.x() = mapped_odom_msg->pose.pose.orientation.x;
  q_map_curr.y() = mapped_odom_msg->pose.pose.orientation.y;
  q_map_curr.z() = mapped_odom_msg->pose.pose.orientation.z;
  q_map_curr.w() = mapped_odom_msg->pose.pose.orientation.w;
  t_map_curr.x() = mapped_odom_msg->pose.pose.position.x;
  t_map_curr.y() = mapped_odom_msg->pose.pose.position.y;
  t_map_curr.z() = mapped_odom_msg->pose.pose.position.z;
  double map_time = mapped_odom_msg->header.stamp.toSec();

  euler_map = Quat2Euler(q_map_curr)*180/M_PI;

//   destFile2 <<setw(10)<< frame_count2 <<setw(15)<< map_time <<setw(15)<< t_map_curr.x() <<setw(15)<< t_map_curr.y() <<setw(15)<< t_map_curr.z() <<setw(15)\
//            << euler_map.x() <<setw(15)<< euler_map.y() <<setw(15)<< euler_map.z() << endl;

  frame_count2++;
  new_mapped_odometry_ = true;
}

bool HasNewData(){
    return new_local_odometry_ && new_mapped_odometry_ && (frame_count1 == frame_count2);
}

void ProcessSaving(){

    if(!HasNewData()){
        return;
    }
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Has New Data<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    new_local_odometry_ = false;
    new_mapped_odometry_ = false;

  destFile <<setw(10)<< frame_count1 <<setw(15)<< time_local_odometry_.toSec() <<setw(15)<< t_odom_curr.x() <<setw(15)<< t_odom_curr.y() <<setw(15)<< t_odom_curr.z() <<setw(15)\
           << euler_odom.x() <<setw(15)<< euler_odom.y() <<setw(15)<< euler_odom.z() <<setw(15)<< t_map_curr.x() <<setw(15)<< t_map_curr.y() <<setw(15)<< t_map_curr.z() <<setw(15)\
           << euler_map.x() <<setw(15)<< euler_map.y() <<setw(15)<< euler_map.z() << endl;

}

int main(int argc, char** argv)
{
    // destFile1.setf(ios::left);  //设置对齐方式（左对齐）
    // // destFile1.fill('0');
    // destFile1.setf(ios::fixed, ios::floatfield);  //设定为fixed模式，以小数点表示浮点数
    // destFile1.precision(4);  //设置小数点位数（精度）

    // destFile2.setf(ios::left);
    // destFile2.setf(ios::fixed, ios::floatfield); 
    // destFile2.precision(4);

    destFile.setf(ios::left);
    destFile.setf(ios::fixed, ios::floatfield); 
    destFile.precision(4);

    ros::init(argc, argv, "save_local_mapped_odom");
    ros::NodeHandle nh("~");
    
    string output_file;
    // nh.getParam("output_local_odom",output_file1);
    // nh.getParam("output_map_to_init",output_file2);
    // destFile1.open(output_file1,ios::out);
    // destFile2.open(output_file2,ios::out);
    nh.getParam("output_local_mapped_odom",output_file);
    destFile.open(output_file,ios::out);

    ros::Subscriber sub_predict_odom_ = nh.subscribe<nav_msgs::Odometry>("/local_laser_odom", 100, LocalOdomHandler); 
    ros::Subscriber sub_map_to_init_ = nh.subscribe<nav_msgs::Odometry>("/lio_map_builder/aft_mapped_to_init", 100, MapOdomHandler);
    
    ros::Rate r(100);
    while(ros::ok()) {
      ProcessSaving();
      ros::spinOnce();
      r.sleep();
    }
    // ros::spin();

    // destFile1.close();
    // destFile2.close();
    destFile.close();

    return 0;
}
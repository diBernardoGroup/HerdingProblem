// Fabrizia Auletta
// fabrizia.auletta@bristol.ac.uk
// June 2020

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>
#include <iostream>
#include <fstream>


class NodeFunctions{

  public: 
    NodeFunctions();  
    void loop(); 
    void modelOdomCallback(nav_msgs::Odometry::ConstPtr);
    void modelNameCallback(const std_msgs::Int32MultiArray::ConstPtr); 
    
  private: 
    ros::NodeHandle nh;
    ros::Subscriber modelOdomSub, modelNameSub;
    
    float currX_[5], currY_[5];
    float currR_[5], currTH_[5];
    float currRdot_[5], currTHdot_[5];
 
    double currHerderHeading_[2], currTargetHeading_[3]; 
    
    bool first_odom_target, first_odom_herder, first_chased;
    
    int chasedID_[2]; 
    
    std::string ChasedFile_name, OdomFile_name, csv_sep;
    std::ofstream OdomFile, ChasedFile, FileStream;  

}; 

NodeFunctions::NodeFunctions()
{

   int queue_size = 100; 
   
   modelOdomSub = nh.subscribe("/odom", queue_size, &NodeFunctions::modelOdomCallback, this); 
   modelNameSub = nh.subscribe("/herder/chased_target",queue_size,&NodeFunctions::modelNameCallback, this);
  
   
   first_odom_herder = false;
   first_odom_target = false; 
   first_chased = false;  
   
   csv_sep = ";";
   ChasedFile_name = "/home/phab/catkin_ws/src/herd_ros/ChasedTargetID_.csv"; 
   OdomFile_name = "/home/phab/catkin_ws/src/herd_ros/OdomAgents.csv"; 
      
}

// callback for target_to_chase model name 
void NodeFunctions::modelNameCallback(const std_msgs::Int32MultiArray::ConstPtr msg)
{
  int i = 0; 
  
  for(std::vector<int>::const_iterator it = msg->data.begin(); it !=msg->data.end(); it++)
  {
    chasedID_[i] = * it;
    i++;     
  }
   
   
  
  first_chased = true; 
  
}

void NodeFunctions::modelOdomCallback(nav_msgs::Odometry::ConstPtr msg)    
{
  
  for(int i = 0; i < 2; i++)
  {
	  if(msg->child_frame_id == "herder_"+std::to_string(i)+"_base_link")
	  {
	  currX_[i] = msg->pose.pose.position.x; 
	  currY_[i] = msg->pose.pose.position.y; 
	  
	  currR_[i] = sqrt(pow(currX_[i],2) + pow(currY_[i],2)); 
	  currTH_[i] = atan2(currY_[i], currX_[i]); 
	  
	  tf::Quaternion quatHerder_(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	  double dummy; 
	  tf::Matrix3x3(quatHerder_).getRPY(dummy, dummy, currHerderHeading_[i]); 
	  
	  
	  currRdot_[i] = msg->twist.twist.linear.x; 
	  currTHdot_[i] = msg->twist.twist.angular.z;  
	  
	  first_odom_herder = true; 
  	  } //endif
  
  } //endfor
  
   for(int j = 2; j < 5; j++) 
   {
	  if (msg->child_frame_id == "target_"+std::to_string(j-2)+"_base_link")
	  {
	  
	  currX_[j] = msg->pose.pose.position.x; 
	  currY_[j] = msg->pose.pose.position.y; 
	  
	  currR_[j] = sqrt(pow(currX_[j],2) + pow(currY_[j],2)); 
	  currTH_[j] = ::atan2(currY_[j], currX_[j]); 
	  
	  tf::Quaternion quat_(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	  double dummy; 
	  tf::Matrix3x3(quat_).getRPY(dummy, dummy, currTargetHeading_[j]); 
	  
	  first_odom_target = true; 
  	  } //endif
  
  } //endfor

}



void NodeFunctions::loop()
{

  while(!first_chased){
  
  ros::spinOnce(); 
  sleep(0.5); 
  
  
  }
  
 
  while(!first_odom_herder){
  
  ros::spinOnce(); 
  sleep(0.5); 
  
  }
  
  
  while(!first_odom_target){
  
  ros::spinOnce(); 
  sleep(0.5); 
  
  }
  
    // write firt line of each files
    ChasedFile.open (ChasedFile_name);
    OdomFile.open(OdomFile_name); 
    
    OdomFile << "" << csv_sep; 
    
    for(int i = 0; i < 2; i++)
     {
      OdomFile <<"Herder_" + std::to_string(i) << csv_sep << "; ; ; ; ;";
      ChasedFile <<"Herder_" + std::to_string(i) << csv_sep;
     }
     
    for(int j = 2; j < 5; j++) 
      OdomFile <<"Target_" + std::to_string(j - 2)<< csv_sep << "; ; ; ; ;"; 
    
    OdomFile <<std::endl; 
    ChasedFile <<std::endl;
    
    OdomFile << "time;";
    for(int i = 0; i < 5; i++)
     OdomFile << "pos_x ; pos_y ; pos_r ; pos_th_ ; vel_r ; vel_th ;"; 
    
    OdomFile <<std::endl;    
    
   ros::Rate ratedt(10); 
   ros::Time current_time, last_time;  
   
   
  while(ros::ok())
  { 
  

    current_time = ros::Time::now(); 
    
    float dt = (current_time - last_time).toSec();  
    
    // write on ChasedFile the chased targets
    for(int i = 0; i < 2; i++)
     ChasedFile << chasedID_[i] << csv_sep;
 
    ChasedFile <<std::endl;
    
    
    // write on OdomFile odometric information on the agents 
    OdomFile << current_time << csv_sep; 
    for(int i = 0; i <5; i++)
    {
      OdomFile << currX_[i] << csv_sep; 
      OdomFile << currY_[i] << csv_sep; 
      OdomFile << currR_[i] << csv_sep; 
      OdomFile << currTH_[i] << csv_sep; 
      OdomFile << currRdot_[i] << csv_sep; 
      OdomFile << currTHdot_[i] << csv_sep;  
    }
    
    OdomFile << std::endl; 
   
    
    ros::spinOnce();
    ratedt.sleep(); 
    last_time = current_time; 

  }
  
  
  ChasedFile.close();
  OdomFile.close(); 

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "SaveData"); //
  NodeFunctions func; 
  func.loop(); 
  
  return 0; 
  
}
   



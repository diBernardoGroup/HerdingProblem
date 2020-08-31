// Fabrizia Auletta
// fabrizia.auletta@bristol.ac.uk
// June 2020

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <string>
#include <iostream>

#define PI 3.1415926535897931

// ros node to update the state of the herder agents

class NodeFunctions{

  public: 
    NodeFunctions(); 
    void loop(); 
    void modelOdomCallback(nav_msgs::Odometry::ConstPtr);
    void modelNameCallback(const std_msgs::Int32MultiArray::ConstPtr); 
    
  private: 
    ros::NodeHandle nh;
    ros::Subscriber modelOdomSub, modelNameSub;
    ros::Publisher modelStatePub;

    float currHerderX_, currHerderY_;
    float currTargetX_, currTargetY_;  
 
    float currHerderR_, currHerderTH_; 
    float currTargetR_, currTargetTH_; 
    
    float currHerderRdot_, currHerderTHdot_; 
 
    double currHerderHeading_, currTargetHeading_; 
    
    bool first_odom_target, first_odom_herder, first_chased;
    
    bool wait_odom; 
    
    int chasedID_[2]; 
    
    std::string string;
    int herderN_;   

}; 

NodeFunctions::NodeFunctions()
{

   int queue_size = 100; 
   
   ros::param::get("~my_string",string); 
   ros::param::get("~herder_number", herderN_); 

   modelOdomSub = nh.subscribe("/odom", queue_size, &NodeFunctions::modelOdomCallback, this); 
   modelNameSub = nh.subscribe("/herder/chased_target",queue_size,&NodeFunctions::modelNameCallback, this);
   
   modelStatePub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_"+std::to_string(herderN_), queue_size);
    
   
   first_odom_herder = false;
   first_odom_target = false; 
   first_chased = false;  
   wait_odom = true; 
      
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
   
  //ROS_INFO("read the name of the chased %i!", chasedID_[herderN_]); 
  
  first_chased = true; 
  
}

void NodeFunctions::modelOdomCallback(nav_msgs::Odometry::ConstPtr msg)
{
  if(msg->child_frame_id == "herder_"+std::to_string(herderN_)+"_base_link")
  {
  currHerderX_ = msg->pose.pose.position.x; 
  currHerderY_ = msg->pose.pose.position.y; 
  
  currHerderR_ = sqrt(pow(currHerderX_,2) + pow(currHerderY_,2)); 
  currHerderTH_ = atan2(currHerderY_, currHerderX_); 
  
  tf::Quaternion quatHerder_(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double dummy; 
  tf::Matrix3x3(quatHerder_).getRPY(dummy, dummy, currHerderHeading_); 
  
  
  currHerderRdot_ = msg->twist.twist.linear.x; 
  currHerderTHdot_ = msg->twist.twist.angular.z;  
  
  first_odom_herder = true; 
  
  }else if (msg->child_frame_id == "target_"+std::to_string(chasedID_[herderN_])+"_base_link")
    {
  
  currTargetX_ = msg->pose.pose.position.x; 
  currTargetY_ = msg->pose.pose.position.y; 
  
  currTargetR_ = ::sqrt(::pow(currTargetX_,2) + ::pow(currTargetY_,2)); 
  currTargetTH_ = ::atan2(currTargetY_, currTargetX_); 
  
  tf::Quaternion quat_(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double dummy; 
  tf::Matrix3x3(quat_).getRPY(dummy, dummy, currTargetHeading_); 
  
  first_odom_target = true; 

  //ROS_INFO("%i", chasedID_[herderN_]); 
  
  }
  
  if(first_odom_herder)
    wait_odom = false; 
  
  
}



void NodeFunctions::loop()
{

  while(!first_chased){
  
  ros::spinOnce(); 
  sleep(0.5); 

  }
  
  /**/
  while(!first_odom_herder){
  
  ros::spinOnce(); 
  sleep(0.5); 
  
  }
  
  
  while(!first_odom_target){
  
  ros::spinOnce(); 
  sleep(0.5); 
  
  }
  

  ROS_INFO("%s the herder_%d",string.c_str(),herderN_);

   ros::Rate ratedt(10); 
   ros::Time current_time, last_time; 
   
   const int br = 1; 
   const int er = 92 ; 
   const int bth = 1 ; 
   const int eth = 60; 
   const int rstar = 1; 
   const int deltar = 1;  
   
   float HerderX_;  
   float HerderY_; 
   float HerderR_ = 0.0; 
   float HerderTH_= 0.0;
   float HerderRdot_= 0.0;
   float HerderTHdot_= 0.0;
   float HerderRdotdot_;
   float HerderTHdotdot_;
   
   bool relocate = false;

   int csi = 1; 
   
    
   last_time = ros::Time::now(); 
   
   geometry_msgs::Twist HerderState; 

  while(ros::ok())
  { 
  

    current_time = ros::Time::now(); 
    
    float dt = (current_time - last_time).toSec();  
    
    
    if (currTargetR_ > rstar)
     csi = 1; 
    else
     csi = 0; 

    
    /* compute r_dotdot = u_r and th_dot_dot = u_th accelerations by novel model in world frame*/
    HerderRdotdot_ = - br * currHerderRdot_ - er * (currHerderR_ - csi * (currTargetR_+ deltar) - (1 - csi) * (rstar + deltar)); 
    HerderTHdotdot_ = - bth * currHerderTHdot_ - eth * (currHerderTH_ - csi * currTargetTH_); 
    
    /* update r_dot and th_dot velocities in world frame */
    HerderRdot_ = currHerderRdot_ + HerderRdotdot_ * dt;  
    HerderTHdot_ = currHerderTHdot_ + HerderTHdotdot_ * dt; 
    
    
    /* update r and th position in world frame */
    HerderR_ = currHerderR_ + HerderRdot_ * dt;  
    HerderTH_ = currHerderTH_ + HerderTHdot_ * dt ; 
     
    
    /* compute x and y position in world frame */
    HerderX_ = HerderR_ * cos(HerderTH_); 
    HerderY_ = HerderR_ * sin(HerderTH_); 
    
    //ROS_INFO("x = %f, y = %f", HerderX_, HerderY_);
      

    // cartesian regulation 
    float k1 = 0.125; 
    float k2 = 0.25; 
	
    float x_e = currHerderX_ - HerderX_; 
    float y_e = currHerderY_ - HerderY_; 
    
    float phi_e = atan2(HerderY_, HerderX_) - currHerderHeading_;
    
    if(fabs(phi_e) > PI)
	phi_e = phi_e - 2 * PI * ((phi_e>0)?1:-1); 
	
    float k = 1; 
    
    float x_vel = - k1 * (x_e * cos(currHerderHeading_) + y_e * sin(currHerderHeading_)); 
    float z_vel = k2 * (phi_e + k * PI ) ; 
    

    HerderState.linear.x = x_vel; 
    HerderState.angular.z = z_vel; 
    

    modelStatePub.publish(HerderState); 
    ros::spinOnce();
    ratedt.sleep(); 
    
    last_time = current_time; 

  }


}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "HerderState_0"); //
  NodeFunctions func; 
  func.loop(); 
  
  return 0; 
  
}
   



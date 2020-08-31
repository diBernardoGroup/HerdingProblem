// Fabrizia Auletta
// fabrizia.auletta@bristol.ac.uk
// June 2020

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

#include <tf/tf.h>

#include <cmath>
#include <string>

#include <random>
#define PI 3.1415926535897931

// ros node to update the state of the target agents


class NodeFunctions{

  public: 
    NodeFunctions();  
    void loop(); 
    void modelOdomCallback(nav_msgs::Odometry::ConstPtr);
    float DistanceFromHerder(float, float); 
  private: 
    ros::NodeHandle nh;
    ros::Subscriber modelOdomSub;
    ros::Publisher modelStatePub;
    ros::Publisher modelGazeboStatePub; 

    bool first_odom_target, first_odom_herder;
    
    float BrownMag; 
    float RepMag; 
    
    float currX_[3]; 
    float currY_[3]; 
    float currR_[3]; 
    double currTH_[3]; 
    
    double currHeading_[3]; 

    float pos_th_e0 , pos_th_e1, pos_e; 
    float des_yaw, yaw_e; 
     
    std::string string;
    int targetN_; 

}; 

NodeFunctions::NodeFunctions()
{

   int queue_size = 100; 
   
   ros::param::get("~my_string",string); 
   ros::param::get("~target_number", targetN_); 
   
   modelOdomSub = nh.subscribe("/odom", queue_size, &NodeFunctions::modelOdomCallback, this); 
   modelStatePub = nh.advertise<geometry_msgs::Twist>("/target_"+std::to_string(targetN_)+"_cmd_vel", queue_size);
   modelGazeboStatePub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", queue_size); 
   
   
   BrownMag = 1; 
   RepMag = 2; 
   
   first_odom_target = false; 
   first_odom_herder = false; 
   
   
   
   
}

void NodeFunctions::modelOdomCallback(nav_msgs::Odometry::ConstPtr msg)
{
  
  if(msg->child_frame_id == "target_"+std::to_string(targetN_)+"_base_link")
  {
	  int index = 0; 
	  
	  currX_[index] = msg->pose.pose.position.x; 
	  currY_[index] = msg->pose.pose.position.y; 
	  
	  currR_[index] = ::sqrt(::pow(currX_[index],2) + ::pow(currY_[index],2)); 
	  currTH_[index] = ::atan2(currY_[index], currX_[index]); 
	  
	  tf::Quaternion quat_(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	  double dummy; 
	  tf::Matrix3x3(quat_).getRPY(dummy, dummy, currHeading_[index]); 
	  
	  first_odom_target = true; 
  
  }
  else if (msg->child_frame_id == "herder_0_base_link")
    {
	  int index = 1; 
	  
	  currX_[index] = msg->pose.pose.position.x; 
	  currY_[index] = msg->pose.pose.position.y; 
	  
	  currR_[index] = ::sqrt(::pow(currX_[index],2) + ::pow(currY_[index],2)); 
	  currTH_[index] = ::atan2(currY_[index], currX_[index]); 
	  
	  tf::Quaternion quat_(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	  double dummy; 
	  tf::Matrix3x3(quat_).getRPY(dummy, dummy, currHeading_[index]); 
  
  
  }
  else if (msg->child_frame_id == "herder_1_base_link")
    {
	  int index = 2; 
	  
	  currX_[index] = msg->pose.pose.position.x; 
	  currY_[index] = msg->pose.pose.position.y; 
	  
	  currR_[index] = ::sqrt(::pow(currX_[index],2) + ::pow(currY_[index],2)); 
	  currTH_[index] = ::atan2(currY_[index], currX_[index]); 
	  
	  tf::Quaternion quat_(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	  double dummy; 
	  tf::Matrix3x3(quat_).getRPY(dummy, dummy, currHeading_[index]); 
	  
	  first_odom_herder = true; 
  }
  
     // from herder 
     
     pos_th_e0 = sqrt(pow(currX_[0] - currX_[1],2) + pow(currY_[0] - currY_[1],2)); 
     pos_th_e1 = sqrt(pow(currX_[0] - currX_[2],2) + pow(currY_[0] - currY_[2],2)); 
     
     // from goal (0,0)
     
     pos_e = sqrt(pow(currX_[0],2) + pow(currY_[0],2)); 
     des_yaw = atan2(currY_[0] - 0, currX_[0] - 0) + PI * ((atan2(currY_[0]-0, currX_[0]-0)>PI)?1:-1); 
     
     yaw_e = des_yaw - currHeading_[0]; 
  
  
}

float NodeFunctions::DistanceFromHerder(float HerderX_, float HerderY_)
{
     
   float distanceX = pow(currX_[0] - HerderX_,2); 
   float distanceY = pow(currY_[0] - HerderY_,2); 
   float distanceAbs = sqrt(distanceX + distanceY); 
   float repulden = pow(distanceAbs,3); 

   return repulden; 
}

void NodeFunctions::loop()
{


    while(!first_odom_herder)
    {
	ros::spinOnce(); 
	sleep(0.5); 
    }
  
  
    while(!first_odom_target)
    {
	ros::spinOnce(); 
	sleep(0.5); 
    }
  
   ROS_INFO("%s  target_%d",string.c_str(),targetN_);
  
   ros::Rate ratedt(10);  
   ros::Time current_time, last_time; 
   
   std::default_random_engine generator; 
   std::normal_distribution<float> distribution(0,sqrt(0.1));

   geometry_msgs::Twist State; 
   gazebo_msgs::ModelState GazeboState; 
   
   bool reorientated = false;
   
   float delta_rmin = 1.5, rmin = 1.0;  
   
  while(ros::ok())
  {
    

    ///////////// target dynamics /////////////////////////
     


     float x_vel = 0.0, z_vel= 0.0, k = 1.0; 
     
     float BrownX_ = BrownMag * distribution(generator);  
     float BrownY_ = BrownMag * distribution(generator); 
    


     if((pos_th_e0 > delta_rmin) || (pos_th_e1 > delta_rmin))  
     // if relative distance b/w herders and target greather than threashold delta_rmin = 1 --> reorient towards goal (to change in brownian)
     {
     
       //ROS_INFO("ranx %f rany %f", BrownX_, BrownY_); 
       
    
	      if(fabs(yaw_e) > PI)  // check on angles
	      { 
		 yaw_e = yaw_e - 2 * PI * ((yaw_e>0)?1:-1);
		 k = -1; 
	      }
	
         
         float rep = pos_th_e0/pow(pos_th_e0,2) + pos_th_e1/pow(pos_th_e1,2); 
         
         x_vel = 0.5 * fabs(BrownX_); 
         z_vel = 2 * BrownY_; 
         
   
	      State.linear.x = x_vel; 
	      State.angular.z = z_vel; 
	      modelStatePub.publish(State); 
    }
    


    if((pos_th_e0 < delta_rmin) || (pos_th_e1 < delta_rmin)) 
    {
    
      //ROS_INFO("herder in range of target_%i", targetN_); 
    
	      if(!reorientated)  // if not oriented towards goal
	      { 
	       // ROS_INFO("target_%i reorientated", targetN_); 
	       
	    	GazeboState.model_name = "my_target_"+std::to_string(targetN_); 
	    	GazeboState.pose.position.x = currX_[0]; 
	    	GazeboState.pose.position.y = currY_[0]; 
	    	GazeboState.pose.orientation = tf::createQuaternionMsgFromYaw(des_yaw);
	    	GazeboState.reference_frame="world"; 
	    	modelGazeboStatePub.publish(GazeboState); 
	    	reorientated = true; 
	      }
	      else
	      {
	      
	 
	    	if(fabs(pos_e)>rmin) // if outside goal region 
		{
		  
		  //ROS_INFO("target_%i moving", targetN_);
		  first_odom_target = false; 
		  
		  x_vel = 2 * pos_e; 
		  z_vel = 0.0; 
		}

	      }
      
	      State.linear.x = x_vel; 
	      State.angular.z = z_vel; 
	      modelStatePub.publish(State); 
    
    }else if(fabs(des_yaw)>0.2)
      reorientated = false; 
      
    if(fabs(pos_e)<rmin)
    {
      x_vel = 0.0; 
      z_vel = 0.0; 
      State.linear.x = x_vel; 
      State.angular.z = z_vel; 
      modelStatePub.publish(State);
    }
    
    ros::spinOnce();
    ratedt.sleep();    

  }

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "TargetState"); 
  NodeFunctions func; 
  func.loop(); 
  
  return 0; 
  
}
   



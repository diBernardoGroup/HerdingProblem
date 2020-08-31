#!/usr/bin/env python
## Fabrizia Auletta
## fabrizia.auletta@bristol.ac.uk
## June 2020

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
import numpy as np


## global var
first_reading = False
chasedPub = rospy.Publisher('/herder/chased_target', Int32MultiArray , queue_size = 1)
pub = rospy.Publisher('chatter',String,queue_size = 10)
chasedID_ = Int32MultiArray()
HerderPOS_ = np.empty(shape=(4,2))  ## N_coordinates x N_herders  -- coordinates = X,Y,R,TH
TargetPOS_ = np.empty(shape=(4,3))  ## N_coordinates x N_targets  -- coordinates = X,Y,R,TH

###########################################################################

def glob(Target_rpos):
  
  chased = np.flip(np.argsort(Target_rpos),axis=0)[0:2]
  
  return chased 

###########################################################################

def stat(Target_pos):
  
  chased = np.zeros(shape=(2))
  temp0 = np.empty(shape=(Target_pos.shape[1]))
  temp1 = np.empty(shape=(Target_pos.shape[1]))
  
  
  Target_angle = Target_pos[3,:]
  index_sorted = np.flip(np.argsort(Target_pos[2,:]),axis=0)
  Target_angle_sorted = Target_angle[index_sorted]
  
  #rospy.loginfo("target angles : %f %f %f", Target_angle_sorted[0], Target_angle_sorted[1], Target_angle_sorted[2])
  count0 = 0; 
  count1 = 0; 
  
  for i in range(0,Target_pos.shape[1]):
    if Target_angle_sorted[i] < 0:
      temp0[count0] = index_sorted[i]
      count0 +=1
      
  for i in range(0,Target_pos.shape[1]):
    if Target_angle_sorted[i] >= 0:
      temp1[count1] = index_sorted[i]
      count1 +=1
      
  chased[0] = temp0[0] 
  chased[1] = temp1[0]
  
  
  return chased

###########################################################################

def lf(Target_pos, Herder_pos):
  
	
	Herder_angle = (Herder_pos[3,:] + 2 * np.pi) % (2 * np.pi)
	Target_angle = (Target_pos[3,:] + 2 * np.pi) % (2 * np.pi)
	index_sorted = np.flip(np.argsort(Target_pos[2,:]),axis=0)
	Target_angle_sorted = Target_angle[index_sorted]
	
	chased = np.ones(shape=(2)) * (Target_pos.shape[1] + 1)

	LB_0 = Herder_angle[0] - (np.pi / Herder_pos.shape[1])
	UB_0 = LB_0 + (2 * np.pi) / Herder_pos.shape[1]
	
	found = 0

	for i in range(0,3):
	    if Target_angle_sorted[i] < UB_0:
		if Target_angle_sorted[i] >= LB_0:
		    if found == 0:
		        chased[0] = index_sorted[i]  
		        found = 1
		    
	found = 0            
		
	for i in range(0,3):
	    if Target_angle_sorted[i] < LB_0:
		if Target_angle_sorted[i] != chased[0]:
		    if found == 0:
		        chased[1] = index_sorted[i]  
		        found = 1
	
	return chased

###########################################################################

def p2p(Target_pos, Herder_pos):
  
	
	Herder_angle = (Herder_pos[3,:] + 2 * np.pi) % (2 * np.pi)
	Target_angle = (Target_pos[3,:] + 2 * np.pi) % (2 * np.pi)
	index_sorted = np.flip(np.argsort(Target_pos[2,:]),axis=0)
	Target_angle_sorted = Target_angle[index_sorted]

	chased = np.ones(shape=(2)) * (Target_pos.shape[1] + 1)

	LB_0 = Herder_angle[0] - (np.fabs(Herder_angle[0] - Herder_angle[1]) / Herder_pos.shape[1])
	UB_0 = LB_0 + (2 * np.pi) / Herder_pos.shape[1]

	found = 0

	for i in range(0,3):
	    if Target_angle_sorted[i] < UB_0:
		if Target_angle_sorted[i] >= LB_0:
		    if found == 0:
		        chased[0] = index_sorted[i]  
		        found = 1
		    
	found = 0            
		
	for i in range(0,3):
	    if Target_angle_sorted[i] < LB_0:
		if Target_angle_sorted[i] != chased[0]:
		    if found == 0:
		        chased[1] = index_sorted[i]  
		        found = 1
	
	return chased

###########################################################################

def odom_callback(data):

  ## read positions
  
  for i in range(0,2):
    if data.child_frame_id == "herder_"+str(i)+"_base_link":
      xpos = data.pose.pose.position.x 
      ypos = data.pose.pose.position.y
      rpos = np.sqrt(np.power(xpos,2)+np.power(ypos,2))
      thpos = np.arctan2(ypos, xpos)
      HerderPOS_[:,i] = [xpos,ypos,rpos,thpos]
   
  for j in range(0,3):
    if data.child_frame_id == "target_"+str(j)+"_base_link":
      xpos = data.pose.pose.position.x 
      ypos = data.pose.pose.position.y
      rpos = np.sqrt(np.power(xpos,2)+np.power(ypos,2))
      thpos = np.arctan2(ypos, xpos)
      TargetPOS_[:,j] = [xpos,ypos,rpos,thpos]
  
  first_reading = True

  
  ## compute task division
  
  chasedID_.data = [0,1]
  
  task = rospy.get_param("~taskDivision")
  
  if task == "global":
    chasedID_.data = glob(TargetPOS_[2,:])
  if task == "static":
    chasedID_.data = stat(TargetPOS_)
  if task == "leaderfollower":
    chasedID_.data = lf(TargetPOS_, HerderPOS_)
  if task == "peer2peer":
    chasedID_.data = p2p(TargetPOS_, HerderPOS_)
    
  ## publish topic
  chasedPub.publish(chasedID_)
  
###########################################################################

def loop():
  
  rospy.Subscriber('/odom', Odometry, odom_callback)
  rospy.spin()

###########################################################################

if __name__ == '__main__':
  try:
    rospy.init_node('TaskDivision',anonymous = True)  
    rospy.loginfo("chasing with %s strategy",rospy.get_param("~taskDivision"))  
    loop()
  
  except rospy.ROSInterruptException:
    pass

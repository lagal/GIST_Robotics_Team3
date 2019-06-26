#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/JointPosition.h"

#include "sensor_msgs/JointState.h"

float x; //= 3144;  // subscribe node
float y; 

float pre_x;
float pre_y;

float d;
float h = 0.2*1000;//0.1041*1000;

float l0 = 0.218*1000;
float l1 = 0.13*1000;
float l2 = 0.124*1000;
float l3 = 0.126*1000;

float th3 = -20*(3.1415926535/180);

double e=1;

float joint1;
float joint2;
float joint3;
float joint4;

float th_initial;//=-90*(3.141592/180);

int count;
int count2;

float th2;
float th1;

int point;
int point2;

int operation1;

int op1;

int count3;
int count4;

//float x;
//float y=h-l0-l3*sin(th3);


void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	x=msg->data[0];
	y=msg->data[1];
	d = sqrt(x*x+y*y);
	if ( x == 0 ) x= 12;
	if ( y == 0 ) y= 12;
	if( x != pre_x || y!= pre_y ) {operation1 = 1; d = sqrt(x*x+y*y); pre_x = x; pre_y = y;}	

	else 
	     { 
	operation1 = 0;
	pre_x = x;
	pre_y = y; 
	     }
}

void chatterCallback2(const sensor_msgs::JointState::ConstPtr& msg)
{
	joint1=msg->position[2];
	joint2=msg->position[3];
	joint3=msg->position[4];
	joint4=msg->position[5];

}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "inverse");

  ros::NodeHandle n;

  ros::ServiceClient chatter_pub = n.serviceClient<open_manipulator_msgs::SetJointPosition>("/arm/moveit/set_joint_position");
  ros::ServiceClient chatter_pub2 = n.serviceClient<open_manipulator_msgs::SetJointPosition>("/om_with_tb3/gripper");
  ros::Subscriber sub = n.subscribe("operation",1,chatterCallback);
  ros::Subscriber sub2 = n.subscribe("om_with_tb3/joint_states",1,chatterCallback2);

  th2 = -1*(asin((-d*d-h*h+l1*l1+l2*l2)/(2*l1*l2)));
  th1 = -1*(3.141592/2-acos((l1+l2*sin(th2))/sqrt(d*d+h*h))-atan(h/d));
  th_initial = 0.1;
  op1 = 0;
  x = 12;
while(1){

      	open_manipulator_msgs::SetJointPosition srv;
      	open_manipulator_msgs::SetJointPosition srv2;
      	open_manipulator_msgs::SetJointPosition srv3;
      	open_manipulator_msgs::SetJointPosition srv4;
      	open_manipulator_msgs::SetJointPosition srv5;
      	open_manipulator_msgs::SetJointPosition srv6;

    	srv3.request.joint_position.joint_name.resize(1); // first
    	srv3.request.joint_position.position.resize(1); 

    	srv3.request.planning_group = "gripper_group";
    	srv3.request.path_time = 2;
    	srv3.request.joint_position.joint_name[0] = "gripper_1";
    	srv3.request.joint_position.position[0] = 0.02;
    	srv3.request.joint_position.max_accelerations_scaling_factor = 0.1;
    	srv3.request.joint_position.max_velocity_scaling_factor = 0.1;

if ( x== 12 ) ROS_INFO("Let's start");
if ( x== 3144 ) // process died // 
{ 		
		// reach the trashcan // 
		if ( count3 == 0){
		srv5.request.joint_position.joint_name.resize(4);
    		srv5.request.joint_position.position.resize(4); 

    		srv5.request.planning_group = "arm";
    		srv5.request.path_time = 2;

    		srv5.request.joint_position.joint_name[0] = "joint1";
    		srv5.request.joint_position.joint_name[1] = "joint2";
    		srv5.request.joint_position.joint_name[2] = "joint3";
    		srv5.request.joint_position.joint_name[3] = "joint4";

    		srv5.request.joint_position.position[0] = 90*(3.141592/180);
    		srv5.request.joint_position.position[1] = 10*(3.141592/180);
    		srv5.request.joint_position.position[2] = 0;
    		srv5.request.joint_position.position[3] = 0;

    		srv5.request.joint_position.max_accelerations_scaling_factor = 0.1;
    		srv5.request.joint_position.max_velocity_scaling_factor = 0.1;
    
    		if (chatter_pub.call(srv5))
  		 {      
   		  ROS_INFO("get the position");
  		 }
		sleep(5); // wait for second - reach the trash can....
		// arm position go go go
		  chatter_pub2.call(srv3); // gripper open
count3++;}
		//back to initial position after trash can
	   srv6.request.joint_position.joint_name.resize(4);
    	   srv6.request.joint_position.position.resize(4); 

    	   srv6.request.planning_group = "arm";
    	   srv6.request.path_time = 2;

     	   srv6.request.joint_position.joint_name[0] = "joint1";
     	   srv6.request.joint_position.joint_name[1] = "joint2";
     	   srv6.request.joint_position.joint_name[2] = "joint3";
     	   srv6.request.joint_position.joint_name[3] = "joint4";

	
	

      	   srv6.request.joint_position.position[0] = 0;
      	   srv6.request.joint_position.position[1] = -90*(3.141592/180);
      	   srv6.request.joint_position.position[2] =  60*(3.141592/180);
      	   srv6.request.joint_position.position[3] =  30*(3.141592/180);

    
      	   srv6.request.joint_position.max_accelerations_scaling_factor = 0.1;
      	   srv6.request.joint_position.max_velocity_scaling_factor = 0.1;
		//op1 = 1;
           chatter_pub.call(srv6);
	   count3 = 0;
	   x=12;
	   y=12;
}		
if ( x != 12 && x != 3144)
   { 
  // ROS_INFO("%f**%f",th1,th2);
          if( isnan(th1) || isnan(th2) ){
	  th1 = joint2; 
	  th2 = joint3;}
	if ((point2 == 1 ) && (d < 300))
	 {
         // op1 = 0;
	  point2 = 0;
 	  th2 = -1*(asin((-d*d-h*h+l1*l1+l2*l2)/(2*l1*l2)));
          th1 = -1*(3.141592/2-acos((l1+l2*sin(th2))/sqrt(d*d+h*h))-atan(h/d));
	  //sleep(5);
		// theta2, theta1 back to initial	
	 }ROS_INFO("%f**!!@@@@@@@",th1);

	
 // stay

	while(1)
	{ ROS_INFO("Hello");
	 if (operation1 ==1) break;
	//ros::spinOnce();	 
	}
	//ROS_INFO("%f",operation1);

        while(1)
 	 {   //   ros::spinOnce();
                  ROS_INFO("he");
		if ( count == 0)
		{
			chatter_pub2.call(srv3);
			ROS_INFO("get the gripper_position1");
			count++;
		}
  	 	else break;
		 
         }
	//ROS_INFO("%f**%f",joint2,th1);
	//ROS_INFO("%f*************%f**%f",th1,th2,joint2 - th1);

	while ( (joint2 - th1) > 0.05 || (joint2 - th1) < -0.05)
	{
	//{ // only position operation not gripper
   		ros::spinOnce();
		ROS_INFO("gooooooooooood");

    		srv.request.joint_position.joint_name.resize(4);
    		srv.request.joint_position.position.resize(4); 

    		srv.request.planning_group = "arm";
    		srv.request.path_time = 2;

    		srv.request.joint_position.joint_name[0] = "joint1";
    		srv.request.joint_position.joint_name[1] = "joint2";
    		srv.request.joint_position.joint_name[2] = "joint3";
    		srv.request.joint_position.joint_name[3] = "joint4";

    		srv.request.joint_position.position[0] = 0;
    		srv.request.joint_position.position[1] = th1;
    		srv.request.joint_position.position[2] = th2;
    		srv.request.joint_position.position[3] = th3;

    		srv.request.joint_position.max_accelerations_scaling_factor = 0.1;
    		srv.request.joint_position.max_velocity_scaling_factor = 0.1;
    
    		//chatter_pub.call(srv);
  		if (chatter_pub.call(srv))
  		 {      
   			ROS_INFO("get the position");
  		 }
//	ROS_INFO("%f***********%f",joint2,th1);
	} // go to object position
	sleep(2);

	float k = joint2-th1;
	ROS_INFO("%f****%f*****%f",joint2,th1,k);

//	while (( (joint2 - th1) < 0.05  && (joint2 - th1) > 0) ||  ( (joint2 - th1) > -0.05  && (joint2 - th1) ) < 0 )
//	{
	  //  ros::spinOnce();
	    ROS_INFO("hello");
            th_initial=-90*(3.141592/180);
	    srv2.request.joint_position.joint_name.resize(1);
	    srv2.request.joint_position.position.resize(1); 

	    srv2.request.planning_group = "gripper_group";
	    srv2.request.path_time = 2;
	    srv2.request.joint_position.joint_name[0] = "gripper_1";
	    srv2.request.joint_position.position[0] = -0.01;
	    srv2.request.joint_position.max_accelerations_scaling_factor = 0.1;
	    srv2.request.joint_position.max_velocity_scaling_factor = 0.1;
	//   if(chatter_pub2.call(srv2))
	//{
	  // ROS_INFO("get the gripper_position2");  
	   point = 1;
//	   break;
 	//} // gripper operation.
//       }//    sleep(3);
         th_initial = joint2-0.7;//add
 while (point == 1 && ( (joint2 - th_initial) > 0.05 || (joint2 - th_initial) < -0.05) )
	{  ROS_INFO("step4");
	   chatter_pub2.call(srv2);
	  // ros::spinOnce();
    	   srv4.request.joint_position.joint_name.resize(4);
    	   srv4.request.joint_position.position.resize(4); 

    	   srv4.request.planning_group = "arm";
    	   srv4.request.path_time = 2;

     	   srv4.request.joint_position.joint_name[0] = "joint1";
     	   srv4.request.joint_position.joint_name[1] = "joint2";
     	   srv4.request.joint_position.joint_name[2] = "joint3";
     	   srv4.request.joint_position.joint_name[3] = "joint4";

	
	

      	   srv4.request.joint_position.position[0] = 0;
      	   srv4.request.joint_position.position[1] = -90*(3.141592/180);
      	   srv4.request.joint_position.position[2] =  60*(3.141592/180);
      	   srv4.request.joint_position.position[3] =  30*(3.141592/180);

    
      	   srv4.request.joint_position.max_accelerations_scaling_factor = 0.1;
      	   srv4.request.joint_position.max_velocity_scaling_factor = 0.1;
    
	if (chatter_pub.call(srv4))
   {
      //  ROS_INFO("%f***%f",joint2,th_initial);
	count = 0;
        point = 0;
	point2 = 1;
	th_initial = 0.1;
	x=12;
   }//if
  }//while
sleep(10);
}// else if 3144

ros::spinOnce();
} //this one	
  return 0;
}

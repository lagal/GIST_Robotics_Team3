
#if 1
#include <iostream>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>

using namespace cv;
using namespace std;

double x_odom;
double y_odom;
double qx;
double qy;
double qz;
double qw;
double r[5];

RNG rng(12345);
void ChatterCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  x_odom=msg->pose.pose.position.x;
  y_odom=msg->pose.pose.position.y;
  qw=msg->pose.pose.orientation.w;
  qx=msg->pose.pose.orientation.x;
  qy=msg->pose.pose.orientation.y;
  qz=msg->pose.pose.orientation.z;
}

int main(int argc, char **argv) {
	Mat frame, frame_HSV, frame_result;
	VideoCapture cap(0);
	ros::init(argc, argv, "position_estimation");
	ros::NodeHandle n;
	ros::Subscriber sub2= n.subscribe("/om_with_tb3/odom",1,ChatterCallback2);
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("operation", 1);			
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float32MultiArray>("navigation", 1); 		

	ros::Publisher chatter_pub3 = n.advertise<geometry_msgs::Twist>("om_with_tb3/cmd_vel", 1);
	geometry_msgs::Twist pstamp;

	double position[5][2];
	double distance[5][2];	
  	Mat imgOriginal;
	Mat imgHSV;
	Mat limited_imgHSV;
  	Mat imgThresholded;
  	Mat redThresholded;
  	Mat greenThresholded;
  	Mat blackThresholded;
 
        vector<vector<Point>>contours_red;
  	vector<vector<Point>>contours_green;
  	vector<vector<Point>>contours_black;
  		
  	vector<Vec4i> hierarchy_red;
  	vector<Vec4i> hierarchy_blue;
  	vector<Vec4i> hierarchy_green;
  	vector<Vec4i> hierarchy_black;
	
	double yaw;
	double px,py;
	int j =0; 
  	
	int l=1;
	double dist;
	double angle;

	double max;
	int num;
	Scalar color = Scalar(255,0,0);		
	position[2][0]=520;
	position[2][1]=20;
	position[4][0]=520;
	position[4][1]=20;
			
	while (ros::ok()) 
	{
		std_msgs::Float32MultiArray msg;
		cap >> imgOriginal; 
  		if (imgOriginal.empty())break;

  		imgOriginal.copyTo(imgHSV);
  		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); 
			
		imgHSV=imgHSV(cv::Rect(70,60,540,350));  //구역 제한
		if(l%2==0&&r[l]==0)
		{		
			for(int i=0; i<100;i++)
			{
			  if(yaw > 0.05) 
			  {
			    pstamp.angular.z = 0.1;
			    cout<<"high"<<endl;
			  }
			
			  if(yaw < -0.05) 
			  {
			    pstamp.angular.z = -0.1;
			    cout<<"low"<<endl;
			  }
			  chatter_pub3.publish(pstamp);
			  
			  if(yaw<0.01&&yaw>-0.01) 
			  {	
				pstamp.angular.z = 0;
				chatter_pub3.publish(pstamp);
				cout<<"ByeBye!"<<endl;
				r[l]=1;
				sleep(10);
				break;
			  }
			}
  		}
		if(l==3&&r[l]==0)
		{		
			for(int i=0; i<100;i++)
			{
			  if(yaw > 3.15 || yaw<-3.15) 
			  {
				pstamp.angular.z = 0.1;
				cout<<"high"<<endl;
			  }
			
			  if(yaw < 3.13 || yaw >-3.13) 
			  {
			  	pstamp.angular.z = -0.1;
			  	cout<<"low"<<endl;
			  }
			
			  chatter_pub3.publish(pstamp);
			  int a=(yaw< 3.15)&&(yaw>3.13);
			  int b=(yaw>-3.15)&&(yaw<-3.13);
			if(a||b) 
			{	
				pstamp.angular.z = 0;
				chatter_pub3.publish(pstamp);
				cout<<"ByeBye!"<<endl;
				r[l]=1;
				sleep(15);
				break;
			}
			}
  		}
  //------thresholding-----
  //red
  inRange(imgHSV, Scalar(150, 60, 30), Scalar(190, 255, 255), redThresholded);
  //green
  inRange(imgHSV, Scalar(40, 100, 82), Scalar(90, 250, 255), greenThresholded);
  //black(turtlebot)
  inRange(imgHSV, Scalar(0, 0, 0), Scalar(180,200,40),blackThresholded);


  // Find Contour
  //4-(1)Find position of red object

  if (j<=10)     
  {
	GaussianBlur(redThresholded, redThresholded, Size(3, 3), 0, 0);	

    	findContours(redThresholded, contours_red, hierarchy_red, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<Moments> mu(contours_red.size());
	vector<Point2f> mc(contours_red.size());
	vector<Vec4i> lines_red;
	for (int i = 0; i < contours_red.size(); i++)
	{
		mu[i] = moments(contours_red[i], false);
	}
	max=0;
	num=0;
	
	
	for (int i = 0; i < contours_red.size(); i++)
	{
		if(max<mu[i].m00)
		{
		  max=mu[i].m00;
		  num=i;
		}		
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}
	Mat drawing = Mat::zeros(imgHSV.size(), CV_8UC3);
	Scalar color = Scalar(255,0,0);
	drawContours(imgOriginal, contours_red, num, color, 2, 8, hierarchy_red, 0, Point());
	circle(imgOriginal, mc[num], 4, color, -1, 8, 0);


	px=mc[num].x;
	py=mc[num].y;
  }
  
  else if (j<=20)
  {

	GaussianBlur(greenThresholded, greenThresholded, Size(3, 3), 0, 0);	
   	findContours(greenThresholded, contours_green, hierarchy_green, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<Moments> mu(contours_green.size());
	vector<Point2f> mc(contours_green.size());
	
	for (int i = 0; i < contours_green.size(); i++)
	{
		mu[i] = moments(contours_green[i], false);
	}
	max=0;
	num=0;
	//  Get the mass centers:
	for (int i = 0; i < contours_green.size(); i++)
	{
		if(max<mu[i].m00)
		{
		  max=mu[i].m00;
		  num=i;
		}		
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}
	Mat drawing = Mat::zeros(imgHSV.size(), CV_8UC3);
	Scalar color = Scalar(255,0,0);
	drawContours(imgOriginal, contours_green, num, color, 2, 8, hierarchy_green, 0, Point());
	circle(imgOriginal, mc[num], 4, color, -1, 8, 0);
	px=mc[num].x;
	py=mc[num].y;
  }
 
  else
  {
	
    	findContours(blackThresholded, contours_black, hierarchy_black, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<Moments> mu(contours_black.size());
	vector<Point2f> mc(contours_black.size());
	
	for (int i = 0; i < contours_black.size(); i++)
	{
		mu[i] = moments(contours_black[i], false);
	}
	max=0;
	num=0;
	//  Get the mass centers:
	for (int i = 0; i < contours_black.size(); i++)
	{
		if(max<mu[i].m00)
		{
		  max=mu[i].m00;
		  num=i;
		}		
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}
	Mat drawing = Mat::zeros(imgHSV.size(), CV_8UC3);
	
	drawContours(imgOriginal, contours_black, num, color, 2, 8, hierarchy_black, 0, Point());
	circle(imgOriginal, mc[num], 4, color, -1, 8, 0);
	px=mc[num].x;
	py=mc[num].y;
	
  }
 
// Find position of each object

  if(j==10)
  {
	position[1][0]=px;
	position[1][1]=py;
	//cout<<"red position is"<<position[1][0]<<"\t" << position[1][1]<<endl;
  }

  
  else if(j==20)
  {
	position[3][0]=px;
	position[3][1]=py;
	//cout<<"green position is"<<position[5][0]<<"\t" << position[5][1]<<endl;
  }
  

  if(j>20)
  {
	position[0][0]=px;
	position[0][1]=py;
	for (int i=0;i<=8;i++)
	{
		distance[i][0]=(position[i][0]-px)/115;
		distance[i][1]=(position[i][1]-py)/115;
	}
	dist=sqrt(distance[l][0]*distance[l][0]+distance[l][1]*distance[l][1]);
	//cout<<"dist is "<<dist<<endl;	
	angle = -atan2(distance[l][1],distance[l][0]);		
  }


  //cout << "x : " << cx << "        y : " <<  cy << endl;
  namedWindow("Contours", CV_WINDOW_AUTOSIZE);
  imshow("Black", blackThresholded);
  imshow("red", redThresholded);
  imshow("green", greenThresholded);
  imshow("frame", imgOriginal);

  double siny_cosp = +2.0 * (qw * qz + qx * qy);
  double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);  
  yaw = atan2(siny_cosp, cosy_cosp);
 	
    cout<<"yaw is"<<yaw<< "angle is "<<angle<<endl;
				
  	if(j>20){	
		if(dist<0.2) //가까이 다가갔을때
		{	
			  pstamp.linear.x = 0;
			  pstamp.linear.y = 0;
			  pstamp.linear.z = 0;
	
			  pstamp.angular.x = 0;
			  pstamp.angular.y = 0;
			  pstamp.angular.z = 0;

			
		

			msg.data.resize(2);	//로봇팔 퍼블리시. 전달하는 내용은 object 위치
			chatter_pub3.publish(pstamp); 			

			if(l%2==1)
			{       
				msg.data[0]=distance[l][0]*1000;
  				msg.data[1]=distance[l][1]*1000;
				chatter_pub.publish(msg);
				sleep(10);
				l++; // gripper - operation
			}
			else if(l%2==0)
			{
				for(int i=0; i<100;i++)
				{
			  	  if(yaw > 0.05) 
			  	  {
			    		pstamp.angular.z = 0.1;
			    		cout<<"high"<<endl;
			  	  }
			
			  	  if(yaw < -0.05) 
			  	  {
			    		pstamp.angular.z = -0.1;
			    		cout<<"low"<<endl;
			  	  }
			  	  chatter_pub3.publish(pstamp);
			  
			  	  if(yaw<0.01&&yaw>-0.01) 
			  	  {	
					pstamp.angular.z = 0;
					chatter_pub3.publish(pstamp);
					cout<<"ByeBye!"<<endl;
					sleep(10);
					break;
			  	  }
				}
  		
				msg.data.resize(2);
  				msg.data[0]=3144;
  				msg.data[1]=3144;
				chatter_pub.publish(msg);
			  	l++;
			}
		}
		
		
		else
		{			  
			if(yaw>(angle-0.01) && yaw<(angle+0.01)) 
				{						
				pstamp.linear.x = 0;//0.3;
			  	pstamp.linear.y = 0;
			 	pstamp.linear.z = 0;
	
			  	pstamp.angular.x = 0;
			  	pstamp.angular.y = 0;
				pstamp.angular.z = 0;
				}
			if(yaw < angle) {pstamp.angular.z =-0;
			cout<<"bye"<<endl;}
			
			if(yaw > angle) {pstamp.angular.z =0;
			cout<<"hello"<<endl;}
			msg.data.resize(2);
  			msg.data[0]=0;
  			msg.data[1]=0;
			//로봇팔 퍼블리시. 전달하는 내용은 object 위치
			chatter_pub3.publish(pstamp);
			chatter_pub.publish(msg);
		 	
		  }
	}
		ros::spinOnce();
	
	j++;		
	int key = waitKey(5);
  	if (key == 27) break;
	}
}

#endif


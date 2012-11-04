/* tf_listener.cpp
Stephanie Cockrell
7/19/2012

subscribes to the robot-to-kinect transform
transforms the kinect data

outputs txt files:
tf_pts - After transform.
tf_pts_before - Before transform.

*/

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

// for the writing to file and stuff- to test the output
#include <iostream>
#include <fstream>

#include <sstream>

#include "std_msgs/String.h"

#include "/home/stephanie/drivers/nite/build/Nite-1.3.1.5/Include/XnVNite.h"

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

using namespace std;

bool savedPts=false;

//ofstream myfile_tf; 

void transformPoint(const tf::TransformListener& listener){
	//kinect stuff:
	XnStatus nRetVal = XN_STATUS_OK;
	xn::Context context;
	xn::DepthGenerator depth;
	// Initialization
	nRetVal = context.Init();
	//CHECK_RC(nRetVal, "Initialize context");
	nRetVal = depth.Create(context);
	//CHECK_RC(nRetVal, "Create Depth");
	
	// Tell the context object to start generating data
	nRetVal = context.StartGeneratingAll();
	//CHECK_RC(nRetVal, "Start Generating All Data");

	const XnDepthPixel* pDepthMap;
	XnPoint3D realWorld[XN_VGA_Y_RES*XN_VGA_X_RES];
	XnPoint3D pointList[XN_VGA_Y_RES*XN_VGA_X_RES];

	nRetVal = context.WaitOneUpdateAll(depth);
	//CHECK_RC(nRetVal, "Updating depth");
	// Get the new depth map
	pDepthMap = depth.GetDepthMap();

	for (int y=0; y<XN_VGA_Y_RES; y++){
		for(int x=0;x<XN_VGA_X_RES;x++){
		        pointList[y * XN_VGA_X_RES + x ].X =x;
		        pointList[y * XN_VGA_X_RES + x ].Y =y;
		        pointList[y * XN_VGA_X_RES + x ].Z = (short) pDepthMap[y *XN_VGA_X_RES + x];
		}
	} 

	depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

	double calib_x_start=XN_VGA_X_RES*1/4; // what proportion of the width you start at, for both angle bins and height bins
	double calib_x_end=XN_VGA_X_RES*3/4;
	double calib_y_start=XN_VGA_Y_RES*1/4;
	double calib_y_end = XN_VGA_Y_RES*1;

	ofstream myfile_tf; 
	myfile_tf.open ("tf_pts.txt");
	ofstream myfile_tf2; 
	myfile_tf2.open ("tf_pts_before.txt");

	for(int i=calib_x_start; i<calib_x_end; i++){
		for(int j=calib_y_start; j<calib_y_end; j++){
			laser_point.point.x=realWorld[j*XN_VGA_X_RES + i].Y; //Y;
			laser_point.point.y=-realWorld[j*XN_VGA_X_RES + i].X; //X; 
			//laser_point.point.x=realWorld[j*XN_VGA_X_RES + i].X;
			//laser_point.point.y=realWorld[j*XN_VGA_X_RES + i].Y;
			laser_point.point.z=realWorld[j*XN_VGA_X_RES + i].Z;
			  try{
			    geometry_msgs::PointStamped base_point;
			    listener.transformPoint("base_link", laser_point, base_point);
/*
			    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
				laser_point.point.x, laser_point.point.y, laser_point.point.z,
				base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
*/

				myfile_tf << base_point.point.x << ", ";
				myfile_tf << base_point.point.y << ", ";
				myfile_tf << base_point.point.z << "\n";
				myfile_tf2 << laser_point.point.x << ", ";
				myfile_tf2 << laser_point.point.y << ", ";
				myfile_tf2 << laser_point.point.z << "\n";
			  }
			  catch(tf::TransformException& ex){
			    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
			  }

		}
	}

	myfile_tf.close();
	myfile_tf2.close();



  //just an arbitrary point in space
  //laser_point.point.x = 1.0;
  //laser_point.point.y = 0.2;
  //laser_point.point.z = 0.0;

/*
  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
*/
	savedPts=true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));
  //tf::TransformListener listener();

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

	printf("okay everyone, we're right before the spin while loop\n");
	while(!savedPts){
		ros::spinOnce();
		printf("spun once\n");
	}
	printf("kay did the spin thing\n");
  //ros::spin();

}

/* tf_listener3.cpp
Stephanie Cockrell
7/19/2012

subscribes to the robot-to-kinect transform
transforms the kinect data
transforms a set of artificial pts

outputs 4 txt files:
tf_pts_data - for the real data. After transform.
tf_pts_data_before - for the real data. Before transform.
tf_pts_plane - for the artificial data. After transform.
tf_pts_plane_before - for the artificial data. Before transform.

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

	// this stuff is for the ideal best-fit pts
	ifstream plane_file("plane_params.txt");
	string A_str, B_str, C_str, D_str;
	getline(plane_file,A_str);
	getline(plane_file,B_str);
	getline(plane_file,C_str);
	getline(plane_file,D_str);
	double A=atof(A_str.c_str());
	double B=atof(B_str.c_str());
	double C=atof(C_str.c_str());
	double D=atof(D_str.c_str());
	plane_file.close();

	int n_pts=6;
	double y_list[]={-400, -400, -100, -100, 300, 300};
	double x_list[]={200, -200, 200, -200, 400, -400}; // x in kinect's ref frame
	double z_list[n_pts];
	for(int i=0;i<n_pts;i++){
		z_list[i]=(-D - A*x_list[i] - B*y_list[i])/C;
	}

  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();
/*
	double calib_x_start=XN_VGA_X_RES*3/8; // what proportion of the width you start at, for both angle bins and height bins
	double calib_x_end=XN_VGA_X_RES*5/8;
	double calib_y_start=XN_VGA_Y_RES*3/8;
	double calib_y_end = XN_VGA_Y_RES*1;
*/
	double calib_x_start=XN_VGA_X_RES*1/4; // what proportion of the width you start at, for both angle bins and height bins
	double calib_x_end=XN_VGA_X_RES*3/4;
	double calib_y_start=XN_VGA_Y_RES*1/4;
	double calib_y_end = XN_VGA_Y_RES*1;

	ofstream myfile_tf_1; 
	myfile_tf_1.open ("tf_pts_data.txt");
	ofstream myfile_tf2_1; 
	myfile_tf2_1.open ("tf_pts_data_before.txt");
	ofstream myfile_tf_2; 
	myfile_tf_2.open ("tf_pts_plane.txt");
	ofstream myfile_tf2_2; 
	myfile_tf2_2.open ("tf_pts_plane_before.txt");

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

				myfile_tf_1 << base_point.point.x << ", ";
				myfile_tf_1 << base_point.point.y << ", ";
				myfile_tf_1 << base_point.point.z << "\n";
				myfile_tf2_1 << laser_point.point.x << ", ";
				myfile_tf2_1 << laser_point.point.y << ", ";
				myfile_tf2_1 << laser_point.point.z << "\n";
			  }
			  catch(tf::TransformException& ex){
			    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
			  }

		}
	}
	for(int i=0; i<n_pts; i++){
		laser_point.point.x=y_list[i];
		laser_point.point.y=-x_list[i]; 
		laser_point.point.z=z_list[i];
		  try{
		    geometry_msgs::PointStamped base_point;
		    listener.transformPoint("base_link", laser_point, base_point);

			myfile_tf_2 << base_point.point.x << ", ";
			myfile_tf_2 << base_point.point.y << ", ";
			myfile_tf_2 << base_point.point.z << "\n";
			myfile_tf2_2 << laser_point.point.x << ", ";
			myfile_tf2_2 << laser_point.point.y << ", ";
			myfile_tf2_2 << laser_point.point.z << "\n";
		  }
		  catch(tf::TransformException& ex){
		    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
		  }

	}

	myfile_tf_1.close();
	myfile_tf2_1.close();
	myfile_tf_2.close();
	myfile_tf2_2.close();

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

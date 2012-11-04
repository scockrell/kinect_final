//#include "/home/stephanie/code/dev_stacks/kinect/NITE/Include/XnVNite.h"

// the following 3 things i'm adding because now i'm gonna run this in ros.  source: http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <math.h>

// for the writing to file and stuff- to test the output
#include <iostream>
#include <fstream>

// to do the ros msg i have this source http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Sensors
#include </opt/ros/electric/stacks/common_msgs/sensor_msgs/msg_gen/cpp/include/sensor_msgs/LaserScan.h>
//#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
//using namespace sensor_msgs;

using sensor_msgs::PointCloud2;

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
pcl::PointCloud<pcl::PointXYZ> cloud; // global variable

void xYToRTheta(double x, double y, double* result);
void rotate(double x, double y, double z, double theta, double* result);

bool savedPts=false;

//int _tmain(int argc, _TCHAR* argv[]){

void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
	printf("in cloudCallBack\n");
	pcl::fromROSMsg(*cloud_msg, cloud);
}
void kinectToLaserScan(const tf::TransformListener& listener){
	printf("in kinectToLaserScan\n");
	// to publish:
	//ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50); // maybe this should be in main?

	double laser_frequency = 40;
	ros::Rate r(15); //run at 15 Hz		

	int angleMax=90;
	double* distances=new double[2*angleMax];
	int* distances_index=new int[2*angleMax];
	double* distances_height=new double[2*angleMax];
	double unknownDist=-0.01; // units: m

	double* finalX=new double[2*angleMax];
	double* finalY=new double[2*angleMax];
	double* finalZ=new double[2*angleMax];
	double tempX, tempY, tempZ;

	//double* rotatedResult=new double[3];
	double* tempRTheta=new double[2];
	double floorThreshold=-.4; //-.27; //-950; //-950; // magical floor threshold: -950 yay!

	//int angleIndex;
	int angleIndex2;

	int real_x_dim=640;
	int real_y_dim=480;
	int x_res=real_x_dim;
	int y_res=real_y_dim; //clean this up- why do i have so many variables that are the same thing?	

	double minDist;
	int minIndexJ;
	double minX;

	ros::Time scan_time;
	sensor_msgs::LaserScan scan;

	ofstream myfile_tf; 
	myfile_tf.open ("tf_pts.txt");
	ofstream myfile_tf2; 
	myfile_tf2.open ("tf_pts_before.txt");

  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  //laser_point.header.frame_id = "base_laser";
  laser_point.header.frame_id = "camera_depth_frame";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

		// hey guys, x is horizontal, y is vertical, z is depth.

		for(int i=0; i<2*angleMax; i++){
			distances[i]=unknownDist;
			distances_index[i]=-1;
			distances_height[i]=0;
			//printf("distances[%d] = %f\n",i,distances[i]);
			finalX[i]=0;
			finalY[i]=0;
			finalZ[i]=0;
		}

		geometry_msgs::PointStamped base_point;

		for(int i=0; i<x_res; i++){ 
			minDist=3.0; // 3 m because hey, why not.
			for(int j=0; j<y_res; j++){
				laser_point.point.x=(cloud.at(i,j)).x; 
				laser_point.point.y=(cloud.at(i,j)).y; 
				laser_point.point.z=(cloud.at(i,j)).z;

				try{
					//geometry_msgs::PointStamped base_point;
					listener.transformPoint("base_link", laser_point, base_point);

					myfile_tf << base_point.point.x << ", ";
					myfile_tf << base_point.point.y << ", ";
					myfile_tf << base_point.point.z << "\n";
					myfile_tf2 << laser_point.point.x << ", ";
					myfile_tf2 << laser_point.point.y << ", ";
					myfile_tf2 << laser_point.point.z << "\n";

				}
				catch(tf::TransformException& ex){
					ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_frame\" to \"base_link\": %s", ex.what());
				}
				//printf("base_point x %f y %f z %f\n",base_point.point.x,base_point.point.y,base_point.point.z);

				//rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,-tiltAngle, rotatedResult);
				//if(rotatedResult[1]>floorThreshold && rotatedResult[2]<minDist && rotatedResult[2]>0){

				//printf("base_point.point.z %f > floorThreshold %f && base_point.point.y %f < minDist %f\n",base_point.point.z,floorThreshold,base_point.point.y,minDist);
				//printf("base_point.point.z %f < floorThreshold %f && abs(base_point.point.y) %f < minDist %f \n",base_point.point.z,floorThreshold,abs(base_point.point.y),minDist);


				// for some reason, z and y are defined weird so they are the negative of what you'd think. so, changing the ifs...
				if(base_point.point.z<floorThreshold && abs(base_point.point.y)<minDist){
				//if(base_point.point.z>floorThreshold && base_point.point.y<minDist && base_point.point.y>0){
					//minDist=base_point.point.y;
					minDist=abs(base_point.point.y);
					minIndexJ=j;
					minX=base_point.point.x;
					tempX=base_point.point.x;
					tempY=base_point.point.y;
					tempZ=base_point.point.z;
				}
			}


			//xYToRTheta(minDist,realWorld[minIndexJ*x_res + i].X, tempRTheta);
			//xYToRTheta(minDist,base_point.point.x, tempRTheta); // might need to switch x and y
			xYToRTheta(minDist,minX, tempRTheta); // might need to switch x and y
/*
			if(angleMax-(int)tempRTheta[1]<90){ 
				angleIndex2=90-angleMax+(int)tempRTheta[1];
			}
			else{
				angleIndex2=270-angleMax+(int)tempRTheta[1];
			}
*/

			//printf("minDist %f base_point.point.x %f tempRTheta[0] %f tempRTheta[1] %f\n",minDist, base_point.point.x, tempRTheta[0], tempRTheta[1]);

			//printf("%d %d %f\n",(int) tempRTheta[1], minDist,base_point.point.x);
			angleIndex2=(int)tempRTheta[1]+90;

			//printf("angleIndex2 %d ",angleIndex2);

			if(distances[angleIndex2]<0 || distances[angleIndex2]>tempRTheta[0]){
				distances[angleIndex2]=tempRTheta[0];
				distances_index[angleIndex2]=i;
				finalX[angleIndex2]=tempX;
				finalY[angleIndex2]=tempY;
				finalZ[angleIndex2]=tempZ;

			}			
		}

		myfile_tf.close();
		myfile_tf2.close();

		// write to file
		ofstream myfile;
		myfile.open ("lidar_result.txt");
		if(!myfile){
			printf("oh my goodness, the file didn't open\n");
		}
		for(int i=0;i<2*angleMax; i++){
			//myfile << "Writing this to a file.\n";
			myfile << distances[i] << ", " << distances_index[i] << ", " << finalX[i] << ", " << finalY[i] << ", " << finalZ[i] << endl;
			//printf("wrote to file, i = %d\n",i);
		}

		myfile.close();

		scan_time = ros::Time::now();

		//populate the LaserScan message
		//sensor_msgs::LaserScan scan; //declaring this BEFORE the while loop instead
		scan.header.stamp = scan_time;
		//scan.header.frame_id = "laser_frame";
		scan.header.frame_id = "/base_link";
		scan.angle_min = -1.57;
		scan.angle_max = 1.57;
		scan.angle_increment = 3.14 / (2*angleMax);
		scan.time_increment = (1 / laser_frequency) / (2*angleMax);
		scan.range_min = -1; //0.0;
		scan.range_max = 100.0;

		scan.set_ranges_size(2*angleMax);
		//scan.set_intensities_size(2*angleMax);
		for(unsigned int i = 0; i < 2*angleMax; ++i){
			//scan.ranges[i] = distances[i];
			scan.ranges[i] = distances[i]; 
			//scan.intensities[i] = intensities[i];
		}
		savedPts=true;

		scan_pub.publish(scan);
		r.sleep();
	//}


	// eh my attempt to free memory
	delete [] distances;
}
void xYToRTheta(double x, double y, double* result){
	result[0]=sqrt(x*x+y*y);
	result[1]=atan2(y,x)*180/3.14159;
}
void rotate(double x, double y, double z, double theta, double* result){
	// so for this case, it needs to be (x, depth, height)
	double thetaRad=theta*3.14159/180;
	result[0]=x;
	result[1]=cos(thetaRad)*y - sin(thetaRad)*z;
	result[2]=sin(thetaRad)*y + cos(thetaRad)*z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));
  //tf::TransformListener listener();

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&kinectToLaserScan, boost::ref(listener)));

	ros::Subscriber cloud_sub=n.subscribe("/camera/depth/points",100,cloudCallBack);

	printf("okay everyone, we're right before the spin while loop\n");
/*
	while(!savedPts){
		ros::spinOnce();
		printf("spun once\n");
	}
*/
	ros::spin();
	printf("kay did the spin thing\n");

	return 0;

}

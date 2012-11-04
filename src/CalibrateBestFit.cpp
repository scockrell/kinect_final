// possibly useful: http://www.ros.org/doc/api/nao_openni/html/namespacexn.html
// Include OpenNI
//#include </home/mobilerobots/kinect/OpenNI/Include/XnCppWrapper.h>
//#include </home/stephanie/code/dev_stacks/kinect/OpenNI/Include/XnCppWrapper.h>
// Include NITE
//#include "/home/mobilerobots/kinect/NITE/Nite-1.4.0.5/Include/XnVNite.h"
//#include "/home/stephanie/code/dev_stacks/kinect/NITE/Include/XnVNite.h"
//#include "/home/stephanie/drivers/nite/build/Nite-1.3.1.5/Include/XnVNite.h" // commenting this out, but this is the useful one

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

//#include "bestfit.cpp"
#include "bestfit2.h" // this is so messy

//to publish the transform once you get it
#include <tf/transform_broadcaster.h>

// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
*/

using namespace std;
//using namespace sensor_msgs;

using sensor_msgs::PointCloud2;

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

// Global variables here
ros::Publisher             cloud_pub_;
//image_transport::Publisher image_pub_;
//string window_name_;

/*
// This macro checks the return code that all OpenNI calls make
// and throws an error if the return code is an error. Use this
// after all calls to OpenNI objects. Great for debugging.
#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

#define _TCHAR char
#define _tmain main
*/

void xYToRTheta(double x, double y, double* result);
void rotate(double x, double y, double z, double theta, double* result);

//ros::Rate r(15); //run at 15 Hz	
//tf::TransformBroadcaster broadcaster;

bool transformCalculated=false;
double roll2, pitch2, yaw2;
double x_shift, y_shift, z_shift;

//int _tmain(int argc, _TCHAR* argv[]){
void allCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){

	// Convert the image from ROS format to PCL format
	//PointCloudXYZRGB cloud;
	//pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*cloud_msg, cloud);

/*
	//
	// Variables

	// Keep track of the return code of all OpenNI calls
	XnStatus nRetVal = XN_STATUS_OK;
	// context is the object that holds most of the things related to OpenNI
	xn::Context context;
	// The DepthGenerator generates a depth map that we can then use to do 
	// cool stuff with. Other interesting generators are gesture generators
	// and hand generators.
	xn::DepthGenerator depth;

	//
	// Initialization

	// Initialize context object
	nRetVal = context.Init();
	//CHECK_RC(nRetVal, "Initialize context");
	// Create the depth object
	nRetVal = depth.Create(context);
	//CHECK_RC(nRetVal, "Create Depth");

	// Tell the context object to start generating data
	nRetVal = context.StartGeneratingAll();
	//CHECK_RC(nRetVal, "Start Generating All Data");
*/

	// to publish:
	//ros::init(argc, argv, "kinect_tf_publisher");
	//ros::NodeHandle n;
	//ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
	//ros::Rate r(15); //run at 15 Hz	
	//tf::TransformBroadcaster broadcaster;

	double laser_frequency = 40;	

	int unknownDist=-2000; 

	//double* rotatedResult=new double[3];
	double floorThreshold=-950; //-950; // magical floor threshold: -950 yay!

	//int tiltAngle=-13; //-20; //60; //45; //15; // in degrees
	// magical angle: -13 yay!

	int total_iterations=10;
	int count_iterations=0;

	int real_x_dim=640;
	int real_y_dim=480;

	//int x_res=XN_VGA_X_RES;
	//int y_res=XN_VGA_Y_RES;
	int x_res=real_x_dim;
	int y_res=real_y_dim; //clean this up- why do i have so many variables that are the same thing?	

	int minDist;
	int minIndexJ;

	int realXMin=-4000;
	int realXMax=4000;
	int realZMin=0;
	int realZMax=3000;

	int finalXRes=100;
	int finalZRes=finalXRes*(realZMax-realZMin)/(realXMax-realXMin);

	double approxX;
	double approxY;
	double approxZ;
	//double approxXOld, approxYOld, approxZOld;

	int smoothConstant=100;
	double* yArray=new double[smoothConstant];
	double* zArray=new double[smoothConstant];

	printf("got there line 120\n");
	//double topDownMap[finalXRes][finalZRes]; //=new double[finalXRes][finalZRes];
	//printf("after topDownMap\n");
	double gradientMap[x_res][y_res-1];
	printf("after gradientMap\n");
	//double simpleMap1[finalXRes][finalZRes-1];
	//printf("after simpleMap\n");
	//printf("got ther eline 124\n");

	double calib_x_start=x_res*1/4; // what proportion of the width you start at, for both angle bins and height bins
	double calib_x_end=x_res*3/4;
	double calib_y_start=y_res*1/4;
	double calib_y_end = y_res*1;

	int angleBinsSum=0;
	double angleBinsMin=0;
	double angleBinsMax= 89;
	//int angleBinsCount=angleBinsMax-angleBinsMin;
	int angleBinsCount=100;
	int tempBinIndex;
	int wonkyCount;
	int noDataCount;
	int* angleBins=new int[angleBinsCount];
	double* binAvgs=new double[angleBinsCount];
	double tempAngle;

/*
	const XnDepthPixel* pDepthMap;
	printf("got there line 131\n");
	//XnPoint3D realWorld[real_x_dim*real_y_dim];
	printf("got ther eline 132\n");
	// source: https://groups.google.com/group/openni-dev/browse_thread/thread/e5aebba0852f8803?pli=1
	XnPoint3D pointList[XN_VGA_Y_RES*XN_VGA_X_RES];
	printf("got ther eline 134\n");
*/

	double allowedGradient=250; // dude i know 250 isn't right, i just picked something
	printf("got toehre line 128\n");

	ros::Time scan_time;
	sensor_msgs::LaserScan scan;

	printf("got ther eline 138\n");
	// Main loop
//	while(angleBinsSum==0){
		printf("got ther eline 139\n");
/*
		// Wait for new data to be available
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
*/

		for(int i=0;i<angleBinsCount;i++){
			angleBins[i]=0;
			binAvgs[i]=0;
		}
		wonkyCount=0;
		noDataCount=0;

		//depth.ConvertProjectiveToRealWorld(XN_VGA_Y_RES*XN_VGA_X_RES, pointList, realWorld); 

		// hey guys, x is horizontal, y is vertical, z is depth.
/*
		for(int i=0; i<finalXRes; i++){
			for(int j=0; j<finalZRes; j++){
				topDownMap[i][j]=unknownDist;
			}
		}
*/
		printf("got there line 238\n");
		for(int i=0; i<x_res; i++){ 
		// this loop calculates an angle at each pt and puts all the angles in bins
		//for(int i=x_res/4; i<3*x_res/4; i++){ 
			//printf("%d ",i);
			minDist=1000000;
			for(int i2=0; i2<smoothConstant; i2++){
				// initialize yArray and zArray
				// the purpose of yArray and zArray- store values of y and z which are smoothConstant units above the previous- 
				//for smoothing

				//printf("i2 = %d i = %d\n",i2,i);
				//yArray[i2]=realWorld[i2*x_res + i].Z;
				//zArray[i2]=realWorld[i2*x_res + i].Y; // and no one cares about x
				//if(i==0){printf("yArray1[%d] = %f zArray1[%d] = %f x1 = %f\n",i2,yArray[i2],i2,zArray[i2],realWorld[i2*x_res + i].X);}
				//if(i==0){printf("yArray1[%d] = %f zArray1[%d] = %f x1 = %f\n",i2,yArray[i2],i2,zArray[i2],realWorld[i2*x_res + i].X);}

				yArray[i2]=(cloud.at(i,i2)).y;
				zArray[i2]=(cloud.at(i,i2)).z; 
				//if(i==0){printf("yArray[%d] = %f zArray[%d] = %f x = %f\n",i2,yArray[i2],i2,zArray[i2],(cloud.at(i,i2)).x);}
			}
			for(int j=0;j<smoothConstant; j++){
				gradientMap[i][j]=0;
			}
			for(int j=smoothConstant; j<y_res; j++){
			//for(int j=y_res/4; j<3*y_res/4; j++){
/*
				approxX=realWorld[j*x_res + i].X;
				approxY=realWorld[j*x_res + i].Z;
				approxZ=realWorld[j*x_res + i].Y;
*/
				approxX=(cloud.at(i,j)).x;
				approxY=(cloud.at(i,j)).y;
				approxZ=(cloud.at(i,j)).z;

				tempAngle=atan2(yArray[0]-approxY,approxZ-zArray[0])*180.0/3.14159;
				//tempAngle=atan2(zArray[0]-approxZ,yArray[0]-approxY)*180.0/3.14159;
				//tempAngle=floor(atan2(approxZ-zArray[0],approxY-yArray[0])*180.0/3.14159+.5);
				//tempAngle=floor(atan2(approxY-approxYOld,approxZ-approxZOld)*180.0/3.14159+.5);
				if(tempAngle<0){
					tempAngle=tempAngle+180;
				}
				tempBinIndex=(int)(floor((tempAngle-angleBinsMin)/(angleBinsMax-angleBinsMin) * angleBinsCount));
				//printf("tempAngle = %f tempBinIndex = %d\n",tempAngle,tempBinIndex);

				if(approxY==0 || yArray[0]==0){
					noDataCount++;
				}
				else if(i>=calib_x_start && i<=calib_x_end && j>=calib_y_start && j<=calib_y_end){
				// totally not the most efficient way to program this but it works
					if(tempBinIndex>=0 && tempBinIndex<angleBinsCount){
						angleBins[tempBinIndex]++;
						binAvgs[tempBinIndex]=(tempAngle + (angleBins[tempBinIndex]-1)*binAvgs[tempBinIndex])/angleBins[tempBinIndex];
					}
					else{
						wonkyCount++;	
/*
						printf("%d %f ",tempBinIndex,tempAngle);
						if(wonkyCount%10 == 0){
							printf("\n");
						}	
*/		
					}
				}

				//gradientMap is now for the angles
				gradientMap[i][j]=tempAngle;

				for(int i2=0;i2<smoothConstant-1; i2++){
					yArray[i2]=yArray[i2+1];
					zArray[i2]=zArray[i2+1];
				}
				yArray[smoothConstant-1]=approxY;
				zArray[smoothConstant-1]=approxZ;
			}
	
		}
		printf("\nfreakin got there line 251\n");
/*
		ofstream myfile2; // this really should be declared outside of the while loop, but i'm only running it once so whatever
		myfile2.open ("anglemap.txt");
		for(int j=0; j<y_res-1; j++){
			for(int i=0;i<x_res; i++){
				myfile2 << gradientMap[i][j] << ", " ;
			}
			myfile2 << "\n" ;
		}
		myfile2.close();
		printf("gpot here line 262\n");
*/

		angleBinsSum=0;
		for(int i=0;i<angleBinsCount;i++){
			//printf("%d %d \n",i,angleBins[i]);
			printf("%f %d %f\n",i*(angleBinsMax-angleBinsMin)/angleBinsCount + angleBinsMin,angleBins[i],binAvgs[i]);
			angleBinsSum+=angleBins[i];
		}
		printf("\nangleBinsSum = %d\n%d x %d = %d\nwonkyCount = %d\nnoDataCount = %d\n",angleBinsSum,x_res,y_res,x_res*y_res,wonkyCount,noDataCount);

		count_iterations++;
		printf("count_iterations = %d\n",count_iterations);

		//r.sleep();
//	}

	if(angleBinsSum>0){
		// okay, now you have all the stuff in bins.  so now find the bin with the most
		int maxBinIndex=-1;
		double maxBinValue=-1;
		for(int i=0; i<angleBinsCount; i++){
			if(angleBins[i]>maxBinValue){
				maxBinValue=angleBins[i];
				maxBinIndex=i;
			}
		}
		double finalAngle;
		if(maxBinIndex>0 && maxBinIndex<angleBinsCount-1){
			finalAngle=(binAvgs[maxBinIndex-1]*angleBins[maxBinIndex-1] + binAvgs[maxBinIndex]*angleBins[maxBinIndex] + binAvgs[maxBinIndex+1]*angleBins[maxBinIndex+1])/(angleBins[maxBinIndex-1]+angleBins[maxBinIndex]+angleBins[maxBinIndex+1]);
		}
		else{ // screw it
			finalAngle=binAvgs[maxBinIndex];
		}

		// okay now rotate everything to get heights, put in bins, etc, find the height of kinect
		double* rotatedResult=new double[3];
		double heightBinsMin=0; //-2; //-2000; 
		//meaning, the height bin values start at heightBinsMin and go to heightBinsMax
		double heightBinsMax= 2; //0;
		int heightBinsCount=500;

		int* heightBins=new int[heightBinsCount];
		double* heightBinAvgs=new double[heightBinsCount];
		double tempHeight;

		for(int i=0;i<heightBinsCount;i++){
			heightBins[i]=0;
			heightBinAvgs[i]=0;
		}

		ofstream myfile2; 
		myfile2.open ("before_rotate.txt");
		ofstream myfile3; 
		myfile3.open ("after_rotate.txt");

		int heightOutOfRangeCount=0;
		for(int i=calib_x_start; i<calib_x_end; i++){
			for(int j=calib_y_start; j<calib_y_end; j++){
				//rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,finalAngle, rotatedResult);
				rotate((cloud.at(i,j)).x,(cloud.at(i,j)).y,(cloud.at(i,j)).z,90-finalAngle, rotatedResult);

				myfile2 << (cloud.at(i,j)).x << ", " << (cloud.at(i,j)).y << ", " << (cloud.at(i,j)).z << "\n";
				myfile3 << rotatedResult[0] << ", " << rotatedResult[1] << ", " << rotatedResult[2] << "\n";
				tempHeight=rotatedResult[2];
				tempBinIndex=(int)(floor((tempHeight-heightBinsMin)/(heightBinsMax-heightBinsMin) * heightBinsCount));

				if(rotatedResult[2]!=0){
					if(tempBinIndex>=0 && tempBinIndex<heightBinsCount){
						heightBins[tempBinIndex]++;
						heightBinAvgs[tempBinIndex]=(tempHeight + (heightBins[tempBinIndex]-1)*heightBinAvgs[tempBinIndex])/heightBins[tempBinIndex];
					}
					else{
						heightOutOfRangeCount++;
					}

				}
			}		
		}

		myfile2.close();
		myfile3.close();

		// so all the heights are in bins.  print them out

		for(int i=0;i<heightBinsCount;i++){
			printf("%f %d %f\n",i*(heightBinsMax-heightBinsMin)/heightBinsCount + heightBinsMin,heightBins[i],heightBinAvgs[i]);
		}
		printf("heightOutOfRangeCount = %d\n",heightOutOfRangeCount);


		// okay, now you have all the heights in bins.  so now find the bin with the most
		maxBinIndex=-1;
		maxBinValue=-1;
		for(int i=0; i<heightBinsCount; i++){
			//printf("%f %d %f\n",i*(heightBinsMax-heightBinsMin)/heightBinsCount + heightBinsMin,heightBins[i],heightBinAvgs[i]);
			if(heightBins[i]>maxBinValue){
				maxBinValue=heightBins[i];
				maxBinIndex=i;
			}
		}
		double finalHeight;
		int floorCount;
		if(maxBinIndex>0 && maxBinIndex<heightBinsCount-1){
			finalHeight=(heightBinAvgs[maxBinIndex-1]*heightBins[maxBinIndex-1] + heightBinAvgs[maxBinIndex]*heightBins[maxBinIndex] + heightBinAvgs[maxBinIndex+1]*heightBins[maxBinIndex+1])/(heightBins[maxBinIndex-1]+heightBins[maxBinIndex]+heightBins[maxBinIndex+1]);
			floorCount=heightBins[maxBinIndex-1]+heightBins[maxBinIndex]+heightBins[maxBinIndex+1];
		}
		else{ // screw it
			finalHeight=binAvgs[maxBinIndex];
			floorCount=heightBins[maxBinIndex]; // floorCount is how many points are in the plane of the floor
		}

		// now write-to-file all the pts in those 3 (height) bins
		ofstream myfile_planefit; 
		myfile_planefit.open ("planefitpts.txt");
		ofstream myfile_reject; 
		myfile_reject.open ("rejectpts.txt");

		printf("floorCount = %d\n",floorCount);
		printf("got there line 358 whatever man\n");
		//float planeFitData[floorCount*3]=new float[floorCount*3];
		float* planeFitData=new float[floorCount*3];
		int planeFitCounter=0;
		printf("got ther line 306 yah\n");

		for(int i=calib_x_start; i<calib_x_end; i++){
			for(int j=calib_y_start; j<calib_y_end; j++){
		//for(int i=0; i<x_res; i++){ 
		//	for(int j=0; j<y_res; j++){
				//rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,finalAngle, rotatedResult);
				rotate((cloud.at(i,j)).x,(cloud.at(i,j)).y,(cloud.at(i,j)).z,90-finalAngle, rotatedResult);
				tempHeight=rotatedResult[2];
				tempBinIndex=(int)(floor((tempHeight-heightBinsMin)/(heightBinsMax-heightBinsMin) * heightBinsCount));
				//printf("maxBinIndex = %d tempBinIndex = %d\n",maxBinIndex,tempBinIndex);

				if(abs(tempBinIndex-maxBinIndex)<=1){
					//printf("planeFitCounter = %d floorCount*3 = %d\n",planeFitCounter,floorCount*3);
	/*
					planeFitData[planeFitCounter]=realWorld[j*x_res + i].X;
					planeFitCounter++;
					planeFitData[planeFitCounter]=realWorld[j*x_res + i].Y;
					planeFitCounter++;
					planeFitData[planeFitCounter]=realWorld[j*x_res + i].Z;
					planeFitCounter++;

					myfile_planefit << realWorld[j*x_res + i].X << ", ";
					myfile_planefit << realWorld[j*x_res + i].Y << ", ";
					myfile_planefit << realWorld[j*x_res + i].Z << "\n";
	*/

					planeFitData[planeFitCounter]=(cloud.at(i,j)).x;
					planeFitCounter++;
					planeFitData[planeFitCounter]=(cloud.at(i,j)).y;
					planeFitCounter++;
					planeFitData[planeFitCounter]=(cloud.at(i,j)).z;
					planeFitCounter++;
				}
				else{
	/*
					myfile_reject << realWorld[j*x_res + i].X << ", ";
					myfile_reject << realWorld[j*x_res + i].Y << ", ";
					myfile_reject << realWorld[j*x_res + i].Z << "\n";
	*/
				}

			//	if(i>x_res/4 && i<3*x_res/4 && j>y_res/4){
	/*
					myfile_planefit << realWorld[j*x_res + i].X << ", ";
					myfile_planefit << realWorld[j*x_res + i].Y << ", ";
					myfile_planefit << realWorld[j*x_res + i].Z << "\n";
	*/
			//	}


			}		
		}
		printf("planeFitCounter = %d floorCount*3 = %d FINAL\n",planeFitCounter,floorCount*3);
		myfile_planefit.close();
		myfile_reject.close();
		float plane[4];
		getBestFitPlane(floorCount,planeFitData,sizeof(float)*3,0,0,plane);
		printf("Plane: %f %f %f %f\r\n", plane[0], plane[1], plane[2],plane[3] );
		double planeFitAngle=acos(abs(plane[1]))*180/3.14159;
		printf("planeFitAngle = %f\n",planeFitAngle);

		printf("finalAngle  = %f\n",finalAngle);
		printf("finalHeight = %f\n",finalHeight);

		ofstream plane_file; 
		plane_file.open ("plane_params.txt");
		plane_file << plane[0] << "\n";
		plane_file << plane[1] << "\n";
		plane_file << plane[2] << "\n";
		plane_file << plane[3] << "\n";
		plane_file.close();

		ofstream myfile_calibration; 
		myfile_calibration.open ("calibration.txt");

		myfile_calibration << finalAngle  << "\n";
		myfile_calibration << finalHeight << "\n";

		myfile_calibration.close();
	/*
		double pitch_radians=-asin(-plane[0]);
		double roll_radians=-asin(plane[1]/cos(pitch_radians));
		double roll_realityCheck=acos(plane[2]/cos(pitch_radians));
	*/
	/*
		double pitch_radians=asin(-plane[1]);
		double roll_radians=asin(-plane[0]/cos(pitch_radians));
		double roll_realityCheck=acos(plane[2]/cos(pitch_radians));
		printf("roll_realityCheck %f\n",roll_realityCheck*180/3.14159);
	*/
		double roll_radians=-asin(-plane[0]);
		double pitch_radians=asin(plane[1]/cos(roll_radians));
		double pitch_realityCheck=acos(plane[2]/cos(roll_radians));
		printf("pitch_realityCheck %f\n",pitch_realityCheck*180/3.14159);

		//double roll_radians=-asin(abs(plane[0]));
		//double roll_radians=-acos(plane[2]/sqrt(plane[0]*plane[0] + plane[2]*plane[2]));
		if(plane[0]*plane[2] < 0){
			printf("hey flipping the sign on roll\n");
			roll_radians=-roll_radians;
		}

		if (pitch_radians>1.57){
			printf("oh and pitch is in 2nd quadrant\n");
			pitch_radians=3.14159-pitch_radians;
		}
		double yaw_radians=0;

		printf("roll = %f\npitch = %f\nyaw = %f\n",roll_radians*180/3.14159,pitch_radians*180/3.14159,yaw_radians*180/3.14159);
	/*
		double x_shift=.1; // in m.  and i just guessed this.
		double y_shift=0;
		double z_shift=-.001*(plane[0]*x_shift*1000 + plane[3])/plane[2];
	*/
		x_shift=.1; // in m //100; // in mm
		y_shift=0;
		z_shift=-(plane[0]*x_shift - plane[3])/plane[2] + .3234;

		printf("x = %f\ny = %f\nz = %f\n",x_shift,y_shift,z_shift);

		if(plane[2] < 0){
			printf("hey changing the sign on all the plane params\n");
			plane[0]=-plane[0];
			plane[1]=-plane[1];
			plane[2]=-plane[2];
			plane[3]=-plane[3];
		}

		//double A=plane[0];
		//double B=plane[1];
		//double A=-plane[1]; // interesting that this is negative
		double A=plane[0]; // dude, no way this can be negative
		double B=plane[1];
		double C=plane[2];
		double D=plane[3];

		double x_axis[]={ sqrt(B*B + C*C) , -A*B/sqrt(B*B + C*C) , -A*C/sqrt(B*B + C*C) };
		double y_axis[]={ 0 , C/sqrt(B*B + C*C) , -B/sqrt(B*B + C*C) };
		double z_axis[]={ A , B , C };

		printf("x_axis %f , %f , %f\ny_axis %f , %f , %f\nz_axis %f , %f , %f\n",x_axis[0],x_axis[1],x_axis[2],y_axis[0],y_axis[1],y_axis[2],z_axis[0],z_axis[1],z_axis[2]);

		btMatrix3x3 rotationMatrix = btMatrix3x3(x_axis[0],x_axis[1],x_axis[2],y_axis[0],y_axis[1],y_axis[2],z_axis[0],z_axis[1],z_axis[2]);
		//btMatrix3x3 rotationMatrix = btMatrix3x3(x_axis[0],y_axis[0],z_axis[0],x_axis[1],y_axis[1],z_axis[1],x_axis[2],y_axis[2],z_axis[2]);

		//double roll2, pitch2, yaw2;
		rotationMatrix.getEulerYPR(yaw2,pitch2,roll2);
		printf("roll2 = %f\npitch2 = %f\nyaw2 = %f\n",roll2*180/3.14159,pitch2*180/3.14159,yaw2*180/3.14159);
		if(roll2<0){
			printf("flipped pitch, roll, yaw signs\n");
			roll2=-roll2;
			pitch2=-pitch2;
			yaw2=-yaw2;
		}

		transformCalculated=true;
	}

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

int main (int argc, char** argv)
{
	printf("okay starting\n");
  // Initialize ROS
	ros::init (argc, argv, "listener");

	ros::NodeHandle nh;
	printf("just declared the nodehandle\n");
	ros::Subscriber cloud_sub=nh.subscribe("/camera/depth/points",100,allCB);
	tf::TransformBroadcaster broadcaster;
	ros::Rate r(15); //run at 15 Hz	

	ros::start(); // hopefully this will take care of the ros::Time::now() error
	printf("just did ros::start()\n");
	
	while( !transformCalculated ){
  		ros::spinOnce();
		printf("spun once\n");
	}
	printf("okay everyone, we calculated the transform\n");

	// publish the transform
	//while( ros::ok() ){
	while(nh.ok()){
		//printf("got there line 550\n");
		//ros::start(); // this is such a hack, i am sure this doesn't belong here and will cause problems
		//printf("got there line 552\n");
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll2,pitch2,yaw2), tf::Vector3(x_shift, y_shift, z_shift)),ros::Time::now(),"base_link", "camera_link")); // derp
		//broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(pitch2,roll2,yaw2), tf::Vector3(x_shift, y_shift, z_shift)),ros::Time::now(),"base_link", "camera_link")); 
		// WHOA DON'T FORGET TO COME BACK AND FIX THIS. THIS IS HACKED TOGETHER- roll and pitch are switched
		r.sleep();
	}
	printf("kay done with that publishing business\n");

  
	return 0;
}

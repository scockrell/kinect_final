/*
CalibrateBestFit.cpp
by Stephanie Cockrell
November 5, 2012
This program subscribes to the Kinect point cloud, does some calculations with slope and best-fit planes, and publishes the transform between Kinect's and robot's reference frames.
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>

// for the writing to file
#include <iostream>
#include <fstream>

#include "bestfit2.h" // some code I didn't write, for fitting the best-fit plane

//to publish the transform once you get it
#include <tf/transform_broadcaster.h>

// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;

using sensor_msgs::PointCloud2;

ros::Publisher             cloud_pub_;

void rotate(double x, double y, double z, double theta, double* result);

bool transformCalculated=false;
double roll2, pitch2, yaw2;
double x_shift, y_shift, z_shift;

void allCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
	// Convert the image from ROS format to PCL format
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*cloud_msg, cloud);

	int real_x_dim=640;
	int real_y_dim=480;

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

	int smoothConstant=100;
	double* yArray=new double[smoothConstant];
	double* zArray=new double[smoothConstant];

	double gradientMap[x_res][y_res-1];

	double calib_x_start=x_res*1/4; // what proportion of the width you start at, for both angle bins and height bins
	double calib_x_end=x_res*3/4;
	double calib_y_start=y_res*1/4;
	double calib_y_end = y_res*1;

	int angleBinsSum=0;
/*
	double angleBinsMin=0;
	double angleBinsMax= 89;
	int angleBinsCount=100;
*/
	double angleBinsMin=.35; //these are for SLOPES now instead of angles
	double angleBinsMax= 2.75;
	int angleBinsCount=96;
	int tempBinIndex;
	int wonkyCount;
	int noDataCount;
	int* angleBins=new int[angleBinsCount];
	double* binAvgs=new double[angleBinsCount];
	double tempAngle;

	for(int i=0;i<angleBinsCount;i++){
		angleBins[i]=0;
		binAvgs[i]=0;
	}
	wonkyCount=0;
	noDataCount=0;

	ofstream myfile_allpts; 
	myfile_allpts.open ("allpts.txt");

	for(int i=0; i<x_res; i++){ 
		for(int j=0; j<y_res; j++){
			// write to file
			myfile_allpts << (cloud.at(i,j)).x << ", ";
			myfile_allpts << (cloud.at(i,j)).y << ", ";
			myfile_allpts << (cloud.at(i,j)).z << "\n";
		}
	// this loop calculates an angle at each pt and puts all the angles in bins
		minDist=1000000;
		for(int i2=0; i2<smoothConstant; i2++){
			// initialize yArray and zArray
			// the purpose of yArray and zArray- store values of y and z which are smoothConstant units above the previous- 
			//for smoothing

			yArray[i2]=(cloud.at(i,i2)).y;
			zArray[i2]=(cloud.at(i,i2)).z; 
		}
		for(int j=0;j<smoothConstant; j++){
			gradientMap[i][j]=0;
		}
		for(int j=smoothConstant; j<y_res; j++){
			approxX=(cloud.at(i,j)).x;
			approxY=(cloud.at(i,j)).y;
			approxZ=(cloud.at(i,j)).z;

			tempAngle=(yArray[0]-approxY)/(approxZ-zArray[0]);
/*
			tempAngle=atan2(yArray[0]-approxY,approxZ-zArray[0])*180.0/3.14159;
			if(tempAngle<0){
				tempAngle=tempAngle+180;
			}
*/
			tempBinIndex=(int)(floor((tempAngle-angleBinsMin)/(angleBinsMax-angleBinsMin) * angleBinsCount));

			if(approxY==0 || yArray[0]==0){
				noDataCount++;
			}
			else if(i>=calib_x_start && i<=calib_x_end && j>=calib_y_start && j<=calib_y_end){
				if(tempBinIndex>=0 && tempBinIndex<angleBinsCount){
					angleBins[tempBinIndex]++;
					binAvgs[tempBinIndex]=(tempAngle + (angleBins[tempBinIndex]-1)*binAvgs[tempBinIndex])/angleBins[tempBinIndex];
				}
				else{
					wonkyCount++;	
					// how many times is it not even in the range of angles we were expecting		
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

	myfile_allpts.close();

	angleBinsSum=0;
	for(int i=0;i<angleBinsCount;i++){
		angleBinsSum+=angleBins[i];
		printf("%f %d %f\n",i*(angleBinsMax-angleBinsMin)/angleBinsCount + angleBinsMin,angleBins[i],binAvgs[i]);
	}
	printf("angleBinsSum = %d\n%d x %d = %d\nwonkyCount = %d\nnoDataCount = %d\n",angleBinsSum,x_res,y_res,x_res*y_res,wonkyCount, noDataCount);

	if(angleBinsSum>0){ // if angleBinsSum==0 then the loop will run again. sometimes you don't get data the 1st time through
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
			finalAngle=atan( (binAvgs[maxBinIndex-1]*angleBins[maxBinIndex-1] + binAvgs[maxBinIndex]*angleBins[maxBinIndex] + binAvgs[maxBinIndex+1]*angleBins[maxBinIndex+1])/(angleBins[maxBinIndex-1]+angleBins[maxBinIndex]+angleBins[maxBinIndex+1]) )*180.0/3.14159;
/*
			finalAngle=(binAvgs[maxBinIndex-1]*angleBins[maxBinIndex-1] + binAvgs[maxBinIndex]*angleBins[maxBinIndex] + binAvgs[maxBinIndex+1]*angleBins[maxBinIndex+1])/(angleBins[maxBinIndex-1]+angleBins[maxBinIndex]+angleBins[maxBinIndex+1]);
*/
		}
		else{ // screw it
			//finalAngle=binAvgs[maxBinIndex];
			finalAngle=atan( binAvgs[maxBinIndex] )*180.0/3.14159;
		}

		// okay now rotate everything to get heights, put in bins, etc, find the height of kinect
		double* rotatedResult=new double[3];
		double heightBinsMin=0; 
		//meaning, the height bin values start at heightBinsMin and go to heightBinsMax
		double heightBinsMax= 2; 
		int heightBinsCount=500;

		int* heightBins=new int[heightBinsCount];
		double* heightBinAvgs=new double[heightBinsCount];
		double tempHeight;

		for(int i=0;i<heightBinsCount;i++){
			heightBins[i]=0;
			heightBinAvgs[i]=0;
		}
/*
		// uncomment these if you want to troubleshoot the point cloud itself.
		ofstream myfile2; 
		myfile2.open ("before_rotate.txt");
		ofstream myfile3; 
		myfile3.open ("after_rotate.txt");
*/
		int heightOutOfRangeCount=0;
		for(int i=calib_x_start; i<calib_x_end; i++){
			for(int j=calib_y_start; j<calib_y_end; j++){
				//rotate(realWorld[j*x_res + i].X,realWorld[j*x_res + i].Y,realWorld[j*x_res + i].Z,finalAngle, rotatedResult);
				rotate((cloud.at(i,j)).x,(cloud.at(i,j)).y,(cloud.at(i,j)).z,90-finalAngle, rotatedResult);
/*
				myfile2 << (cloud.at(i,j)).x << ", " << (cloud.at(i,j)).y << ", " << (cloud.at(i,j)).z << "\n";
				myfile3 << rotatedResult[0] << ", " << rotatedResult[1] << ", " << rotatedResult[2] << "\n";
*/
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
/*
		myfile2.close();
		myfile3.close();
*/

		// okay, now you have all the heights in bins.  so now find the bin with the most
		maxBinIndex=-1;
		maxBinValue=-1;
		for(int i=0; i<heightBinsCount; i++){
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

/*
		// now write-to-file all the pts in those 3 (height) bins
		ofstream myfile_planefit; 
		myfile_planefit.open ("planefitpts.txt");
		ofstream myfile_reject; 
		myfile_reject.open ("rejectpts.txt");
*/

		ofstream myfile_planefit; 
		myfile_planefit.open ("planefitpts.txt");

		printf("floorCount = %d\n",floorCount);
		float* planeFitData=new float[floorCount*3];
		int planeFitCounter=0;

		for(int i=calib_x_start; i<calib_x_end; i++){
			for(int j=calib_y_start; j<calib_y_end; j++){
				rotate((cloud.at(i,j)).x,(cloud.at(i,j)).y,(cloud.at(i,j)).z,90-finalAngle, rotatedResult);
				tempHeight=rotatedResult[2];
				tempBinIndex=(int)(floor((tempHeight-heightBinsMin)/(heightBinsMax-heightBinsMin) * heightBinsCount));

				if(abs(tempBinIndex-maxBinIndex)<=1){
					planeFitData[planeFitCounter]=(cloud.at(i,j)).x;
					planeFitCounter++;
					planeFitData[planeFitCounter]=(cloud.at(i,j)).y;
					planeFitCounter++;
					planeFitData[planeFitCounter]=(cloud.at(i,j)).z;
					planeFitCounter++;

					myfile_planefit << (cloud.at(i,j)).x << ", ";
					myfile_planefit << (cloud.at(i,j)).y << ", ";
					myfile_planefit << (cloud.at(i,j)).z << "\n";
				}
				else{
	/*
					//uncomment if you want the point cloud data
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
/*
		myfile_planefit.close();
		myfile_reject.close();
*/
		float plane[4];
		getBestFitPlane(floorCount,planeFitData,sizeof(float)*3,0,0,plane);
		printf("Plane: %f %f %f %f\r\n", plane[0], plane[1], plane[2],plane[3] );
		double planeFitAngle=acos(abs(plane[1]))*180/3.14159;
		printf("planeFitAngle = %f\n",planeFitAngle);

		printf("finalAngle  = %f\n",finalAngle);
		printf("finalHeight = %f\n",finalHeight);
/*
		// stores the plane parameters in files- not really necessary
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
*/
		// now calculate roll, pitch, yaw and set up the transform
		double roll_radians=-asin(-plane[0]);
		double pitch_radians=asin(plane[1]/cos(roll_radians));
		double pitch_realityCheck=acos(plane[2]/cos(roll_radians));
		printf("pitch_realityCheck %f\n",pitch_realityCheck*180/3.14159);

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

		x_shift=.1; // in m 
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

		double A=plane[0]; // dude, no way this can be negative
		double B=plane[1];
		double C=plane[2];
		double D=plane[3];

		double x_axis[]={ sqrt(B*B + C*C) , -A*B/sqrt(B*B + C*C) , -A*C/sqrt(B*B + C*C) };
		double y_axis[]={ 0 , C/sqrt(B*B + C*C) , -B/sqrt(B*B + C*C) };
		double z_axis[]={ A , B , C };

		printf("x_axis %f , %f , %f\ny_axis %f , %f , %f\nz_axis %f , %f , %f\n",x_axis[0],x_axis[1],x_axis[2],y_axis[0],y_axis[1],y_axis[2],z_axis[0],z_axis[1],z_axis[2]);

		btMatrix3x3 rotationMatrix = btMatrix3x3(x_axis[0],x_axis[1],x_axis[2],y_axis[0],y_axis[1],y_axis[2],z_axis[0],z_axis[1],z_axis[2]);

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

	while(nh.ok()){
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll2,pitch2,yaw2), tf::Vector3(x_shift, y_shift, z_shift)),ros::Time::now(),"base_link", "camera_link")); 
		r.sleep();
	}
	printf("kay done with that publishing business\n");

	return 0;
}

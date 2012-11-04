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

#include <ctime>

using namespace std;
//using namespace sensor_msgs;

using sensor_msgs::PointCloud2;

// Shorthand for our point cloud type
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
pcl::PointCloud<pcl::PointXYZ> cloud; // global variable

bool savedPts=false;

double realXMin=-2;
double realXMax=2;
double realYMin=-3;
double realYMax=0;

int finalXRes=400;
int finalYRes=finalXRes*(realYMax-realYMin)/(realXMax-realXMin);
double xStep=(realXMax-realXMin)/finalXRes; // the length of each pixel in m
double yStep=(realYMax-realYMin)/finalYRes;

double getX(int i);
double getY(int j);
/*
int[][] generateAvgMask(int i, int j);

float topDownMap1[finalXRes][finalYRes]; //orig height data
float topDownMap[finalXRes][finalYRes]; // after filling in tiny holes
float topDownMap2[finalXRes][finalYRes]; //after avg filter
int countMap[finalXRes][finalYRes];
float gradientMap1[finalXRes][finalYRes];
float gradientMap2[finalXRes][finalYRes];
float finalMap[finalXRes][finalYRes];
*/
void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
	printf("in cloudCallBack\n");
	pcl::fromROSMsg(*cloud_msg, cloud);
}
void processKinectData(const tf::TransformListener& listener){
	printf("in processKinectData\n");

	int real_x_dim=640;
	int real_y_dim=480;
	int x_res=real_x_dim;
	int y_res=real_y_dim; //clean this up- why do i have so many variables that are the same thing?	



	int approxX;
	int approxY;
	double approxZ;
/*
	double topDownMap1[finalXRes][finalYRes]; //orig height data
	double topDownMap[finalXRes][finalYRes]; // after filling in tiny holes
	double topDownMap2[finalXRes][finalYRes]; //after avg filter
	int countMap[finalXRes][finalYRes];
	double gradientMap1[finalXRes][finalYRes];
	double gradientMap2[finalXRes][finalYRes];
	double finalMap[finalXRes][finalYRes];
*/

	float topDownMap1[finalXRes][finalYRes]; //orig height data
	float topDownMap[finalXRes][finalYRes]; // after filling in tiny holes
	float topDownMap2[finalXRes][finalYRes]; //after avg filter
	int countMap[finalXRes][finalYRes];
	float gradientMap1[finalXRes][finalYRes];
	float gradientMap2[finalXRes][finalYRes];
	float finalMap[finalXRes][finalYRes];
	float finalMap2[finalXRes][finalYRes];

	ofstream myfile_height; 
	myfile_height.open ("map_height.txt");
	ofstream myfile_gradient1; 
	myfile_gradient1.open ("map_gradient1.txt");
	ofstream myfile_gradient2; 
	myfile_gradient2.open ("map_gradient2.txt");
	ofstream myfile_final; 
	myfile_final.open ("map_final.txt");
	ofstream myfile_final2; 
	myfile_final2.open ("map_final2.txt");

	//we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
	geometry_msgs::PointStamped laser_point;
	laser_point.header.frame_id = "camera_depth_frame";

	laser_point.header.stamp = ros::Time();

	geometry_msgs::PointStamped base_point;

	// start timer here
	clock_t startTime=clock();
/*
	double unknownDist=.1;
	double unknownGradient=-.05;
*/
	float unknownDist=.1;
	float unknownGradient=-.05;
	for(int i=0; i<finalXRes; i++){
		for(int j=0; j<finalYRes; j++){
			topDownMap1[i][j]=unknownDist;
			topDownMap[i][j]=unknownDist;
			topDownMap2[i][j]=unknownDist;
			countMap[i][j]=0;
			gradientMap1[i][j]=unknownGradient;
			gradientMap2[i][j]=unknownGradient;
			finalMap[i][j]=-2; // -2 means it hasn't been filled in yet
			finalMap2[i][j]=-2; // -2 means it hasn't been filled in yet
		}
	}

	for(int i=0; i<x_res; i++){ 
		for(int j=0; j<y_res; j++){
			laser_point.point.x=(cloud.at(i,j)).x; 
			laser_point.point.y=(cloud.at(i,j)).y; 
			laser_point.point.z=(cloud.at(i,j)).z;

			try{
				//geometry_msgs::PointStamped base_point;
				listener.transformPoint("base_link", laser_point, base_point);
/*
				myfile_height << base_point.point.x << ", ";
				myfile_height << base_point.point.y << ", ";
				myfile_height << base_point.point.z << "\n";
*/
/*
				myfile_tf2 << laser_point.point.x << ", ";
				myfile_tf2 << laser_point.point.y << ", ";
				myfile_tf2 << laser_point.point.z << "\n";
*/
			}
			catch(tf::TransformException& ex){
				ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_frame\" to \"base_link\": %s", ex.what());
			}
			approxX=(int)((base_point.point.x-realXMin)*finalXRes/(realXMax-realXMin));
			approxY=(int)((base_point.point.y-realYMin)*finalYRes/(realYMax-realYMin));
			approxZ=base_point.point.z;
			//if((i+j)%1000==0){printf("%d %d %d\n",approxX,approxY,approxZ);}
			if(approxX>=0 && approxX<=finalXRes && approxY>=0 && approxY<=finalYRes){
				if(topDownMap[approxX][approxY]>approxZ){
					topDownMap[approxX][approxY]=(countMap[approxX][approxY]*topDownMap[approxX][approxY] + approxZ)/(countMap[approxX][approxY] + 1);
				countMap[approxX][approxY]++;
				}
			}
		}			
	}
/*
	// now we throw the filter on it
	int filterLength=30;
	int filterLength_1=filterLength/2; // integer division!
	int filterLength_2=filterLength-filterLength_1;
	int filterCount=0;
	double filterSum=0;
	for (int i=0; i<finalXRes; i++){
		for(int j=0; j<finalYRes; j++){
			if(topDownMap1[i][j]!=unknownDist){ // only do this for pts with data!
				filterCount=0;
				filterSum=0;
				for(int j1=max(0,j-filterLength_1); j1<min(finalYRes,j+filterLength_2); j1++){
					if(topDownMap1[i][j1]!=unknownDist){
						filterCount++;
						filterSum+=topDownMap1[i][j1];
					}
				}
				topDownMap[i][j]=filterSum/filterCount; // and there's no way filterCount=0, because (i,j) DEFINITELY had data
			}
		}
	}
*/
	// now we fill in holes in the data, if they have data on all 4 sides or 2 opposite sides
	int fillInCount1=1;
	int fillInCount2=0;
	int fillInCount3=0;
	while(fillInCount1+fillInCount2+fillInCount3>0){
		fillInCount1=0;
		fillInCount2=0;
		fillInCount3=0;
		for(int i=1; i<finalXRes-1; i++){
			for(int j=1; j<finalYRes; j++){
				if(topDownMap[i][j]==unknownDist){
					if(topDownMap[i-1][j]!=unknownDist && topDownMap[i+1][j]!=unknownDist){
						if(topDownMap[i][j-1]!=unknownDist && topDownMap[i][j+1]!=unknownDist){
							topDownMap[i][j]=.25*(topDownMap[i-1][j]+topDownMap[i+1][j]+topDownMap[i][j-1]+topDownMap[i][j+1]);
							fillInCount1++;
						}
						else{
							topDownMap[i][j]=.5*(topDownMap[i-1][j]+topDownMap[i+1][j]);
							fillInCount2++;
						}
					}
					else if(topDownMap[i][j-1]!=unknownDist && topDownMap[i][j+1]!=unknownDist){
						topDownMap[i][j]=.5*(topDownMap[i][j-1]+topDownMap[i][j+1]);
						fillInCount3++;
					}
					else{
						finalMap[i][j]=-1; // -1 means unknown
						finalMap2[i][j]=-1; // -1 means unknown
					}
				}
			}
		}
		printf("fillInCount = %d %d %d\n",fillInCount1,fillInCount2,fillInCount3);
	}
/*
	double smoothM1=.02; //the number of m from each pt to the one you are averaging with
	double smoothM2=.3;
*/
	float smoothM1=.02; //the number of m from each pt to the one you are averaging with
	float smoothM2=.3;
	int smoothPixels1=(int)(round(smoothM1/xStep)); // I really hope xStep and yStep are the same
	int smoothPixels2=(int)(round(smoothM2/xStep)); 
	if(smoothPixels1<1){
		smoothPixels1=1;
	}
	if(smoothPixels2<1){
		smoothPixels2=1;
	}
	int smoothPixels1_2=smoothPixels1/2; // INTEGER DIVISION!!!!!!
	int smoothPixels1_1=smoothPixels1-smoothPixels1_2;
	int smoothPixels2_2=smoothPixels2/2; // INTEGER DIVISION!!!!!!
	int smoothPixels2_1=smoothPixels2-smoothPixels2_2;
	//printf("filterLength = %d\n",filterLength);
	printf("xStep = %f yStep = %f smoothM1 = %f smoothPixels1 = %d smoothM2 = %f smoothPixels2 = %d\n",xStep, yStep,smoothM1,smoothPixels1,smoothM2,smoothPixels2);
/*
	double heightDiffX1, heightDiffX2;
	double heightDiffY1, heightDiffY2;
*/
	float heightDiffX1, heightDiffX2;
	float heightDiffY1, heightDiffY2;

	//double thr1=.008; // separate floor from bumps
	//double thr2=.034; // separate bumps from obstacles
/*
	double thr1=.75;
	double thr2=1.8;
*/
	float thr1=.75;
	float thr2=1.8;
	for (int i=smoothPixels1_1; i<finalXRes-smoothPixels1_2;i++){
		for(int j=smoothPixels1_1;j<finalYRes-smoothPixels1_2;j++){
			if(finalMap[i][j]!=-1){ // don't do this for the pts with no data
				if(topDownMap[i+smoothPixels1_2][j]!=unknownDist && topDownMap[i-smoothPixels1_1][j]!=unknownDist && topDownMap[i][j+smoothPixels1_2]!=unknownDist && topDownMap[i][j-smoothPixels1_1]!=unknownDist){ // if all 4 adjacent cells have data
					heightDiffX1=abs(topDownMap[i][j]-topDownMap[i+smoothPixels1_2][j]);
					heightDiffX2=abs(topDownMap[i-smoothPixels1_1][j]-topDownMap[i][j]);
					heightDiffY1=abs(topDownMap[i][j]-topDownMap[i][j+smoothPixels1_2]);
					heightDiffY2=abs(topDownMap[i][j-smoothPixels1_1]-topDownMap[i][j]);
					// perhaps add something about pts adjacent to pts with no data

					//gradientMap1[i][j]=sqrt(heightDiffX2*heightDiffX2 + heightDiffY2*heightDiffY2);
					gradientMap1[i][j]= sqrt( (heightDiffY1+heightDiffY2)*(heightDiffY1+heightDiffY2) + (heightDiffX1+heightDiffX2)*(heightDiffX1+heightDiffX2))/smoothM1;
					//gradientMap1[i][j]= sqrt( (heightDiffY1+heightDiffY2)*(heightDiffY1+heightDiffY2) )/smoothM1;
					if(gradientMap1[i][j]>thr2){ // obstacle
						finalMap[i][j]=3;
						finalMap2[i][j]=3;
					}
					else if(gradientMap1[i][j]>thr1){ //bump
						finalMap[i][j]=2;
						finalMap2[i][j]=2;
					}
				}
				else{
					finalMap[i][j]=-1; // no data if not all 4 adj have data
					finalMap2[i][j]=-1; // no data if not all 4 adj have data
				}
			}
		}
	}
	// perhaps right here we put something about tiny noise bits that got classified as a bump?
/*
	for (int i=smoothPixels2_1; i<finalXRes-smoothPixels2_2;i++){
		for(int j=smoothPixels2_1;j<finalYRes-smoothPixels2_2;j++){
			heightDiffX1=abs(topDownMap[i][j]-topDownMap[i+smoothPixels2_2][j]);
			heightDiffX2=abs(topDownMap[i-smoothPixels2_1][j]-topDownMap[i][j]);
			heightDiffY1=abs(topDownMap[i][j]-topDownMap[i][j+smoothPixels2_2]);
			heightDiffY2=abs(topDownMap[i][j-smoothPixels2_1]-topDownMap[i][j]);
			//gradientMap1[i][j]=sqrt(heightDiffX*heightDiffX + heightDiffY*heightDiffY)/smoothM1;
			//gradientMap2[i][j]= sqrt( (heightDiffY1+heightDiffY2)*(heightDiffY1+heightDiffY2) + (heightDiffX1+heightDiffX2)*(heightDiffX1+heightDiffX2))/smoothM2;
			gradientMap2[i][j]= sqrt( (heightDiffY1+heightDiffY2)*(heightDiffY1+heightDiffY2) )/smoothM2;
		}
	}
*/
	clock_t timeBeforeAvgMask=clock();
	int avgMask[smoothPixels2][smoothPixels2];
	int avgMaskCount=0;
	int avgMaskChanges=0;
	int i2,j2,jmax1,jmax2;
	bool generateAvgMask=true;
	// this will populate topDownMap2:
	for(int i=0;i<finalXRes;i++){
	//for(int i=finalXRes-1;i>=0;i--){
		//for(int j=0;j<finalYRes;j++){
		for(int j=finalYRes-1;j>=0;j--){
			if(finalMap[i][j]==-2){ // if it hasn't been classified yet- so it must be floor or ramp
				
				if(generateAvgMask){
					for(int i1=0;i1<smoothPixels2;i1++){
						for(int j1=0;j1<smoothPixels2;j1++){ // initialize the avg mask
							avgMask[i1][j1]=0;
						}
					}
					avgMask[smoothPixels2_1][smoothPixels2_1]=1; // initialize so the pt in the middle is 1
					avgMaskCount=1;
					avgMaskChanges=1;

					// okay now i am going to populate the averaging mask- expand from the center
					while(avgMaskChanges>0){
						avgMaskChanges=0;
						for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
							for(int j1=max(0,j-smoothPixels2_1);j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
								i2=i1-i+smoothPixels2_1;
								j2=j1-j+smoothPixels2_1;
								if(avgMask[i2][j2]==1){ // now set its 4 (ish) neighbors to 1
									if(i2>=1){
										if(avgMask[i2-1][j2]==0 && finalMap[i1-1][j1]==-2){
											avgMask[i2-1][j2]=1;
											avgMaskChanges++;
										}
									}
									if(i2<=smoothPixels2 - 2){
										if(avgMask[i2+1][j2]==0 && finalMap[i1+1][j1]==-2){
											avgMask[i2+1][j2]=1;
											avgMaskChanges++;
										}
									}
									if(j2>=1){
										if(avgMask[i2][j2-1]==0 && finalMap[i1][j1-1]==-2){
											avgMask[i2][j2-1]=1;
											avgMaskChanges++;
										}
									}
									if(j2<=smoothPixels2 - 2){
										if(avgMask[i2][j2+1]==0 && finalMap[i1][j1+1]==-2){
											avgMask[i2][j2+1]=1;
											avgMaskChanges++;
										}
									}
								}
							}
						}
						avgMaskCount+=avgMaskChanges;
						generateAvgMask=false;
					}//end of while loop to populate avg mask
				}
/*
				else{ // if you don't need to generate the entire thing
					jmax1=min(finalYRes,j+smoothPixels2_2)-1;
					jmax2=jmax1-j+smoothPixels2_1;
					for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
						i2=i1-i+smoothPixels2_1;
						if(avgMask[i2][max(0,j-smoothPixels2_1) -j+smoothPixels2_1]==1){
						//if(avgMask[i2][0]==1){ //updating avgMaskCount for removal of 1st col
							avgMaskCount--;
						}
						for(int j1=max(0,j-smoothPixels2_1)+1;j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
						//for(int j1=1;j1<smoothPixels2;j1++){ 
							//first shift over everything by 1 column 
							j2=j1-j+smoothPixels2_1;
							avgMask[i2][j2-1]=avgMask[i2][j2];
							//avgMask[i2][j1-1]=avgMask[i2][j1];
						}
					}
					// now to figure out avgMask[i2][smoothPixels2-1] (the last column)
					//if(finalYRes>j+smoothPixels2_2){ //if we're not going off the edge of the map
					if(jmax2==smoothPixels2-1){ //if we're not going off the edge of the map
						for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
							i2=i1-i+smoothPixels2_1;
							// now to figure out avgMask[i2][jmax2] (the last column)
							if(finalMap[i1][jmax1-1]==-2 && finalMap[i1][jmax1]==-2){
								//cell in new column and cell in adj col are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2][jmax2-1];
								//avgMask[i2][smoothPixels2-1]=avgMask[i2][smoothPixels2-2];
							}
							else if(finalMap[i1][jmax1]!=-2){
								//new cell is obstacle/bump
								avgMask[i2][jmax2]=0;
								//avgMask[i2][smoothPixels2-1]=0;
							}
							else if(i1==max(0,i-smoothPixels2_1)){
								//if it's the absolute top corner
								avgMask[i2][jmax2]=-1;
								//avgMask[i2][smoothPixels2-1]=-1;
							}
							else if(finalMap[i1-1][jmax1]==-2 && finalMap[i1][jmax1]==-2){
								//if new cell and the one above are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2-1][jmax2];
								//avgMask[i2][smoothPixels2-1]=avgMask[i2-1][smoothPixels2-1];
							}
							else if(i1==min(finalXRes,i+smoothPixels2_2)-1){
								// if it's the one in the absolute bottom corner and we haven't figured it out yet
								avgMask[i2][jmax2]=0;
								//avgMask[i2][smoothPixels2-1]=0;
							}
							else{
								avgMask[i2][jmax2]=-1; //unknown for now
								//avgMask[i2][smoothPixels2-1]=-1; //unknown for now
							}
							if(avgMask[i2][jmax2]==1){
							//if(avgMask[i2][smoothPixels2-1]==1){
								avgMaskCount++;
							}
						}
						for(int i1=min(finalXRes,i+smoothPixels2_2)-2;i1>=max(0,i-smoothPixels2_1);i1--){
							i2=i1-i+smoothPixels2_1;
							if(avgMask[i2][jmax2]==-1){
							//if(avgMask[i2][smoothPixels2-1]==-1){
								if(finalMap[i1+1][jmax1]==-2 && finalMap[i1][jmax1]==-2){
									//new cell and the one below are both floor/ramp
									avgMask[i2][jmax2]=avgMask[i2+1][jmax2];
									//avgMask[i2][smoothPixels2-1]=avgMask[i2+1][smoothPixels2-1];
									if(avgMask[i2][jmax2]==1){
									//if(avgMask[i2][smoothPixels2-1]==1){
										avgMaskCount++;
									}
								}
								else{
									avgMask[i2][jmax2]=0;
									//avgMask[i2][smoothPixels2-1]=0;
								}
							}
						}
					}

				}
*/



				else{ // if you don't need to generate the entire thing
					jmax1=max(0,j-smoothPixels2_1);
					jmax2=jmax1-j+smoothPixels2_1;
					//printf("jmax2 = %d\n",jmax2);
					for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
						i2=i1-i+smoothPixels2_1;
						if(avgMask[i2][smoothPixels2-1]==1){
						//if(avgMask[i2][0]==1){ //updating avgMaskCount for removal of 1st col
							avgMaskCount--;
						}
						for(int j1=smoothPixels2-2;j1>=0;j1--){ 
						//for(int j1=1;j1<smoothPixels2;j1++){ 
							//first shift over everything by 1 column 
							//j2=j1-j+smoothPixels2_1;
							avgMask[i2][j1+1]=avgMask[i2][j1];
							//avgMask[i2][j1-1]=avgMask[i2][j1];
						}
					}
					// now to figure out avgMask[i2][smoothPixels2-1] (the last column)
					//if(finalYRes>j+smoothPixels2_2){ //if we're not going off the edge of the map
					if(jmax2==0){ //if we're not going off the edge of the map
						for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
							i2=i1-i+smoothPixels2_1;
							// now to figure out avgMask[i2][jmax2] (the last column)
							if(finalMap[i1][jmax1+1]==-2 && finalMap[i1][jmax1]==-2){
								//cell in new column and cell in adj col are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2][jmax2+1];
								//avgMask[i2][smoothPixels2-1]=avgMask[i2][smoothPixels2-2];
							}
							else if(finalMap[i1][jmax1]!=-2){
								//new cell is obstacle/bump
								avgMask[i2][jmax2]=0;
								//avgMask[i2][smoothPixels2-1]=0;
							}
							else if(i1==max(0,i-smoothPixels2_1)){
								//if it's the absolute top corner
								avgMask[i2][jmax2]=-1;
								//avgMask[i2][smoothPixels2-1]=-1;
							}
							else if(finalMap[i1-1][jmax1]==-2 && finalMap[i1][jmax1]==-2){
								//if new cell and the one above are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2-1][jmax2];
								//avgMask[i2][smoothPixels2-1]=avgMask[i2-1][smoothPixels2-1];
							}
							else if(i1==min(finalXRes,i+smoothPixels2_2)-1){
								// if it's the one in the absolute bottom corner and we haven't figured it out yet
								avgMask[i2][jmax2]=0;
								//avgMask[i2][smoothPixels2-1]=0;
							}
							else{
								avgMask[i2][jmax2]=-1; //unknown for now
								//avgMask[i2][smoothPixels2-1]=-1; //unknown for now
							}
							if(avgMask[i2][jmax2]==1){
							//if(avgMask[i2][smoothPixels2-1]==1){
								avgMaskCount++;
							}
						}
						for(int i1=min(finalXRes,i+smoothPixels2_2)-2;i1>=max(0,i-smoothPixels2_1);i1--){
							i2=i1-i+smoothPixels2_1;
							if(avgMask[i2][jmax2]==-1){
							//if(avgMask[i2][smoothPixels2-1]==-1){
								if(finalMap[i1+1][jmax1]==-2 && finalMap[i1][jmax1]==-2){
									//new cell and the one below are both floor/ramp
									avgMask[i2][jmax2]=avgMask[i2+1][jmax2];
									//avgMask[i2][smoothPixels2-1]=avgMask[i2+1][smoothPixels2-1];
									if(avgMask[i2][jmax2]==1){
									//if(avgMask[i2][smoothPixels2-1]==1){
										avgMaskCount++;
									}
								}
								else{
									avgMask[i2][jmax2]=0;
									//avgMask[i2][smoothPixels2-1]=0;
								}
							}
						}
					}
				}


				topDownMap2[i][j]=0;
				for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
					for(int j1=max(0,j-smoothPixels2_1);j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
						i2=i1-i+smoothPixels2_1;
						j2=j1-j+smoothPixels2_1;
						topDownMap2[i][j]+=topDownMap[i1][j1]*avgMask[i2][j2];
					}
				}
				topDownMap2[i][j]=topDownMap2[i][j]/avgMaskCount;
			}
			else{
				generateAvgMask=true;
			}
		}
	}

	double thr3=.000425;
	// now subtract to find the difference between floor and ramp
	for (int i=1; i<finalXRes;i++){
		for(int j=1;j<finalYRes;j++){
			if(finalMap[i][j]==-2){
				heightDiffX1=abs(topDownMap2[i][j]-topDownMap2[i-1][j]);
				heightDiffY1=abs(topDownMap2[i][j]-topDownMap2[i][j-1]);
				gradientMap2[i][j]=sqrt(heightDiffX1*heightDiffX1 + heightDiffY1*heightDiffY1);
				//gradientMap2[i][j]= sqrt( (heightDiffY1+heightDiffY2)*(heightDiffY1+heightDiffY2) )/smoothM2;
				if(gradientMap2[i][j]>=thr3){
					finalMap[i][j]=1; // ramp
				}
				else{
					finalMap[i][j]=0; // floor
				}
			}
		}
	}



	generateAvgMask=true;


	for(int i=0;i<finalXRes;i++){
		for(int j=0;j<finalYRes;j++){
			topDownMap2[i][j]=unknownDist;
			if(finalMap2[i][j]==-2){ // if it hasn't been classified yet- so it must be floor or ramp
				if(generateAvgMask){
					for(int i1=0;i1<smoothPixels2;i1++){
						for(int j1=0;j1<smoothPixels2;j1++){ // initialize the avg mask
							avgMask[i1][j1]=0;
						}
					}
					avgMask[smoothPixels2_1][smoothPixels2_1]=1; // initialize so the pt in the middle is 1
					avgMaskCount=1;
					avgMaskChanges=1;

					// okay now i am going to populate the averaging mask- expand from the center
					while(avgMaskChanges>0){
						avgMaskChanges=0;
						for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
							for(int j1=max(0,j-smoothPixels2_1);j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
								i2=i1-i+smoothPixels2_1;
								j2=j1-j+smoothPixels2_1;
								if(avgMask[i2][j2]==1){ // now set its 4 (ish) neighbors to 1
									if(i2>=1){
										if(avgMask[i2-1][j2]==0 && finalMap2[i1-1][j1]==-2){
											avgMask[i2-1][j2]=1;
											avgMaskChanges++;
										}
									}
									if(i2<=smoothPixels2 - 2){
										if(avgMask[i2+1][j2]==0 && finalMap2[i1+1][j1]==-2){
											avgMask[i2+1][j2]=1;
											avgMaskChanges++;
										}
									}
									if(j2>=1){
										if(avgMask[i2][j2-1]==0 && finalMap2[i1][j1-1]==-2){
											avgMask[i2][j2-1]=1;
											avgMaskChanges++;
										}
									}
									if(j2<=smoothPixels2 - 2){
										if(avgMask[i2][j2+1]==0 && finalMap2[i1][j1+1]==-2){
											avgMask[i2][j2+1]=1;
											avgMaskChanges++;
										}
									}
								}
							}
						}
						avgMaskCount+=avgMaskChanges;
						generateAvgMask=false;
					}//end of while loop to populate avg mask
				}

				else{ // if you don't need to generate the entire thing
					jmax1=min(finalYRes,j+smoothPixels2_2)-1;
					jmax2=jmax1-j+smoothPixels2_1;
					for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
						i2=i1-i+smoothPixels2_1;
						if(avgMask[i2][max(0,j-smoothPixels2_1) -j+smoothPixels2_1]==1){
						//if(avgMask[i2][0]==1){ //updating avgMaskCount for removal of 1st col
							avgMaskCount--;
						}
						for(int j1=max(0,j-smoothPixels2_1)+1;j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
						//for(int j1=1;j1<smoothPixels2;j1++){ 
							//first shift over everything by 1 column 
							j2=j1-j+smoothPixels2_1;
							avgMask[i2][j2-1]=avgMask[i2][j2];
							//avgMask[i2][j1-1]=avgMask[i2][j1];
						}
					}
					// now to figure out avgMask[i2][smoothPixels2-1] (the last column)
					//if(finalYRes>j+smoothPixels2_2){ //if we're not going off the edge of the map
					if(jmax2==smoothPixels2-1){ //if we're not going off the edge of the map
						for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
							i2=i1-i+smoothPixels2_1;
							// now to figure out avgMask[i2][jmax2] (the last column)
							if(finalMap2[i1][jmax1-1]==-2 && finalMap2[i1][jmax1]==-2){
								//cell in new column and cell in adj col are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2][jmax2-1];
								//avgMask[i2][smoothPixels2-1]=avgMask[i2][smoothPixels2-2];
							}
							else if(finalMap2[i1][jmax1]!=-2){
								//new cell is obstacle/bump
								avgMask[i2][jmax2]=0;
								//avgMask[i2][smoothPixels2-1]=0;
							}
							else if(i1==max(0,i-smoothPixels2_1)){
								//if it's the absolute top corner
								avgMask[i2][jmax2]=-1;
								//avgMask[i2][smoothPixels2-1]=-1;
							}
							else if(finalMap2[i1-1][jmax1]==-2 && finalMap2[i1][jmax1]==-2){
								//if new cell and the one above are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2-1][jmax2];
								//avgMask[i2][smoothPixels2-1]=avgMask[i2-1][smoothPixels2-1];
							}
							else if(i1==min(finalXRes,i+smoothPixels2_2)-1){
								// if it's the one in the absolute bottom corner and we haven't figured it out yet
								avgMask[i2][jmax2]=0;
								//avgMask[i2][smoothPixels2-1]=0;
							}
							else{
								avgMask[i2][jmax2]=-1; //unknown for now
								//avgMask[i2][smoothPixels2-1]=-1; //unknown for now
							}
							if(avgMask[i2][jmax2]==1){
							//if(avgMask[i2][smoothPixels2-1]==1){
								avgMaskCount++;
							}
						}
						for(int i1=min(finalXRes,i+smoothPixels2_2)-2;i1>=max(0,i-smoothPixels2_1);i1--){
							i2=i1-i+smoothPixels2_1;
							if(avgMask[i2][jmax2]==-1){
							//if(avgMask[i2][smoothPixels2-1]==-1){
								if(finalMap2[i1+1][jmax1]==-2 && finalMap2[i1][jmax1]==-2){
									//new cell and the one below are both floor/ramp
									avgMask[i2][jmax2]=avgMask[i2+1][jmax2];
									//avgMask[i2][smoothPixels2-1]=avgMask[i2+1][smoothPixels2-1];
									if(avgMask[i2][jmax2]==1){
									//if(avgMask[i2][smoothPixels2-1]==1){
										avgMaskCount++;
									}
								}
								else{
									avgMask[i2][jmax2]=0;
									//avgMask[i2][smoothPixels2-1]=0;
								}
							}
						}
					}

				}
			
				topDownMap2[i][j]=0;
				for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
					for(int j1=max(0,j-smoothPixels2_1);j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
						i2=i1-i+smoothPixels2_1;
						j2=j1-j+smoothPixels2_1;
						topDownMap2[i][j]+=topDownMap[i1][j1]*avgMask[i2][j2];
					}
				}
				topDownMap2[i][j]=topDownMap2[i][j]/avgMaskCount;
			}
			else{
				generateAvgMask=true;
			}
		}
	}
	double avgMaskTime=(std::clock() - timeBeforeAvgMask)/(double)CLOCKS_PER_SEC;
	printf("avgMaskTime = %f sec\n",avgMaskTime);

	//double thr3=.000425;
	// now subtract to find the difference between floor and ramp
	for (int i=1; i<finalXRes;i++){
		for(int j=1;j<finalYRes;j++){
			if(finalMap2[i][j]==-2){
				heightDiffX1=abs(topDownMap2[i][j]-topDownMap2[i-1][j]);
				heightDiffY1=abs(topDownMap2[i][j]-topDownMap2[i][j-1]);
				gradientMap2[i][j]=sqrt(heightDiffX1*heightDiffX1 + heightDiffY1*heightDiffY1);
				//gradientMap2[i][j]= sqrt( (heightDiffY1+heightDiffY2)*(heightDiffY1+heightDiffY2) )/smoothM2;
				if(gradientMap2[i][j]>=thr3){
					finalMap2[i][j]=1; // ramp
				}
				else{
					finalMap2[i][j]=0; // floor
				}
			}
		}
	}

	// end timer here
	double calculationTime=(std::clock() - startTime)/(double)CLOCKS_PER_SEC;
	printf("calculationTime = %f sec\n",calculationTime);

	for(int i=0;i<finalXRes; i++){
		for(int j=0; j<finalYRes; j++){
			myfile_height << topDownMap[i][j] << ", " ;
			myfile_gradient1 << gradientMap1[i][j] << ", " ;
			myfile_gradient2 << gradientMap2[i][j] << ", " ;
			myfile_final << finalMap[i][j] << ", " ;
			myfile_final2 << finalMap2[i][j] << ", " ;
		}
		myfile_height << "\n" ;
		myfile_gradient1 << "\n" ;
		myfile_gradient2 << "\n" ;
		myfile_final << "\n" ;
		myfile_final2 << "\n" ;
	}

	myfile_height.close();
	myfile_gradient1.close();
	myfile_gradient2.close();
	myfile_final.close();
	myfile_final2.close();

	savedPts=true;

}
double getX(int i){
	//gets the x dimension, in mm, for pixel (i,j)
	return i*(realXMax-realXMin)/finalXRes + realXMin;
}
double getY(int j){
	//gets the y dimension, in mm, for pixel (i,j)
	return j*(realYMax-realYMin)/finalYRes + realYMin;
}
/*
int[][] generateAvgMask(int i, int j){
	int smoothPixels2=30;
	int avgMask[smoothPixels2][smoothPixels2];

	int avgMaskCount=0;
	int avgMaskChanges=0;
	int i2,j2;

	for(int i1=0;i1<smoothPixels2;i1++){
		for(int j1=0;j1<smoothPixels2;j1++){ // initialize the avg mask
			avgMask[i1][j1]=0;
		}
	}
	avgMask[smoothPixels2_1][smoothPixels2_1]=1; // initialize so the pt in the middle is 1
	avgMaskCount=1;
	avgMaskChanges=1;

	// okay now i am going to populate the averaging mask- expand from the center
	while(avgMaskChanges>0){
		avgMaskChanges=0;
		for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
			for(int j1=max(0,j-smoothPixels2_1);j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
				i2=i1-i+smoothPixels2_1;
				j2=j1-j+smoothPixels2_1;
				if(avgMask[i2][j2]==1){ // now set its 4 (ish) neighbors to 1
					if(i2>=1){
						if(avgMask[i2-1][j2]==0 && finalMap[i1-1][j1]==-2){
							avgMask[i2-1][j2]=1;
							avgMaskChanges++;
						}
					}
					if(i2<=smoothPixels2 - 2){
						if(avgMask[i2+1][j2]==0 && finalMap[i1+1][j1]==-2){
							avgMask[i2+1][j2]=1;
							avgMaskChanges++;
						}
					}
					if(j2>=1){
						if(avgMask[i2][j2-1]==0 && finalMap[i1][j1-1]==-2){
							avgMask[i2][j2-1]=1;
							avgMaskChanges++;
						}
					}
					if(j2<=smoothPixels2 - 2){
						if(avgMask[i2][j2+1]==0 && finalMap[i1][j1+1]==-2){
							avgMask[i2][j2+1]=1;
							avgMaskChanges++;
						}
					}
				}
			}
		}
		avgMaskCount+=avgMaskChanges;
	}//end of while loop to populate avg mask

	return avgMask;
}
*/
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));
  //tf::TransformListener listener();

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&processKinectData, boost::ref(listener)));

	ros::Subscriber cloud_sub=n.subscribe("/camera/depth/points",100,cloudCallBack);

	printf("okay everyone, we're right before the spin while loop\n");

	while(!savedPts){
		ros::spinOnce();
		printf("spun once\n");
	}

	//ros::spin();
	printf("kay did the spin thing\n");

	return 0;

}

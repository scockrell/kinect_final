/*
FinalMap.cpp
by Stephanie Cockrell
Nov 5, 2012
This program subscribes to the Kinect point cloud and the transform (published by calibrate_bestfit).
It outputs a txt file which is the final map- classifying each point as unknown (0), level floor (1), ramp (2), bumps (3), and obstacles (4).
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <math.h>

// for the writing to file
#include <iostream>
#include <fstream>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

// PCL includes (point cloud)
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ctime>

using namespace std;

using sensor_msgs::PointCloud2;

pcl::PointCloud<pcl::PointXYZ> cloud; // global variable

bool savedPts=false; // this is so that it only runs once- when savedPts=true then it ends

double realXMin=-2; // units: meters
double realXMax=2;
double realYMin=-3;
double realYMax=0;

int finalXRes=400;
int finalYRes=finalXRes*(realYMax-realYMin)/(realXMax-realXMin);
double xStep=(realXMax-realXMin)/finalXRes; // the length of each pixel in m
double yStep=(realYMax-realYMin)/finalYRes;

void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
	// subscribes to the point cloud
	printf("in cloudCallBack\n");
	pcl::fromROSMsg(*cloud_msg, cloud);
}
void processKinectData(const tf::TransformListener& listener){
	// this method does all the processing
	printf("in processKinectData\n");

	int real_x_dim=640;
	int real_y_dim=480;
	int x_res=real_x_dim;
	int y_res=real_y_dim; //clean this up- why do i have so many variables that are the same thing?	

	int approxX;
	int approxY;
	double approxZ;

	//double topDownMap1[finalXRes][finalYRes]; //orig height data
	double topDownMap[finalXRes][finalYRes]; // after filling in tiny holes
	double topDownMap2[finalXRes][finalYRes]; //after avg filter
	int countMap[finalXRes][finalYRes]; // how many pts got put in each cell in the height map- because you need to average them
	double gradientMap1[finalXRes][finalYRes]; // subtracting adjacent cells in topDownMap
	double gradientMap2[finalXRes][finalYRes]; // subtracting adjacent cells in topDownMap2
	double finalMap[finalXRes][finalYRes]; // the FINAL output!

	// you can uncomment these if you want to see the intermediate maps and stuff
/*
	ofstream myfile_height; 
	myfile_height.open ("map_height.txt"); 
	ofstream myfile_heightRaw; 
	myfile_heightRaw.open ("map_height_raw.txt"); 
*/
/*
	ofstream myfile_gradient1; 
	myfile_gradient1.open ("map_gradient1.txt");
	ofstream myfile_gradient2; 
	myfile_gradient2.open ("map_gradient2.txt");
*/
	ofstream myfile_final; 
	myfile_final.open ("map_final.txt");

	geometry_msgs::PointStamped laser_point;
	laser_point.header.frame_id = "camera_depth_frame";

	laser_point.header.stamp = ros::Time();

	geometry_msgs::PointStamped base_point;

	// start timer here
	clock_t startTime=clock(); // just to see how long this process takes. about 2 seconds in all.

	double unknownDist=.1;
	double unknownGradient=-.05;

	for(int i=0; i<finalXRes; i++){
		for(int j=0; j<finalYRes; j++){
		//	topDownMap1[i][j]=unknownDist;
			topDownMap[i][j]=unknownDist;
			topDownMap2[i][j]=unknownDist;
			countMap[i][j]=0;
			gradientMap1[i][j]=unknownGradient;
			gradientMap2[i][j]=unknownGradient;
			finalMap[i][j]=-2; // -2 means it hasn't been filled in yet
		}
	}

	// this for loop, with the transform, takes 1.5 seconds to run. so slow...
	for(int i=0; i<x_res; i++){ 
		for(int j=0; j<y_res; j++){
			laser_point.point.x=(cloud.at(i,j)).x; 
			laser_point.point.y=(cloud.at(i,j)).y; 
			laser_point.point.z=(cloud.at(i,j)).z;

			// transform pt into robot's reference frame
			try{
				listener.transformPoint("base_link", laser_point, base_point);

			}
			catch(tf::TransformException& ex){
				ROS_ERROR("Received an exception trying to transform a point from \"camera_depth_frame\" to \"base_link\": %s", ex.what());
			}
			approxX=(int)((base_point.point.x-realXMin)*finalXRes/(realXMax-realXMin));
			approxY=(int)((base_point.point.y-realYMin)*finalYRes/(realYMax-realYMin));
			approxZ=base_point.point.z;
			if(approxX>=0 && approxX<finalXRes && approxY>=0 && approxY<finalYRes){
				topDownMap[approxX][approxY]=(countMap[approxX][approxY]*topDownMap[approxX][approxY] + approxZ)/(countMap[approxX][approxY] + 1);
				countMap[approxX][approxY]++;
			}
		}			
	}
/*
	//outputting the original height map, before fill ins
	for(int i=0;i<finalXRes; i++){
		for(int j=0; j<finalYRes; j++){
			myfile_heightRaw << topDownMap[i][j] << ", " ;
		}
		myfile_heightRaw << "\n" ;
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
					}
				}
			}
		}
		//printf("fillInCount = %d %d %d\n",fillInCount1,fillInCount2,fillInCount3);
	}
	//double fillInTime=(std::clock() - timeBeforeFillIn)/(double)CLOCKS_PER_SEC;
	//printf("fillInTime = %f sec\n",fillInTime);

	double smoothM1=.02; //the number of m from each pt to the one you are averaging with
	double smoothM2=.3;

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
	//printf("xStep = %f yStep = %f smoothM1 = %f smoothPixels1 = %d smoothM2 = %f smoothPixels2 = %d\n",xStep, yStep,smoothM1,smoothPixels1,smoothM2,smoothPixels2);

	double heightDiffX1, heightDiffX2;
	double heightDiffY1, heightDiffY2;

	// play with these thresholds if you disagree with me about what your robot can and can't run over
	double thr1=.75; // separate floor from bumps
	double thr2=1.8; // separate bumps from obstacles

	for (int i=smoothPixels1_1; i<finalXRes-smoothPixels1_2;i++){
		for(int j=smoothPixels1_1;j<finalYRes-smoothPixels1_2;j++){
			if(finalMap[i][j]!=-1){ // don't do this for the pts with no data
				if(topDownMap[i+smoothPixels1_2][j]!=unknownDist && topDownMap[i-smoothPixels1_1][j]!=unknownDist && topDownMap[i][j+smoothPixels1_2]!=unknownDist && topDownMap[i][j-smoothPixels1_1]!=unknownDist){ // if all 4 adjacent cells have data
					heightDiffX1=abs(topDownMap[i][j]-topDownMap[i+smoothPixels1_2][j]);
					heightDiffX2=abs(topDownMap[i-smoothPixels1_1][j]-topDownMap[i][j]);
					heightDiffY1=abs(topDownMap[i][j]-topDownMap[i][j+smoothPixels1_2]);
					heightDiffY2=abs(topDownMap[i][j-smoothPixels1_1]-topDownMap[i][j]);

					gradientMap1[i][j]= sqrt( (heightDiffY1+heightDiffY2)*(heightDiffY1+heightDiffY2) + (heightDiffX1+heightDiffX2)*(heightDiffX1+heightDiffX2))/smoothM1;
					if(gradientMap1[i][j]>thr2){ // obstacle
						finalMap[i][j]=3;
					}
					else if(gradientMap1[i][j]>thr1){ //bump
						finalMap[i][j]=2;
					}
				}
				else{
					finalMap[i][j]=-1; // no data if not all 4 adj have data
				}
			}
		}
	}

	int avgMask[smoothPixels2][smoothPixels2];
	int avgMaskCount=0;
	int avgMaskChanges=0;
	int i2,j2,jmax1,jmax2;
	bool generateAvgMask=true;
	// this will populate topDownMap2:
	for(int i=0;i<finalXRes;i++){
		for(int j=0;j<finalYRes;j++){
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
						generateAvgMask=false; // comment out this line if you want to regenerate the mask at every point- but it's SO SLOW! It takes like 10 seconds.
					}//end of while loop to populate avg mask
				}
				else{ // if you don't need to generate the entire thing
					jmax1=min(finalYRes,j+smoothPixels2_2)-1;
					jmax2=jmax1-j+smoothPixels2_1;
					for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
						i2=i1-i+smoothPixels2_1;
						if(avgMask[i2][max(0,j-smoothPixels2_1) -j+smoothPixels2_1]==1){ //updating avgMaskCount for removal of 1st col
							avgMaskCount--;
						}
						for(int j1=max(0,j-smoothPixels2_1)+1;j1<min(finalYRes,j+smoothPixels2_2);j1++){ 
							//first shift over everything by 1 column 
							j2=j1-j+smoothPixels2_1;
							avgMask[i2][j2-1]=avgMask[i2][j2];
						}
					}
					// now to figure out avgMask[i2][smoothPixels2-1] (the last column)
					if(jmax2==smoothPixels2-1){ //if we're not going off the edge of the map
						for(int i1=max(0,i-smoothPixels2_1);i1<min(finalXRes,i+smoothPixels2_2);i1++){
							i2=i1-i+smoothPixels2_1;
							if(finalMap[i1][jmax1-1]==-2 && finalMap[i1][jmax1]==-2){
								//cell in new column and cell in adj col are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2][jmax2-1];
							}
							else if(finalMap[i1][jmax1]!=-2){
								//new cell is obstacle/bump
								avgMask[i2][jmax2]=0;
							}
							else if(i1==max(0,i-smoothPixels2_1)){
								//if it's the absolute top corner
								avgMask[i2][jmax2]=-1;
							}
							else if(finalMap[i1-1][jmax1]==-2 && finalMap[i1][jmax1]==-2){
								//if new cell and the one above are both floor/ramp
								avgMask[i2][jmax2]=avgMask[i2-1][jmax2];
							}
							else if(i1==min(finalXRes,i+smoothPixels2_2)-1){
								// if it's the one in the absolute bottom corner and we haven't figured it out yet
								avgMask[i2][jmax2]=0;
							}
							else{
								avgMask[i2][jmax2]=-1; //unknown for now
							}
							if(avgMask[i2][jmax2]==1){
								avgMaskCount++;
							}
						}
					
						for(int i1=min(finalXRes,i+smoothPixels2_2)-2;i1>=max(0,i-smoothPixels2_1);i1--){
							i2=i1-i+smoothPixels2_1;
							if(avgMask[i2][jmax2]==-1){
								if(finalMap[i1+1][jmax1]==-2 && finalMap[i1][jmax1]==-2){
									//new cell and the one below are both floor/ramp
									avgMask[i2][jmax2]=avgMask[i2+1][jmax2];
									if(avgMask[i2][jmax2]==1){
										avgMaskCount++;
									}
								}
								else{
									avgMask[i2][jmax2]=0;
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
	//double avgMaskTime=(std::clock() - timeBeforeAvgMask)/(double)CLOCKS_PER_SEC;
	//printf("avgMaskTime = %f sec\n",avgMaskTime);

	double thr3=.000425;
	// now subtract to find the difference between floor and ramp
	for (int i=1; i<finalXRes;i++){
		for(int j=1;j<finalYRes;j++){
			if(finalMap[i][j]==-2){
				heightDiffX1=abs(topDownMap2[i][j]-topDownMap2[i-1][j]);
				heightDiffY1=abs(topDownMap2[i][j]-topDownMap2[i][j-1]);
				gradientMap2[i][j]=sqrt(heightDiffX1*heightDiffX1 + heightDiffY1*heightDiffY1);
				if(gradientMap2[i][j]>=thr3){
					finalMap[i][j]=1; // ramp
				}
				else{
					finalMap[i][j]=0; // floor
				}
			}
		}
	}
	clock_t endTime=clock()-startTime;
	// end timer here
	double calculationTime=(std::clock() - startTime)/(double)CLOCKS_PER_SEC;
	printf("calculationTime = %f sec\n",calculationTime);


	// now write the output to file
	for(int i=0;i<finalXRes; i++){
		for(int j=0; j<finalYRes; j++){

//			myfile_height << topDownMap[i][j] << ", " ;
/*
			myfile_gradient1 << gradientMap1[i][j] << ", " ;
			myfile_gradient2 << gradientMap2[i][j] << ", " ;
*/
			myfile_final << finalMap[i][j] << ", " ;
		}

//		myfile_height << "\n" ;
/*
		myfile_gradient1 << "\n" ;
		myfile_gradient2 << "\n" ;
*/
		myfile_final << "\n" ;
	}


//	myfile_height.close();
//	myfile_heightRaw.close();
/*
	myfile_gradient1.close();
	myfile_gradient2.close();
*/
	myfile_final.close();

	savedPts=true; // so it only loops until this method executes once

} 
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

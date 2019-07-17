
/**
 * @file lasor_spot.cpp
 * @brief This file consists of code that converts the eliminates the background in the video by frame differencing and preserves the laser spot..
 * 
 * The code has been executed on Jetson Nano.
 * 
 * @author Siddhant Jajoo.
 * @date 06/26/2019
 * 
 * @copyright Copyright (c) 2019. 
 */


#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;


#define WINDOW				("VIDEO WINDOW")
//#define G_BAND_WINDOW		("G BAND GRAYSCALE FRAME")
//#define MEDIAN_WINDOW		("APPLIED MEDIAN FILTER TO G BAND GRAYSCALE FRAME")
//#define DESTINATION			("Screenshots and Images/before_median_filter.ppm")


int main( int argc, char** argv )
{
	IplImage* laser_frame;
	IplImage* prev_frame;
	IplImage* diff_frame;
	char k;
	
	
	if (argc ==1)
	{
		cout << "Input Video as an argument.\n";		
		exit(EXIT_FAILURE);
	}
	
	//Naming Windows
	namedWindow(WINDOW, WINDOW_AUTOSIZE);
	
	CvCapture* capture = cvCreateFileCapture(argv[1]);
	if(!capture)
	{
		cout << "Could not open or find the reference Video.\n";
		exit(EXIT_FAILURE);
	}
	
	
	laser_frame = cvQueryFrame(capture);
	prev_frame = cvCreateImage(cvGetSize(laser_frame), IPL_DEPTH_8U, 3);
	diff_frame = cvCreateImage(cvGetSize(laser_frame), IPL_DEPTH_8U, 3);
	
	prev_frame = cvCloneImage(laser_frame);


	while(1)
	{
		laser_frame = cvQueryFrame(capture);
		
		cvSub(laser_frame, prev_frame, diff_frame);
		prev_frame = cvCloneImage(laser_frame);  
	
		cvShowImage(WINDOW, laser_frame);
		
		k = cvWaitKey(10);		
		if(k == 27) break;
	}
	
	//Splitting the Image colour channels to obtain G band grayscale image
	//split(laser_frame, bgr);
		
	//imshow(WINDOW, laser_frame);
	cvReleaseImage(&prev_frame);
	cvReleaseImage(&diff_frame);
	cvReleaseCapture(&capture);
	destroyWindow(WINDOW);
	
}

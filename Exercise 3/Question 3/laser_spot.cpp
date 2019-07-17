
/**
 * @file lasor_spot.cpp
 * @brief This file consists of code that eliminates the background in the video by frame differencing and preserves the laser spot..
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
#define NAME				("output.avi")

int main( int argc, char** argv )
{
	Mat laser_frame;
	Mat prev_frame;
	Mat diff_frame;
//	Mat gray;
	char k;
	
	//Check for correct number of arguments
	if (argc ==1)
	{
		cout << "Input Video as an argument.\n";		
		exit(EXIT_FAILURE);
	}
	
	//Naming Windows
	namedWindow(WINDOW, WINDOW_AUTOSIZE);
	
	VideoCapture capture(argv[1]);
	VideoWriter outputvideo;
	
	//Reading and saving the first frame
	capture.read(laser_frame);
	if(laser_frame.empty())
	{
		cout << "Cannot find Video Reference.\n";
	}
	prev_frame = laser_frame.clone();
//	cvtColor(laser_frame, prev_frame, COLOR_BGR2GRAY);
	
	//Get input video size.
	Size outputsize = Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH), (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	
	//Set Output Video Parameters
	outputvideo.open(NAME, capture.get(CV_CAP_PROP_FOURCC), capture.get(CV_CAP_PROP_FPS), outputsize, true);
	if(!outputvideo.isOpened())
	{
		cout << "Could not open Output video for writer.\n";
		exit(EXIT_FAILURE);
	}
	
	while(1)
	{
		//Reading the next frame
		capture.read(laser_frame);
		if(laser_frame.empty())
		{
			cout << "Video Duration over.\n";
			break;
		}
		
//		cvtColor(laser_frame, gray, COLOR_BGR2GRAY);		
//		diff_frame = gray - prev_frame;
//		prev_frame = gray.clone();  
		
		//Comparing the current and previous frame
		diff_frame = laser_frame - prev_frame;
		
		//Saving the current frame for next iteration.
		prev_frame = laser_frame.clone();  

		//Display
		imshow(WINDOW, diff_frame);
		
		//Write the frames in a video format.
		outputvideo.write(diff_frame);
		
		k = cvWaitKey(10);		
		if(k == 27) break;
	}
	
	//Destroy Window
	destroyWindow(WINDOW);
		
	return 0;
}

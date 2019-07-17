
/**
 * @file grayscale.cpp
 * @brief This file consists of code that converts the video frames into G band grayscale and preserves the laser spot by frame differencing.
 * The individual frames are stored in .pgm format.
 * 
 * The code has been executed on Jetson Nano.
 * 
 * @author Siddhant Jajoo.
 * @date 06/27/2019
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

int main( int argc, char** argv )
{
	Mat laser_frame;
	Mat bgr[3];
	char k;
	int i = 0;
	char out[50];
	
	
	//Check for correct number of arguments
	if (argc ==1)
	{
		cout << "Input Video as an argument.\n";		
		exit(EXIT_FAILURE);
	}
	
	//Naming Windows
	namedWindow(WINDOW, WINDOW_AUTOSIZE);
	
	VideoCapture capture(argv[1]);
	
	
	while(1)
	{
		//Reading the next frame
		capture.read(laser_frame);
		if(laser_frame.empty())
		{
			cout << "Video Duration over.\n";
			break;
		}
		
		//Splitting the Image colour channels to obtain G band grayscale image
		split(laser_frame, bgr);
		
		//Display
		imshow(WINDOW, bgr[1]);
		
		sprintf(out, "./Grayscale_PGM_frames/grayscale_frame_%d.pgm", i);
		imwrite(out, bgr[1]);
		
		i++;
		
		k = cvWaitKey(10);		
		if(k == 27) break;
	}
	
	//Destroy Window
	destroyWindow(WINDOW);
		
	return 0;
}

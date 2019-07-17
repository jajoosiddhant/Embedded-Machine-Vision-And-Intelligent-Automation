
/**
 * @file tracking_clutter.cpp
 * @brief This file consists of threshold function and a COM detection function to track the laser spot.
 * The laser spot is tracked with the help of a crosshair.
 * 
 * The code has been executed on Jetson Nano.
 * 
 * @author Siddhant Jajoo.
 * @date 06/28/2019
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
#define LINE_THICKNESS 		(2)
#define LINE_TYPE 			(8)
#define LINE_SHIFT 			(0)
#define NAME				("output_tracking.avi")


//Function Declaration
Mat threshold_singlechannel(Mat laser_frame);
void find_xleft(int i, int j);
void find_xright(int i, int j);
void find_ytop(int i,int j);
void find_ybottom(int i, int j);
void find_com(void);
Mat background_eliminate(Mat img);


//Variable Declaration
int xleft, xright, ytop, ybottom;
int yleft, yright, xtop, xbottom; 
int x,y;
int flag;
int maxm;
char pixel;
Mat frame, laser_frame, prev_frame, gray, nobg;
Scalar yellow(0,255,255);
	

int main( int argc, char** argv )
{
	char k;
	flag = 0;
	
	//Check for correct number of arguments
	if (argc == 1)
	{
		cout << "Input Video as an argument.\n";		
		exit(EXIT_FAILURE);
	}
	
	//Naming Windows
	namedWindow(WINDOW, WINDOW_AUTOSIZE);
	
	VideoCapture capture(argv[1]);
	VideoWriter outputvideo;
	
	//Get input video size.
	Size outputsize = Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH), (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
	

	
	//Set Output Video Parameters
	outputvideo.open(NAME, capture.get(CV_CAP_PROP_FOURCC), capture.get(CV_CAP_PROP_FPS), outputsize, true);
	if(!outputvideo.isOpened())
	{
		cout << "Could not open Output video for writer.\n";
		exit(EXIT_FAILURE);
	}


	//Reading and saving the first frame
	capture.read(frame);
	if(frame.empty())
	{
		cout << "Cannot find Video Reference.\n";
	}
	
	cvtColor(frame, laser_frame, COLOR_BGR2GRAY);
	prev_frame = laser_frame.clone();
	
	
	while(1)
	{
		//Reading the next frame
		capture.read(frame);
		if(frame.empty())
		{
			cout << "Video Duration over.\n";
			break;
		}

		//Initializing Co-ordinates
		xleft = frame.cols;
		xright = 0;
		ytop = frame.rows;
		ybottom = 0;
	
		yleft = 0;
		yright = 0;
		xtop = 0;
		xbottom = 0;
		
		
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		
		//Eliminating Background
		nobg = background_eliminate(gray);
		
		//For debugging purpose		
		/*
		cout << "Before Threshold" << endl;
		cout << "xleft = "<< xleft << endl;
		cout << "xright = "<< xright << endl;
		cout << "ytop = "<< ytop << endl;
		cout << "ybottom = "<< ybottom << endl;
		*/
		
		//Converting to binary image
		for(int i = 0; i < nobg.rows; i++)
		{
			for(int j = 0; j < nobg.cols; j++)
			{
				if(nobg.at<uchar>(i,j) > 30)
				{
					nobg.at<uchar>(i,j) = 255;
				}
				else
				{
					nobg.at<uchar>(i,j) = 0;
				}
			}
		}
		

		//Median Filter to eliminate noise	
		medianBlur(nobg, laser_frame, 5);

		laser_frame = threshold_singlechannel(laser_frame);

		//Fir debugging Purpose
		/*
		cout << "After Threshold"<< endl;
		cout << "xleft = "<< xleft << endl;
		cout << "xright = "<< xright << endl;
		cout << "ytop = "<< ytop << endl;
		cout << "ybottom = "<< ybottom << endl;
		*/
		
		
		if(flag)
		{
			line(frame, Point(0, y), Point(frame.cols, y), yellow, LINE_THICKNESS, LINE_TYPE, LINE_SHIFT);
			line(frame, Point(x, 0), Point(x,frame.rows), yellow, LINE_THICKNESS, LINE_TYPE, LINE_SHIFT);			
			flag = 0;
		}		
		
		//Display
		imshow(WINDOW, frame);
		
		//Write the frames in a video format.
		outputvideo.write(laser_frame);
	
		k = cvWaitKey(10);		
		if(k == 27) break;
	}
	
	//Destroy Window
	destroyWindow(WINDOW);
		
	return 0;
}



//Functionto take frame difference and eliminate background
Mat background_eliminate(Mat img)
{
	Mat diff_frame;
			
	//Comparing the current and previous frame
	diff_frame = img - prev_frame;
	
	//Saving the current frame for next iteration.
	prev_frame = img.clone();
	
	return diff_frame;  
}


//Threshold function
Mat threshold_singlechannel(Mat th_img)
{
	uchar value;

	//i corresponds to rows, j corresponds to columns
	for(int i = 0; i < th_img.rows; i++)
	{
		for(int j = 0; j < th_img.cols; j++)
		{
			value = th_img.at<uchar>(i,j);
					
			if(value == 255)
			{			
				//Find xleft, xright, ytop, ybottom.
				find_xleft(i, j);
				find_xright(i, j);
				find_ytop(i, j);
				find_ybottom(i, j);
				
				find_com();

				flag = 1;			
			}
		}
	}
	
	return th_img;
}



//Function to find leftmost x of laser spot in the frame.
void find_xleft(int i, int j)
{
	if(j < xleft)
	{
		xleft = j;
		yleft = i;
	}
}


//Function to find the rightmost x of laser spot in the frame.
void find_xright(int i, int j)
{
	if(j > xright)
	{
		xright = j;
		yright = i;
	}
}

//Function to find the topmost y of laser spot in the frame.
void find_ytop(int i, int j)
{
	if(i < ytop)
	{
		ytop = i;
		xtop = j;
	}
}


//Function to find the bottommost y of laser spot in the frame.
void find_ybottom(int i, int j)
{
	if(i > ybottom)
	{
		ybottom = i;
		xbottom = j;
	}	
}


//Function to find the COM of the laser spot.
void find_com(void)
{
	
	x = xleft + (xright - xleft)/2; 
	y = ytop + (ybottom - ytop)/2;

}


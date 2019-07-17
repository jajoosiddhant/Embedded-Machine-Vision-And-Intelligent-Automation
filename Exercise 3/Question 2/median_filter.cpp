
/**
 * @file median_filter.cpp
 * @brief Thus file consists of code that converts the colour image to the G band grayscale image.
 * It shows and outputs the G band image as well as the Grayscal image.
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


#define COLOUR_WINDOW		("ORIGINAL FRAME")
#define G_BAND_WINDOW		("G BAND GRAYSCALE FRAME")
#define MEDIAN_WINDOW		("APPLIED MEDIAN FILTER TO G BAND GRAYSCALE FRAME")
#define DESTINATION			("Screenshots and Images/before_median_filter.ppm")


int main( int argc, char** argv )
{
	Mat laser_frame, median_frame;
	Mat bgr[3];
	
	if (argc ==1)
	{
		cout << "Input Image as an argument.\n";		
		exit(EXIT_FAILURE);
	}
	
	//Naming Windows
	namedWindow(COLOUR_WINDOW, WINDOW_AUTOSIZE);
	namedWindow(G_BAND_WINDOW, WINDOW_AUTOSIZE);
	namedWindow(MEDIAN_WINDOW, WINDOW_AUTOSIZE);
	
	//Reading the image
	laser_frame = imread(argv[1], CV_LOAD_IMAGE_COLOR);

	//Checking Image Data
	if(!laser_frame.data)
	{
		cout << "Could not open or find the reference image.\n";
		exit(EXIT_FAILURE);
	}

	//Splitting the Image colour channels to obtain G band grayscale image
	split(laser_frame, bgr);
	
//	cout<<bgr[1].channels();
	
	//Applying Median Filter to Grayscale Image
	medianBlur(bgr[1], median_frame, 3);
	
	imshow(COLOUR_WINDOW, laser_frame);
	imshow(G_BAND_WINDOW, bgr[1]);
	imshow(MEDIAN_WINDOW, median_frame);

//	cout<<median_frame.channels();
	
	while(waitKey(0)!=27);

	imwrite("output_original_frame.jpg", laser_frame);
	imwrite("output_gband.jpg", bgr[1]);
	imwrite("output_median_frame.jpg", median_frame);
	
	destroyWindow(COLOUR_WINDOW);
	destroyWindow(G_BAND_WINDOW);
	destroyWindow(MEDIAN_WINDOW);
}

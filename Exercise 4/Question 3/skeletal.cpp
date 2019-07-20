/*
 * @file skeletal.c
 * @brief This code applies skeletal transform to the frames continuously obtained from and the camera.
 * It aslso creates an mpeg4 video and saves the frames in a folder frames_snapshot.
 * 
 * The code has been executed on Jetson Nano.
 * 
 * @author Siddhant Jajoo
 * @date 07/16/2019
 * @copyright Copyright (c) 2019
 */
 
 
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;


#define NAME 		("output.mp4")
#define THRESHOLD	(100)

//Function Declarations
Mat skeletal_transform(Mat src);


int main( int argc, char** argv )
{
	
	Mat frame;
	Mat src;
	int k = 0;
	char out[50];
	
    VideoCapture capture(0);
	VideoWriter outputvideo;
	
	capture.open(0);
	if(!capture.isOpened())
	{
		cout << "Cannot open camera." << endl;
		exit(EXIT_FAILURE);
	}
	
	Size outputsize = Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH), (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
		
	outputvideo.open(NAME, CV_FOURCC('M', 'P', '4', 'V'), capture.get(CV_CAP_PROP_FPS), outputsize, true);
	if(!outputvideo.isOpened())
	{
		cout << "Could not open Output video for writer." << endl;
		exit(EXIT_FAILURE);
	}
	
    while(1)
    {
        capture.read(frame);
     
        if(frame.empty()) 
        {
			cout << "Cannot capture frame.\n" << endl;
			break;
		}

        imshow("Original Video", frame);

		
		//Applying Skeletal Transform on the obtained frame.
		src = skeletal_transform(frame);
			
		cvtColor(src, frame, CV_GRAY2BGR);
        
        imshow("Skeletal Transformed Video", frame);
        
        sprintf(out, "./frames_snapshot/frame_%d.jpg", k);
        imwrite(out, frame);
        outputvideo.write(frame);
        
        k++;
        
        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }
    
}



Mat skeletal_transform(Mat src)
{
	Mat gray, binary, blur;
	
	cvtColor(src, gray, CV_BGR2GRAY);
	imshow("Grayscale Image", gray);
	
	
	threshold(gray, binary, THRESHOLD, 255, CV_THRESH_BINARY);
	binary = 255-binary;
	imshow("Binary Image", binary);

	
	medianBlur(binary, blur, 5);
	imshow("Median Blur Image", blur);
	
	Mat skel(blur.size(), CV_8UC1, Scalar(0));
	Mat temp;
	Mat eroded;
	Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
	bool done;
	int iterations=0;

	do
	{
		erode(blur, eroded, element);
		dilate(eroded, temp, element);
		subtract(blur, temp, temp);
		bitwise_or(skel, temp, skel);
		eroded.copyTo(blur);

		done = (countNonZero(blur) == 0);
		iterations++;
 
	} while (!done && (iterations < 100));

	cout << "iterations=" << iterations << endl;
 
	return skel;
}

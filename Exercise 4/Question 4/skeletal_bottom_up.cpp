/*
 * @file skeletal_bottom_up.c
 * @brief This code applies skeletal transform using the bottom up to the frames continuously obtained from the camera.
 * The frames obtained are first applied frame differencing followed by creating a binary image and then applies the skeletal transform.
 * It aslso creates an mpeg4 video and saves the frames in a folder frames_snapshot.
 * 
 * The code was executed on Jetson-nano.
 * 
 * @author Siddhant Jajoo
 * @date 07/16/2019
 * @copyright Copyright (c) 2019
 */
 
 
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;


#define NAME 		("output.mp4")
#define THRESHOLD	(100)

//Function Declarations
Mat skeletal_transform(Mat src);

int cnt_frame;

int main( int argc, char** argv )
{
	
	Mat frame;
	Mat prev_frame;
	Mat diff_frame;
	Mat src;
	int k = 0;
	char out[50];	
	struct timeval start, stop;
	cnt_frame = 0;
	
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

	/*
	//Reading Initial Frame
    capture.read(frame);
    if(frame.empty())
    {
		cout << "Cannot read Video from Camera" << endl;
	}
	//Cloning initial frame for differencing
	prev_frame = frame.clone();
	*/
	
	gettimeofday(&start, NULL);
	
    while(1)
    {

		//Reading next frame
		capture.read(frame);
        if(frame.empty()) 
        {
			cout << "Cannot capture frame.\n" << endl;
			break;
		}

        imshow("Original Video", frame);

		/* Uncomment the below lines if using frame differencing
		//Comparing current and previous frame.
		diff_frame = frame - prev_frame;
		
		//Saving the next frame for next iteration.
		prev_frame = frame.clone();
		
		//Applying Skeletal Transform on the obtained frame.
		src = skeletal_transform(diff_frame);
		*/
			
		//Applying Skeletal Transform on the obtained frame. (Comment this line if using frame differencing)
		src = skeletal_transform(frame);
			
		cvtColor(src, frame, CV_GRAY2BGR);
        
        imshow("Skeletal Transformed Video", frame);
        
        sprintf(out, "./frames_snapshot/frame_%d.jpg", k);
        imwrite(out, frame);
        outputvideo.write(frame);
        
        k++;
        
        char c = cvWaitKey(10);
        if( c == 27 ) break;
    }
    
    
    gettimeofday(&stop, NULL);    
    cout << "Duration: " << (stop.tv_sec - start.tv_sec) << " seconds." << endl;
	cout << "Average Frame rate = " << cnt_frame/((int)(stop.tv_sec - start.tv_sec)) << endl;
    
}



Mat skeletal_transform(Mat src)
{
	Mat gray, binary, blur;
	int sigma, chi;
	
	uchar a0,a1,a2,a3,a4,a5,a6,a7,a8;
	
	cvtColor(src, gray, CV_BGR2GRAY);
	imshow("Grayscale Image", gray);
	
	
	threshold(gray, binary, THRESHOLD, 255, CV_THRESH_BINARY);
	binary = 255-binary;
	imshow("Binary Image", binary);

	medianBlur(binary, blur, 5);
	imshow("Median Blur Image", blur);
	
	Mat skel(blur.size(), CV_8UC1, Scalar(0));
	bool done;
	int iterations=0;

	do
	{
		done = true;
		for(int i = 1; i < blur.rows -1; i++)
		{
			for(int j = 1; j < blur.cols-1; j++)
			{
				a0 = blur.at<uchar>(i,j);
				
				if(a0 == 255)
				{
					a1 = blur.at<uchar>(i,j+1);
					a2 = blur.at<uchar>(i-1,j+1);
					a3 = blur.at<uchar>(i-1,j);
					a4 = blur.at<uchar>(i-1,j-1);
					a5 = blur.at<uchar>(i,j-1);
					a6 = blur.at<uchar>(i+1,j-1);
					a7 = blur.at<uchar>(i+1,j);
					a8 = blur.at<uchar>(i+1,j+1);
					
					//Calculating Sigma = A1 + A2 + A3 + A4 + A5 + A6 + A7 + A8
					sigma = a1 + a2 + a3 + a4 + a5 + a6 + a7 + a8;
					
					//Calculating Chi
					chi = (int)(a1 != a3) + (int)(a3 != a5) + (int)(a5 != a7) + (int)(a7 != a1)
							+2*((int)((a2>a1)&&(a2>a3)) + (int)((a4>a3)&&(a4>a5)) + (int)((a6>a5)&&(a6>a7)) + (int)((a8>a7)&&(a8>a1)));
					
					
					if((chi == 2) && (sigma != 255))
					{
						//North Point		
						if((a3 == 0) && (a7 == 255))
						{
							blur.at<uchar>(i,j) = 0;
							done = false;
						}
						else if((a3 == 255) && (a7 == 0))
						{
							blur.at<uchar>(i,j) = 0;
							done = false;
						}
						//East Point		
						else if((a5 == 255) && (a1 == 0))
						{
							blur.at<uchar>(i,j) = 0;
							done = false;
						}
						//West Point		
						else if((a5 == 0) && (a1 == 255))
						{
							blur.at<uchar>(i,j) = 0;
							done = false;
						}
					}
				}
			}
		} 


		iterations++;
 
	} while (!done && (iterations < 100));

	cout << "iterations=" << iterations << endl;
	cnt_frame++;
 
	return blur;
}

/*
 * @file skeletal.c
 * @brief 
 * 
 * @author Siddhant Jajoo
 * @date 07/16/2019
 *
 */
 
 
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

//Function Declarations
Mat skeletal_transform(Mat src);


int main( int argc, char** argv )
{
	
	Mat src;
	IplImage *frame;
		
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    CvCapture* capture = cvCreateCameraCapture(0);


    while(1)
    {
        frame=cvQueryFrame(capture);
     
        if(!frame) 
        {
			cout << "Cannot capture frame.\n" << endl;
			break;
		}

		//Converting IplImage to Mat image and displaying it.
		src = cvarrToMat(frame);
        imshow("Original Video", src);

		src = skeletal_transform(src);
        
        imshow("Skeletal Transformed Video", src);
        
        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }

    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");
    
}



Mat skeletal_transform(Mat src)
{
	Mat gray, binary, blur;
	
	cvtColor(src, gray, CV_BGR2GRAY);
	
	threshold(gray, binary, 70, 255, CV_THRESH_BINARY);
	binary = 255-binary;
	
	medianBlur(binary, blur, 1);
	
	
	Mat skel(mfblur.size(), CV_8UC1, Scalar(0));
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

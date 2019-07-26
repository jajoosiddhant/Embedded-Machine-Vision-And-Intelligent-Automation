/*
 * @file lane_detect.c
 * @brief 
 * It aslso creates an mpeg4 video and saves the frames in a folder frames_snapshot.
 * 
 * The code was executed on Jetson-nano.
 * 
 * @author Siddhant Jajoo
 * @date 07/26/2019
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
#include "opencv2/objdetect.hpp"

using namespace cv;
using namespace std;

#define NAME 		("output.mp4")
#define THRESHOLD	(100)

//Canny Threshold Values
#define CANNY_THRESHOLD_1			(40)
#define CANNY_THRESHOLD_2			(120)

//Hough Lines Threshold Values
#define	HOUGH_THRESHOLD				(70)
#define	HOUGH_MIN_LINE_LENGTH		(10)
#define	HOUGH_MAX_LINE_GAP			(50)

//Function Declarations
Mat detect_lanes(Mat edge, Mat frame);
Mat detect_vehicle(Mat edge, Mat frame);
void help(void);

//Global Variable Declaration
int cnt_frame;
CascadeClassifier vehicle_cascade;
const string vehicle_cascade_name("cars.xml");

void help()
{
 cout << "\nThis program demonstrates line finding with the Hough transform.\n"
         "Usage:\n"
         "./houghlines <Video_name>\n" << endl;
}


int main(int argc, char** argv)
{
	Mat src;
	Mat frame, edge, blur;
	//Uncomment if using split and not cvtColor.
	//Mat bgr[3];
	Mat gray;
	Mat test, test1;
	struct timeval start, stop;
	
	//To store the path where frames would be saved.
	char out[50];	
	int k = 0;
	char c = 0;
	cnt_frame = 0;
	
	
    VideoCapture capture(argv[1]);
	VideoWriter outputvideo;
	
	//capture.open(argv[1]);
	if(!capture.isOpened())
	{
		cout << "Cannot open camera." << endl;
		help();
		exit(EXIT_FAILURE);
	}
	
	Size outputsize = Size((int)capture.get(CV_CAP_PROP_FRAME_WIDTH), (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT));
		
	outputvideo.open(NAME, CV_FOURCC('M', 'P', '4', 'V'), capture.get(CV_CAP_PROP_FPS), outputsize, true);
	if(!outputvideo.isOpened())
	{
		cout << "Could not open Output video for writer." << endl;
		exit(EXIT_FAILURE);
	}

	
	if( !vehicle_cascade.load(vehicle_cascade_name) )
    {
        cout << "Error loading Vehicle cascade\n";
        exit(EXIT_FAILURE);
    }
	
	
	gettimeofday(&start, NULL);
	
	while(capture.read(src))
	{	
	
		cvtColor(src, gray, CV_RGB2GRAY);
		GaussianBlur( gray, blur, Size(5,5), 0, 0, BORDER_DEFAULT );
		Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);	//Can change to false

				
		//Another method is to split and just obtain the Green Channel.
		//split(src, bgr);
		//GaussianBlur( bgr[1], blur, Size(5,5), 0, 0, BORDER_DEFAULT )
		//Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);

		imshow("Canny", edge);
		frame = src.clone();
		cvtColor(edge, test, CV_GRAY2RGB);
		cvtColor(edge, test1, CV_GRAY2RGB);
		
		//Detecting lanes and drawing lines.
		test = detect_lanes(edge, test);

		//Detecting Vehicles.
		frame = detect_vehicle(gray, test1);

		//imshow("source", src);
		imshow("Detected lines", test);
		imshow("Detected Vehicles", frame);
		sprintf(out, "./frames_snapshot/frame_%d.jpg", k);
        imwrite(out, frame);
        outputvideo.write(frame);
        
        k++;
        cnt_frame++;
        
        c = cvWaitKey(10);
        if( c == 27 ) break;
	
	}
	
	gettimeofday(&stop, NULL);    
    cout << "Duration: " << (stop.tv_sec - start.tv_sec) << " seconds." << endl;
	cout << "Average Frame rate = " << cnt_frame/((int)(stop.tv_sec - start.tv_sec)) << endl;
	return 0;
}


/**
 * @brief This function detects straight lines and draws lines over those detected lines.
 * @param edge The edge detected frame.
 * @param frame The frame on which the lines are drawn.
 * @return Mat frame. 
 */
Mat detect_lanes(Mat edge, Mat frame)
{
	vector<Vec4i> lines;
	HoughLinesP(edge, lines, 1, CV_PI/180, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP );
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		line( frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
	}
	return frame;
}


/**
 * @brief This function detects vehicles and draws rectangles over those detected vehicles.
 * @param edge The edge detected frame.
 * @param frame The frame on which the lines are drawn.
 * @return Mat frame. 
 */
Mat detect_vehicle(Mat edge, Mat frame)
{
	vector<Rect> vehicle;
	vehicle_cascade.detectMultiScale(edge, vehicle);
	
	for ( size_t i = 0; i < vehicle.size(); i++ )
    {
		 rectangle(frame, vehicle[i], CV_RGB(255, 0, 0) );		
    }

	return frame;
}

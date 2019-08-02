/*
 * @file lane_detect.c
 * @brief 
 * It also creates an mpeg4 video and saves the frames in a folder frames_snapshot.
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
#define	HOUGH_THRESHOLD				(40)
#define	HOUGH_MIN_LINE_LENGTH		(10)
#define	HOUGH_MAX_LINE_GAP			(80)

//Function Declarations
Mat preprocess(Mat src);
Mat detect_draw_lanes(Mat edge, Mat frame);
Mat detect_draw_vehicle(Mat edge, Mat frame);
Mat getroi(Mat edge);
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

	//Loading the .xml file
	if( !vehicle_cascade.load(vehicle_cascade_name) )
    {
        cout << "Error loading Vehicle cascade\n";
        exit(EXIT_FAILURE);
    }
	
	//Noting the start of processing time.
	gettimeofday(&start, NULL);

	Mat src;
	Mat src_half, src_res, gray;
	Mat contrast;	
	Mat hls, hsv, mask;
	Mat white, yellow;
	Mat lanes_detect;
	Mat blur, edge;
	Mat roi_mask;
	Mat canny_roi;
	vector<Vec4i> lines;
	double slope;
	
	while(capture.read(src))
	{	

		//Preprocess frames
		//Reduce the resolution by 2
		pyrDown(src, src_res);
		
		//Crop the image to get bottom half because sky is not required. Thus eliminating half the pixels.
		src_half = src_res( Rect( 0, src_res.rows/2, src_res.cols, src_res.rows/2) );
		imshow("Original", src_half);		
		
		//Convert to grayscale
		cvtColor(src_half, gray, COLOR_BGR2GRAY);
		
		//Increase Contrast to detect white lines and remove any disturbance due to road color.
		equalizeHist(gray, contrast);
		imshow("Contrast Image", contrast);
		//imshow("Half image and resolution and gray", gray);
		
		 
		//Convert Original Image to HLS
		//Shows white as yellow.
		cvtColor(src_half, hls, COLOR_BGR2HLS);
		imshow("HLS", hls);
		
		//Lower value of Saturation makes changes. i.e middle one. Reducing the lower saturation value includes yellow lanes and background as well	
		//If want to incorpotate yellow lines change lower threshold of saturation to 70 or keep 100.
		inRange(hls, Scalar(20,100,0), Scalar(40,255,50), white);				//brightness can be 0 to 50 also.	
		imshow("HLS white", white);
		
		/*
			//ALTERNATE
		//Convert Original Image to HLS
		//Shows white as yellow.
		cvtColor(src_half, hls, COLOR_RGB2HLS);
		imshow("HLS", hls);
		
		//Lower value of Saturation makes changes. i.e middle one. Reducing the lower saturation value includes yellow lanes and background as well	
		//If want to incorpotate yellow lines change lower threshold of saturation to 70 or keep 100.
		inRange(hls, Scalar(60,100,30), Scalar(100,255,255), white);				//brightness can be 0 to 50 also.	
		imshow("HLS white", white);
		*/


		
		// Original
		//Convert Original Image to HSV
		//Shows yellow as yellow.
		cvtColor(src_half, hsv, COLOR_BGR2HSV);
		imshow("HSV", hsv);
		
		//Brightness makes the difference (eliminates background) - Do not change brightness value. i.e the last value (80)
		//Saturation value makes changes i.e middle one. If want to make the yellow lines more bold increase upper saturation value. above 100 introduces background.
		inRange(hsv, Scalar(20,80,80), Scalar(40,255,255), yellow); 					
		imshow("HSV YELLOW", yellow);
		
		
		bitwise_or(white, yellow, mask);
		imshow("Mask", mask);
		
		bitwise_and(contrast, mask, lanes_detect);
		imshow("lanes Detected", lanes_detect);
		
		//Applying gaussian filter to reduce noise followed by canny transform for edge detection.
		GaussianBlur( lanes_detect, blur, Size(5,5), 0, 0, BORDER_DEFAULT );
		Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);	//Can change to false

		//Add ROI function here
		Point roi_pt[1][3];
		int num = 3;
		roi_mask = Mat::zeros(Size(src_half.cols, src_half.rows), CV_8U);
		//Points for ROI mask
		roi_pt[0][0] = Point(src_half.cols/2, 0);					//Apex
		roi_pt[0][1] = Point(0, src_half.rows);						//Bottom left vertice
		roi_pt[0][2] = Point(src_half.cols, src_half.rows);			//Bottomk right vertice
		const Point* pts_list[1] = {roi_pt[0]};
		fillPoly(roi_mask, pts_list, &num, 1, 255, 8);
		
		//imshow("ROI MASK", roi_mask);

		//imshow("Canny", edge);

		bitwise_and(edge, roi_mask, canny_roi);
		//imshow("Canny Mask", canny_roi);
		
		//Detect and Draw Lines
		HoughLinesP(canny_roi, lines, 1, CV_PI/180, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP );
		for( size_t i = 0; i < lines.size(); i++ )
		{
			Vec4i l = lines[i];
			
			slope = (double)(l[3] - l[1])/(l[2] - l[0]);
			if (slope > 0.5 || slope < (-0.5))
			{
				line( src_res, Point(l[0], l[1] + src_res.rows/2), Point(l[2], l[3] + src_res.rows/2), Scalar(0,0,255), 3, CV_AA);	
			}
			
		}

		//Can use pyrUp to -revert to original resolution before displaying.
		imshow("Final", src_res);


//		frame = src.clone();
//		cvtColor(edge, test, CV_GRAY2RGB);
//		cvtColor(edge, test1, CV_GRAY2RGB);
		
		//Detecting lanes and drawing lines.
		//test = detect_draw_lanes(gray, src);

		//Detecting Vehicles.
//		test = detect_draw_vehicle(gray, gray);

		//imshow("source", src);
		//imshow("Detected lines", test);
//		imshow("Detected Vehicles", test);
//		sprintf(out, "./frames_snapshot/frame_%d.jpg", k);
//        imwrite(out, frame);
//        outputvideo.write(frame);

        
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


Mat preprocess(Mat src)
{
	Mat src_half, src_res, gray;
	
	//Crop the image to get bottom half because sky is not required. Thus eliminating half the pixels.
	src_half = src( Rect( 0, src.rows/2, src.cols, src.rows/2) );
	
	//Reduce the resolution by 2
	pyrDown(src_half, src_res);
		
	return src_res;
}




/**
 * @brief This function detects straight lines and draws lines over those detected lines.
 * @param edge The edge detected frame.
 * @param frame The frame on which the lines are drawn.
 * @return Mat frame. 
 */
Mat detect_draw_lanes(Mat src_res, Mat frame)
{
	Mat gray, equalize, hls, hls_filter, blur, edge;
	Mat white, yellow, hello, mask;
	
	//Convert the image to grayscale
	cvtColor(src_res, gray, CV_RGB2GRAY);
	imshow("Gray Image", gray);
	
	//Increase Contrast
	equalizeHist(gray, equalize);
	imshow("Contrast Image", equalize);
	
	//Convert Original Image to HSL
	cvtColor(src_res, hls, COLOR_BGR2HLS);
	imshow("HLS", hls);

//	inRange(hls, Scalar(0,50,0), Scalar(0,255,0), white);
//	imshow("HLS white", white);
	
//	inRange(hls, Scalar(0,63,48), Scalar(62,74,57), white);
//	imshow("HLS Yellow", yellow);
	
//	bitwise_or(white, yellow, mask)
//	imshow("Mask", mask);
	
//	bitwise_and(equalize, white, hello);
//	imshow("Hello", hello);
	//inRange(hls, Scalar());
	
	
	
	
	
	//Applying gaussian filter to reduce noise followed by canny transform for edge detection.
	GaussianBlur( gray, blur, Size(5,5), 0, 0, BORDER_DEFAULT );
	Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);	//Can change to false
			
	//Another method is to split and just obtain the Green Channel.
	//split(src, bgr);
	//GaussianBlur( bgr[1], blur, Size(5,5), 0, 0, BORDER_DEFAULT )
	//Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);
	
	
	//Crop the image over here
	
	
	//Detect and Draw Lines
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
Mat detect_draw_vehicle(Mat edge, Mat frame)
{
	vector<Rect> vehicle;
	
	vehicle_cascade.detectMultiScale(edge, vehicle);
	
	for ( size_t i = 0; i < vehicle.size(); i++ )
    {
		//vehicle[i].y += 359;
		//vehicle[i].x += 359;
		rectangle(frame, vehicle[i], CV_RGB(255, 0, 0) );		
    }

	return frame;
}




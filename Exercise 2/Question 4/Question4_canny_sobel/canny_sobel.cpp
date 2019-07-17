#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <syslog.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
//using namespace std;

//Function Declaration
void analysis(void);

//Global Variables for Analysis
struct timeval timestamp;
struct timeval start;
struct timeval stop;
struct timeval prev_start;
int time_count;
int count;
int frame_sum;
int avg;
int ignore;
int worst_frame_rate;

char k;

//Global Variables for canny Transform
Mat src, src_gray;
Mat dst, detected_edges;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Canny/Sobel Transform";



//Global Variables for sobel Transform
Mat grad;
int scale = 1;
int delta = 0;
int ddepth = CV_16S;


/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  imshow( window_name, dst );
 }



int main( int argc, char** argv )
{
	char x;
	printf("Enter 'C' or 'c' for Canny Transform.\n");
	printf("Enter 'S' or 's' for Sobel Transform.\n");
	scanf("%c", &x);

	//Initializing Global Variables
	time_count = 0;
	count = 0;
	frame_sum = 0;
	ignore = 2;
	worst_frame_rate = 31;

	CvCapture* capture = cvCreateCameraCapture(0);
	IplImage* frame;


	while(1)
	{		
		if( x == 'C' || x == 'c' )
	    {
			analysis();
						
	        frame = cvQueryFrame(capture);
			if(!frame) break;
			
			src = cvarrToMat(frame);
			
			/// Create a matrix of the same type and size as src (for dst)
			dst.create( src.size(), src.type() );

			/// Convert the image to grayscale
			cvtColor( src, src_gray, CV_BGR2GRAY );


			/// Create a Trackbar for user to enter threshold
			createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

			/// Show the image
			CannyThreshold(0, 0);

			/// Wait until user exit program by pressing a key  
	
	        k = cvWaitKey(33);
	        if( k == 27 ) break;
	    }
	    else if( x == 'S' || x == 's')
	    {
			int c;

			analysis();
						
	        frame = cvQueryFrame(capture);
			if(!frame) break;
			
			src = cvarrToMat(frame);
			
			GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
	
			/// Convert it to gray
			cvtColor( src, src_gray, CV_RGB2GRAY );
	
			/// Create window
			namedWindow( window_name, CV_WINDOW_AUTOSIZE );
	
			/// Generate grad_x and grad_y
			Mat grad_x, grad_y;
			Mat abs_grad_x, abs_grad_y;
			 
			/// Gradient X
			//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
			Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );   
			convertScaleAbs( grad_x, abs_grad_x );
	
			/// Gradient Y 
			//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
			Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );   
			convertScaleAbs( grad_y, abs_grad_y );
	
			/// Total Gradient (approximate)
			addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
	
			imshow( window_name, grad );

			/// Wait until user exit program by pressing a key  
	
	        k = cvWaitKey(33);
	        if( k == 27 ) break;
	      
		}    
	}  
	cvReleaseCapture(&capture);
    cvDestroyWindow(window_name);
}



void analysis(void)
{	
	//Note Timestamp for frame capture
	gettimeofday(&start, NULL);
	
	//Count number of frames.
	if(start.tv_sec == prev_start.tv_sec)
	{
		count++;
	}
	else
	{
		//syslog(LOG_CRIT,"Time Count = %d", time_count);
		syslog(LOG_CRIT,"Frame Count = %d", count+1);
		
		//keeping the count of number of seconds
		time_count++;
		
		//Ignoring the first set because it might contain inaccurate frame count because of program startin inmidway of a second.
		if(!ignore)
		{
			//Calculating worst frame rate
			//count+1 because counting starts from 0
			if((count+1) < worst_frame_rate)
			{
				worst_frame_rate = (count+1);
			}
			
			//Calculating sum of frame count in every second to evaluate average over 60 sconds.
			frame_sum += (count+1);			
			//syslog(LOG_CRIT,"FRAME_SUM %d", frame_sum);
		}
		
		//Resettng the flag and count number.
		if (ignore > 0)
		{
			ignore = ignore - 1;
		}
		count = 0;
	}
	
	//Evaluating Average frame rate at the end of 60 seconds
	if(time_count == 61)
	{
		avg = frame_sum/60;
		syslog(LOG_CRIT,"Ideal number of frames in 60 seconds is 1800");			
		syslog(LOG_CRIT,"Total frames in 60 seconds = %d", frame_sum);			
		syslog(LOG_CRIT,"Average Frame Rate = %d", avg);
		syslog(LOG_CRIT,"Worst Frame Rate = %d", worst_frame_rate);
		syslog(LOG_CRIT,"Jitter = %d frames which is %d usecs", 1800-frame_sum, ((1800-frame_sum)*1000000)/30);					
		exit(1);
	}
	
	//Save to compare in next iteration
	prev_start = start;
	
	
	//Print Timestamp
	syslog(LOG_CRIT, "Frame Timestamp sec=%d, usec=%d\n", (int)(start.tv_sec), (int)start.tv_usec);
	
}


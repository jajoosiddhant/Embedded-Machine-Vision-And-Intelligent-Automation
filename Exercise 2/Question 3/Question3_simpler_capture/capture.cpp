/*
 *
 *  Example by Sam Siewert 
 *
 */
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>
#include <syslog.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;



int main( int argc, char** argv )
{
    cvNamedWindow("Capture Example", CV_WINDOW_AUTOSIZE);
    CvCapture* capture = cvCreateCameraCapture(0);
    IplImage* frame;
	struct timeval timestamp;
	struct timeval start;
	struct timeval stop;
	struct timeval prev_start;
	int time_count = 0;
	int count = 0;
	int frame_sum = 0;
	int avg;
	int ignore = 2;
	int worst_frame_rate = 31;
	
    while(1)
    {
		//Note Timestamp for frame capture
		gettimeofday(&start, NULL);
		
		//Save to compare in next iteration
		//prev_start = start;
		
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
			
			//Ignoring the first set because it might contain inaccurate frame count because of program starting in midway of a second.
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
		
		
		
        frame=cvQueryFrame(capture);
		
		     
        if(!frame) break;

        cvShowImage("Capture Example", frame);

        char c = cvWaitKey(33);
        if( c == 27 ) break;
    }

    cvReleaseCapture(&capture);
    cvDestroyWindow("Capture Example");
    
};

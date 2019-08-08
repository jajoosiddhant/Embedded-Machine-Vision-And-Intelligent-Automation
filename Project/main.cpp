#include "main.h"
/*
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <signal.h>
#include <semaphore.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"

using namespace cv;
using namespace std;

#define COLS	(320)
#define ROWS	(240)
#define NSEC_PER_SEC	(1000000000)

#define PED_DETECT_TH	(0)
#define LANE_FOLLOW_TH	(1)
#define SIGN_RECOG_TH	(2)
#define VEH_DETECT_TH	(3)

#define NUM_THREADS	(4)

bool exit_cond, done;
char output_frame[40];
int iterations, cpucore, g_frame_cnt, numberOfProcessors;

struct timespec g_start_time, g_stop_time, g_diff_time;
typedef struct
{
    int threadIdx;
} threadParams_t;

pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];

Mat g_frame;

//int delta_t(struct timespec *, struct timespec *, struct timespec *);

//void* pedestrian_detect(void*);
//void* lane_follower(void*);
//void* sign_recog(void*);
//void* vehicle_detect(void*);
*/

void signal_handler(int signo, siginfo_t *info, void *extra)
{
	exit_cond = true;
}

void set_signal_handler(void)
{
	struct sigaction action;

	action.sa_flags = SA_SIGINFO;
	action.sa_sigaction = signal_handler;

	if(sigaction(SIGINT, &action, NULL) == -1)
	{
		printf("Error calling signal handler\n");
		exit(EXIT_FAILURE);
	}
}

/* Function for computing time difference between two input timespec structures and saving in third timespec structure.
 * Reference is provided to seqgen.c by Prof. Sam Siewert for delta_t function 
 * */
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
	int dt_sec=stop->tv_sec - start->tv_sec;
	int dt_nsec=stop->tv_nsec - start->tv_nsec;

	if(dt_sec >= 0)
	{
		if(dt_nsec >= 0)
		{
			delta_t->tv_sec=dt_sec;
			delta_t->tv_nsec=dt_nsec;
		}
		else
		{
			delta_t->tv_sec=dt_sec-1;
			delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
		}
	}
	else
	{
		if(dt_nsec >= 0)
		{
			delta_t->tv_sec=dt_sec;
			delta_t->tv_nsec=dt_nsec;
		}
		else
		{
			delta_t->tv_sec=dt_sec-1;
			delta_t->tv_nsec=NSEC_PER_SEC+dt_nsec;
		}
	}
	return(1);
}


void* pedestrian_detect(void* threadp)
{
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;

	Mat mat, resz_mat;
//	vector<Rect> found_loc;	
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
			
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);
	

	while(1)
	{
		sem_wait(&sem_main);
		//semaphore from main
		mat = g_frame.clone();
		cvtColor(mat, mat, CV_BGR2GRAY);
		resize(mat, resz_mat, Size(COLS, ROWS));
		//	resize(mat, resz_mat, Size(800, 533));

		hog.detectMultiScale(resz_mat, img_char.found_loc, 0, Size(8, 8), Size(0, 0), 1.05, 2, false);

		//cout << "Hello Pedestrians" << endl;
		frame_cnt++;
		sem_post(&sem_pedestrian);
//		for(int i=0; i<found_loc.size(); i++)
//		{
//			rectangle(resz_mat, found_loc[i], (0, 0, 255), 4);
//		}

//		imshow("Detector", resz_mat);
//		imshow("Video", mat);


//		c = cvWaitKey(10);
		if((c == 27) || (exit_cond))
		{
			break;
		}
	}

	//Calculating FPS for pedestrian detection
	fps_calc(start_time, frame_cnt, FPS_PEDESTRIAN);

	pthread_exit(NULL);
}


void* lane_follower(void* threadp)
{
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;
		
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);

	while(1)
	{
		sem_wait(&sem_main);

	//	cout << "Hello lane detect" << endl;

		frame_cnt++;

		sem_post(&sem_lane);

		usleep(50000);
		
		if((c == 27) || (exit_cond))
		{
			break;
		}

	}
	
	//Calculating FPS for lane detection
	fps_calc(start_time, frame_cnt, FPS_LANE);
	
	pthread_exit(NULL);

}


void* sign_recog(void* threadp)
{
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;
		
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);


	for(int i=0; i<250; i++)
	{
		i++;
		if((c == 27) || (exit_cond))
		{
			break;
		}
	}
	
	//Calculating FPS for sign detection
	fps_calc(start_time, frame_cnt, FPS_SIGN);

	pthread_exit(NULL);

}


void* vehicle_detect(void* threadp)
{
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;
		
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);

	for(int i=0; i<250; i++)
	{
		i++;
		if((c == 27) || (exit_cond))
		{
			break;
		}
	}
	
	//Calculating FPS for sign detection
	fps_calc(start_time, frame_cnt, FPS_VEHICLE);

	pthread_exit(NULL);

}

int main(int argc, char** argv)
{
	int frame_cnt = 0;
	struct timespec start_time;
	exit_cond = false;
	
	if(argc < 2)
	{
		cout << endl << "Usage: ./smart_car input_vdeo_file" << endl;
		cout << "Exiting Program";
		exit(EXIT_FAILURE);
	}

	//Declaring VideoCapture and VideoWriter objects to read and write videos
	VideoCapture capture(argv[1]);
//	VideoWriter output_v;
//	Size size = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH)/2,
//			 (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);		//Size of capture object, height and width
//	output_v.open("output.avi", CV_FOURCC('M','P','4','V'), cap.get(CV_CAP_PROP_FPS), size, true);	//Opens output object

	
	Mat detector;

	set_signal_handler();

	//Initializing Semaphores
	sem_create();

	//Setting up core affinity for different services
	thread_core_set();

	//Creatintg 4 threads for individual services.
	thread_create();
		      
	clock_gettime(CLOCK_REALTIME, &start_time);

	while(1)
	{
		capture >> g_frame;

		sem_post(&sem_main);
		sem_post(&sem_main);
		//sem_post(&sem_main);
		//sem_post(&sem_main);
		
		detector = g_frame.clone();
		resize(detector, detector, Size(COLS, ROWS));



		for(int i=0; i<img_char.found_loc.size(); i++)
		{
			rectangle(detector, img_char.found_loc[i], (0, 0, 255), 4);
		}
		
		//Counting number of frames
		frame_cnt++;
		
		sem_wait(&sem_pedestrian);
		sem_wait(&sem_lane);
		//sem_wait(&sem_vehicle);
		//sem_wait(&sem_sign);
		
		
		c = waitKey(33);
		if((c == 27) || (exit_cond))
		{
			break;
		}

		imshow("Video", g_frame);
		imshow("Detector", detector);
	}
	
	//Calculating Average FPS
	fps_calc(start_time, frame_cnt, FPS_SYSTEM);

	//Joining threads
	for(int i=0;i<NUM_THREADS;i++)
	{
		pthread_join(threads[i], NULL);
	}
	
	cout << "Exiting program" << endl;


	//Destroying all Semaphores
	sem_destroy_all();
	
	return 0;
}


/**
 * @brief This function creates semaphores for the required threads.
 * @param void
 * @return void
 */
void sem_create(void)
{
	//Initializing Semaphores
	if(sem_init(&sem_main, 0, 0) == -1)
		handle_error("ERROR: sem_init for sem_main");
	if(sem_init(&sem_pedestrian, 0, 0) == -1)
		handle_error("ERROR: sem_init for sem_pedestrian");
	if(sem_init(&sem_lane, 0, 0) == -1)
		handle_error("ERROR: sem_init for sem_lane");
	if(sem_init(&sem_vehicle, 0, 0) == -1)
		handle_error("ERROR: sem_init for sem_vehicle");
	if(sem_init(&sem_sign, 0, 0) == -1)
		handle_error("ERROR: sem_init for sem_sign");	
}


/**
 * @brief This function sets core affinity to individual required services.
 * @param void
 * @return void
 */
void thread_core_set(void)
{
	cpu_set_t allcpuset;
	cpu_set_t threadcpu;
	
	cout << "This system has " << get_nprocs_conf() << " processors configured and " << get_nprocs() << " processors available" << endl << endl;
	
	CPU_ZERO(&allcpuset);
	for(int i=0; i < get_nprocs_conf(); i++)
	{
		CPU_SET(i, &allcpuset);
	}
	
	for(int i=0; i < NUM_THREADS; i++)
	{
		int rc, coreid;
		
		//Setting individual cores to individual services
		CPU_ZERO(&threadcpu);
		coreid=i%get_nprocs_conf();
		cout << "Setting thread " << i << " to core " << coreid << endl;
		CPU_SET(coreid, &threadcpu);
		for(int idx=0; idx<get_nprocs_conf(); idx++)
		{
			if(CPU_ISSET(idx, &threadcpu))  
			{
				cout << " CPU-" << idx << endl;
			}
		}
		cout << "Launching thread " << i << endl << endl;

		//handle error condition here
		rc=pthread_attr_init(&rt_sched_attr[i]);
		rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

		threadParams[i].threadIdx=i;
	}

}


/**
 * @brief This function creates threads for the required services.
 * @param void
 * @return void
 */
void thread_create(void)
{
	//Pedestrian Detection Thread
	
	pthread_create(&threads[PED_DETECT_TH],   							// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[PED_DETECT_TH]),     		// use default attributes
			pedestrian_detect, 											// thread function entry point
			(void *)&(threadParams[PED_DETECT_TH]) 						// parameters to pass in
		      );


	//Lane Detection Thread
	pthread_create(&threads[LANE_FOLLOW_TH],   							// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[LANE_FOLLOW_TH]),     		// use default attributes
			lane_follower, 												// thread function entry point
			(void *)&(threadParams[LANE_FOLLOW_TH]) 					// parameters to pass in
		      );


	//Sign Detection Thread
	pthread_create(&threads[SIGN_RECOG_TH],   							// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[SIGN_RECOG_TH]),     		// use default attributes
			sign_recog, 												// thread function entry point
			(void *)&(threadParams[SIGN_RECOG_TH]) 						// parameters to pass in
		      );

	//Vehicle Detection Thread
	pthread_create(&threads[VEH_DETECT_TH],   							// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[VEH_DETECT_TH]),     		// use default attributes
			vehicle_detect, 											// thread function entry point
			(void *)&(threadParams[VEH_DETECT_TH]) 						// parameters to pass in
		      );

}


/**
 * @brief This function prints individual thread CPU information.
 * @param threadParams The parameter passed while creating pthread.
 * @return void
 */
void threadcpu_info(threadParams_t* threadParams)
{
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
	cout << "Thread " << threadParams->threadIdx << " initialized" << endl;
	cout << "Thread idx=" << threadParams->threadIdx << " running on core " << sched_getcpu() << ", affinity contained:" << endl;
	cout << "With PID = " << syscall(SYS_gettid) << endl;
	for(int i=0; i<get_nprocs_conf(); i++)
	{
		if(CPU_ISSET(i, &cpuset))
		{
			cout << " CPU-" << i << endl;
		}
	}
	
}


/**
 * @brief This function prints the average FPS of the specified thread.
 * @param start The start time of the particular thread.
 * @param fps_thread The thread whose fps needs to be calculated. Substitute Macros defined in main.h
 * @param frame_cnt The number of frames processed.
 * @return void
 */
void fps_calc(struct timespec start_time, int frame_cnt, uint8_t fps_thread)
{
	struct timespec stop_time, diff_time;
	
	switch(fps_thread)
	{
		case FPS_SYSTEM:
		{
			exit_cond = 1;
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "OVERALL FPS:" << endl;
			cout << "Number of frames: "<< frame_cnt << endl;
			cout << "Duration: "<< diff_time.tv_sec << endl;
			cout << "Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			sem_post(&sem_main);
			sem_post(&sem_main);
			sem_post(&sem_main);
			sem_post(&sem_main);
			break;
		}

		case FPS_PEDESTRIAN:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "PEDESTRIAN FPS:" << endl;
			cout << "Number of frames: "<< frame_cnt << endl;
			cout << "Duration: "<< diff_time.tv_sec << endl;
			cout << "Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			break;
		}
		
		case FPS_LANE:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "LANE FPS:" << endl;
			cout << "Number of frames: "<< frame_cnt << endl;
			cout << "Duration: "<< diff_time.tv_sec << endl;
			cout << "Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			break;
		}
		
		case FPS_VEHICLE:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "VEHICLE FPS:" << endl;
			cout << "Number of frames: "<< frame_cnt << endl;
			cout << "Duration: "<< diff_time.tv_sec << endl;
			cout << "Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			break;
		}
		
		case FPS_SIGN:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "SIGN FPS:" << endl;
			cout << "Number of frames: "<< frame_cnt << endl;
			cout << "Duration: "<< diff_time.tv_sec << endl;
			cout << "Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			break;
		}
	}
	
}


/**
 * @brief This function destroys the created semaphores.
 * @param void
 * @return void
 */
void sem_destroy_all(void)
{
	sem_destroy(&sem_main);
	sem_destroy(&sem_pedestrian);
	sem_destroy(&sem_lane);
}

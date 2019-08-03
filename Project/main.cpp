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
		exit(1);
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
	char c;
	int frame_cnt = 0;
	struct timespec start_time, stop_time, diff_time;
	cpu_set_t cpuset;
	pthread_t thread;
	pthread_mutex_t mutex;
	
	Mat mat, resz_mat;
//	vector<Rect> found_loc;
	
	threadParams_t* threadParams = (threadParams_t*)threadp;
	printf("Thread %d initialized\n", threadParams->threadIdx);

	thread = pthread_self();
	cpucore = sched_getcpu();
	CPU_ZERO(&cpuset);
	pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
	
	printf("Thread idx=%d running on core %d, affinity contained:", threadParams->threadIdx, cpucore);
	for(int i=0; i<numberOfProcessors; i++)
		if(CPU_ISSET(i, &cpuset))  printf(" CPU-%d ", i);
	printf("\n");
	
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	
    	clock_gettime(CLOCK_REALTIME, &start_time);
	while(1)
	{
		sem_wait(&th1_sem);
		//semaphore from main
		mat = g_frame.clone();
		resize(mat, resz_mat, Size(COLS, ROWS));
		//	resize(mat, resz_mat, Size(800, 533));

		hog.detectMultiScale(resz_mat, img_char.found_loc, 0, Size(4, 4), Size(8, 8), 1.05, 2, false);

		sem_post(&main_sem);
//		for(int i=0; i<found_loc.size(); i++)
//		{
//			rectangle(resz_mat, found_loc[i], (0, 0, 255), 4);
//		}

//		imshow("Detector", resz_mat);
//		imshow("Video", mat);
		frame_cnt++;

		c = cvWaitKey(10);
		if((c == 27) || (exit_cond))
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);
			printf("\nPedestrian thread number of frames: %d\n", frame_cnt);
			printf("Pedestrian duration: %ld seconds\n", diff_time.tv_sec);
			printf("Pedestrian average FPS: %ld\n", (frame_cnt/diff_time.tv_sec));
			printf("Exiting thread %d\n", threadParams->threadIdx);
			break;
		}
	}
}


void* lane_follower(void* threadp)
{
	for(int i=0; i<250; i++)
	{
		i++;
	}
	pthread_exit(NULL);

}


void* sign_recog(void* threadp)
{
	for(int i=0; i<250; i++)
	{
		i++;
	}
	pthread_exit(NULL);

}


void* vehicle_detect(void* threadp)
{
	for(int i=0; i<250; i++)
	{
		i++;
	}
	pthread_exit(NULL);

}

int main(int argc, char** argv)
{
	int rc, coreid;
	char c;
	cpu_set_t allcpuset;
	cpu_set_t threadcpu;

	useconds_t usec = 10;
	g_frame_cnt = 0;
	VideoCapture capture(argv[1]);

	Mat detector;

	set_signal_handler();
	exit_cond = false;

	if(sem_init(&th1_sem, 0, 0) == -1)
		handle_error("sem_init");
	if(sem_init(&main_sem, 0, 0) == -1)
		handle_error("sem_init");

//	VideoWriter output_v;
//	Size size = Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH)/2,
//			 (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT)/2);		//Size of capture object, height and width
//	output_v.open("output.avi", CV_FOURCC('M','P','4','V'), cap.get(CV_CAP_PROP_FPS), size, true);	//Opens output object

	numberOfProcessors = get_nprocs_conf(); 
	printf("This system has %d processors configured and %d processors available.\n\n", numberOfProcessors, get_nprocs());

	CPU_ZERO(&allcpuset);

	for(int i=0; i < numberOfProcessors; i++)
		CPU_SET(i, &allcpuset);


	clock_gettime(CLOCK_REALTIME, &g_start_time);

	for(int i=0; i < NUM_THREADS; i++)
	{
		CPU_ZERO(&threadcpu);
		coreid=i%numberOfProcessors;
		printf("\nSetting thread %d to core %d ", i, coreid);
		CPU_SET(coreid, &threadcpu);
		for(int idx=0; idx<numberOfProcessors; idx++)
			if(CPU_ISSET(idx, &threadcpu))  
				printf(" CPU-%d ", idx);
		printf("\nLaunching thread %d\n", i);

		rc=pthread_attr_init(&rt_sched_attr[i]);
		rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);

		threadParams[i].threadIdx=i;
	}


	pthread_create(&threads[PED_DETECT_TH],   // pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[PED_DETECT_TH]),     // use default attributes
			pedestrian_detect, // thread function entry point
			(void *)&(threadParams[PED_DETECT_TH]) // parameters to pass in
		      );
	usleep(usec);

	pthread_create(&threads[LANE_FOLLOW_TH],   // pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[LANE_FOLLOW_TH]),     // use default attributes
			lane_follower, // thread function entry point
			(void *)&(threadParams[LANE_FOLLOW_TH]) // parameters to pass in
		      );
	usleep(usec);
	
	pthread_create(&threads[SIGN_RECOG_TH],   // pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[SIGN_RECOG_TH]),     // use default attributes
			sign_recog, // thread function entry point
			(void *)&(threadParams[SIGN_RECOG_TH]) // parameters to pass in
		      );
	usleep(usec);

	pthread_create(&threads[VEH_DETECT_TH],   // pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[VEH_DETECT_TH]),     // use default attributes
			vehicle_detect, // thread function entry point
			(void *)&(threadParams[VEH_DETECT_TH]) // parameters to pass in
		      );
	usleep(usec);

	while(1)
	{
		capture >> g_frame;

		sem_post(&th1_sem);
		detector = g_frame.clone();
		resize(detector, detector, Size(COLS, ROWS));
		//semaphores for threads

		sem_wait(&main_sem);
		g_frame_cnt++;
		for(int i=0; i<img_char.found_loc.size(); i++)
		{
			rectangle(detector, img_char.found_loc[i], (0, 0, 255), 4);
		}
		
		c = cvWaitKey(33);
		if((c == 27) || (exit_cond))
		{
			clock_gettime(CLOCK_REALTIME, &g_stop_time);
			delta_t(&g_stop_time, &g_start_time, &g_diff_time);		//Obtain time difference
			printf("\nNumber of frames: %d\n", g_frame_cnt);
			printf("Duration: %ld seconds\n", g_diff_time.tv_sec);
			printf("Average FPS: %ld\n", (g_frame_cnt/g_diff_time.tv_sec));
			break;
		}
		imshow("Video", g_frame);
		imshow("Detector", detector);
	}
	for(int i=0;i<NUM_THREADS;i++)
		pthread_join(threads[i], NULL);
	return 0;
}

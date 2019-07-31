#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <signal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

#define NSEC_PER_SEC	(1000000000)
#define THRESHOLD	(100)
#define MAX_THRESHOLD	(255)
#define NUM_THREADS	(4)

int frame_cnt, numberOfProcessors;
bool exit_cond;
char output_frame[40];
IplImage* capture_frame;
CvCapture* capture; 
//VideoCapture cap(0);			//Capture object is camera

typedef struct
{
    int threadIdx;
} threadParams_t;

// POSIX thread declarations and scheduling attributes
//
pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];
pthread_mutex_t mutex;

void signal_handler(int, siginfo_t *, void *);
void set_signal_handler(void);
int delta_t(struct timespec *, struct timespec *, struct timespec *);

void* processing_th(void*);


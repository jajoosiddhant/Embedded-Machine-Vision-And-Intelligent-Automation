#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <signal.h>

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

int g_frame_cnt, numberOfProcessors;
bool exit_cond;
typedef struct
{
    int threadIdx;
} threadParams_t;

pthread_mutex_t mutex;

Mat g_frame;

int delta_t(struct timespec *, struct timespec *, struct timespec *);

void* pedestrian_detect(void*);
void* lane_follower(void*);
void* sign_recog(void*);
void* vehicle_detect(void*);


bool done;
char c;
int iterations, cpucore, frame_cnt;
vector<Rect> found_loc;
struct timespec start_time, stop_time, diff_time;

pthread_t thread;
cpu_set_t cpuset;

Mat mat, resize_mat;


pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];

void signal_handler(int, siginfo_t *, void *);
void set_signal_handler(void);

#endif

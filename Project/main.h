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


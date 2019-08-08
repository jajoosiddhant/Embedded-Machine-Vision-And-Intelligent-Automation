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
#include <sys/syscall.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"

using namespace cv;
using namespace std;

#define COLS								(320)
#define ROWS								(240)
#define NSEC_PER_SEC						(1000000000)

//Waitkey Keys
#define ESC									(27)

#define PED_DETECT_TH						(0)
#define LANE_FOLLOW_TH						(1)
#define SIGN_RECOG_TH						(2)
#define VEH_DETECT_TH						(3)
#define NUM_THREADS							(4)

//FPS calculation Macros
#define FPS_PEDESTRIAN						(1)
#define FPS_LANE							(2)
#define FPS_VEHICLE							(3)
#define FPS_SIGN							(4)
#define FPS_SYSTEM							(5)


#define handle_error(msg) \
	{ \
		perror(msg); \
		exit(EXIT_FAILURE); \
	} \

typedef struct
{
    int threadIdx;
} threadParams_t;


struct img_cooordinates
{
	vector<Rect> found_loc;
} img_char;

pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];

bool exit_cond;
char c, output_frame[40];
Mat g_frame;
sem_t sem_main, sem_pedestrian, sem_lane, sem_vehicle, sem_sign;


//Function Declarations
void sem_create(void);
void sem_destroy_all(void);
void thread_create(void);
void threadcpu_info(threadParams_t* threadParams);
void thread_core_set(void);
void fps_calc(struct timespec start, int frame_cnt, uint8_t fps_thread);
//int delta_t(struct timespec *, struct timespec *, struct timespec *);

//void* pedestrian_detect(void*);
//void* lane_follower(void*);
//void* sign_recog(void*);
//void* vehicle_detect(void*);


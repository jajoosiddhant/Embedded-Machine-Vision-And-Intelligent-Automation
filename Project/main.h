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
#include <X11/Xlib.h>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"

using namespace cv;
using namespace std;

//#define USING_CALTECH_VIDEO

#ifdef USING_CALTECH_VIDEO
#define COLS							(320)
#define ROWS							(240)
#endif

#ifndef USING_CALTECH_VIDEO
#define COLS							(320)
#define ROWS							(180)
#endif

#define NSEC_PER_SEC						(1000000000)

//Waitkey Keys
#define ESC							(27)

#define PED_DETECT_TH						(0)
#define LANE_FOLLOW_TH						(1)
#define SIGN_RECOG_TH						(2)
#define VEH_DETECT_TH						(3)
#define NUM_THREADS						(4)

//FPS calculation Macros
#define FPS_PEDESTRIAN						(1)
#define FPS_LANE						(2)
#define FPS_VEHICLE						(3)
#define FPS_SIGN						(4)
#define FPS_SYSTEM						(5)

//Sides
#define LEFT							(1)
#define RIGHT							(2)

//Canny Threshold Values
#define CANNY_THRESHOLD_1					(40)
#define CANNY_THRESHOLD_2					(120)

//Hough Lines Threshold Values
#define	HOUGH_THRESHOLD						(20)
#define	HOUGH_MIN_LINE_LENGTH					(10)
#define	HOUGH_MAX_LINE_GAP					(50)


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
	vector<Rect> traffic;
	vector<Rect> found_loc;				//Rectangle Coordinates for pedestrian
	vector<Rect> vehicle_loc;				//Rectangle Coordinates for Vehicle
	Vec4i g_left;
	Vec4i g_right;
} img_char;


//Variable Declarations
pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];
struct sched_param rt_param[NUM_THREADS];
int rt_max_prio, rt_min_prio;
struct sched_param main_param;
pthread_attr_t main_attr;

int enable[4] = {0};
bool exit_cond;
char c, output_frame[40];
Mat g_frame;
sem_t sem_main, sem_pedestrian, sem_lane, sem_vehicle, sem_sign;
mutex mute_ped, mute_lane, mute_vehicle, mute_sign;

//For Vehicle Detection
CascadeClassifier vehicle_cascade;
const string vehicle_cascade_name("cars.xml");

//Function Declarations
void sem_create(void);
void sem_destroy_all(void);
void thread_create(void);
void threadcpu_info(threadParams_t* threadParams);
void thread_core_set(void);
void fps_calc(struct timespec start, int frame_cnt, uint8_t fps_thread);
void set_thread_attr(void);
void print_scheduler(void);
void* pedestrian_detect(void* threadp);
void* lane_follower(void* threadp);
void* sign_recog(void* threadp);
void* vehicle_detect(void* threadp);
int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t);
void signal_handler(int signo, siginfo_t *info, void *extra);
void set_signal_handler(void);
void print_scope(void);
void help(void);

//Functions related to lane detect. Copy these in master.
Mat preprocess(Mat src);
Mat equalize(Mat src_half);
Mat create_mask(Mat src_half);
Mat detect_lanes(Mat contrast, Mat mask, Mat roi_mask);
Mat roi_mask(Mat src_half);
void process_lanes(vector<Vec4i> lane, int side);

#include "main.h"

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


void* processing_th(void* threadp)
{
	bool done;
	int iterations=0, cpucore;
	char c;
	pthread_t thread;
	cpu_set_t cpuset;
	
	Mat src, gray, binary, mfblur,temp, eroded, RGB_skel;
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
	
	while(1)
	{
		pthread_mutex_lock(&mutex);
		capture_frame = cvQueryFrame(capture);
		pthread_mutex_unlock(&mutex);
		src = cvarrToMat(capture_frame);
//		imshow("Video Stream", src);		// show original source image and wait for input to next step

		cvtColor(src, gray, CV_BGR2GRAY);	// show graymap of source image and wait for input to next step
//		imshow("graymap 1", gray);

		threshold(gray, binary, THRESHOLD, MAX_THRESHOLD, CV_THRESH_BINARY);	// show bitmap of source image and wait for input to next step

		binary = MAX_THRESHOLD - binary;			//Perform image subtraction sinceforeground is generally darker than background
//		imshow("binary 1", binary);

		medianBlur(binary, mfblur, 3);		// To remove median filter, just replace blurr value with 1
//		imshow("medianblur 1", mfblur);

		/*This section of code was adapted from the following post, which was based in turn on the Wikipedia description of a morphological skeleton 
		 * http://felix.abecassis.me/2011/09/opencv-morphological-skeleton */

		Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
		Mat skel(mfblur.size(), CV_8UC1, Scalar(0));
		iterations = 0;
		do
		{
			erode(mfblur, eroded, element);		//Erode boundary edges of object
			dilate(eroded, temp, element);		//Extend original object to compensate for loss of data
			subtract(mfblur, temp, temp);		//Subtract to obtain only increased sections of object
			bitwise_or(skel, temp, skel);		//Save increased sections of each iteration in skel
			eroded.copyTo(mfblur);

			done = (countNonZero(mfblur) == 0);	//Check for completely black image
			iterations++;

		} while (!done && (iterations < 100));
		cvtColor(skel, RGB_skel, CV_GRAY2BGR);		// show graymap of source image and wait for input to next step
		sprintf(output_frame, "./frames/frame%d.jpg", frame_cnt);
		frame_cnt++;
//		printf("In thread %d \n", threadParams->threadIdx);

		imwrite(output_frame, RGB_skel);		//save output frames in folder

//		c = cvWaitKey(33);
		if((c==27) || (exit_cond))
		{
			printf("Exiting thread %d\n", threadParams->threadIdx);
			pthread_exit(NULL);
		}
	}
}


int main(int argc, char** argv)
{
	int rc, coreid;
	char c;
	struct timespec start_time, stop_time, diff_time;
	cpu_set_t allcpuset;
	cpu_set_t threadcpu;
	
	useconds_t usec = 10;
	frame_cnt = 0;
	
	capture = cvCreateCameraCapture(0);
//	if(!cap.isOpened())  			// check if we succeeded
//		return -1;

	set_signal_handler();
	exit_cond = false;

	numberOfProcessors = get_nprocs_conf(); 
	printf("This system has %d processors configured and %d processors available.\n\n", numberOfProcessors, get_nprocs());

	CPU_ZERO(&allcpuset);

	for(int i=0; i < numberOfProcessors; i++)
		CPU_SET(i, &allcpuset);


	clock_gettime(CLOCK_REALTIME, &start_time);

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

		pthread_create(&threads[i],   // pointer to thread descriptor
				(pthread_attr_t*)&(rt_sched_attr[i]),     // use default attributes
				processing_th, // thread function entry point
				(void *)&(threadParams[i]) // parameters to pass in
			      );
		usleep(usec);
	}
	printf("\nAll threads initialized\n");
	while(1)
	{
		c = cvWaitKey(33);
		if((c == 27) || (exit_cond))
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			printf("Number of frames: %d\n", frame_cnt);
			printf("Duration: %ld seconds\n", diff_time.tv_sec);
			printf("Average FPS: %ld\n", (frame_cnt/diff_time.tv_sec));
			break;
		}
	}
	for(int i=0;i<NUM_THREADS;i++)
		pthread_join(threads[i], NULL);
	return 0;
}

#include "include.h"

void* pedestrian_detect(void* threadp)
{
	iterations = 0;
	frame_cnt = 0;

	cvNamedWindow("Detector", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Video", CV_WINDOW_AUTOSIZE);
	
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
		pthread_mutex_lock(&mutex);
		mat = g_frame.clone();
		pthread_mutex_unlock(&mutex);
	
		resize(mat, resize_mat, Size(COLS, ROWS));
		//	resize(mat, resize_mat, Size(800, 533));

		hog.detectMultiScale(resize_mat, found_loc, 0, Size(4, 4), Size(8, 8), 1.05, 2, false);
		for(int i=0; i<found_loc.size(); i++)
		{
			rectangle(resize_mat, found_loc[i], (0, 0, 255), 4);
		}

		imshow("Detector", resize_mat);
		imshow("Video", mat);
		frame_cnt++;


//		c = cvWaitKey(33);
		if((c==27) || (exit_cond))
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);
			printf("PEDESTRIAN DETECTOR:\n");
			printf("Number of frames: %d\n", frame_cnt);
			printf("Duration: %ld seconds\n", diff_time.tv_sec);
			printf("Average FPS: %ld\n", (frame_cnt/diff_time.tv_sec));
			printf("Exiting thread %d\n\n", threadParams->threadIdx);
			pthread_exit(NULL);
		}
	}
}

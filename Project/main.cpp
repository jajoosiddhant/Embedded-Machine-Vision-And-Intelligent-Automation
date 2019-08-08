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
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
			
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);
	
	while(1)
	{
		sem_wait(&sem_main);	//semaphore from main
		mat = g_frame.clone();
		cvtColor(mat, mat, CV_BGR2GRAY);
		resize(mat, resz_mat, Size(COLS, ROWS));	//resize to 320x240

		hog.detectMultiScale(resz_mat, img_char.found_loc, 0, Size(8, 8), Size(0, 0), 1.05, 2, false);
		sem_post(&sem_pedestrian);

//		cout << "Hello Pedestrians" << endl;
		frame_cnt++;

		if((c == 27) || (exit_cond))
		{
			break;
		}
	}

	//Calculating FPS for pedestrian detection
//	fps_calc(start_time, frame_cnt, FPS_PEDESTRIAN);

	pthread_exit(NULL);
}


void* lane_follower(void* threadp)
{
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;
	Mat src;
	Mat src_half, contrast, mask;		
	Mat detect_lanes, blur, edge;
	Mat canny_roi;
	//Mat src_res;
	
	double slope;
	int x1, x2, y1, y2;
		
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);

	while(1)
	{
		sem_wait(&sem_main);
		src = g_frame.clone();
	
		//Preprocess frames
		src_half = preprocess(src);		
		
		//Return a contrast image
		contrast = equalize(src_half);
		
		//Returns a mask to detect lanes
		mask = create_mask(src_half);
		  
		
		//Creating Polygon ROI
		Point roi_pt[1][4];
		int num = 4;

		Mat roi_mask = Mat::zeros(Size(src_half.cols, src_half.rows), CV_8U);
		//Points for ROI mask
		roi_pt[0][0] = Point(2*src_half.cols/5, src_half.rows/5);					//Apex
		roi_pt[0][1] = Point(3*src_half.cols/5, src_half.rows/5);
		roi_pt[0][3] = Point(src_half.cols/5, src_half.rows);						//Bottom left vertice
		roi_pt[0][2] = Point(4*src_half.cols/5 , src_half.rows);			//Bottomk right vertice
		const Point* pts_list[1] = {roi_pt[0]};
		fillPoly(roi_mask, pts_list, &num, 1, 255, 8);						//Change to fillConvexPolly for faster results
	

		//Detect lanes
		vector<Vec4i> lines;
		vector<Vec4i> left;
		vector<Vec4i> right;
		
		bitwise_and(contrast, mask, detect_lanes);
//		imshow("lanes Detected", detect_lanes);
		
		//Applying gaussian filter to reduce noise followed by canny transform for edge detection.
		GaussianBlur( detect_lanes, blur, Size(5,5), 0, 0, BORDER_DEFAULT );
		Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);	//Can change to false

		bitwise_and(edge, roi_mask, canny_roi);
		imshow("Canny Mask", canny_roi);
		
		//Detect and Draw Lines
		HoughLinesP(canny_roi, lines, 1, CV_PI/180, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP );
		for( size_t i = 0; i < lines.size(); i++ )
		{
			Vec4i l = lines[i];
			
			slope = (double)((l[3] - l[1])/(double)(l[2] - l[0]));
			if (slope > 0.5)
			{
				right.push_back(lines[i]);
			}				
			else if (slope < (-0.5))
			{
				left.push_back(lines[i]);
			}
		}
				
		process_lanes(left, LEFT);
		process_lanes(right, RIGHT);


		frame_cnt++;
		sem_post(&sem_lane);

		if((c == 27) || (exit_cond))
		{
			break;
		}

	}
	
	//Calculating FPS for lane detection
//	fps_calc(start_time, frame_cnt, FPS_LANE);
	
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
//	fps_calc(start_time, frame_cnt, FPS_SIGN);

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
//	fps_calc(start_time, frame_cnt, FPS_VEHICLE);

	pthread_exit(NULL);

}

int main(int argc, char** argv)
{
	int frame_cnt = 0;
	struct timespec start_time;
	exit_cond = false;
	Mat detector;
	
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

		//Post semaphore 4 times, once for each of the 4 threads
		sem_post(&sem_main);
		sem_post(&sem_main);
		//sem_post(&sem_main);
		//sem_post(&sem_main);
		
		
		//Counting number of frames
		frame_cnt++;
		
		sem_wait(&sem_pedestrian);
		sem_wait(&sem_lane);
		//sem_wait(&sem_vehicle);
		//sem_wait(&sem_sign);

		detector = g_frame.clone();
		resize(detector, detector, Size(COLS, ROWS));
		for(int i=0; i<img_char.found_loc.size(); i++)
		{
			rectangle(detector, img_char.found_loc[i], (0, 0, 255), 4);
		}


//		pyrDown(g_frame, g_frame_res);
//		line( g_frame_res, Point(img_char.g_left[0], img_char.g_left[1] + g_frame_res.rows/2), Point(img_char.g_left[2], img_char.g_left[3] + g_frame_res.rows/2), Scalar(0,0,255), 3, CV_AA);
//		line( g_frame_res, Point(img_char.g_right[0], img_char.g_right[1] + g_frame_res.rows/2), Point(img_char.g_right[2], img_char.g_right[3] + g_frame.rows/2), Scalar(0,0,255), 3, CV_AA);
		//pyrDown(g_frame_res, g_frame);


		
		c = waitKey(5);
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
	int config_Processors = get_nprocs_conf();
	cpu_set_t allcpuset;
	cpu_set_t threadcpu;
	
	cout << "This system has " << config_Processors << " processors configured and " << get_nprocs() << " processors available" << endl << endl;
	
	CPU_ZERO(&allcpuset);
	for(int i=0; i < config_Processors; i++)
	{
		CPU_SET(i, &allcpuset);
	}
	
	for(int i=0; i < NUM_THREADS; i++)
	{
		int rc, coreid;
		
		//Setting individual cores to individual services
		CPU_ZERO(&threadcpu);
		coreid=i%config_Processors;
		cout << "Setting thread " << i << " to core " << coreid << endl;
		CPU_SET(coreid, &threadcpu);
		for(int idx=0; idx<config_Processors; idx++)
		{
			if(CPU_ISSET(idx, &threadcpu))  
			{
				cout << " CPU-" << idx << endl;
			}
		}
		cout << "Launching thread " << i << endl << endl;

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
	pthread_create(&threads[PED_DETECT_TH],   					// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[PED_DETECT_TH]),     		// use default attributes
			pedestrian_detect, 						// thread function entry point
			(void *)&(threadParams[PED_DETECT_TH]) 				// parameters to pass in
		      );

	//Lane Detection Thread
	pthread_create(&threads[LANE_FOLLOW_TH],   					// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[LANE_FOLLOW_TH]),     		// use default attributes
			lane_follower, 							// thread function entry point
			(void *)&(threadParams[LANE_FOLLOW_TH]) 			// parameters to pass in
		      );

	//Sign Detection Thread
	pthread_create(&threads[SIGN_RECOG_TH],   					// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[SIGN_RECOG_TH]),     		// use default attributes
			sign_recog, 							// thread function entry point
			(void *)&(threadParams[SIGN_RECOG_TH]) 				// parameters to pass in
		      );

	//Vehicle Detection Thread
	pthread_create(&threads[VEH_DETECT_TH],   					// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[VEH_DETECT_TH]),     		// use default attributes
			vehicle_detect, 						// thread function entry point
			(void *)&(threadParams[VEH_DETECT_TH]) 				// parameters to pass in
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
	cout << "Thread idx=" << threadParams->threadIdx << " running on core " << sched_getcpu() << ", affinity contained:";
	for(int i=0; i<get_nprocs_conf(); i++)
	{
		if(CPU_ISSET(i, &cpuset))
		{
			cout << " CPU-" << i;
		}
	}
	cout << " with PID = " << syscall(SYS_gettid) << endl;
	
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



//Functions related to lane detection.

/**
 * @brief This function reduces the resolution by 2 and eliminates the top part of the image.
 * @param src The image that needs to be processed.
 * @return The processed image.
 */
Mat preprocess(Mat src)
{
	Mat src_res, src_half;
	
	//Reduce the resolution by 2
	pyrDown(src, src_res);
	
	//Crop the image to get bottom half because sky is not required. Thus eliminating half the pixels.
	src_half = src_res( Rect( 0, src_res.rows/2, src_res.cols, src_res.rows/2) );
	
	//imshow("Original", src_half);
	
	return src_half;
}


/**
 * @brief This function converts the image into grayscale and improves contrast.
 * @param src_half The image that needs to be processed.
 * @return The processed image.
 */
Mat equalize(Mat src_half)
{
	Mat gray, contrast;
	
	//Convert to grayscale
	cvtColor(src_half, gray, COLOR_BGR2GRAY);
	
	//Increase Contrast to detect white lines and remove any disturbance due to road color.
	equalizeHist(gray, contrast);
	
	//imshow("Contrast Image", contrast);
	
	return contrast;
}


/**
 * @brief This function creates mask to detect lanes.
 * @param src_half The image that needs to be processed.
 * @return The processed image.
 */
Mat create_mask(Mat src_half)
{
	Mat hls, white;
	Mat hsv, yellow;
	Mat mask;
	
	//Convert Original Image to HLS
	cvtColor(src_half, hls, COLOR_BGR2HLS);								//Shows white as yellow.
	//imshow("HLS", hls);
	
	//Lower value of Saturation makes changes. i.e middle one. Reducing the lower saturation value includes yellow lanes and background as well	
	//If want to incorpotate yellow lines change lower threshold of saturation to 70 or keep 100.
	inRange(hls, Scalar(20,100,0), Scalar(40,255,50), white);				//brightness can be 0 to 50 also.	
	//imshow("HLS white", white);	

	//Convert Original Image to HSV
	cvtColor(src_half, hsv, COLOR_BGR2HSV);								//Shows yellow as yellow.
	//imshow("HSV", hsv);
	
	//Brightness makes the difference (eliminates background) - Do not change brightness value. i.e the last value (80)
	//Saturation value makes changes i.e middle one. If want to make the yellow lines more bold increase upper saturation value. above 100 introduces background.
	inRange(hsv, Scalar(20,80,80), Scalar(40,255,255), yellow); 					
	//imshow("HSV YELLOW", yellow);
	
	
	bitwise_or(white, yellow, mask);
	//imshow("Mask", mask);

	return mask;
}

//Not required. Can delete.
Mat detect_lanes(Mat contrast, Mat mask, Mat roi_mask)
{
	Mat detect_lanes, blur, edge;
	Mat canny_roi;
	vector<Vec4i> lines;
	vector<Vec4i> left;
	vector<Vec4i> right;
	
	bitwise_and(contrast, mask, detect_lanes);
	imshow("lanes Detected", detect_lanes);
	
	//Applying gaussian filter to reduce noise followed by canny transform for edge detection.
	GaussianBlur( detect_lanes, blur, Size(5,5), 0, 0, BORDER_DEFAULT );
	Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);	//Can change to false

	bitwise_and(edge, roi_mask, canny_roi);
	imshow("Canny Mask", canny_roi);
	
	//Detect and Draw Lines
	HoughLinesP(canny_roi, lines, 1, CV_PI/180, HOUGH_THRESHOLD, HOUGH_MIN_LINE_LENGTH, HOUGH_MAX_LINE_GAP );
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		
		double slope = (double)((l[3] - l[1])/(double)(l[2] - l[0]));
		if (slope > 0.5 || slope < (-0.5))
		{
			//cout << "y = " << l[0] << "and threshold" << src_res.cols/2 << endl;
			if(l[0] < (g_frame.cols/2))
			{
				left.push_back(lines[i]);
			}
			else
			{
				right.push_back(lines[i]);
			}
			
			line( g_frame, Point(l[0], l[1] + g_frame.rows/2), Point(l[2], l[3] + g_frame.rows/2), Scalar(0,0,255), 3, CV_AA);	
		}
		
	}
}


//Not required. Can delete.
Mat roi_mask(Mat src_half)
{
		cout << "finished creating Mask ------------" << endl;
		fflush(stdout);
	//Creating Polygon ROI
	Point roi_pt[1][4];
	int num = 4;
	cout << "finished creating Mask ------------" << endl;
	Mat roi_mask = Mat::zeros(Size(src_half.cols, src_half.rows), CV_8U);
	//Points for ROI mask
	roi_pt[0][0] = Point(2*src_half.cols/5, src_half.rows/5);					//Apex
	roi_pt[0][1] = Point(3*src_half.cols/5, src_half.rows/5);
	roi_pt[0][3] = Point(src_half.cols/5, src_half.rows);						//Bottom left vertice
	roi_pt[0][2] = Point(4*src_half.cols/5 , src_half.rows);			//Bottomk right vertice
	const Point* pts_list[1] = {roi_pt[0]};
	fillPoly(roi_mask, pts_list, &num, 1, 255, 8);						//Change to fillConvexPolly for faster results
	
	imshow("ROI MASK", roi_mask);
	cout << "finished creating Mask" << endl;
	return roi_mask;
}


void process_lanes(vector<Vec4i> lane, int side)
{
	//Reset Coordinates
	int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
			
	for( size_t i = 0; i < lane.size(); i++ )
	{
		Vec4i c = lane[i];
		
		x1 += c[0];
		y1 += c[1];
		x2 += c[2];
		y2 += c[3]; 
	}
	
	x1 = x1/lane.size();
	y1 = y1/lane.size();
	x2 = x2/lane.size();
	y2 = y2/lane.size();
			
	if(side == LEFT)
	{
		img_char.g_left[0] = x1;
		img_char.g_left[1] = y1;
		img_char.g_left[2] = x2;
		img_char.g_left[3] = y1;
		
//		line( abcd, Point(x1, y1 + abcd.rows/2), Point(x2, y2 + abcd.rows/2), Scalar(0,0,255), 3, CV_AA);
//		imshow("ABCD", abcd);
	}
	else if(side == RIGHT)
	{
		img_char.g_right[0] = x1;
		img_char.g_right[1] = y1;
		img_char.g_right[2] = x2;
		img_char.g_right[3] = y1;
	}		
//	line( g_frame, Point(x1, y1 + g_frame.cols/2), Point(x2, y2 + g_frame.rows/2), Scalar(0,0,255), 3, CV_AA);		
		
}

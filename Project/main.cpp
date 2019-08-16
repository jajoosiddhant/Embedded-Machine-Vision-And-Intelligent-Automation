/**
 * @file main.cpp
 * @brief This file consists of all the functions required for implementing services such as pedestrian detection, lane detection, vehicle detection and traffic signal detection.
 *
 * The Code has been executed on JETSON NANO(linux).
 *
 * @author Siddhant Jajoo and Sarthak Jain
 * @date 04/19/2019
 *
 * @copyright Copyright (c) 2019
 *
 */


#include "main.h"


int main(int argc, char** argv)
{
	XInitThreads();
	int frame_cnt = 0;
	struct timespec start_time;
	exit_cond = false;
	Mat detector;
	int rc;
	pid_t mainpid;
	int flag = 1;
	int opt;
	bool options = false;
	char output_frames[50];

	int radius;
	string text;
	Point ped_rect[2];
	Point vehicle_rect[2];
	Point sign_rect[2];

	struct timespec temp_start, temp_stop, temp_diff;

	if(argc < 4)
		help();

	while((opt = getopt(argc, argv, "aplvs")) != -1)
	{
		options = true;
		switch(opt)
		{
			case 'a':
				enable[PED_DETECT_TH] = 1;
				enable[LANE_FOLLOW_TH] = 1;
				enable[SIGN_RECOG_TH] = 1;
				enable[VEH_DETECT_TH] = 1;
				break;
			case 'p':
				enable[PED_DETECT_TH] = 1;
				break;
			case 'l':
				enable[LANE_FOLLOW_TH] = 1;
				break;
			case 's':
				enable[SIGN_RECOG_TH] = 1;
				break;
			case 'v':
				enable[VEH_DETECT_TH] = 1;
				break;
			default:
				help();
				break;
		}
		if(optind >= argc)
		{
			help();
		}
	}
	if(!options)
		help();

	//Declaring VideoCapture and VideoWriter objects to read and write videos
	VideoCapture capture(argv[optind]);
	VideoWriter output_v;
	Size size = Size(COLS*2, ROWS*2);
	output_v.open(argv[optind+1], CV_FOURCC('M','P','4','V'), capture.get(CV_CAP_PROP_FPS), size, true);

	//Load Vehicle xml
	if( !vehicle_cascade.load(vehicle_cascade_name) )
		handle_error("Error loading vehicle cascade")


	//Initializing Semaphores and Signal Handler.
	set_signal_handler();
	sem_create();	

	//Main thread affinity
	cout << " Main thread has PID = " << syscall(SYS_gettid) << endl;
	cpu_set_t main_cpu; 
	CPU_ZERO(&main_cpu);
	CPU_SET(0, &main_cpu);
	sched_setaffinity(0, sizeof(main_cpu), &main_cpu);	

	//Fetching and printing Max and Min priority values for SCHED_FIFO.
	mainpid=getpid();
	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	rt_min_prio = sched_get_priority_min(SCHED_FIFO);
	cout << "MAX priority= " << rt_max_prio << endl;
	cout << "MIN priority= " << rt_min_prio << endl;

	//Setting highest priority to main which will act as the scheduler.
	rc = sched_getparam(mainpid, &main_param);
	main_param.sched_priority = rt_max_prio - 1;
	rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
	if(rc < 0) 
	{
		perror("Cannot Set priority of Main thread.\n");
		exit(EXIT_FAILURE);
	}

	//Printing the set scheduled policy, scope and priorities value.
	print_scheduler();
	print_scope();

	//Setting thread attributes
	set_thread_attr();

	//Setting up core affinity for different services
	thread_core_set();

	//Creatintg 4 threads for individual services.
	thread_create();
	
	//note Start time to calculate average FPS.	      
	clock_gettime(CLOCK_REALTIME, &start_time);

	// Create Sequencer thread, which like a cyclic executive, is highest prio
	cout << "STARTING SCHEDULER" << endl;
	
	while(1)
	{
//		clock_gettime(CLOCK_REALTIME, &temp_start);			//uncomment during testing
		capture >> g_frame;		
		
		//Counting number of frames
		frame_cnt++;
		
		// Pedestrian Service = RT_MAX-20 @10Hz
		if((frame_cnt % 3) == 0)
		{
			sem_post(&sem_pedestrian);
		}
		
		// Lane Detection Service = RT_MAX-20 @15Hz
		if((frame_cnt % 2) == 0)
		{
			sem_post(&sem_lane);
		}
	     
		// Vehicle Service = RT_MAX-20 @15Hz
		if((frame_cnt % 2) == 0)
		{
			sem_post(&sem_vehicle);
		}
		
		// Sign Service = RT_MAX-20 @ 7.5Hz
		if((frame_cnt % 4) == 0)
		{
			sem_post(&sem_sign);
		}
        	
		//Sleep initially once to give the other threads to process and store values in global values
		if(flag)
		{
			sleep(1);
			flag = 0;
		}
		
		detector = g_frame.clone();
		resize(detector, detector, Size(COLS*2, ROWS*2));
	
		//Drawing function for pedestrian here
		mute_ped.lock();
		for(int i=0; i<img_char.found_loc.size(); i++)
		{
			ped_rect[0].x = (img_char.found_loc[i].x)*2;
			ped_rect[0].y = (img_char.found_loc[i].y)*2;
			ped_rect[1].x = (img_char.found_loc[i].x + img_char.found_loc[i].width)*2;
			ped_rect[1].y = (img_char.found_loc[i].y + img_char.found_loc[i].height)*2;
			rectangle(detector, ped_rect[0], ped_rect[1], CV_RGB(255, 255, 255), 4);
		}
		mute_ped.unlock();
		
		//Drawing frunction for lanes here
		mute_lane.lock();
		line(detector, Point(img_char.g_left[0], img_char.g_left[1] + 180), Point(img_char.g_left[2], img_char.g_left[3] + 180), CV_RGB(255,0,0), 3, CV_AA);	
		line(detector, Point(img_char.g_right[0], img_char.g_right[1] + 180), Point(img_char.g_right[2], img_char.g_right[3] + 180), CV_RGB(255,0,0), 3, CV_AA);
		mute_lane.unlock();

		//Drawing function for Vehicles here
		mute_vehicle.lock();
		for(int i=0; i<img_char.vehicle_loc.size(); i++)
		{
			vehicle_rect[0].x = img_char.vehicle_loc[i].x;
			vehicle_rect[0].y = img_char.vehicle_loc[i].y + 180;
			vehicle_rect[1].x = img_char.vehicle_loc[i].x + img_char.vehicle_loc[i].width;
			vehicle_rect[1].y = img_char.vehicle_loc[i].y + img_char.vehicle_loc[i].height + 180;
			radius = cvRound((img_char.vehicle_loc[i].width + img_char.vehicle_loc[i].height)*0.25*1.2);
			if(radius < 20)
				text = "Speed up";
			else if((radius >= 20) && (radius < 28))
				text = "Maintain speed";
			else if(radius <= 28)
				text = "Slow down";
//			cout << "Thresholding value: " << radius << endl;
			rectangle(detector, vehicle_rect[0], vehicle_rect[1], CV_RGB(0, 0, 255));
		}
		putText(detector, text, Point(0, 24), FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 0, 255), 2, 8, false);
		mute_vehicle.unlock();

		//Drawing function for traffic sign here		
		mute_sign.lock();
		for(int i=0; i<img_char.traffic.size(); i++)
		{
			sign_rect[0].x = (img_char.traffic[i].x)*2;
			sign_rect[0].y = (img_char.traffic[i].y)*2;
			sign_rect[1].x = (img_char.traffic[i].x + img_char.traffic[i].width)*2;
			sign_rect[1].y = (img_char.traffic[i].y + img_char.traffic[i].height)*2;
			rectangle(detector, sign_rect[0], sign_rect[1], CV_RGB(0, 255, 0));
		}
		mute_sign.unlock();


//		imshow("Video", g_frame);		//Uncomment to view original video
		c = waitKey(1);
		imshow("Detector", detector);
//		sprintf(output_frames, "./frames_snapshot/frame%d.jpg", frame_cnt);
//		imwrite(output_frames, detector);
		output_v.write(detector);
		
//		clock_gettime(CLOCK_REALTIME, &temp_stop);			//uncomment during testing
//		delta_t(&temp_stop, &temp_start, &temp_diff);
//		printf("Time elapsed in waiting: %lu nsecs\n", temp_diff.tv_nsec);		//uncomment during testing
		
		if((c == 27) || (exit_cond))
		{
			break;
		}

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
	destroyAllWindows();
	
	return 0;
}


/**
 * @brief Callback function for pedestrian detection thread.
 * @param threadp 
 * @return void
 */
void* pedestrian_detect(void* threadp)
{
	
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;
	vector<Rect> local_found_loc;

	Mat mat, resz_mat;
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
			
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);
	
	while(1)
	{

		sem_wait(&sem_pedestrian);			//semaphore from main

		mat = g_frame.clone();
		cvtColor(mat, mat, CV_BGR2GRAY);
		resize(mat, resz_mat, Size(COLS, ROWS));			//resize to 320x240

		hog.detectMultiScale(resz_mat, local_found_loc, 0, Size(8, 8), Size(0, 0), 1.05, 2, false);
		
		mute_ped.lock();
		img_char.found_loc.clear();
		img_char.found_loc = local_found_loc;
		mute_ped.unlock();
		
		frame_cnt++;
	
		
		if((c == 27) || (exit_cond))
		{
			break;
		}
	}

	//Calculating FPS for pedestrian detection
	fps_calc(start_time, frame_cnt, FPS_PEDESTRIAN);

	pthread_exit(NULL);
}


/**
 * @brief Callback function for lane detection thread.
 * @param threadp 
 * @return void
 */
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

	
	double slope;
	int x1, x2, y1, y2;
		
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);

	while(1)
	{
		sem_wait(&sem_lane);
	
	
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
		roi_pt[0][0] = Point(2*src_half.cols/5, src_half.rows/5);			//Apex
		roi_pt[0][1] = Point(3*src_half.cols/5, src_half.rows/5);
		roi_pt[0][3] = Point(src_half.cols/5, src_half.rows);				//Bottom left vertice
		roi_pt[0][2] = Point(4*src_half.cols/5 , src_half.rows);			//Bottomk right vertice
		const Point* pts_list[1] = {roi_pt[0]};
		fillPoly(roi_mask, pts_list, &num, 1, 255, 8);				//Change to fillConvexPolly for faster results
	

		//Detect lanes
		vector<Vec4i> lines;
		vector<Vec4i> left;
		vector<Vec4i> right;
		
		bitwise_and(contrast, mask, detect_lanes);
		//imshow("lanes Detected", detect_lanes);
		
		//Applying gaussian filter to reduce noise followed by canny transform for edge detection.
		GaussianBlur( detect_lanes, blur, Size(5,5), 0, 0, BORDER_DEFAULT );
		Canny(blur, edge, CANNY_THRESHOLD_1, CANNY_THRESHOLD_2, 3, true);		//Can change to false for faster results. tradeoff: Accuracy.

		bitwise_and(edge, roi_mask, canny_roi);
		//imshow("Canny Mask", canny_roi);
		
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

		if((c == 27) || (exit_cond))
		{
			break;
		}

	}
	
	//Calculating FPS for lane detection
	fps_calc(start_time, frame_cnt, FPS_LANE);
	
	pthread_exit(NULL);

}


/**
 * @brief Callback function for traffic signal detection thread.
 * @param threadp 
 * @return void
 */
void* sign_recog(void* threadp)
{
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;	
	Mat mat, resz_mat;
	vector<Rect> local_traffic;
	
	CascadeClassifier cascade_traffic;
	if(!cascade_traffic.load("./traffic_light.xml"))
		handle_error("Error loading traffic light cascade")
	
	//Printing thread information 
	threadcpu_info(threadParams);
	
	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);


	while(1)
	{
		sem_wait(&sem_sign);
		mat = g_frame.clone();
		mat = mat(Rect(0, 0, mat.cols, mat.rows/2));
		resize(mat, resz_mat, Size(mat.cols/2, mat.rows/2));
//		cvtColor(mat, mat, CV_BGR2GRAY);

		cascade_traffic.detectMultiScale(resz_mat, local_traffic, 1.1, 2, 0 | CASCADE_SCALE_IMAGE, Size(4, 4), resz_mat.size()/* Size(30, 30)*/);
//		cascade_traffic.detectMultiScale(resz_mat, local_traffic, 1.1, 3, CASCADE_DO_CANNY_PRUNING, Size(0, 0), resz_mat.size()/* Size(30, 30)*/);
						
		mute_sign.lock();
		img_char.traffic.clear();
		img_char.traffic = local_traffic;
		mute_sign.unlock();

		frame_cnt++;

		if((c == 27) || (exit_cond))
		{
			break;
		}
	}
	
	//Calculating FPS for sign detection
	fps_calc(start_time, frame_cnt, FPS_SIGN);

	pthread_exit(NULL);

}


/**
 * @brief Callback function for vehicle detection thread.
 * @param threadp 
 * @return void
 */
void* vehicle_detect(void* threadp)
{
	//Variable Declaration
	threadParams_t* threadParams = (threadParams_t*)threadp;
	struct timespec start_time;
	int frame_cnt = 0;
	Mat src, src_half, gray, blur;
	vector<Rect> local_vehicle_loc;

	
	//Printing thread information 
	threadcpu_info(threadParams);

	//Note Start time to calculate FPS
   	clock_gettime(CLOCK_REALTIME, &start_time);

	while(1)
	{
		sem_wait(&sem_vehicle);
		
		src = g_frame.clone();
		
		src_half = preprocess(src);
		cvtColor(src_half, gray, CV_RGB2GRAY);

		vehicle_cascade.detectMultiScale(gray, local_vehicle_loc, 1.2, 4, 0, Size(16, 16), gray.size());

		mute_vehicle.lock();
		img_char.vehicle_loc.clear();
		img_char.vehicle_loc = local_vehicle_loc;
		mute_vehicle.unlock();
		
		frame_cnt++;

		
		if((c == 27) || (exit_cond))
		{
			break;
		}
	}
	
	//Calculating FPS for sign detection
	fps_calc(start_time, frame_cnt, FPS_VEHICLE);

	pthread_exit(NULL);

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
	
	
	CPU_ZERO(&threadcpu);
	CPU_SET(1, &threadcpu);
	CPU_SET(2, &threadcpu);
	CPU_SET(3, &threadcpu);
	
	for(int i=0; i < NUM_THREADS; i++)
	{
		int rc;
		
		cout << "Setting thread " << i << endl;

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
	if(enable[PED_DETECT_TH])
	{
		rt_param[0].sched_priority=rt_max_prio-2;
		pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
		pthread_create(&threads[PED_DETECT_TH],   				// pointer to thread descriptor
				(pthread_attr_t*)&(rt_sched_attr[PED_DETECT_TH]),     	// use default attributes
				pedestrian_detect, 					// thread function entry point
				(void *)&(threadParams[PED_DETECT_TH]) 			// parameters to pass in
			      );
	}

	//Lane Detection Thread
	if(enable[LANE_FOLLOW_TH])
	{
	rt_param[1].sched_priority=rt_max_prio-2;
	pthread_attr_setschedparam(&rt_sched_attr[1], &rt_param[1]);
	pthread_create(&threads[LANE_FOLLOW_TH],   					// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[LANE_FOLLOW_TH]),     		// use default attributes
			lane_follower, 							// thread function entry point
			(void *)&(threadParams[LANE_FOLLOW_TH]) 			// parameters to pass in
		      );
	}

	//Sign Detection Thread
	if(enable[SIGN_RECOG_TH])
	{
	rt_param[2].sched_priority=rt_max_prio-2;
	pthread_attr_setschedparam(&rt_sched_attr[2], &rt_param[2]);
	pthread_create(&threads[SIGN_RECOG_TH],   					// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[SIGN_RECOG_TH]),     		// use default attributes
			sign_recog, 							// thread function entry point
			(void *)&(threadParams[SIGN_RECOG_TH]) 				// parameters to pass in
		      );
	}

	//Vehicle Detection Thread
	if(enable[VEH_DETECT_TH])
	{
	rt_param[3].sched_priority=rt_max_prio-2;
	pthread_attr_setschedparam(&rt_sched_attr[3], &rt_param[3]);
	pthread_create(&threads[VEH_DETECT_TH],   					// pointer to thread descriptor
			(pthread_attr_t*)&(rt_sched_attr[VEH_DETECT_TH]),     		// use default attributes
			vehicle_detect, 						// thread function entry point
			(void *)&(threadParams[VEH_DETECT_TH]) 				// parameters to pass in
		      );
	}
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
 * @brief This function sets the thread attributes and its priority.
 * @param void 
 * @return void
 */
void set_thread_attr(void)
{
	int i, rc;
	
	for(i=0; i < NUM_THREADS; i++)
	{

		rc=pthread_attr_init(&rt_sched_attr[i]);
		rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
		rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
		//rc=pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu);
		pthread_attr_setscope(&rt_sched_attr[i], PTHREAD_SCOPE_SYSTEM);

		rt_param[i].sched_priority=rt_max_prio-2;
		pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);

		threadParams[i].threadIdx=i;
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
			cout << "OVERALL Number of frames: "<< frame_cnt << endl;
			cout << "OVERALL Duration: "<< diff_time.tv_sec << endl;
			cout << "OVERALL Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			sem_post(&sem_pedestrian);
			sem_post(&sem_lane);
			sem_post(&sem_vehicle);
			sem_post(&sem_sign);

			break;
		}

		case FPS_PEDESTRIAN:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "PEDESTRIAN FPS:" << endl;
			cout << "PEDESTRIAN Number of frames: "<< frame_cnt << endl;
			cout << "PEDESTRIAN Duration: "<< diff_time.tv_sec << endl;
			cout << "PEDESTRIAN Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			break;
		}
		
		case FPS_LANE:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "LANE FPS:" << endl;
			cout << "LANE Number of frames: "<< frame_cnt << endl;
			cout << "LANE Duration: "<< diff_time.tv_sec << endl;
			cout << "LANE Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			break;
		}
		
		case FPS_VEHICLE:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "VEHICLE FPS:" << endl;
			cout << "VEHICLE Number of frames: "<< frame_cnt << endl;
			cout << "VEHICLE Duration: "<< diff_time.tv_sec << endl;
			cout << "VEHICLE Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
			break;
		}
		
		case FPS_SIGN:
		{
			clock_gettime(CLOCK_REALTIME, &stop_time);
			delta_t(&stop_time, &start_time, &diff_time);		//Obtain time difference
			cout << endl << "SIGN FPS:" << endl;
			cout << "SIGN Number of frames: "<< frame_cnt << endl;
			cout << "SIGN Duration: "<< diff_time.tv_sec << endl;
			cout << "SIGN Average FPS: " << (frame_cnt/diff_time.tv_sec) << endl;
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
	cvtColor(src_half, hls, COLOR_BGR2HLS);					//Shows white as yellow.
	//imshow("HLS", hls);
	
	//Lower value of Saturation makes changes. i.e middle one. Reducing the lower saturation value includes yellow lanes and background as well	
	//If want to incorpotate yellow lines change lower threshold of saturation to 70 or keep 100 but this also includes noise so do not reduce the middle lower value from the current one.
	inRange(hls, Scalar(20,115,10), Scalar(40,255,40), white);					//Lower saturation can be 118.	
	//imshow("HLS white", white);									//Shows white as yellow.

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



/**
 * @brief Function to process and get coordinates for left and right lane.
 * @param lane The lane vector obtained from HoughlinesP.
 * @param side The macro LEFT or RIGHT which specifies the coordinates the lane vector belongs to. 
 * @return void
 */
void process_lanes(vector<Vec4i> lane, int side)
{

	//Reset Coordinates
	double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
	int xtop, xbottom; //ytop, ybottom;
	double slope;
	double intercept;
	
	//Origin is top left corner
	if(side == LEFT)
	{
		if(lane.size() == 0)
		{
			avg_slope_left = 0;
			avg_intercept_left = 0;
			count_left = 1;
			nolane_count_left++;
			if(nolane_count_left > 10)
			{
				mute_lane.lock();
				img_char.g_left[0] = 0;
				img_char.g_left[1] = 0;
				img_char.g_left[2] = 0;
				img_char.g_left[3] = 0;
				nolane_count_left = 0;
				mute_lane.unlock(); 
			}
			return;
		}
		
		//Resetting the count for no left lanes detected
		nolane_count_left = 0;
		
		for( size_t i = 0; i < lane.size(); i++ )
		{
			Vec4i c = lane[i];
			
			x1 += c[0];
			y1 += c[1];
			x2 += c[2];
			y2 += c[3]; 
		}
		
		
		//Calculating avergae values of detected left lane coordinates
		x1 = x1/lane.size();
		y1 = y1/lane.size();
		x2 = x2/lane.size();
		y2 = y2/lane.size();

		
		//Returning if by any chance average of the two x coordinates are equal. Eliminating division by zero when calculating slope.
		if(x1 == x2)
		{
			return;
		}
		
		//Calculating Slope and Intercept
		slope = (double)(y2 - y1)/(double)(x2 - x1);
		intercept = (double)y2 - (double)(slope*x2);		
		
		//Assigning the maximum and minimum values for left lane detected in y direction to draw lines.
		if(ybottom_left < y1)
		{
			ybottom_left = y1;
		}
		if(ytop_left > y2)
		{
			ytop_left = y2;
		}
		
		
		if(slope < (-0.5))
		{
			//Averaging slope and intercept with history to get consistent results.
			avg_slope_left = avg_slope_left + (double)(slope - avg_slope_left)/count_left;
			avg_intercept_left = avg_intercept_left + (double)(intercept - avg_intercept_left)/count_left;
			
			//Finding corresponding x-coordinates
			xtop = (ytop_left - avg_intercept_left)/avg_slope_left;
			xbottom = (ybottom_left - avg_intercept_left)/avg_slope_left;
						
			count_left++;			
		
			mute_lane.lock();
			img_char.g_left[0] = (int)xtop;
			img_char.g_left[1] = (int)ytop_left;
			img_char.g_left[2] = (int)xbottom;
			img_char.g_left[3] = (int)ybottom_left;
			mute_lane.unlock();
		}
	}
	else if(side == RIGHT)
	{
		if(lane.size() == 0)
		{
			avg_slope_right = 0;
			avg_intercept_right = 0;
			count_right = 1;
			nolane_count_right++;
			if(nolane_count_right > 10)
			{
				mute_lane.lock();
				img_char.g_right[0] = 0;
				img_char.g_right[1] = 0;
				img_char.g_right[2] = 0;
				img_char.g_right[3] = 0;
				nolane_count_right = 0;
				mute_lane.unlock();
			}
			return;
		}

		//Resetting the count for no right lanes detected
		nolane_count_right = 0;		

		for( size_t i = 0; i < lane.size(); i++ )
		{
			Vec4i c = lane[i];
			
			x1 += c[0];
			y1 += c[1];
			x2 += c[2];
			y2 += c[3]; 
		}
		
		
		//Calculating avergae values of detected right lane coordinates
		x1 = x1/lane.size();
		y1 = y1/lane.size();
		x2 = x2/lane.size();
		y2 = y2/lane.size();
	
		//Returning if by any chance average of the two x coordinates are equal. Eliminating division by zero when calculating slope.
		if(x1 == x2)
		{
			return;
		}
		
		//Calculating Slope and Intercept
		slope = (double)(y2 - y1)/(double)(x2 - x1);
		intercept = (double)y2 - (double)(slope*x2);		
		
		//Assigning the maximum and minimum values for right lane detected in y direction to draw lines.
		if(ybottom_right < y2)
		{
			ybottom_right = y2;
		}
		if(ytop_right > y1)
		{
			ytop_right = y1;
		}
		
		
		if(slope > (0.5))
		{
			//Averaging slope and intercept with history to get consistent results.
			avg_slope_right = avg_slope_right + (double)(slope - avg_slope_right)/count_right;
			avg_intercept_right = avg_intercept_right + (double)(intercept - avg_intercept_right)/count_right;

			//Finding corresponding x-coordinates
			xtop = (ytop_right - avg_intercept_right)/avg_slope_right;
			xbottom = (ybottom_right - avg_intercept_right)/avg_slope_right;
						
			count_right++;			
		
			mute_lane.lock();
			img_char.g_right[0] = (int)xtop;
			img_char.g_right[1] = (int)ytop_right;
			img_char.g_right[2] = (int)xbottom;
			img_char.g_right[3] = (int)ybottom_right;
			mute_lane.unlock();
		}
	}		
}


/**
 * @brief The function prints the PTHREAD scope.
 * @param void 
 * @return void
 */
void print_scope(void)
{
	int scope;

	pthread_attr_getscope(&main_attr, &scope);
	if(scope == PTHREAD_SCOPE_SYSTEM)
	{
		cout << "PTHREAD SCOPE SYSTEM" << endl;
	}
	else if (scope == PTHREAD_SCOPE_PROCESS)
	{
		cout << "PTHREAD SCOPE PROCESS" << endl;
	}
	else
	{
		cout << "PTHREAD SCOPE UNKNOWN" << endl;
	}
}


/**
 * @brief The function prints the scheduling policy set.
 * @param void 
 * @return void
 */
void print_scheduler(void)
{
	int schedType;

	schedType = sched_getscheduler(getpid());

	switch(schedType)
	{
		case SCHED_FIFO:
			cout << "Pthread Policy is SCHED_FIFO" << endl;
			break;
		case SCHED_OTHER:
			cout << "Pthread Policy is SCHED_OTHER" << endl;
			exit(EXIT_FAILURE);
			break;
		case SCHED_RR:
			cout << "Pthread Policy is SCHED_RR" << endl;
			exit(EXIT_FAILURE);
			break;
		default:
			cout << "Pthread Policy is UNKNOWN" << endl;
			exit(EXIT_FAILURE);
	}
}


/**
 * @brief Callback function for signal interruption.
 * @param signo
 * @param info
 * @param extra 
 * @return void
 */
void signal_handler(int signo, siginfo_t *info, void *extra)
{
	exit_cond = true;
}


/**
 * @brief The function sets the signal handler for interrupt signal i.e ctrl + c.
 * @param void 
 * @return void
 */
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


/**
 * @brief Function for computing time difference between two input timespec structures and saving in third timespec structure.
 * @reference Reference is provided to seqgen.c by Prof. Sam Siewert for delta_t function.
 * @param stop Stop time timespec address
 * @param start start time timespec address
 * @param delta_t The timespec address where the calculated time difference needs to be stored.
 * @return void
 */
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


/**
 * @brief Function that prints the correct command line arguments to be input.
 * @param void
 * @return void
 */
void help(void)
{
	cout << endl << "Usage: sudo ./smart_car detection_type_1 detection_type_2 ....detection_type_4 input_video_file output_video_file.mp4";
	cout << endl << "-a for all detection tasks";
	cout << endl << "-p for pedestrian detection";
	cout << endl << "-l for lane following";
	cout << endl << "-v for vehicle detection";
	cout << endl << "-s for road-sign recognition";
	cout << endl << "Exiting Program" << endl;
	exit(EXIT_FAILURE);
}

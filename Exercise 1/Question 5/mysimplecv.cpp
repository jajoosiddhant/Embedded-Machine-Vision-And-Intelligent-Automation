
//Reference : https://docs.opencv.org/3.4/dc/da3/tutorial_copyMakeBorder.html


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>


#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/contrib/contrib.hpp"

using namespace cv;
using namespace std;

// Default resolution is 360p
#define VRES_ROWS (360)
#define HRES_COLS (640)
#define BORDER_WIDTH (4)
#define LINE_THICKNESS (1)
#define LINE_TYPE (8)
#define LINE_SHIFT (0)

#define ESC_KEY (27)

// Buffer for highest resolution visualization possible
unsigned char imagebuffer[1440*2560*3]; 	// 1440 rows, 2560 cols/row, 3 channel

int main(int argc, char **argv)
{
	int border_top, border_bottom, border_left, border_right;
    int hres = HRES_COLS;
    int vres = VRES_ROWS;
    int border_type = BORDER_CONSTANT;
    int line_thickness = LINE_THICKNESS;
    int line_type = LINE_TYPE;
    int line_shift = LINE_SHIFT;
    
    Scalar blue(255,0,0);
    Scalar yellow(0,255,255);

    Mat basicimage(vres, hres, CV_8UC3, imagebuffer);
    
    
    printf("hres=%d, vres=%d\n", hres, vres);

	
	
    // interactive computer vision loop 
    namedWindow("Annotated Image", CV_WINDOW_AUTOSIZE);

    // read in default image
    if(vres == 360)
        basicimage = imread("Cactus360p.jpg", CV_LOAD_IMAGE_COLOR);
    else if(vres == 720)
        basicimage = imread("Cactus720p.jpg", CV_LOAD_IMAGE_COLOR);
    else if(vres == 1080)
        basicimage = imread("Cactus1080p.jpg", CV_LOAD_IMAGE_COLOR);
    else if(vres == 1440)
        basicimage = imread("Cactus1440p.jpg", CV_LOAD_IMAGE_COLOR);

    if(!basicimage.data)  // Check for invalid input
    {
        printf("Could not open or find the refernece starting image\n");
        exit(-1);
    }

    

	resize(basicimage, basicimage, Size(), 0.5, 0.5, INTER_LINEAR);
	
	hres = hres/2;
	vres = vres/2;
	printf("After Resizing, Resolution: hres = %d, vres=%d.\n", hres, vres);
	
	border_top = border_bottom = border_left = border_right = BORDER_WIDTH;
	
	//Adding Blue Border to the Image
	copyMakeBorder(basicimage, basicimage, border_top, border_bottom, border_left, border_right, border_type, blue);


	//Add crosshair yellow colour
	line(basicimage, Point((hres/2) - 20,vres/2), Point((hres/2) + 20,vres/2), yellow, line_thickness, line_type, line_shift);
	
	line(basicimage, Point(hres/2,(vres/2) - 20), Point(hres/2,(vres/2) + 20), yellow, line_thickness, line_type, line_shift);
	
	circle(basicimage, Point(hres/2,vres/2), 20, yellow, line_thickness, line_type, line_shift);
	

    // Interactive LOOP
    while(1)
    {

        imshow("Annotated Image", basicimage);  

		imwrite("Annotated_image.jpg", basicimage);
		
        // set pace on shared memory sampling in msecs
        char c = cvWaitKey(10);

        if(c == ESC_KEY)
        {
            exit(1);
        }

    }
 
    return 1;

}

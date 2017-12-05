
#include <iostream>
#include <time.h>
#include <fstream>

//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
////#include "opencv2/legacy/legacy.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/stitching/stitcher.hpp"

#include "opencv3\opencv.hpp"
#include <stdio.h>
#include "Stitching.h"



using namespace cv;
using namespace std;


////debug lib
//#pragma comment(lib, "opencvLib\\debug\\opencv_calib3d248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_core248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_highgui248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_imgproc248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_video248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_stitching248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_features2d248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_nonfree248d.lib")
//#pragma comment(lib, "opencvLib\\debug\\opencv_flann248d.lib")

////release lib
//#pragma comment(lib, "opencvLib\\release\\opencv_legacy248.lib")
//#pragma comment(lib, "opencvLib\\release\\opencv_core248.lib")
//#pragma comment(lib, "opencvLib\\release\\opencv_highgui248.lib")
//#pragma comment(lib, "opencvLib\\release\\opencv_imgproc248.lib")
//#pragma comment(lib, "opencvLib\\release\\opencv_video248.lib")
//#pragma comment(lib, "opencvLib\\release\\opencv_stitching248.lib")
//
//#pragma comment(lib, "opencvLib\\release\\opencv_features2d248.lib")
//#pragma comment(lib, "opencvLib\\release\\opencv_nonfree248.lib")
//#pragma comment(lib, "opencvLib\\release\\opencv_flann248.lib")

//opencv 3.x 版本的库
#pragma comment(lib, "opencv3\\lib\\opencv_core320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_calib3d320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_objdetect320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_imgcodecs320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_highgui320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_imgproc320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_features2d320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_flann320d.lib")
#pragma comment(lib, "opencv3\\lib\\opencv_xfeatures2d320d.lib")



void main()
{
	Mat srcImage[2], dst;
	Mat srcImageResize[2];
	srcImage[0] = imread("images//railtracks_01.jpg", 1);
	srcImage[1] = imread("images//railtracks_02.jpg", 1);
	float ratio = 0.2;
	if (srcImage[0].cols > 1500 || srcImage[0].rows > 1500)
	{
		resize(srcImage[0], srcImageResize[0], Size(srcImage[0].cols*ratio, srcImage[0].rows*ratio));
		resize(srcImage[1], srcImageResize[1], Size(srcImage[1].cols*ratio, srcImage[1].rows*ratio));

		stitching(srcImageResize, 1, dst, 1);
	}
	else
	{
		stitching(srcImage, 1, dst, 1);
	}
	
	/*srcImage[0] = imread("leftRectangularImg.jpg", 1);
	srcImage[1] = imread("rightRectangularImg.jpg", 1);

	Mat trans0 = Mat(srcImage[0].cols, srcImage[0].rows, srcImage[0].type());
	Mat srcImage0_270 = Mat(srcImage[0].cols, srcImage[0].rows, srcImage[0].type());

	Mat trans1 = Mat(srcImage[1].cols, srcImage[1].rows, srcImage[1].type());
	Mat srcImage1_270 = Mat(srcImage[1].cols, srcImage[1].rows, srcImage[1].type());
	
	transpose(srcImage[0], trans0);
	flip(trans0, srcImage0_270, 0);

	flip(srcImage[1], srcImage[1], 0);
	transpose(srcImage[1], trans1);
	flip(trans1, srcImage1_270, 0);

	Mat tempImage[2];
	srcImage0_270.copyTo(tempImage[0]);
	srcImage1_270.copyTo(tempImage[1]);*/
	
	//SPHP算法实现
	
	
	imshow("stitching result",dst);
	imwrite("dst.jpg", dst);
	waitKey(0);



}



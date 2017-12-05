#include <iostream>
#include <time.h>
#include <fstream>

//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/legacy/legacy.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/stitching/stitcher.hpp"

#include "opencv3\opencv.hpp"

#include "coorTrans.h"


#define NUMOFIMAGES 2
using namespace cv;
using namespace std;


void findMinMaxValue(vector<Point2f> corners_out, float &minx, float &maxx, float &miny, float &maxy);
void mapping(Mat* inputImages, int imgW_out, int imgH_out, double rowStep, double colStep, Mat* mapX, Mat* mapY, Mat *dst, Mat *mask);
void textureMapping(Mat* srcImages, int gridSize, int imgW, int imgH, int warpType, Mat*T, Mat H01, Mat* H, Mat *H_matrices,Mat* c1para, double theta, Mat &dst, double ub1, double ub2, int zero_On);
Mat blending(Mat *dsts, Mat *masks, int type);

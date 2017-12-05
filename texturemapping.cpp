#include "texturemapping.h"
//

void textureMapping(Mat* srcImages, int gridSize, int imgW, int imgH, int warpType, Mat*T, Mat H01, Mat* H, Mat *H01_matrices,Mat* c1para, double theta, Mat &dst, double ub1, double ub2, int zero_On)
{

	int rowNum = ceil((double)imgH / gridSize);
	int colNum = ceil((double)imgW / gridSize);
	double rowStep = (double)(imgH - 1) / (rowNum - 1);
	double colStep = (double)(imgW - 1) / (colNum - 1);

	double X = 0, Y = 0;
	double X_temp = 0, Y_temp = 0;
	double X_map = 0, Y_map = 0;

	Mat mapX[2], mapY[2];
	mapX[0] = Mat(rowNum, colNum, CV_64F, Scalar(0));
	mapX[0].copyTo(mapX[1]);
	mapX[0].copyTo(mapY[0]);
	mapX[0].copyTo(mapY[1]);

	double minX[2], maxX[2], minY[2], maxY[2];

	double u, v;

	Mat S = c1para[4];
	double s00 = S.at<double>(0, 0); double s01 = S.at<double>(0, 1); double s02 = S.at<double>(0, 2);
	double s10 = S.at<double>(1, 0); double s11 = S.at<double>(1, 1); double s12 = S.at<double>(1, 2);

	double a0 = c1para[0].at<double>(0, 0); double a1 = c1para[0].at<double>(1, 0); double a2 = c1para[0].at<double>(2, 0);//a

	double b0 = c1para[1].at<double>(0, 0); double b1 = c1para[1].at<double>(1, 0); double b2 = c1para[1].at<double>(2, 0);//b

	double e0 = c1para[2].at<double>(0, 0); double e1 = c1para[2].at<double>(1, 0); double e2 = c1para[2].at<double>(2, 0);//e

	double f0 = c1para[3].at<double>(0, 0); double f1 = c1para[3].at<double>(1, 0); double f2 = c1para[3].at<double>(2, 0);//f
	double a3, e3;// , b3, e3, f3;

	if (zero_On != 0)
	{
		a3 = c1para[0].at<double>(3, 0);
		// b3 = c1para[1].at<double>(3, 0);
		e3 = c1para[2].at<double>(3, 0);
		//f3 = c1para[3].at<double>(3, 0);
	}

	int  y = 0, x = 0;
	for (int i = 0; i < NUMOFIMAGES; i++)
	{
		double h00 = H01.at<double>(0, 0); double h01 = H01.at<double>(0, 1); double h02 = H01.at<double>(0, 2);
		double h10 = H01.at<double>(1, 0); double h11 = H01.at<double>(1, 1); double h12 = H01.at<double>(1, 2);
		double h20 = H01.at<double>(2, 0); double h21 = H01.at<double>(2, 1); double h22 = H01.at<double>(2, 2);

		/*double h00_top = H01_matrices[1].at<double>(0, 0); double h01_top = H01_matrices[1].at<double>(0, 1); double h02_top = H01_matrices[1].at<double>(0, 2);
		double h10_top = H01_matrices[1].at<double>(1, 0); double h11_top = H01_matrices[1].at<double>(1, 1); double h12_top = H01_matrices[1].at<double>(1, 2);
		double h20_top = H01_matrices[1].at<double>(2, 0); double h21_top = H01_matrices[1].at<double>(2, 1); double h22_top = H01_matrices[1].at<double>(2, 2);

		double h00_down = H01_matrices[2].at<double>(0, 0); double h01_down = H01_matrices[2].at<double>(0, 1); double h02_down = H01_matrices[2].at<double>(0, 2);
		double h10_down = H01_matrices[2].at<double>(1, 0); double h11_down = H01_matrices[2].at<double>(1, 1); double h12_down = H01_matrices[2].at<double>(1, 2);
		double h20_down = H01_matrices[2].at<double>(2, 0); double h21_down = H01_matrices[2].at<double>(2, 1); double h22_down = H01_matrices[2].at<double>(2, 2);
		double h00, h01, h02, h10, h11, h12, h20, h21, h22;*/

		y = 0;
		for (double row = 0; row < imgH; row = row + rowStep)
		{
			
			double *dataX = mapX[i].ptr<double>(y);
			double *dataY = mapY[i].ptr<double>(y);
			x = 0;

		/*	if (row < imgH / 2)
			{
				h00 = h00_top;  h01 = h01_top;  h02 = h02_top;
				h10 = h10_top;  h11 = h11_top;  h12 = h12_top;
				h20 = h20_top;  h21 = h21_top;  h22 = h22_top;
			}
			else
			{
				h00 = h00_down;  h01 = h01_down;  h02 = h02_down;
				h10 = h10_down;  h11 = h11_down;  h12 = h12_down;
				h20 = h20_down;  h21 = h21_down;  h22 = h22_down;
			}*/

			for (double col = 0; col < imgW; col = col + colStep)
			{
				applyTransform(col, row, T[i], X, Y);//[X, Y] = apply_transform(X0, Y0, T{i});
				applyTransform(X, Y, H[i], X_temp, Y_temp);//[tmpx, tmpy] = apply_transform(X, Y, H{ref, i});

				//[out_x, out_y] = c1_warp_ver2(tmpx, tmpy, c1para);
				u = cos(theta)*X_temp + sin(theta)*Y_temp;
				v = -sin(theta)*X_temp + cos(theta)*Y_temp;

				if (u <= ub1)
				{
					//这里是乘以H矩阵
					double value = (double)(h00*X_temp + h01*Y_temp + h02) / (h20*X_temp + h21*Y_temp + h22);//在这里出现了很大的偏差,是因为matlab里H矩阵用的是c1param中的H，这个H是H01，而我以为是数组H中的H
					dataX[x] = value;
					dataY[x] = (double)(h10*X_temp + h11*Y_temp + h12) / (h20*X_temp + h21*Y_temp + h22);

				}
				else if (u > ub1&&u <= ub2)
				{
					//这里是过渡地带
					if (zero_On == 0)
					{
						dataX[x] = (a0*u*u + a1*u + a2)*v + b0*u*u + b1*u + b2;
						dataY[x] = (e0*u*u + e1*u + e2)*v + f0*u*u + f1*u + f2;
					}
					else
					{
						dataX[x] = (a0*u*u*u + a1*u*u + a2*u + a3)*v + b0*u*u + b1*u + b2;
						dataY[x] = (e0*u*u*u + e1*u*u + e2*u + e3)*v + f0*u*u + f1*u + f2;//这里一开始把e3写成了a3，导致最终的生成的图像有问题
					}
				}
				else
				{
					//这里是乘以S矩阵
					dataX[x] = (double)(s00*X_temp + s01*Y_temp + s02);
					dataY[x] = (double)(s10*X_temp + s11*Y_temp + s12);
				}

				x++;
			}
			y++;
		}
		//cout << mapX[i] << endl;
		minMaxLoc(mapX[i], &minX[i], &maxX[i], 0, 0);
		minMaxLoc(mapY[i], &minY[i], &maxY[i], 0, 0);
	}

	//这里的处理只支持两张图片
	double all_minX = min(minX[0], minX[1]);     double all_maxX = max(maxX[0], maxX[1]);
	double all_minY = min(minY[0], minY[1]);     double all_maxY = max(maxY[0], maxY[1]);

	// reorganize
	double tt_array[3][3] = { { 1, 0, -all_minX }, { 0, -1, all_maxY }, { 0, 0, 1 } };//T1

	Mat tt = Mat(3, 3, CV_64F, tt_array);

	Mat mask = Mat(mapX[0].rows, mapX[0].cols, CV_8U, Scalar(1));
	Mat xy_mat[2];
	xy_mat[0] = Mat(mapX[0].rows*mapX[0].cols, 3, CV_64F);
	xy_mat[1] = Mat(mapX[1].rows*mapX[1].cols, 3, CV_64F);
	Mat Mask_double = Mat(mask.rows, mask.cols, CV_64F);
	mask.convertTo(Mask_double, CV_64F);
	for (int i = 0; i < NUMOFIMAGES; i++)
	{
		matrix2Array(mapX[i], mask).copyTo(xy_mat[i].col(0));
		matrix2Array(mapY[i], mask).copyTo(xy_mat[i].col(1));
		matrix2Array(Mask_double, mask).copyTo(xy_mat[i].col(2));
		//Mat temp = xy_mat[i]*tt;//matlab代码里用的是apply_transform函数，意思与这个不一样
		Mat temp = applyTransform(xy_mat[i], tt);
		array2Matrix(temp.col(0), mask, mapX[i]);//这里也用到了array2Matrix注意值的正确性
		array2Matrix(temp.col(1), mask, mapY[i]);

		minMaxLoc(mapX[i], &minX[i], &maxX[i], 0, 0);

		minMaxLoc(mapY[i], &minY[i], &maxY[i], 0, 0);
	}

	all_minX = min(minX[0], minX[1]);    all_maxX = max(maxX[0], maxX[1]);
	all_minY = min(minY[0], minY[1]);    all_maxY = max(maxY[0], maxY[1]);

	/*这里没有加matlab中scale down这一步，因为matlab代码中这一
	部分是为了防止内存溢出，可以通过减小图片尺寸完成*/
	int tr_x = 5, tr_y = 5;
	double ttt_array[3][3] = { { 1, 0, tr_x }, { 0, 1, tr_y }, { 0, 0, 1 } };
	Mat T3 = Mat(3, 3, CV_64F, ttt_array);
	for (int i = 0; i < 2; i++)
	{
		mapX[i] = mapX[i] + tr_x;
		mapY[i] = mapY[i] + tr_y;
	}

	int imgW_out = ceil(all_maxX - all_minX) + 1 + 2 * tr_x;
	int imgH_out = ceil(all_maxY - all_minY) + 1 + 2 * tr_y;

	//mapping求出来的图像和mask都是每一张图像的输出需要经过blending
	Mat dsts[2];
	Mat masks[2];

	//mapping对应的就是matlab中textureMapping.exe
	cout << mapX[1] << endl;
	mapping(srcImages, imgW_out, imgH_out, rowStep, colStep, mapX, mapY, dsts, masks);

	imshow("left.jpg", dsts[0]);
	imshow("right.jpg", dsts[1]);
	waitKey(0);
	dst = blending(dsts, masks, 0);
	//mapping(srcImages, imgW_out, imgH_out, rowStep, colStep, mapX, mapY, dst,mask);

}


void mapping(Mat* inputImages, int imgW_out, int imgH_out, double rowStep, double colStep, Mat* mapX, Mat* mapY, Mat *dst, Mat *mask)
{
	dst[0] = Mat(imgH_out, imgW_out, CV_8UC3, Scalar(0, 0, 0));
	dst[0].copyTo(dst[1]);
	mask[0] = Mat(imgH_out, imgW_out, CV_8UC1, Scalar(0));
	mask[0].copyTo(mask[1]);
	vector<Point2f> corners_in, corners_out;
	corners_in.resize(4);
	corners_out.resize(4);
	int m = 0, n = 0;
	Mat tempMatrix;

	////正向插值
	//for (int i = 0; i < NUMOFIMAGES; i++)
	//{
	//	m = 0;
	//	for (float row = 0; row < inputImages[i].rows; row = row + rowStep)
	//	{
	//		if ((m + 1) < mapY[i].rows)
	//		{
	//			double *dataX = mapX[i].ptr<double>(m);
	//			//double *dataX1 = mapX[i].ptr<double>(m+1);
	//			double *dataY = mapY[i].ptr<double>(m);
	//			double *dataY1 = mapY[i].ptr<double>(m + 1);
	//			n = 0;
	//			for (float col = 0; col < inputImages[i].cols; col = col + colStep)
	//			{
	//				if ((n + 1) < mapX[i].cols)
	//				{
	//					
	//					corners_in[0].x = col; corners_in[0].y = row;             corners_in[1].x = col + colStep; corners_in[1].y = row;
	//					corners_in[2].x = col; corners_in[2].y = row + rowStep;   corners_in[3].x = col + colStep; corners_in[3].y = row + rowStep;

	//					//这里原来写的是（dataY+1）[n]这样指针是没有移动的，所以出错了
	//					corners_out[0].x = dataX[n]; corners_out[0].y = dataY[n];        corners_out[1].x = dataX[n + 1]; corners_out[1].y = dataY[n];
	//					corners_out[2].x = dataX[n]; corners_out[2].y = dataY1[n];       corners_out[3].x = dataX[n + 1]; corners_out[3].y = dataY1[n];

	//					//perspectiveTransform(corners_in, corners_out, tempMatrix);
	//					tempMatrix = getPerspectiveTransform(corners_in, corners_out);
	//					double M00 = tempMatrix.at<double>(0, 0);
	//					double M01 = tempMatrix.at<double>(0, 1);
	//					double M02 = tempMatrix.at<double>(0, 2);

	//					double M10 = tempMatrix.at<double>(1, 0);
	//					double M11 = tempMatrix.at<double>(1, 1);
	//					double M12 = tempMatrix.at<double>(1, 2);

	//					for (int y = row; y <row + rowStep; y++)
	//					{
	//						//if (y < inputImages[i].rows)
	//						//{
	//							uchar *data_input = inputImages[i].ptr<uchar>(y);
	//							//uchar *data_output = dst.ptr<uchar>(y);
	//							for (int x = col; x < col + colStep; x++)
	//							{
	//								
	//								//if (x < inputImages[i].cols)
	//								//{
	//								float x_out_f = M00*x + M01*y + M02;
	//								float y_out_f = M10*x + M11*y + M12;
	//								int x_out = roundf(x_out_f);
	//								int y_out = roundf(y_out_f);
	//									

	//								if (y_out >= 0 && y_out < imgH_out&&x_out >= 0 && x_out < imgW_out)
	//								{
	//									if (x_out == 311 && y_out == 66)
	//									{
	//										int test = 0;
	//									}
	//									mask[i].ptr<uchar>(y_out)[x_out] = 1;
	//									dst[i].ptr<uchar>(y_out)[3 * x_out + 0] = data_input[3 * x + 0];
	//									dst[i].ptr<uchar>(y_out)[3 * x_out + 1] = data_input[3 * x + 1];
	//									dst[i].ptr<uchar>(y_out)[3 * x_out + 2] = data_input[3 * x + 2];
	//								}

	//								//}
	//							}
	//						//}

	//					}

	//				}

	//				n++;
	//				//corners_in.clear();
	//				//corners_out.clear();
	//			}
	//		}
	//		m++;
	//	}
	//}

	Mat transMap2dstX[2], transMap2dstY[2];
	transMap2dstX[0] = Mat(dst[0].size(), CV_32F); transMap2dstY[0] = Mat(dst[0].size(), CV_32F);
	transMap2dstX[1] = Mat(dst[1].size(), CV_32F); transMap2dstY[1] = Mat(dst[1].size(), CV_32F);

	for (int i = 0; i < NUMOFIMAGES; i++)
	{
		m = 0;
		for (float row = 0; row < inputImages[i].rows; row = row + rowStep)
		{
			if ((m + 1) < mapY[i].rows)
			{
				double *dataX = mapX[i].ptr<double>(m);
				//double *dataX1 = mapX[i].ptr<double>(m+1);
				double *dataY = mapY[i].ptr<double>(m);
				double *dataY1 = mapY[i].ptr<double>(m + 1);
				n = 0;
				for (float col = 0; col < inputImages[i].cols; col = col + colStep)
				{
					if ((n + 1) < mapX[i].cols)
					{

						corners_in[0].x = col; corners_in[0].y = row;             corners_in[1].x = col + colStep; corners_in[1].y = row;
						corners_in[2].x = col; corners_in[2].y = row + rowStep;   corners_in[3].x = col + colStep; corners_in[3].y = row + rowStep;

						//这里原来写的是（dataY+1）[n]这样指针是没有移动的，所以出错了
						corners_out[0].x = dataX[n]; corners_out[0].y = dataY[n];        corners_out[1].x = dataX[n + 1]; corners_out[1].y = dataY[n];
						corners_out[2].x = dataX[n]; corners_out[2].y = dataY1[n];       corners_out[3].x = dataX[n + 1]; corners_out[3].y = dataY1[n];

						//perspectiveTransform(corners_in, corners_out, tempMatrix);
						tempMatrix = getPerspectiveTransform(corners_in, corners_out);
						
						Mat tempMatrix_inv;				
						invert(tempMatrix, tempMatrix_inv);
						Mat canvas = Mat(dst[i].rows, dst[i].cols,CV_8UC1,Scalar(0));
						Mat tempImage = Mat(dst[i].rows, dst[i].cols, CV_8UC1, Scalar(0));

						line(tempImage, corners_out[0], corners_out[1], Scalar(255));  line(tempImage, corners_out[1], corners_out[3], Scalar(255));
						line(tempImage, corners_out[3], corners_out[2], Scalar(255));  line(tempImage, corners_out[2], corners_out[0], Scalar(255));

						vector<vector<Point> > contours1;
						vector<Vec4i> hierarchy1;
						findContours(tempImage, contours1, hierarchy1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
						drawContours(canvas, contours1, -1, Scalar(255), -1);

						
					
						float minx, maxx, miny, maxy;
						findMinMaxValue(corners_out, minx, maxx, miny, maxy);

						double M00 = tempMatrix_inv.at<double>(0, 0);
						double M01 = tempMatrix_inv.at<double>(0, 1);
						double M02 = tempMatrix_inv.at<double>(0, 2);

						double M10 = tempMatrix_inv.at<double>(1, 0);
						double M11 = tempMatrix_inv.at<double>(1, 1);
						double M12 = tempMatrix_inv.at<double>(1, 2);

						for (int y = miny; y <= maxy; y++)
						{
							if (y >= 0 && y < dst[i].rows)
							{
								uchar *data = canvas.ptr<uchar>(y);
								uchar *dataMak = mask[i].ptr<uchar>(y);
								float *datax = transMap2dstX[i].ptr<float>(y);
								float *datay = transMap2dstY[i].ptr<float>(y);
								for (int x = minx; x <= maxx; x++)
								{
									if (x >= 0 && x < dst[i].cols)
									{

										if (data[x] == 255)
										{
											datax[x] = M00*x + M01*y + M02;
											datay[x] = M10*x + M11*y + M12;

											dataMak[x] = 255;
										}
									}
								}
							}
							
						}

					}

					n++;
					//corners_in.clear();
					//corners_out.clear();
				}
			}
			m++;
		}
		remap(inputImages[i], dst[i], transMap2dstX[i], transMap2dstY[i], CV_INTER_CUBIC, BORDER_CONSTANT, Scalar(0, 0, 0));
	}

	

}

void findMinMaxValue(vector<Point2f> corners_out,float &minx,float &maxx,float &miny,float &maxy)
{
	float minValue = 10000000;
	float maxValue = -100;
	for (int i = 0; i < corners_out.size(); i++)
	{
		if (corners_out[i].x > maxValue)
		{
			maxValue = corners_out[i].x;
		}
		if (corners_out[i].x < minValue)
		{
			minValue = corners_out[i].x;
		}
	}

	minx = minValue;  maxx = maxValue;

	 minValue = 10000000;
	 maxValue = -100;

	 for (int i = 0; i < corners_out.size(); i++)
	 {
		 if (corners_out[i].y > maxValue)
		 {
			 maxValue = corners_out[i].y;
		 }
		 if (corners_out[i].y < minValue)
		 {
			 minValue = corners_out[i].y;
		 }
	 }

	 miny = minValue;  maxy = maxValue;

}

Point calculateOutCenter(Mat locationsOFNonZero)
{
	Point2f outCenter;
	int sumX = 0, sumY = 0;
	int rows = locationsOFNonZero.rows;
	for (int i = 0; i < rows; i++)
	{
		sumX = sumX + locationsOFNonZero.at<Point>(i).x;
		sumY = sumY + locationsOFNonZero.at<Point>(i).y;
	}
	outCenter.x = double(sumX) / rows;
	outCenter.y = double(sumY) / rows;
	return outCenter;
}
void transferTypeOFNonZero(Mat locationsOFNonZero, Mat &locationsOfNonZero2Cols)
{
	int rows = locationsOFNonZero.rows;
	locationsOfNonZero2Cols = Mat(locationsOFNonZero.rows, 2, CV_64F);
	for (int i = 0; i < rows; i++)
	{
		locationsOfNonZero2Cols.ptr<double>(i)[0] = locationsOFNonZero.at<Point>(i).x;
		locationsOfNonZero2Cols.ptr<double>(i)[1] = locationsOFNonZero.at<Point>(i).y;
	}
}
Mat blending(Mat *dsts, Mat *masks, int type)
{
	//0 linear blending
	//1 feather blending
	//2 multi-band blending
	Mat dst;
	Mat dst_mask;
	Mat outWeightmap = Mat(dsts[0].rows, dsts[0].cols, CV_64F, Scalar(0));
	Mat outWeightMask = Mat(dsts[0].rows, dsts[0].cols, CV_8U, Scalar(1));
	Mat locationsOFNonZero, locationsOfNonZero2Cols;
	Point2f outCenter, outCenter_i;
	switch (type)
	{
	case 0:
		for (int i = 0; i < NUMOFIMAGES; i++)
		{
			if (i == 0)
			{
				dsts[i].copyTo(dst);
				masks[i].copyTo(dst_mask);
				findNonZero(masks[i], locationsOFNonZero);
				transferTypeOFNonZero(locationsOFNonZero, locationsOfNonZero2Cols);
				outCenter.x = mean(locationsOfNonZero2Cols.col(0)).val[0];
				outCenter.y = mean(locationsOfNonZero2Cols.col(1)).val[0];
			}
			else
			{
				findNonZero(masks[i], locationsOFNonZero);
				transferTypeOFNonZero(locationsOFNonZero, locationsOfNonZero2Cols);
				outCenter_i.x = mean(locationsOfNonZero2Cols.col(0)).val[0];
				outCenter_i.y = mean(locationsOfNonZero2Cols.col(1)).val[0];

				Mat vec = Mat(2, 1, CV_64F);
				vec.at<double>(0, 0) = outCenter_i.x - outCenter.x;
				vec.at<double>(1, 0) = outCenter_i.y - outCenter.y;

				Mat intsct_mask = masks[i] & dst_mask;

				findNonZero(intsct_mask, locationsOFNonZero);
				transferTypeOFNonZero(locationsOFNonZero, locationsOfNonZero2Cols);

				Mat locationsOfNonZero2Cols_temp;
				locationsOfNonZero2Cols.copyTo(locationsOfNonZero2Cols_temp);

				locationsOfNonZero2Cols_temp.col(0) = locationsOfNonZero2Cols.col(0) - outCenter.x;
				locationsOfNonZero2Cols_temp.col(1) = locationsOfNonZero2Cols.col(1) - outCenter.y;

				Mat proj_val = locationsOfNonZero2Cols_temp*vec;

				double min_proj_val, max_proj_val;
				minMaxLoc(proj_val, &min_proj_val, &max_proj_val, 0, 0);

				proj_val = (proj_val - min_proj_val) / (max_proj_val - min_proj_val);

				//计算权重图
				for (int row = 0; row < locationsOFNonZero.rows; row++)
				{
					//locationsOfNonZero2Cols第一列是x坐标，第二列是y坐标
					int location_row = locationsOfNonZero2Cols.at<double>(row, 1);
					int location_col = locationsOfNonZero2Cols.at<double>(row, 0);
					double weight = proj_val.at<double>(row, 0);
					outWeightmap.at<double>(location_row, location_col) = weight;
					outWeightMask.at<uchar>(location_row, location_col) = 0;
				}


				//三个区域单独的权重已经有了，下面就是将两个半个的彩图按照权重拼成一个全景图
				Mat mask1 = dst_mask&outWeightMask;
				Mat mask2 = outWeightmap;
				Mat mask3 = masks[i] & outWeightMask;

				

				Mat d_mask1, d_mask3;
				mask1.convertTo(d_mask1, CV_64F); mask3.convertTo(d_mask3, CV_64F);
				imshow("d_mask1", d_mask1);
				imshow("d_mask3", d_mask3);

				//计算最终的全景图
				Mat weight_1 = d_mask1 + (1 - mask2).mul(matNot(mask2));//这里三个mat的数据类型不一样
				Mat weight_2 = mask2 + d_mask3;

				Mat tempOfSim = matNot(mask2);
				Mat tempOfSim_8uc1;
				tempOfSim.convertTo(tempOfSim_8uc1,CV_8U);
				Mat tempOfSim_8uc3;
				vector<Mat> triTempOfSim;

				for (int c = 0; c < 3; c++)
				{
					triTempOfSim.push_back(tempOfSim_8uc1);
					
				}
				merge(triTempOfSim, tempOfSim_8uc3);
				Mat crossAreaOfLeftImage = dsts[0].mul(tempOfSim_8uc3);
				Mat crossAreaOfRightImage = dsts[1].mul(tempOfSim_8uc3);
				imshow("crossAreaOfLeftImage", crossAreaOfLeftImage);
				imshow("crossAreaOfRightImage", crossAreaOfRightImage);

				vector<Mat> channels_1, channels_2;
				for (int c = 0; c < 3; c++)
				{
					channels_1.push_back(weight_1);
					channels_2.push_back(weight_2);
				}
				Mat weight_1_ch3, weight_2_ch3;
				merge(channels_1, weight_1_ch3);

				merge(channels_2, weight_2_ch3);
				Mat d_dst, d_dst_i;
				dst.convertTo(d_dst, weight_1_ch3.type());
				dsts[i].convertTo(d_dst_i, weight_2_ch3.type());
				d_dst = d_dst.mul(weight_1_ch3) + d_dst_i.mul(weight_2_ch3);
			
				d_dst.convertTo(dst,CV_8UC3);
				channels_1.clear();
				channels_2.clear();
			}
		}
		break;
	case 1:
		break;
	case 2:
		break;
	default:
		break;
	}


	return dst;
}


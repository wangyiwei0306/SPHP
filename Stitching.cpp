#include "Stitching.h"
#include "coorTrans.h"
//#include "ASIFT.h"
#include "gms_matcher.h"

//void GmsMatch(Mat &img1, Mat &img2){
//	vector<KeyPoint> kp1, kp2;
//	Mat d1, d2;
//	vector<DMatch> matches_all, matches_gms;
//
//	Ptr<ORB> orb = ORB::create(10000);
//	orb->setFastThreshold(0);
//	orb->detectAndCompute(img1, Mat(), kp1, d1);
//	orb->detectAndCompute(img2, Mat(), kp2, d2);
//
//#ifdef USE_GPU
//	GpuMat gd1(d1), gd2(d2);
//	Ptr<cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
//	matcher->match(gd1, gd2, matches_all);
//#else
//	BFMatcher matcher(NORM_HAMMING);
//	matcher.match(d1, d2, matches_all);
//#endif
//
//	// GMS filter
//	int num_inliers = 0;
//	std::vector<bool> vbInliers;
//	gms_matcher gms(kp1, img1.size(), kp2, img2.size(), matches_all);
//	num_inliers = gms.GetInlierMask(vbInliers, false, false);
//
//	cout << "Get total " << num_inliers << " matches." << endl;
//
//	// draw matches
//	for (size_t i = 0; i < vbInliers.size(); ++i)
//	{
//		if (vbInliers[i] == true)
//		{
//			matches_gms.push_back(matches_all[i]);
//		}
//	}
//
//	Mat show = DrawInlier(img1, img2, kp1, kp2, matches_gms, 1);
//	imshow("show", show);
//	waitKey();
//}

int stitching(Mat* images, int warpType, Mat &dst, int zeroR_on)
{
	/*if (images.size() < 2)
	{
	cout << "images for stitching are not enough" << endl;
	return 0;
	}*/
	Mat T[NUMOFIMAGES];
	int imgW = images[0].cols;
	int imgH = images[0].rows;
	vector<Mat>grayImages(NUMOFIMAGES);
	vector<Mat>doubleImages(NUMOFIMAGES);
	//1 计算平移矩阵
	for (int i = 0; i < NUMOFIMAGES; i++)
	{
		cvtColor(images[i], grayImages[i], CV_BGR2GRAY);
		T[i] = Mat(3, 3, CV_64F);
		T[i].at<double>(0, 0) = 1; T[i].at<double>(0, 1) = 0; T[i].at<double>(0, 2) = -(imgW + 1) / 2;
		T[i].at<double>(1, 0) = 0; T[i].at<double>(1, 1) = -1; T[i].at<double>(1, 2) = (imgH + 1) / 2;
		T[i].at<double>(2, 0) = 0; T[i].at<double>(2, 1) = 0; T[i].at<double>(2, 2) = 1;
		cout << T[i] << endl;

	}

	//计算2相对于1的单应性矩阵
	Mat H_matrices[3];
	Mat tempH = calculateHomography(grayImages[0], grayImages[1], H_matrices);//在这里 要计算全局的theta 也要计算上下区域不同的区域,返回值是全局
	cout << tempH << endl;

	Mat H10 = T[1] * tempH*(T[0].inv());
	Mat H10_top = T[1] * H_matrices[1]*(T[0].inv());
	Mat H10_down = T[1] * H_matrices[2] * (T[0].inv());
	cout << H10 << endl;

	Mat H01 = H10.inv();
	Mat H01_top = H10_top.inv();
	Mat H01_down = H10_down.inv();
	
	/*这里没有计算matlab代码中的相对于ref和tar的单应性矩阵*/

	//对H01矩阵进行归一化
	H10 = H10 / H10.at<double>(2, 2);
	H10_top = H10_top / H10_top.at<double>(2, 2);
	H10_down = H10_down / H10_down.at<double>(2, 2);

	H01 = H01 / H01.at<double>(2, 2);
	H01_top = H01_top / H01_top.at<double>(2, 2);
	H01_down = H01_down / H01_down.at<double>(2, 2);

	Mat H01_matrices[3];
	H01_matrices[0] = H01; H01_matrices[1] = H01_top; H01_matrices[2] = H01_down;


	//normalize(H10, H10, 1, cv::NORM_L2);
	//normalize(H01, H01, 1, cv::NORM_L2);
	cout << H01 << endl;
	//let H0 be the homography from reference to target, extract its parameters for computing our warp
	Mat A, t;
	H01(Rect(0, 0, 2, 2)).copyTo(A);
	H01.rowRange(0, 2).col(2).copyTo(t);

	double h20 = H01.at<double>(2, 0);
	double h21 = H01.at<double>(2, 1);
	double c = sqrtf(h20*h20 + h21*h21);
	double theta = atan2(-h21, -h20);//for compute ub1 and ub2

	//compute ub1 and ub2

	double tmpx[2], tmpy[2];
	double cu[2], cv;//cv 这里没有用，就是为了凑参数
	applyTransform(0, 0, Mat::eye(3, 3, CV_64F), tmpx[0], tmpy[0]);
	applyTransform(0, 0, H10, tmpx[1], tmpy[1]);
	Mat temp = Mat(3, 3, CV_64F);
	temp.at<double>(0, 0) = cosf(theta); temp.at<double>(0, 1) = sinf(theta); temp.at<double>(0, 2) = 0;
	temp.at<double>(1, 0) = -sinf(theta); temp.at<double>(1, 1) = cosf(theta); temp.at<double>(1, 2) = 0;
	temp.at<double>(2, 0) = 0; temp.at<double>(2, 1) = 0; temp.at<double>(2, 2) = 1;
	applyTransform(tmpx[0], tmpy[0], temp, cu[0], cv);
	applyTransform(tmpx[1], tmpy[1], temp, cu[1], cv);

	//这里为什么求最小值和最大值--wyw-matlab
	double oriub1 = min(cu[0], cu[1]);
	double oriub2 = max(cu[0], cu[1]);


	int s_itv = 20;
	double ub1, ub2;
	Mat c1para[5];
	Mat totalcost_table = Mat(51, 91, CV_64F, Scalar(0));//这里的51 和 91是根据下面的循环算出来的
	Mat H[2];
	H[0] = H10;
	H[1] = Mat::eye(3, 3, CV_64F);
	//Mat totalcost_table;
	//可以从for循环提出的运算

	Mat theta_cos_sin = Mat(3, 3, CV_64F);
	theta_cos_sin.at<double>(0, 0) = cosf(theta); theta_cos_sin.at<double>(0, 1) = sinf(theta); theta_cos_sin.at<double>(0, 2) = 0;
	theta_cos_sin.at<double>(1, 0) = -sinf(theta); theta_cos_sin.at<double>(1, 1) = cosf(theta); theta_cos_sin.at<double>(1, 2) = 0;
	theta_cos_sin.at<double>(2, 0) = 0; theta_cos_sin.at<double>(2, 1) = 0; theta_cos_sin.at<double>(2, 2) = 1;

	Mat tempH_rotate[2];
	tempH_rotate[0] = Mat(3, 3, CV_64F); tempH_rotate[1] = Mat(3, 3, CV_64F);
	tempH_rotate[0] = theta_cos_sin*H10;
	tempH_rotate[1] = theta_cos_sin*(Mat::eye(3, 3, CV_64F));
	int j_cost = 0, i_cost = 0;

	for (int j = -300; j < 201; j = j + 10)
	{
		double *data_totalcost = totalcost_table.ptr<double>(j_cost);
		i_cost = 0;
		for (int i = -300; i < 601; i = i + 10)
		{
			ub1 = oriub1 + i;
			ub2 = oriub2 + j;
			if ((ub2 - ub1) < 120)
				continue;
			computeC1Params(H01, t, c, theta, ub1, ub2, zeroR_on, c1para);

			Scalar cost[2];

			for (int num = 0; num < 2; num++)
			{
				Mat reg_mask[3];
			
				int rowNum = ceil((float)imgH / s_itv);
				int colNum = ceil((float)imgW / s_itv);
				reg_mask[0] = Mat(rowNum, colNum, CV_8U, Scalar(0));
				reg_mask[1] = Mat(rowNum, colNum, CV_8U, Scalar(0));
				reg_mask[2] = Mat(rowNum, colNum, CV_8U, Scalar(0));
				Mat mat_x_out, mat_y_out, mat_u, mat_v;
				double y_step = (double)(imgH - 1) / (rowNum - 1);
				double x_step = (double)(imgW - 1) / (colNum - 1);

				mat_x_out = Mat(rowNum, colNum, CV_64F, Scalar(0.0));
				mat_x_out.copyTo(mat_y_out);
				mat_x_out.copyTo(mat_u);
				mat_x_out.copyTo(mat_v);

				int y_index = 0, x_index = 0;
				int nReg_mask0_nonZero = 0, nReg_mask1_nonZero = 0, nReg_mask2_nonZero = 0;
				//cout << T[num] << endl;

				//tmpH = invR * H{ref, i};
			

				for (double y = 0.0; y < imgH; y = y + y_step)
				{
					x_index = 0;
					for (double x = 0.0; x < imgW; x = x + x_step)
					{
						
						double x_out, y_out;
						
						applyTransform(x, y, T[num], x_out, y_out);

						mat_x_out.at<double>(y_index, x_index) = x_out;
						mat_y_out.at<double>(y_index, x_index) = y_out;

						double u, v;
						applyTransform(x_out, y_out, tempH_rotate[num], u, v);
						//cout << u << "," << v << endl;
						mat_u.at<double>(y_index, x_index) = u;
						mat_v.at<double>(y_index, x_index) = v;
						//cout << u << "," << v << endl;
						if (u < ub1)
							reg_mask[0].at<uchar>(y_index, x_index) = 1;
						else if (u >= ub1 && u < ub2)
							reg_mask[1].at<uchar>(y_index, x_index) = 1;
						else
							reg_mask[2].at<uchar>(y_index, x_index) = 1;

						x_index++;
					}
					y_index++;
				}
				
				Mat J11_map = Mat(mat_x_out.rows, mat_x_out.cols, mat_x_out.type(),Scalar(0.0));
				Mat J12_map, J21_map, J22_map;
				J11_map.copyTo(J12_map);
				J11_map.copyTo(J21_map);
				J11_map.copyTo(J22_map);

				for (int reg = 0; reg < 3; reg++)
				{
					Mat x_mask, y_mask, u_mask, v_mask;
					Mat A;

					x_mask = matrix2Array(mat_x_out, reg_mask[reg]);
					//cout << x_mask << endl;
					y_mask = matrix2Array(mat_y_out, reg_mask[reg]);
					//cout << y_mask << endl;
					
					//cout << v_mask << endl;

					if (reg == 0)
						A = H01*H[num];//注意这里计算结果---wyw
					else if (reg == 1)
						A = tempH_rotate[num];
					else if (reg == 2)
						A = c1para[4] * H[num];

					A = A / A.at<double>(2, 2);
					//cout << A << endl;
					//normalize(A, A, 1, NORM_L1);
					
					double h[9];
					h[0] = A.at<double>(0, 0); h[1] = A.at<double>(0, 1); h[2] = A.at<double>(0, 2);
					h[3] = A.at<double>(1, 0); h[4] = A.at<double>(1, 1); h[5] = A.at<double>(1, 2);
					h[6] = A.at<double>(2, 0); h[7] = A.at<double>(2, 1); h[8] = A.at<double>(2, 2);

					//这里有除法，需要自己写一个函数保证矩阵中为0的不参加运算

					Mat src1, common_divisor, common_divisor_2, dst;
					Mat J11, J12, J21, J22;
					common_divisor = (h[6] * x_mask + h[7] * y_mask + 1);
					//cout << common_divisor << endl;
					common_divisor_2 = common_divisor.mul(common_divisor);

					src1 = h[6] * (h[2] + h[0] * x_mask + h[1] * y_mask);
					dotDivide(src1, common_divisor_2, dst);
					J11 = h[0] / common_divisor - dst;

					src1 = h[7] * (h[2] + h[0] * x_mask + h[1] * y_mask);
					//src2 = (h[6] * x_mask + h[7] * y_mask + 1).mul((h[6] * x_mask + h[7] * y_mask + 1));
					dotDivide(src1, common_divisor_2, dst);
					J12 = h[1] / common_divisor - dst;

					src1 = h[6] * (h[5] + h[3] * x_mask + h[4] * y_mask);
					//src2 = (h[6] * x_mask + h[7] * y_mask + 1).mul((h[6] * x_mask + h[7] * y_mask + 1));
					dotDivide(src1, common_divisor_2, dst);
					J21 = h[3] / common_divisor - dst;

					src1 = h[7] * (h[5] + h[3] * x_mask + h[4] * y_mask);
					//src2 = (h[6] * x_mask + h[7] * y_mask + 1).mul((h[6] * x_mask + h[7] * y_mask + 1));
					dotDivide(src1, common_divisor_2, dst);
					J22 = h[4] / common_divisor - dst;

					

					if (reg == 1)
					{
						Mat JT11, JT12, JT21, JT22;
						Mat tmp11, tmp12, tmp21, tmp22;

						u_mask = matrix2Array(mat_u, reg_mask[reg]);
						//cout << u_mask << endl;
						v_mask = matrix2Array(mat_v, reg_mask[reg]);

						Mat u_mask_2, u_mask_3;
						pow(u_mask, 2, u_mask_2);
						pow(u_mask, 3, u_mask_3);

						if (zeroR_on == 0)
						{
							//cout << c1para[0] << endl;
							JT11 = (2 * c1para[0].at<double>(0, 0)*u_mask + c1para[0].at<double>(1, 0)).mul(v_mask) +
								(2 * c1para[1].at<double>(0, 0)*u_mask + c1para[1].at<double>(1, 0));

							//cout << JT11 << endl;

							JT12 = c1para[0].at<double>(0, 0)*(u_mask_2)+
								c1para[0].at<double>(1, 0)*u_mask + c1para[0].at<double>(2, 0);

							JT21 = (2 * c1para[2].at<double>(0, 0)*u_mask + c1para[2].at<double>(1, 0)).mul(v_mask) +
								(2 * c1para[3].at<double>(0, 0)*u_mask + c1para[3].at<double>(1, 0));

							JT22 = c1para[2].at<double>(0, 0)*(u_mask_2)+
								c1para[2].at<double>(1, 0)*u_mask + c1para[2].at<double>(2, 0);
						}
						else
						{
							/*cout << c1para[0] << endl;
							cout << c1para[1] << endl;
							cout << c1para[2] << endl;
							cout << c1para[3] << endl;*/
							JT11 = (3 * c1para[0].at<double>(0, 0)*(u_mask_2)+2 * c1para[0].at<double>(1, 0)*u_mask +
								c1para[0].at<double>(2, 0)).mul(v_mask) + (2 * c1para[1].at<double>(0, 0)*u_mask + c1para[1].at<double>(1, 0));
							
							JT12 = c1para[0].at<double>(0, 0)*(u_mask_3)+c1para[0].at<double>(1, 0)*(u_mask_2)+c1para[0].at<double>(2, 0)*u_mask + c1para[0].at<double>(3, 0);
							
							//cout << c1para[0] << endl;

							JT21 = (3 * c1para[2].at<double>(0, 0)*(u_mask_2)+2 * c1para[2].at<double>(1, 0)*u_mask +
								c1para[2].at<double>(2, 0)).mul(v_mask) + (2 * c1para[3].at<double>(0, 0)*u_mask + c1para[3].at<double>(1, 0));

							//cout << J21 << endl;
							//cout << JT21 << endl;

							JT22 = c1para[2].at<double>(0, 0)*(u_mask_3)+c1para[2].at<double>(1, 0)*(u_mask_2)+c1para[2].at<double>(2, 0)*u_mask + c1para[2].at<double>(3, 0);
						}

						tmp11 = JT11.mul(J11) + JT12.mul(J21);
						tmp12 = JT11.mul(J12) + JT12.mul(J22);
						tmp21 = JT21.mul(J11) + JT22.mul(J21);
						tmp22 = JT21.mul(J12) + JT22.mul(J22);
						
						//cout << J11 << endl;

						J11 = tmp11; 
						//cout << J11 << endl;
						J12 = tmp12; 
						J21 = tmp21; 
						J22 = tmp22;

					}

					//cout << J11 << endl;
					//cout << J12 << endl;
					//cout << J21 << endl;
					//cout << J22 << endl;

					array2Matrix(J11, reg_mask[reg], J11_map);
					array2Matrix(J12, reg_mask[reg], J12_map);
					array2Matrix(J21, reg_mask[reg], J21_map);
					array2Matrix(J22, reg_mask[reg], J22_map);

					//cout << J11_map << endl;
					//cout << J12_map << endl;
					//cout << J21_map << endl;
					//cout << J22_map << endl;

				}

				Scalar s1 = (sum(J11_map) + sum(J22_map)) / (J11_map.cols*J11_map.rows * 2);
				double avg_alpha = s1.val[0];
				Scalar s2 = (sum(J21_map) - sum(J12_map)) / (J11_map.cols*J11_map.rows * 2);
				double avg_beta = s2.val[0];
				//这里之前也写错了
				J11_map = J11_map - avg_alpha;
				J12_map = J12_map + avg_beta;
				J21_map = J21_map - avg_beta;
				J22_map = J22_map - avg_alpha;

				Mat J11_map_2, J12_map_2, J21_map_2, J22_map_2;
				pow(J11_map, 2, J11_map_2); pow(J12_map, 2, J12_map_2);
				pow(J21_map, 2, J21_map_2); pow(J22_map, 2, J22_map_2);

				//cout << J11_map << endl;

				cost[num] = sum(J11_map_2 + J12_map_2 + J21_map_2 + J22_map_2);

			}

			double totalcost = cost[0].val[0] + cost[1].val[0];

			data_totalcost[i_cost] = totalcost;
			i_cost++;
		}
		j_cost++;
	}
	double min_value, max_value;
	Point min_value_loc=findMinLocation(totalcost_table);
	//minMaxLoc(totalcost_table, &min_value, &max_value, &min_value_loc, 0);

	ub1 = oriub1 + min_value_loc.x * 10 - 300;
	ub2 = oriub2 + min_value_loc.y * 10 - 300;

	cout << "ub1=" << ub1 << "," << "ub2=" << ub2 << endl;

	computeC1Params(H01, t, c, theta, ub1, ub2, zeroR_on, c1para);

	int gridSize = 10;
	////texture mapping
	//void textureMapping(Mat* srcImages, int gridSize, int imgW, int imgH, int warpType, Mat*T, Mat* H, Mat* c1para, Mat &dst)
	textureMapping(images, gridSize, imgW, imgH, warpType, T, H01, H, H01_matrices, c1para, theta, dst, ub1, ub2, zeroR_on);

	return 1;
}

Point findMinLocation(Mat totalCostTable)
{
	double minValue = 1000000;
	Point location;
	for (int i = 0; i < totalCostTable.rows; i++)
	{
		double *data = totalCostTable.ptr<double>(i);
		for (int j = 0; j < totalCostTable.cols; j++)
		{
			if (data[j] == 0)
			{
				continue;
			}
			else
			{
				if (data[j] < minValue)
				{
					minValue = data[j];
					location.x = j;
					location.y = i;
				}
			}
		}
	}
	return location;
}

void computeC1Params(Mat H, Mat t, double c, double theta, double ub1, double ub2, int zeroRon, Mat *params)
{
	//params[0-4] a b e f S

	double p1 = H.at<double>(0, 0)*cosf(theta) + H.at<double>(0, 1)*sinf(theta);
	double p2 = H.at<double>(1, 0)*cosf(theta) + H.at<double>(1, 1)*sinf(theta);

	double q1 = -H.at<double>(0, 0)*sinf(theta) + H.at<double>(0, 1)*cosf(theta);
	double q2 = -H.at<double>(1, 0)*sinf(theta) + H.at<double>(1, 1)*cosf(theta);

	//A & E
	Mat G = Mat(4, 4, CV_64F);
	G.at<double>(0, 0) = ub1*ub1; G.at<double>(0, 1) = ub1; G.at<double>(0, 2) = 1; G.at<double>(0, 3) = 0;
	G.at<double>(1, 0) = ub2*ub2; G.at<double>(1, 1) = ub2; G.at<double>(1, 2) = 1; G.at<double>(1, 3) = 1;
	G.at<double>(2, 0) = ub1 * 2; G.at<double>(2, 1) = 1; G.at<double>(2, 2) = 0; G.at<double>(2, 3) = 0;
	G.at<double>(3, 0) = ub2 * 2; G.at<double>(3, 1) = 1; G.at<double>(3, 2) = 0; G.at<double>(3, 3) = 0;

	Mat ga = Mat(4, 1, CV_64F);
	Mat ge = Mat(4, 1, CV_64F);
	Mat sol = Mat(4, 1, CV_64F);

	ga.at<double>(0, 0) = q1 / (1 - c*ub1);
	ga.at<double>(1, 0) = 0;
	ga.at<double>(2, 0) = c*q1 / ((1 - c*ub1) *(1 - c*ub1));
	ga.at<double>(3, 0) = 0;

	ge.at<double>(0, 0) = q2 / (1 - c*ub1);
	ge.at<double>(1, 0) = 0;
	ge.at<double>(2, 0) = c*q2 / ((1 - c*ub1) *(1 - c*ub1));
	ge.at<double>(3, 0) = 0;

	sol = G.inv()*ga;
	sol.rowRange(0, 3).copyTo(params[0]);
	double beta = sol.at<double>(3, 0);

	sol = G.inv()*ge;
	sol.rowRange(0, 3).copyTo(params[2]);
	double alpha = -sol.at<double>(3, 0);

	//compute again if angle of R(x,y) is constrained to be zero
	double scaling = sqrt(alpha*alpha + beta*beta);

	if (zeroRon)
	{
		G.at<double>(0, 0) = ub1*ub1*ub1; G.at<double>(0, 1) = ub1*ub1; G.at<double>(0, 2) = ub1; G.at<double>(0, 3) = 1;
		G.at<double>(1, 0) = ub2*ub2*ub2; G.at<double>(1, 1) = ub2*ub2; G.at<double>(1, 2) = ub2; G.at<double>(1, 3) = 1;
		G.at<double>(2, 0) = ub1 * ub1 * 3; G.at<double>(2, 1) = 2 * ub1; G.at<double>(2, 2) = 1; G.at<double>(2, 3) = 0;
		G.at<double>(3, 0) = ub2 * ub2 * 3; G.at<double>(3, 1) = 2 * ub2; G.at<double>(3, 2) = 1; G.at<double>(3, 3) = 0;

		ga.at<double>(0, 0) = q1 / (1 - c*ub1);
		ga.at<double>(1, 0) = -scaling*sinf(theta);
		ga.at<double>(2, 0) = c*q1 / ((1 - c*ub1) *(1 - c*ub1));
		ga.at<double>(3, 0) = 0;
		sol = G.inv()*ga;
		sol.copyTo(params[0]);
		//cout << params[0] << endl;
		beta = scaling*sinf(theta);

		ge.at<double>(0, 0) = q2 / (1 - c*ub1);
		ge.at<double>(1, 0) = scaling*cosf(theta);//这里之前也写错了。。。。
		ge.at<double>(2, 0) = c*q2 / ((1 - c*ub1) *(1 - c*ub1));
		ge.at<double>(3, 0) = 0;
		sol = G.inv()*ge;
		sol.copyTo(params[2]);
		//cout << params[2] << endl;
		alpha = scaling*cosf(theta);
	}

	// B&F
	G.at<double>(0, 0) = ub1*ub1; G.at<double>(0, 1) = ub1; G.at<double>(0, 2) = 1; G.at<double>(0, 3) = 0;
	G.at<double>(1, 0) = ub2*ub2; G.at<double>(1, 1) = ub2; G.at<double>(1, 2) = 1; G.at<double>(1, 3) = 1;
	G.at<double>(2, 0) = ub1 * 2; G.at<double>(2, 1) = 1; G.at<double>(2, 2) = 0; G.at<double>(2, 3) = 0;
	G.at<double>(3, 0) = ub2 * 2; G.at<double>(3, 1) = 1; G.at<double>(3, 2) = 0; G.at<double>(3, 3) = 0;

	Mat gb = Mat(4, 1, CV_64F);
	Mat gf = Mat(4, 1, CV_64F);

	gb.at<double>(0, 0) = (p1*ub1 + H.at<double>(0, 2)) / (1 - c*ub1);
	gb.at<double>(1, 0) = alpha*ub2;
	gb.at<double>(2, 0) = (p1 + c*H.at<double>(0, 2)) / ((1 - c*ub1)*(1 - c*ub1));
	gb.at<double>(3, 0) = alpha;
	sol = G.inv()*gb;
	sol.rowRange(0, 3).copyTo(params[1]);
	//cout << params[1] << endl;
	double transx = -sol.at<double>(3, 0);

	gf.at<double>(0, 0) = (p2*ub1 + H.at<double>(1, 2)) / (1 - c*ub1);
	gf.at<double>(1, 0) = beta*ub2;
	gf.at<double>(2, 0) = (p2 + c*H.at<double>(1, 2)) / ((1 - c*ub1)*(1 - c*ub1));
	gf.at<double>(3, 0) = beta;

	sol = G.inv()*gf;
	sol.rowRange(0, 3).copyTo(params[3]);
	//cout << params[3] << endl;
	double transy = -sol.at<double>(3, 0);

	//S
	params[4] = Mat(3, 3, CV_64F);
	params[4].at<double>(0, 0) = alpha; params[4].at<double>(0, 1) = -beta; params[4].at<double>(0, 2) = transx;
	params[4].at<double>(1, 0) = beta; params[4].at<double>(1, 1) = alpha; params[4].at<double>(1, 2) = transy;
	params[4].at<double>(2, 0) = 0; params[4].at<double>(2, 1) = 0; params[4].at<double>(2, 2) = 1;

	Mat theta_cos_sin = Mat(3, 3, CV_64F);
	theta_cos_sin.at<double>(0, 0) = cosf(theta); theta_cos_sin.at<double>(0, 1) = sinf(theta); theta_cos_sin.at<double>(0, 2) = 0;
	theta_cos_sin.at<double>(1, 0) = -sinf(theta); theta_cos_sin.at<double>(1, 1) = cosf(theta); theta_cos_sin.at<double>(1, 2) = 0;
	theta_cos_sin.at<double>(2, 0) = 0; theta_cos_sin.at<double>(2, 1) = 0; theta_cos_sin.at<double>(2, 2) = 1;
	params[4] = params[4] * (theta_cos_sin);

	if (zeroRon)
	{
		params[4].at<double>(1, 0) = 0;
		params[4].at<double>(0, 1) = 0;
	}
}




Mat calculateHomography(Mat image1, Mat image2,Mat* Hmatrices)
{
	Mat H_global;
	Mat H_top, H_down;
	vector<cv::KeyPoint> keyPoints_0, keyPoints_1;
	Mat descriptors_0, descriptors_1;
	
	Mat image1Equalized, image2Equalized;
	//equalizeHist(image1, image1Equalized);
	//equalizeHist(image2, image2Equalized);

	//Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();


	//vector<KeyPoint> kp1, kp2;
	Mat d1, d2;
	vector<DMatch> matches_all, matches_gms;

	Ptr<ORB> orb = ORB::create(10000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(image1, Mat(), keyPoints_0, d1);
	orb->detectAndCompute(image2, Mat(), keyPoints_1, d2);

#ifdef USE_GPU
	GpuMat gd1(d1), gd2(d2);
	Ptr<cuda::DescriptorMatcher> matcher = cv::cuda::DescriptorMatcher::createBFMatcher(NORM_HAMMING);
	matcher->match(gd1, gd2, matches_all);
#else
	BFMatcher matcher(NORM_HAMMING);
	matcher.match(d1, d2, matches_all);
#endif

	// GMS filter need opencv 3.x
	int num_inliers = 0;
	std::vector<bool> vbInliers;
	gms_matcher gms(keyPoints_0, image1.size(), keyPoints_1, image2.size(), matches_all);
	num_inliers = gms.GetInlierMask(vbInliers, false, false);

	cout << "Get total " << num_inliers << " matches." << endl;

	// draw matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		if (vbInliers[i] == true)
		{
			matches_gms.push_back(matches_all[i]);
		}
	}



	///*opencv 2.x 版本
	//SiftFeatureDetector sift;
	//sift.detect(image1Equalized, keyPoints_0);
	//sift.detect(image2Equalized, keyPoints_1);
	//SiftDescriptorExtractor pExtractor;
	//pExtractor.compute(image1, keyPoints_0, descriptors_0);
	//pExtractor.compute(image2, keyPoints_1, descriptors_1);*/
	//f2d->detect(image1Equalized, keyPoints_0);
	//f2d->detect(image2Equalized, keyPoints_1);
	//f2d->compute(image1, keyPoints_0, descriptors_0);
	//f2d->compute(image2, keyPoints_1, descriptors_1);
	///*ASiftDetector asift;
	//asift.detectAndCompute(image1, keyPoints_0, descriptors_0);
	//asift.detectAndCompute(image2, keyPoints_1, descriptors_1);*/
	////BruteForceMatcher <L2<double>>matcher;
	//FlannBasedMatcher matcher;
	//vector<Point2f> pointsMatchPre, pointsMatchPost;
	////FlannBasedMatcher matcher;
	//std::vector< DMatch > matches;
	//matcher.match(descriptors_0, descriptors_1, matches);
	//double max_dist = 0; double min_dist = 100;
	////-- Quick calculation of max and min distances between keypoints
	//for (int i = 0; i < descriptors_0.rows; i++)
	//{
	//	double dist = matches[i].distance;
	//	if (dist < min_dist) min_dist = dist;
	//	if (dist > max_dist) max_dist = dist;
	//}
	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n", min_dist);
	////-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	//std::vector< DMatch > good_matches;
	//for (int i = 0; i < descriptors_0.rows; i++)
	//{
	//	if (matches[i].distance < 3.5 * min_dist)
	//	{
	//		good_matches.push_back(matches[i]);
	//	}
	//}
	////-- Localize the object
	//std::vector<Point2f> obj;//1
	//std::vector<Point2f> scene;//2
	//for (int i = 0; i < good_matches.size(); i++)
	//{
	//	//-- Get the keypoints from the good matches
	//	obj.push_back(keyPoints_0[good_matches[i].queryIdx].pt);
	//	scene.push_back(keyPoints_1[good_matches[i].trainIdx].pt);
	//}

	

	std::vector<Point2f> obj_global;//1
	std::vector<Point2f> scene_global;//2

	std::vector<Point2f> obj_top;//1
	std::vector<Point2f> scene_top;//2

	std::vector<Point2f> obj_down;//1
	std::vector<Point2f> scene_down;//2

	for (int i = 0; i < matches_gms.size(); i++)
	{
		//-- Get the keypoints from the good matches
		Point2f kPoint0 = keyPoints_0[matches_gms[i].queryIdx].pt;
		Point2f kPoint1 = keyPoints_1[matches_gms[i].trainIdx].pt;

		obj_global.push_back(kPoint0);
		scene_global.push_back(kPoint1);

		Point2f midPoint;
		midPoint.y = (kPoint0.y + kPoint1.y) / 2;

		if (midPoint.y < (image1.rows) / 2)
		{
			obj_top.push_back(kPoint0);
			scene_top.push_back(kPoint1);
		}
		else
		{
			obj_down.push_back(kPoint0);
			scene_down.push_back(kPoint1);
		}


	}

	H_global = cv::findHomography(obj_global, scene_global, RANSAC);
	cout << "global homography matrix" << endl;
	cout << H_global << endl<<endl;

	H_top = cv::findHomography(obj_top, scene_top, RANSAC);
	cout << "top homography matrix" << endl;
	cout << H_top << endl << endl;

	H_down = cv::findHomography(obj_down, scene_down, RANSAC);
	cout << "bottom homography matrix" << endl;
	cout << H_down << endl << endl;

	Mat img_matches;
	drawMatches(image1, keyPoints_0, image2, keyPoints_1,
		matches_gms, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow("img_matches", img_matches);
	waitKey(0);
	
	Hmatrices[0] = H_global;
	Hmatrices[1] = H_top;
	Hmatrices[2] = H_down;

	return H_global;
}
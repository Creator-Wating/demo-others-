
// demoDlg.cpp: 实现文件
//

#include "stdafx.h"
#include "demo.h"
#include "demoDlg.h"
#include "afxdialogex.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>  //后加1
#include <opencv2/core/core.hpp>  
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/highgui/highgui_c.h>
#include<math.h>
#include<vector>
#include <stack>

using namespace std;
using namespace cv;//后加1

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define MAX_CLASS_NUM 20
#define O_THRESHOLD 2.1
#define  L_THRESHOLD 1.5
#define OBJ_CLASS 7


/*
Input:
src: 待检测连通域的二值化图像
Output:
dst: 标记后的图像
featherList: 连通域特征的清单
return：
连通域数量。
*/
class Feather
{
public:
	int area;
	int label;
	int leftx;
	int upy;
	int width;
	int height;
	Rect boundingbox;
};

Mat show_result(vector<Feather>& featherList, cv::Mat& dst, Mat& edge, int* obj_num, Mat& src, int choose);
void mysort(int* a, int* b, int* c);
void rec_obj(int* obj_num, int* s_area, int* area_idx, float* maxx, float* maxy, float* minx, float* miny, int* bnum, int choose);
int coin_based(int norm_area, int area, float error);
int coin_based2(int norm_area, int area, float error);
int bwLabel(Mat& src, Mat& dst, vector<Feather>& featherList, int choose);
void cannyOperator(Mat& imgOri, Mat& imgOpr);
void DoubleThresholdLink(Mat& imageInput, double lowTh, double highTh);
void DoubleThreshold(Mat& imageInput, const double lowThreshold, const double highThreshold);
void NonLocalMaxValue(const Mat imageInput, Mat& imageOutput, const Mat& theta, const Mat& imageX, const Mat& imageY);
void GradDirection(const Mat imageSource, Mat& imageX, Mat& imageY, Mat& gradXY, Mat& theta);
void Gaussian_kernel(int kernel_size, int sigma, Mat& kernel);
void MVImageToMat(cv::Mat& mat, MVImage& mvImage);
void MatToMVImage(cv::Mat& mat, MVImage& m_image);
double GaussianKenel(int x1, int y1, int x2, int y2, double sigma);
Mat GaussianB1ur(Mat src, int size, int sigma);
Mat otsu_p(Mat dst);


//图片格式转换
void MVImageToMat(cv::Mat& mat, MVImage& mvImage)
{
	mat.release();

	//create new CImage  
	int width = mvImage.GetWidth();
	int height = mvImage.GetHeight();

	mat = cv::Mat(height, width, CV_8UC3);

	BYTE* iPtr = (BYTE*)mvImage.GetBits();

#pragma omp parallel for
	for (int i = 0; i < height; i++)
	{
		BYTE* sPtr = iPtr + i * width * 3;
		uchar* dPtr = mat.ptr<uchar>(i);
		for (int j = 0; j < width; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				dPtr[3 * j + k] = sPtr[j * 3 + k];
			}
		}
	}
}

void MatToMVImage(cv::Mat& mat, MVImage& m_image)
{
	//create new CImage  
	int width = mat.cols;
	int height = mat.rows;
	int channels = mat.channels();

	m_image.Destroy(); //clear  
	m_image.Create(width, height, 8 * channels); //默认图像像素单通道占用1个字节  

	//copy values  
	uchar* ps;
	uchar* pimg = (uchar*)m_image.GetBits(); //A pointer to the bitmap buffer  
	int step = m_image.GetPitch();

	for (int i = 0; i < height; ++i)
	{
		ps = (mat.ptr<uchar>(i));
		for (int j = 0; j < width; ++j)
		{
			if (channels == 1) //gray  
			{
				*(pimg + i * step + j) = ps[j];
			}
			else if (channels == 3) //color  
			{
				for (int k = 0; k < 3; ++k)
				{
					*(pimg + i * step + j * 3 + k) = ps[j * 3 + k];
				}
			}
		}
	}
}

// CdemoDlg 对话框




CdemoDlg::CdemoDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_DEMO_DIALOG, pParent)
	, m_bRun(FALSE)
	, m_hCam(NULL)

{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CdemoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CdemoDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_OpenCam, &CdemoDlg::OnBnClickedOpencam)
	ON_BN_CLICKED(IDC_StartGrab, &CdemoDlg::OnBnClickedStartgrab)
	ON_BN_CLICKED(IDC_CloseCam, &CdemoDlg::OnBnClickedClosecam)
	//ON_BN_CLICKED(IDC_BUTTON1, &CdemoDlg::OnBnClickedButton1)
	ON_EN_CHANGE(IDC_EDIT5, &CdemoDlg::OnEnChangeEdit5)
END_MESSAGE_MAP()


// CdemoDlg 消息处理程序

BOOL CdemoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	MVSTATUS_CODES r;
	r = MVInitLib();
	if (r != MVST_SUCCESS)
	{
		MessageBox(_T("函数库初始化失败！"), _T("提示"), MB_ICONWARNING);
		return TRUE;
	}
	r = MVUpdateCameraList();
	if (r != MVST_SUCCESS)
	{
		MessageBox(_T("查找连接计算机的相机失败！"),_T( "提示"), MB_ICONWARNING);
		return TRUE;
	}
	GetDlgItem(IDC_OpenCam)->EnableWindow(true);
	GetDlgItem(IDC_StartGrab)->EnableWindow(false);
	GetDlgItem(IDC_CloseCam)->EnableWindow(false);


	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CdemoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CdemoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CdemoDlg::DrawImage()
{
	CRect rct;
	GetDlgItem(pic)->GetClientRect(&rct);
	int dstW = rct.Width();
	int dstH = rct.Height();
	CDC *pDC = GetDlgItem(pic)->GetDC();
	{
		pDC->SetStretchBltMode(COLORONCOLOR);
		m_image.Draw(pDC->GetSafeHdc(), 0, 0, dstW, dstH);
	}
	ReleaseDC(pDC);
}
int CdemoDlg::OnStreamCB(MV_IMAGE_INFO *pInfo)
{
	MVInfo2Image(m_hCam, pInfo, &m_image);
	Mat image;
	//格式转换
	//获取图像宽
	int w = m_image.GetWidth();
	//获取图像高
	int h = m_image.GetHeight();
	Mat src;
	Mat src1;
	src.create(h, w, CV_8UC3);
	MVSTATUS_CODES r;//错误枚举对象
	r = MVBayerToBGR(m_hCam, pInfo->pImageBuffer, src.data, w * 3, w, h, m_PixelFormat);
	//图像处理
	GaussianBlur(src,src,Size(9,9),2);
	src = otsu_p(src);
	int choose = 1;

	if (choose == 1)
	{
		Mat structElement2 = getStructuringElement(MORPH_RECT, Size(9, 9), Point(-1, -1));
		erode(src, src, structElement2);
	}

	Mat edge = Mat::zeros(src.rows, src.cols, src.type()); //Canny
	Mat dst;
	cannyOperator(src, edge);//用canny算子进行边缘处理
	vector<Feather> featherList; // 区域生长实现图像分割
	bwLabel(src, dst, featherList, choose);
	int obj_num[OBJ_CLASS] = { 0 }; //特征分类

	src1 =  show_result(featherList, dst, edge, obj_num, src, choose);

	CString text1, text2, text3, text4;
	text1.Format(_T("%d"), obj_num[5]);
	text2.Format(_T("%d"), obj_num[0]);
	text3.Format(_T("%d"), obj_num[1]);
	text4.Format(_T("%d"), obj_num[4]);
	

	GetDlgItem(IDC_EDIT1)->SetWindowText(text1);
	GetDlgItem(IDC_EDIT2)->SetWindowText(text2);
	GetDlgItem(IDC_EDIT3)->SetWindowText(text3);
	GetDlgItem(IDC_EDIT4)->SetWindowText(text4);

	MatToMVImage(src1, m_image);
	DrawImage();
	return 0;
}
int __stdcall StreamCB(MV_IMAGE_INFO *pInfo, ULONG_PTR nUserVal)
{
	CdemoDlg *pDlg = (CdemoDlg *)nUserVal;
	return (pDlg->OnStreamCB(pInfo));
}


void CdemoDlg::OnBnClickedOpencam()
{
	// TODO: 在此添加控件通知处理程序代码
	int nCams = 0;
	MVGetNumOfCameras(&nCams);
	if (nCams == 0)
	{
		MessageBox(_T(" 没 有 找 到 相 机 , 请 确 认 连 接 和 相 机 IP 设 置 "), _T(" 提 示"),MB_ICONWARNING);
			return;
	}
	MVSTATUS_CODES r = MVOpenCamByIndex(0, &m_hCam);
	if (m_hCam == NULL)
	{
		if (r == MVST_ACCESS_DENIED)
			MessageBox(_T(" 无 法 打 开 相 机 ， 可 能 正 被 别 的 软 件 控 制 "),_T( " 提 示 "), MB_ICONWARNING);
		else
			MessageBox(_T("无法打开相机"),_T( "提示"), MB_ICONWARNING);
		return;
	}
	int w, h;
	MVGetWidth(m_hCam, &w);
	MVGetHeight(m_hCam, &h);
	MVGetPixelFormat(m_hCam, &m_PixelFormat);
	m_image.CreateByPixelFormat(w, h, m_PixelFormat);
	GetDlgItem(IDC_OpenCam)->EnableWindow(false);
	GetDlgItem(IDC_StartGrab)->EnableWindow(true);
	GetDlgItem(IDC_CloseCam)->EnableWindow(false);
}


void CdemoDlg::OnBnClickedStartgrab()
{
	// TODO: 在此添加控件通知处理程序代码
	TriggerModeEnums enumMode;
	MVGetTriggerMode(m_hCam, &enumMode);
	if (enumMode != TriggerMode_Off)
	{
		MVSetTriggerMode(m_hCam, TriggerMode_Off);
		Sleep(100);
	}

	MVSetPacketSize(m_hCam, 1316);//修改包的大小

	MVStartGrab(m_hCam, StreamCB, (ULONG_PTR)this);
	m_bRun = true;
	GetDlgItem(IDC_OpenCam)->EnableWindow(false);
	GetDlgItem(IDC_StartGrab)->EnableWindow(false);
	GetDlgItem(IDC_CloseCam)->EnableWindow(true);
}


void CdemoDlg::OnBnClickedClosecam()
{
	// TODO: 在此添加控件通知处理程序代码
	MVStopGrab(m_hCam);
	MVCloseCam(m_hCam);
	m_bRun = false;
	GetDlgItem(IDC_OpenCam)->EnableWindow(true);
	GetDlgItem(IDC_StartGrab)->EnableWindow(false);
	GetDlgItem(IDC_CloseCam)->EnableWindow(false);
}
void CdemoDlg::OnClose()
{
	if (m_bRun != false)
	{
		MVStopGrab(m_hCam);
	}
	MVTerminateLib();
	CDialog::OnClose();
}

//guass&otsu
double GaussianKenel(int x1, int y1, int x2, int y2, double sigma)
{
	const double PI = 4.0*atan(1.0); //圆周率π赋值 
	double absX = pow(abs(x1 - x2), 2);
	double absY = pow(abs(y1 - y2), 2);
	double k = (1 / (2 * PI*sigma*sigma))*exp(-(absX + absY) / (2 * pow(sigma, 2)));
	return k;
}
Mat GaussianB1ur(Mat src, int size, int sigma)
{
	if (size % 2 != 1 || size <= 0)
	{
		cerr << "Filter Size must be a positive odd number!" << endl;
		return src;
	}
	int len = size / 2;
	Mat resultImg = Mat::zeros(src.size(), src.type());
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			double k = 0;
			double f = 0;
			//不对边缘进行处理 
			if ((i - len) >= 0 && (i + len) < src.rows && (j - len) >= 0 && (j + len) < src.cols)
			{
				for (int r = i - len; r <= i + len; r++)
				{
					for (int c = j - len; c <= j + len; c++)
					{
						f = f + src.ptr<uchar>(r)[c] * GaussianKenel(i, j, r, c, sigma);
						k += GaussianKenel(i, j, r, c, sigma);
					}
				}
				int value = f / k;
				if (value < 0)
					value = 0;
				else if (value > 255)
					value = 255;
				resultImg.ptr<uchar>(i)[j] = (uchar)value;
			}
		}
	}
	return resultImg;
}

Mat otsu_p(Mat dst)
{
	//OTSU
	int m, n, q, t_t = 0, L = 256;//m(长)*n（高）=q的图片,t_t是阈值灰度, L为灰度
	int i = 0, j = 0, k = 0;//循环用
	double h_all = 0, h_c0 = 0, h_c1 = 0;//h_all整个图像的均值, h_c0, h_c1
	double p[256] = { 0 }, t[256] = { 0 }, hd[256] = { 0 };// p为对应灰度级的概率，t为每个阈值灰度的判决准则
	double b_0 = 0, b_1 = 0, b_w = 0, b_b = 0, b_t = 0;//b_0,1为类的方差，b_w为类内方差, b_b为类间方差, b_t为总体方差；
	double a = 0, b = 0, r = 0;//循环用
	Mat img_g;

	//img_g = dst;

	cvtColor(dst,img_g,COLOR_RGB2GRAY);

	m = img_g.rows;
	n = img_g.cols;
	q = m * n;


	//求每个灰度的像素数
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			k = img_g.at<uchar>(i, j);
			hd[k] ++;
		}
	}


	//求每个灰度值的概率
	for (i = 0; i < L; i++)
	{
		p[i] = hd[i] / q;
	}


	//求整个图像的均值
	for (i = 0; i < L; i++)
	{
		h_all += i * p[i];
	}

	for (i = 1; i < L - 1; i++)//i为对应每个阈值梯度
	{
		a = 0;
		h_c0 = 0;
		h_c1 = 0;
		b_0 = 0;
		b_1 = 0;
		b_w = 0;
		b_t = 0;
		b_b = 0;

		//求w0
		for (j = 0; j < i + 1; j++)
		{
			a += p[j];
		}

		b = 1 - a;

		//求c0与c1的均值
		for (j = 0; j < i + 1; j++)
		{
			h_c0 += j * p[j] / a;
		}
		for (j = i + 1; j < L; j++)
		{
			h_c1 += j * p[j] / b;
		}

		//求类的方差
		for (j = 0; j < i + 1; j++)
		{
			b_0 += (j - h_c0) * (j - h_c0) * p[j] / a;
		}
		for (j = i + 1; j < L; j++)
		{
			b_1 += (j - h_c1) * (j - h_c1) * p[j] / b;
		}

		//求类内方差
		b_w = a * b_0 + b * b_1;

		//求类间方差
		b_b = a * b*(h_c0 - h_c1)*(h_c0 - h_c1);

		//求总方差
		b_t = b_b + b_w;

		//计算判别准则
		t[i] = b_b / b_t;
	}

	//比较得出t,k判别依据
	for (i = 0; i < L; i++)
	{
		if (t[i] > r)
		{
			r = t[i];
			t_t = i;
		}
	}

	//修改图片
	for (i = 0; i < m; i++)
	{
		uchar* data2 = img_g.ptr<uchar>(i);  //获取第i行的首地址
		for (j = 0; j < n; j++)
		{
			if (*(data2 + j) > t_t)
				*(data2 + j) = 0;
			else
				*(data2 + j) = 255;
		}
	}

	return img_g;
}

/*
计算梯度值和方向
imageSource 原始灰度图
imageX X方向梯度图像
imageY Y方向梯度图像
gradXY 该点的梯度幅值
pointDirection 梯度方向角度
*/
void Gaussian_kernel(int kernel_size, int sigma, Mat& kernel)
{
	const double PI = 3.1415926;
	int m = kernel_size / 2;
	kernel = Mat(kernel_size, kernel_size, CV_32FC1);
	float s = 2 * sigma * sigma;
	for (int i = 0; i < kernel_size; i++)
	{
		for (int j = 0; j < kernel_size; j++)
		{
			int x = i - m;
			int y = j - m;
			kernel.at<float>(i, j) = exp(-(x * x + y * y) / s) / (PI * s);
		}
	}
}

void GradDirection(const Mat imageSource, Mat& imageX, Mat& imageY, Mat& gradXY, Mat& theta)
{
	imageX = Mat::zeros(imageSource.size(), CV_32SC1);
	imageY = Mat::zeros(imageSource.size(), CV_32SC1);
	gradXY = Mat::zeros(imageSource.size(), CV_32SC1);
	theta = Mat::zeros(imageSource.size(), CV_32SC1);
	int rows = imageSource.rows;
	int cols = imageSource.cols;
	int stepXY = imageX.step;
	int step = imageSource.step;
	/*
	Mat.step参数指图像的一行实际占用的内存长度，
	因为opencv中的图像会对每行的长度自动补齐（8的倍数），
	编程时尽量使用指针，指针读写像素是速度最快的，使用at函数最慢。
	*/
	uchar* PX = imageX.data;
	uchar* PY = imageY.data;
	uchar* P = imageSource.data;
	uchar* XY = gradXY.data;
	for (int i = 1; i < rows - 1; i++)
	{
		for (int j = 1; j < cols - 1; j++)
		{
			int a00 = P[(i - 1) * step + j - 1];
			int a01 = P[(i - 1) * step + j];
			int a02 = P[(i - 1) * step + j + 1];
			int a10 = P[i * step + j - 1];
			int a11 = P[i * step + j];
			int a12 = P[i * step + j + 1];
			int a20 = P[(i + 1) * step + j - 1];
			int a21 = P[(i + 1) * step + j];
			int a22 = P[(i + 1) * step + j + 1];
			double gradY = double(a02 + 2 * a12 + a22 - a00 - 2 * a10 - a20);
			double gradX = double(a00 + 2 * a01 + a02 - a20 - 2 * a21 - a22);
			//PX[i*stepXY + j*(stepXY / step)] = abs(gradX);
			//PY[i*stepXY + j*(stepXY / step)] = abs(gradY);
			imageX.at<int>(i, j) = abs(gradX);
			imageY.at<int>(i, j) = abs(gradY);
			if (gradX == 0)
			{
				gradX = 0.000000000001;
			}
			theta.at<int>(i, j) = atan(gradY / gradX) * 57.3;
			theta.at<int>(i, j) = (theta.at<int>(i, j) + 360) % 360;
			gradXY.at<int>(i, j) = sqrt(gradX * gradX + gradY * gradY);
			//XY[i*stepXY + j*(stepXY / step)] = sqrt(gradX*gradX + gradY*gradY);
		}
	}
	convertScaleAbs(imageX, imageX);
	convertScaleAbs(imageY, imageY);
	convertScaleAbs(gradXY, gradXY);
}
/*
局部非极大值抑制
沿着该点梯度方向，比较前后两个点的幅值大小，若该点大于前后两点，则保留，
若该点小于前后两点任意一点，则置为0；
imageInput 输入得到梯度图像
imageOutput 输出的非极大值抑制图像
theta 每个像素点的梯度方向角度
imageX X方向梯度
imageY Y方向梯度
*/
void NonLocalMaxValue(const Mat imageInput, Mat& imageOutput, const Mat& theta, const Mat& imageX, const Mat& imageY)
{
	imageOutput = imageInput.clone();
	int cols = imageInput.cols;
	int rows = imageInput.rows;
	for (int i = 1; i < rows - 1; i++)
	{
		for (int j = 1; j < cols - 1; j++)
		{
			if (0 == imageInput.at<uchar>(i, j))
				continue;
			int g00 = imageInput.at<uchar>(i - 1, j - 1);
			int g01 = imageInput.at<uchar>(i - 1, j);
			int g02 = imageInput.at<uchar>(i - 1, j + 1);
			int g10 = imageInput.at<uchar>(i, j - 1);
			int g11 = imageInput.at<uchar>(i, j);
			int g12 = imageInput.at<uchar>(i, j + 1);
			int g20 = imageInput.at<uchar>(i + 1, j - 1);
			int g21 = imageInput.at<uchar>(i + 1, j);
			int g22 = imageInput.at<uchar>(i + 1, j + 1);
			int direction = theta.at<int>(i, j); //该点梯度的角度值
			int g1 = 0;
			int g2 = 0;
			int g3 = 0;
			int g4 = 0;
			double tmp1 = 0.0; //保存亚像素点插值得到的灰度数
			double tmp2 = 0.0;
			double weight = fabs((double)imageY.at<uchar>(i, j) / (double)imageX.at<uchar>(i, j));
			if (weight == 0)weight = 0.0000001;
			if (weight > 1)
			{
				weight = 1 / weight;
			}
			if ((0 <= direction && direction < 45) || 180 <= direction && direction < 225)
			{
				tmp1 = g10 * (1 - weight) + g20 * (weight);
				tmp2 = g02 * (weight)+g12 * (1 - weight);
			}
			if ((45 <= direction && direction < 90) || 225 <= direction && direction < 270)
			{
				tmp1 = g01 * (1 - weight) + g02 * (weight);
				tmp2 = g20 * (weight)+g21 * (1 - weight);
			}
			if ((90 <= direction && direction < 135) || 270 <= direction && direction < 315)
			{
				tmp1 = g00 * (weight)+g01 * (1 - weight);
				tmp2 = g21 * (1 - weight) + g22 * (weight);
			}
			if ((135 <= direction && direction < 180) || 315 <= direction && direction < 360)
			{
				tmp1 = g00 * (weight)+g10 * (1 - weight);
				tmp2 = g12 * (1 - weight) + g22 * (weight);
			}
			if (imageInput.at<uchar>(i, j) < tmp1 || imageInput.at<uchar>(i, j) < tmp2)
			{
				imageOutput.at<uchar>(i, j) = 0;
			}
		}
	}
}
/*
双阈值的机理是：
指定一个低阈值A，一个高阈值B，一般取B为图像整体灰度级分布的70%，且B为1.5到2倍大小的A；
灰度值小于A的，置为0,灰度值大于B的，置为255；
*/
void DoubleThreshold(Mat& imageInput, const double lowThreshold, const double highThreshold)
{
	int cols = imageInput.cols;
	int rows = imageInput.rows;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			double temp = imageInput.at<uchar>(i, j);
			temp = temp > highThreshold ? (255) : (temp);
			temp = temp < lowThreshold ? (0) : (temp);
			imageInput.at<uchar>(i, j) = temp;
		}
	}
}
/*
连接处理:
灰度值介于A和B之间的，考察该像素点临近的8像素是否有灰度值为255的，
若没有255的，表示这是一个孤立的局部极大值点，予以排除，置为0；
若有255的，表示这是一个跟其他边缘有“接壤”的可造之材，置为255，
之后重复执行该步骤，直到考察完之后一个像素点。
其中的邻域跟踪算法，从值为255的像素点出发找到周围满足要求的点，把满足要求的点设置为255，
然后修改i,j的坐标值，i,j值进行回退，在改变后的i,j基础上继续寻找255周围满足要求的点。
当所有连接255的点修改完后，再把所有上面所说的局部极大值点置为0；（算法可以继续优化）。
参数1，imageInput：输入和输出的梯度图像
参数2，lowTh:低阈值
参数3，highTh:高阈值
*/
void DoubleThresholdLink(Mat& imageInput, double lowTh, double highTh)
{
	int cols = imageInput.cols;
	int rows = imageInput.rows;
	for (int i = 1; i < rows - 1; i++)
	{
		for (int j = 1; j < cols - 1; j++)
		{
			double pix = imageInput.at<uchar>(i, j);
			if (pix != 255)continue;
			bool change = false;
			for (int k = -1; k <= 1; k++)
			{
				for (int u = -1; u <= 1; u++)
				{
					if (k == 0 && u == 0)continue;
					double temp = imageInput.at<uchar>(i + k, j + u);
					if (temp >= lowTh && temp <= highTh)
					{
						imageInput.at<uchar>(i + k, j + u) = 255;
						change = true;
					}
				}
			}
			if (change)
			{
				if (i > 1)i--;
				if (j > 2)j -= 2;
			}
		}
	}
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if (imageInput.at<uchar>(i, j) != 255)
			{
				imageInput.at<uchar>(i, j) = 0;
			}
		}
	}
}
void cannyOperator(Mat& imgOri, Mat& imgOpr)
{
	//高斯滤波
	Mat gausKernel;
	int kernel_size = 5;
	double sigma = 1;
	Gaussian_kernel(kernel_size, sigma, gausKernel);
	Mat gausImage;
	filter2D(imgOri, gausImage, imgOri.depth(), gausKernel);
	//计算XY方向梯度
	Mat imageX, imageY, imageXY;
	Mat theta;
	GradDirection(imgOri, imageX, imageY, imageXY, theta);
	//对梯度幅值进行非极大值抑制
	Mat localImage;
	NonLocalMaxValue(imageXY, localImage, theta, imageX, imageY);
	//双阈值算法检测和边缘连接
	DoubleThreshold(localImage, 60, 100);
	DoubleThresholdLink(localImage, 60, 100);
	imgOpr = localImage.clone();
}


int bwLabel(Mat& src, Mat& dst, vector<Feather>& featherList, int choose)
{
	int rows = src.rows;
	int cols = src.cols;
	int labelValue = 0;
	Point seed, neighbor;
	stack<Point> pointStack; // 堆栈
	int area = 0; // 用于计算连通域的面积
	int leftBoundary = 0; // 连通域的左边界，即外接最小矩形的左边框，横坐标值，依此类推
	int rightBoundary = 0;
	int topBoundary = 0;
	int bottomBoundary = 0;
	//int stacknum = 0;
	Rect box; // 外接矩形框
	Feather feather;
	featherList.clear(); // 清除数组
	dst.release();//对目的图像清除
	dst = src.clone();//将原图像克隆至目的图像
	for (int i = 0; i < rows; i++)
	{
		uchar* pRow = dst.ptr<uchar>(i);//将行矩阵储存进pRow中
		for (int j = 0; j < cols; j++)
		{
			if (pRow[j] == 255)
			{
				//stacknum = 0;
				area = 0;
				labelValue++; // labelValue最大为254，最小为1.
				seed = Point(j, i); // Point（横坐标，纵坐标）,将所有黑色点作为区域生长的种子
				dst.at<uchar>(seed) = labelValue;
				pointStack.push(seed);//将种子入栈
				area++;
				//stacknum++;
				leftBoundary = seed.x;//设置上下左右边界
				rightBoundary = seed.x;
				topBoundary = seed.y;
				bottomBoundary = seed.y;
				while (!pointStack.empty())//当堆栈不为空时
				{
					neighbor = Point(seed.x + 1, seed.y);
					if ((seed.x != (cols - 1)) && (dst.at<uchar>(neighbor) == 255))
					{
						dst.at<uchar>(neighbor) = labelValue;
						pointStack.push(neighbor);
						area++;
						//stacknum++;
						if (rightBoundary < neighbor.x)
							rightBoundary = neighbor.x;
					}
					neighbor = Point(seed.x, seed.y + 1);
					if ((seed.y != (rows - 1)) && (dst.at<uchar>(neighbor) == 255))
					{
						dst.at<uchar>(neighbor) = labelValue;
						pointStack.push(neighbor);
						area++;
						//stacknum++;
						if (bottomBoundary < neighbor.y)
							bottomBoundary = neighbor.y;
					}
					neighbor = Point(seed.x - 1, seed.y);
					if ((seed.x != 0) && (dst.at<uchar>(neighbor) == 255))
					{
						dst.at<uchar>(neighbor) = labelValue;
						pointStack.push(neighbor);
						area++;
						//stacknum++;
						if (leftBoundary > neighbor.x)
							leftBoundary = neighbor.x;
					}
					neighbor = Point(seed.x, seed.y - 1);
					if ((seed.y != 0) && (dst.at<uchar>(neighbor) == 255))
					{
						dst.at<uchar>(neighbor) = labelValue;
						pointStack.push(neighbor);
						area++;
						//stacknum++;
						if (topBoundary > neighbor.y)
							topBoundary = neighbor.y;
					}

					seed = pointStack.top();
					pointStack.pop();
					//stacknum--;
				}
				box = Rect(leftBoundary, topBoundary, rightBoundary - leftBoundary, bottomBoundary - topBoundary);
				//rectangle(src, box, 255);
				feather.area = area;
				if (choose == 1)
				{
					feather.boundingbox = Rect(leftBoundary - 5, topBoundary - 5, rightBoundary - leftBoundary + 10, bottomBoundary - topBoundary + 10);
				}
				else {
					feather.boundingbox = box;
				}
				feather.label = labelValue;
				feather.leftx = leftBoundary;
				feather.upy = topBoundary;
				feather.width = rightBoundary - leftBoundary;
				feather.height = bottomBoundary - topBoundary;
				featherList.push_back(feather);//记录特征
			}
		}
	}
	return  labelValue;
}
//得到识别的最终结果
Mat show_result(vector<Feather>& featherList, cv::Mat& dst, Mat& edge, int* obj_num, Mat& src, int choose)
{
	float bx[MAX_CLASS_NUM] = { 0 };//边缘x坐标均值
	float by[MAX_CLASS_NUM] = { 0 };
	int bnum[MAX_CLASS_NUM] = { 0 };//边缘点的个数
	float maxx[MAX_CLASS_NUM];
	float maxy[MAX_CLASS_NUM];
	float minx[MAX_CLASS_NUM];
	float miny[MAX_CLASS_NUM];
	int f_area[MAX_CLASS_NUM] = { 0 };//每个连通域的面积
	uchar label;
	//dst含有标签,b为边缘矩阵.讲道理b和dst的大小应该一致
	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			//255边缘，0-255 uchar
			if (edge.at<uchar>(i, j) == 255)
			{
				label = dst.at<uchar>(i, j);
				if (label == 0)//label0是背景，无需检测
				{
				}
				else//从label=1开始检测。统计canny提取到的边缘的均值（质心）
				{
					by[label - 1] += i;
					bx[label - 1] += j;
					bnum[label - 1]++;
				}
			}
		}
	}
	// 为了方便观察，可以将label“放大”
	for (int i = 0; i < dst.rows; i++)
	{
		uchar* p = dst.ptr<uchar>(i);
		for (int j = 0; j < dst.cols; j++)
		{
			p[j] = 30 * p[j];
		}
	}
	int i = 0;
	//排序。从而得到硬币在最前。
	//首先，要先将featherlist中的面积提取出来，这样后面可以直接对其做操作。
	for (vector<Feather>::iterator it = featherList.begin(); it < featherList.end(); it++)
	{
		if (bx[i] > 0)
		{
			//得到边缘中心
			bx[i] /= bnum[i];
			by[i] /= bnum[i];
			f_area[i] = it->area;
			rectangle(src, it->boundingbox, Scalar(255, 0, 0), 2);//原图中画出矩形
			minx[i] = min(bx[i] - it->leftx, it->width - bx[i] + it->leftx);//获得四个边界值
			miny[i] = min(by[i] - it->upy, it->height - by[i] + it->upy);
			maxx[i] = max(bx[i] - it->leftx, it->width - bx[i] + it->leftx);
			maxy[i] = max(by[i] - it->upy, it->height - by[i] + it->upy);
		}
		i++;
	}
	//imshow("1", src);

	int area_idx[MAX_CLASS_NUM] = { 0 };
	int s_area[MAX_CLASS_NUM] = { 0 };
	mysort(f_area, s_area, area_idx);
	//sort(f_area, s_area, cmp);
	//cv::sortIdx(f_area, area_idx, CV_SORT_DESCENDING);
	count(obj_num, s_area, area_idx, maxx, maxy, minx, miny, bnum, choose);//recognize objectusing set of features

	return src;
}

void mysort(int* a, int* b, int* c)
{
	//a:原数组，b排序后数组，c：序号
	for (int m = 0; m < MAX_CLASS_NUM; m++)
	{
		c[m] = m;
		b[m] = a[m];
	}
	for (int i = 0; i < MAX_CLASS_NUM; i++)//冒泡排序法。同时用c数组记录idx
	{
		for (int j = 0; j < MAX_CLASS_NUM - i - 1; j++)
		{
			if (b[j] < b[j + 1])
			{
				int temp = b[j];
				b[j] = b[j + 1];
				b[j + 1] = temp;
				int ind_temp = c[j];
				c[j] = c[j + 1];
				c[j + 1] = ind_temp;
			}
		}
	}
}
//对工件计数
void count(int* obj_num, int* s_area, int* area_idx, float* maxx, float* maxy, float* minx, float* miny, int* bnum, int choose)
{
	float all_err[MAX_CLASS_NUM];
	int norm_area = 0;
	int L_area = 0;
	for (int i = 0; i < MAX_CLASS_NUM; i++)
	{
		/*
		#define COIN 0
		#define ROSS 1
		#define ROAM 4
		#define L 5
		*/
		int temp = area_idx[i];
		all_err[temp] = abs(minx[temp] - miny[temp]) + abs(maxx[temp] - maxy[temp]);//获得综合的xy差值作为特征
			//若有硬币则在第一个
		if (s_area[i] > 10000 && s_area[i] < 80000 && all_err[temp] < O_THRESHOLD)//大于一定面积且长宽比小于阈值则为硬币
		{
			obj_num[0]++;//一元硬币个数加1
			norm_area = s_area[i];//得到标准面积
			continue;
		}
		if (norm_area == 0)//若未检测到硬币，直接设定标准面积值
		{
			norm_area = 76843;
		}
		//使用硬币作为标准面积
		int result = -1;
		result = coin_based(norm_area, s_area[i], all_err[temp]);
		if (result >= 0)
		{
			obj_num[result]++;
		}
		/*
		//利用xy差值判断是否为L
		if (all_err[temp] > L_THRESHOLD)
		{
		L_area = s_area[i];
		int result = L_based(L_area, s_area[i], all_err[temp]);
		if (result >= 0)
		{
		obj_num[result]++;
		}
		}*/
	}
		else
		{
		//使用硬币作为标准面积
		int result = -1;//同理
		if (choose == 1
			result = coin_based(norm_area, s_area[i], all_err[temp]);
			if (result >= 0)
			{
				obj_num[result]++;
			}
		}
}
}
int coin_based(int norm_area, int area, float error)//对象检测
{
	float a = norm_area;
	float b = area;
	float c = a / b;
	//此时硬币被正确检出，利用硬币的面积和其他特征进行识别
	if (error < 5 * O_THRESHOLD && c >= 10 && c <= 22)
	{
		//roam螺帽
		return 4;
	}
	if (error > 5 * O_THRESHOLD && error < 40 * O_THRESHOLD && c >= 8 && c <= 17)
	{
		//ross螺丝
		return 1;
	}
	if (error > L_THRESHOLD && c >= 1.1 && c <= 1.8)
	{
		//L型
		return 5;
	}
	else
		return -1;
}


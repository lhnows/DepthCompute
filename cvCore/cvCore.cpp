/*****************************************************************************
 * cvCore.cpp
 *****************************************************************************
 * Copyright (C), Institute of Automation, Chinese Acedemy of Sciences
 *
 * Authors      : 
 *               
 * Date         :2014/12/04
 * Last Update  :2014/12/04
 * Descritption :3D reconstruction from speckle image
 *
 *****************************************************************************/

#include "stdafx.h"

#include "fstream"

/*************************************************
*
*Function:  LoadImgFromFilePath
*Description:   load the reference image from the filepath 
*Input:    char *filepath,int *buffer              
*Output:   int *buffer (grayimage) 
*Return:   none
*Others:         
*************************************************/
void LoadImgFromFilePath(const char *filepath, int *buffer,int rectLeftTopX, int rectLeftTopY, int rectWidth, int rectHeight);

void PreProcess_Truncation(int *refer_img,int toplimit);

void CalculateReferInteg(int *refer_img,int *DJ,int *DJJ,int *refer_I,int *refer_MSI,int *refer_MSII,int hwin,int disparity_begin,int disparity_range);

void AdaptiveFilter_refer(int *refer_img, int *refer_MSI , int winPixTotal);

void CalculateTargetInteg(int *target_img,int *DT,int *DTT,int *target_SI,int *target_SII, int hwin);

void AdaptiveFilter_target(int *target_img, int *target_SI, int winPixTotal);

void CalculateReferTargetInteg(int *refer_I, int *target_img,int *DIJ,int *SIJ,int hwin, int disparity_range);

void CalculateAllCostMatrix(int *refer_SI,int *refer_SII,int *SJ,int *SJJ,int *SIJ,float *costmatrix,int disparity_range,int winPixTotal);

void blockWTA(float *costmatrix,float *finalCost,int disparity_range);

void WTA(float *costmatrix,float *DI ,int hwin,int disparity_begin,int disparity_range);

void TransDepthMap(float *Disparity,float *CostMatrix,int disparity_begin,int disparity_range);

void ResizeDepthImage(float *d);

void SaveDepthMapInTxt(float *D, const char * fileName);

int _tmain(int argc, _TCHAR* argv[])
{
	//paragrams
	const int rectLeftTopX = 200 ,rectLeftTopY = 200, rectWidth = 1648,rectHeight = 768;
	const int toplimit = 1000;
	const int hwin = 10,disparity_begin=-15,disparity_range=37;

	//variable
	int winPixTotal = (hwin+1)*(hwin+1); 

	int *refer_img = new int[WH];		//存储参考图像
	int *target_img = new int[WH];		//存储目标图像
	int *refer_I = new int[disparity_range*WH];		//参考图像移位后的每个像素点对应矩形框内像素和
	int *refer_MSI = new int[disparity_range*WH];	//参考图像移位后的每个像素点对应矩形框内像素和
	int *refer_MSII = new int[disparity_range*WH];	//参考图像移位后的每个像素点对应矩形框内像素平方和
	int *target_SI = new int[WH];					//目标图像的每个像素点对应矩形框内像素和
	int *target_SII = new int[WH];					//目标图像的的每个像素点对应矩形框内像素平方和
	int *DJ = new int[WH];							//参考图像的一阶积分图
	int *DJJ = new int[WH];							//参考图像的二阶积分图
	int *DIJ = new int[WH*disparity_range];							
	int *DT = new int[WH];							//目标图像的一阶积分图
	int *DTT = new int[WH];							//
	int *DM = new int[WH];							//目标图像的一阶积分图
	int *DMM = new int[WH];							//目标图像的二阶积分图

	int *SIJ = new int[disparity_range*WH];			//参考图像与目标图像的点乘的积分图

	
	float *DI = new float[WH];						  //视差深度
	// Variables for GPU
	float *costmatrix = new float[WH*disparity_range];//匹配代价
	float *finalCost = new float[WH*disparity_range]; //匹配代价


	String referImgName;							 //参考图像
	String targetImgName;				
	String SaveImgName;


	//指定原始数据的存储位置
	referImgName	= "TestData\\Reference\\reference.bmp";
	targetImgName	= "TestData\\Reference\\target\\";
	SaveImgName		= "TestData\\Reference\\result\\";	


	
	//参考图像数据读取
	LoadImgFromFilePath(referImgName.c_str(),refer_img, rectLeftTopX,rectLeftTopY,rectWidth,rectHeight);

	//像素值阈值截断
	PreProcess_Truncation(refer_img,toplimit);

	CalculateReferInteg(refer_img,DJ,DJJ,refer_I,refer_MSI,refer_MSII, hwin, disparity_begin, disparity_range);

	AdaptiveFilter_refer(refer_img, refer_MSI,winPixTotal);

	CalculateReferInteg(refer_img,DJ,DJJ,refer_I,refer_MSI,refer_MSII, hwin, disparity_begin, disparity_range);

	IplImage* depthShow=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);  
	cvNamedWindow("depth",1);  

	//遍历dir文件夹下所有jpg图像，作为目标图像进行三维重建
	_finddata_t fileDir;
	char* dir="TestData\\Reference\\数据标定\\*.*";
	long lfDir;
	if((lfDir = _findfirst(dir,&fileDir))==-1l)
		printf("No file is found\n");
	else{
		printf("file list:\n");
		do{
			printf("%s\n",fileDir.name);
			String fn = dir;
			fn.erase(fn.length()-3,fn.length());
			fn = fn + fileDir.name;
			if (fn.find(".bmp") != -1)
			{
				//从目标图像中取出数据
				LoadImgFromFilePath(fn.c_str(),target_img,rectLeftTopX,rectLeftTopY,rectWidth,rectHeight);

				clock_t start0, start; //声明两个时间点
				//double time,time1,time2,time3,time4,time5,time6,timen; //定义运行时间

				start0 = clock(); //获取开始时间
				start = clock();
				PreProcess_Truncation(target_img,toplimit);
				CalculateTargetInteg(target_img,DT,DTT,target_SI,target_SII, hwin);
				AdaptiveFilter_target(target_img, target_SI ,winPixTotal);
				CalculateTargetInteg(target_img,DM,DMM,target_SI,target_SII, hwin);
				printf( "TargetInteg待匹配图像积分图计算时间（CPU）为%.4f 秒\n",(double)(clock() - start) / CLOCKS_PER_SEC);//显示

			
				start = clock(); //获取开始时间
				CalculateReferTargetInteg(refer_I,target_img,DIJ,SIJ,hwin,disparity_range);
				printf( "Refer*TargetInteg参考图像与目标图像乘积积分图计算时间（CPU）为%.4f 秒\n",(double)(clock() - start) / CLOCKS_PER_SEC);//显示

				start = clock(); 
				CalculateAllCostMatrix(refer_MSI,refer_MSII,target_SI,target_SII,SIJ,costmatrix,disparity_range,winPixTotal);
				printf( "CalculateAllCostMatrixZNCC算子计算时间（CPU）为%.4f 秒\n",(double)(clock() - start) / CLOCKS_PER_SEC);//显示

				start = clock();
				blockWTA(costmatrix,finalCost,disparity_range);
				printf("blockWTA ZNCC滤波计算时间（CPU）为%.4f 秒\n",(double)(clock() - start) / CLOCKS_PER_SEC);//显示

				start = clock();
				WTA(finalCost,DI, hwin, disparity_begin, disparity_range);
				printf( "WTA 搜索最优ZNCC计算时间（CPU）为%.4f 秒\n",(double)(clock() - start) / CLOCKS_PER_SEC);//显示

				start =clock();
				TransDepthMap(DI,finalCost, disparity_begin, disparity_range);
				printf( "TransDepthMap 三角变换+插值计算时间（CPU）为%.4f 秒\n",(double)(clock() - start) / CLOCKS_PER_SEC); //CLOCKS_PER_SEC，它用来表示一秒钟会有多少个时钟计时单元，进行计算，完成的时间减去开始的时间获得算法运行时间
				printf( "Total CPU运行时间为%.4f 秒\n",(double)(clock() - start0) / CLOCKS_PER_SEC);//显示

				//对深度图像Resize，从1004x748变成1024x768
				ResizeDepthImage(DI);
				//利用opencv存储和展示图片
				for (int i=0;i<height;i++)
				{
					uchar *ptr = (uchar *) (depthShow->imageData + i * depthShow->widthStep);
					float *ptr1 = (float *) (DI + i*width);
					for (int j=0;j<width;j++)
					{
						ptr[j] = ptr1[width-1-j];
					}
				}

				//解析存储文件名
				fn.erase(fn.length()-4,fn.length());
				SaveImgName =  fn + "_result.jpg";
				double blockDepthValue = 0;

				for(int i = width/2-hwin;i<width/2+hwin;i++)
				{
					for(int j = height/2-hwin; j< height/2+hwin;j++)
					{
						blockDepthValue += DI[i*height + j];
					}
				}
				blockDepthValue /= winPixTotal;
				char blockDepthString[10];
				sprintf(blockDepthString,"%f",blockDepthValue);
				CvFont font;    
				double hScale=1;   
				double vScale=1;    
				int linewidth=2;// 相当于写字的线条    
				// 初始化字体   
				cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,linewidth);//初始化字体，准备写到图片上的   
				cvSaveImage(SaveImgName.c_str(),depthShow);
				cvShowImage("depth",depthShow);
				//cvWaitKey(-1);
				//把更高精度的深度图存储到txt里面去
				String SaveTxtName = fn + ".txt";
				SaveDepthMapInTxt(DI,SaveTxtName.c_str());
			}

		}while( _findnext( lfDir, &fileDir ) == 0 );
	}
	_findclose(lfDir);

	delete refer_img;
	delete target_img;
	delete refer_I;
	delete refer_MSI ;
	delete refer_MSII;
	delete target_SI ;
	delete target_SII ;
	delete DJ ;
	delete DJJ;
	delete DIJ;
	delete SIJ;
	delete DT;
	delete DTT;
	delete DM;
	delete DMM;
	delete costmatrix ; //按层存储
	delete finalCost;
	delete DI ;
	// Comment the following two lines to disable waiting on exit.
	cerr << endl << "Press Enter to exit." << endl;
	while( cin.get() != '\n');
	return 0;
}


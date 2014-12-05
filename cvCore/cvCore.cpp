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

	int *refer_img = new int[WH];		//�洢�ο�ͼ��
	int *target_img = new int[WH];		//�洢Ŀ��ͼ��
	int *refer_I = new int[disparity_range*WH];		//�ο�ͼ����λ���ÿ�����ص��Ӧ���ο������غ�
	int *refer_MSI = new int[disparity_range*WH];	//�ο�ͼ����λ���ÿ�����ص��Ӧ���ο������غ�
	int *refer_MSII = new int[disparity_range*WH];	//�ο�ͼ����λ���ÿ�����ص��Ӧ���ο�������ƽ����
	int *target_SI = new int[WH];					//Ŀ��ͼ���ÿ�����ص��Ӧ���ο������غ�
	int *target_SII = new int[WH];					//Ŀ��ͼ��ĵ�ÿ�����ص��Ӧ���ο�������ƽ����
	int *DJ = new int[WH];							//�ο�ͼ���һ�׻���ͼ
	int *DJJ = new int[WH];							//�ο�ͼ��Ķ��׻���ͼ
	int *DIJ = new int[WH*disparity_range];							
	int *DT = new int[WH];							//Ŀ��ͼ���һ�׻���ͼ
	int *DTT = new int[WH];							//
	int *DM = new int[WH];							//Ŀ��ͼ���һ�׻���ͼ
	int *DMM = new int[WH];							//Ŀ��ͼ��Ķ��׻���ͼ

	int *SIJ = new int[disparity_range*WH];			//�ο�ͼ����Ŀ��ͼ��ĵ�˵Ļ���ͼ

	
	float *DI = new float[WH];						  //�Ӳ����
	// Variables for GPU
	float *costmatrix = new float[WH*disparity_range];//ƥ�����
	float *finalCost = new float[WH*disparity_range]; //ƥ�����


	String referImgName;							 //�ο�ͼ��
	String targetImgName;				
	String SaveImgName;


	//ָ��ԭʼ���ݵĴ洢λ��
	referImgName	= "TestData\\Reference\\reference.bmp";
	targetImgName	= "TestData\\Reference\\target\\";
	SaveImgName		= "TestData\\Reference\\result\\";	


	
	//�ο�ͼ�����ݶ�ȡ
	LoadImgFromFilePath(referImgName.c_str(),refer_img, rectLeftTopX,rectLeftTopY,rectWidth,rectHeight);

	//����ֵ��ֵ�ض�
	PreProcess_Truncation(refer_img,toplimit);

	CalculateReferInteg(refer_img,DJ,DJJ,refer_I,refer_MSI,refer_MSII, hwin, disparity_begin, disparity_range);

	AdaptiveFilter_refer(refer_img, refer_MSI,winPixTotal);

	CalculateReferInteg(refer_img,DJ,DJJ,refer_I,refer_MSI,refer_MSII, hwin, disparity_begin, disparity_range);

	IplImage* depthShow=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);  
	cvNamedWindow("depth",1);  

	//����dir�ļ���������jpgͼ����ΪĿ��ͼ�������ά�ؽ�
	_finddata_t fileDir;
	char* dir="TestData\\Reference\\���ݱ궨\\*.*";
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
				//��Ŀ��ͼ����ȡ������
				LoadImgFromFilePath(fn.c_str(),target_img,rectLeftTopX,rectLeftTopY,rectWidth,rectHeight);

				clock_t start0, start; //��������ʱ���
				//double time,time1,time2,time3,time4,time5,time6,timen; //��������ʱ��

				start0 = clock(); //��ȡ��ʼʱ��
				start = clock();
				PreProcess_Truncation(target_img,toplimit);
				CalculateTargetInteg(target_img,DT,DTT,target_SI,target_SII, hwin);
				AdaptiveFilter_target(target_img, target_SI ,winPixTotal);
				CalculateTargetInteg(target_img,DM,DMM,target_SI,target_SII, hwin);
				printf( "TargetInteg��ƥ��ͼ�����ͼ����ʱ�䣨CPU��Ϊ%.4f ��\n",(double)(clock() - start) / CLOCKS_PER_SEC);//��ʾ

			
				start = clock(); //��ȡ��ʼʱ��
				CalculateReferTargetInteg(refer_I,target_img,DIJ,SIJ,hwin,disparity_range);
				printf( "Refer*TargetInteg�ο�ͼ����Ŀ��ͼ��˻�����ͼ����ʱ�䣨CPU��Ϊ%.4f ��\n",(double)(clock() - start) / CLOCKS_PER_SEC);//��ʾ

				start = clock(); 
				CalculateAllCostMatrix(refer_MSI,refer_MSII,target_SI,target_SII,SIJ,costmatrix,disparity_range,winPixTotal);
				printf( "CalculateAllCostMatrixZNCC���Ӽ���ʱ�䣨CPU��Ϊ%.4f ��\n",(double)(clock() - start) / CLOCKS_PER_SEC);//��ʾ

				start = clock();
				blockWTA(costmatrix,finalCost,disparity_range);
				printf("blockWTA ZNCC�˲�����ʱ�䣨CPU��Ϊ%.4f ��\n",(double)(clock() - start) / CLOCKS_PER_SEC);//��ʾ

				start = clock();
				WTA(finalCost,DI, hwin, disparity_begin, disparity_range);
				printf( "WTA ��������ZNCC����ʱ�䣨CPU��Ϊ%.4f ��\n",(double)(clock() - start) / CLOCKS_PER_SEC);//��ʾ

				start =clock();
				TransDepthMap(DI,finalCost, disparity_begin, disparity_range);
				printf( "TransDepthMap ���Ǳ任+��ֵ����ʱ�䣨CPU��Ϊ%.4f ��\n",(double)(clock() - start) / CLOCKS_PER_SEC); //CLOCKS_PER_SEC����������ʾһ���ӻ��ж��ٸ�ʱ�Ӽ�ʱ��Ԫ�����м��㣬��ɵ�ʱ���ȥ��ʼ��ʱ�����㷨����ʱ��
				printf( "Total CPU����ʱ��Ϊ%.4f ��\n",(double)(clock() - start0) / CLOCKS_PER_SEC);//��ʾ

				//�����ͼ��Resize����1004x748���1024x768
				ResizeDepthImage(DI);
				//����opencv�洢��չʾͼƬ
				for (int i=0;i<height;i++)
				{
					uchar *ptr = (uchar *) (depthShow->imageData + i * depthShow->widthStep);
					float *ptr1 = (float *) (DI + i*width);
					for (int j=0;j<width;j++)
					{
						ptr[j] = ptr1[width-1-j];
					}
				}

				//�����洢�ļ���
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
				int linewidth=2;// �൱��д�ֵ�����    
				// ��ʼ������   
				cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,linewidth);//��ʼ�����壬׼��д��ͼƬ�ϵ�   
				cvSaveImage(SaveImgName.c_str(),depthShow);
				cvShowImage("depth",depthShow);
				//cvWaitKey(-1);
				//�Ѹ��߾��ȵ����ͼ�洢��txt����ȥ
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
	delete costmatrix ; //����洢
	delete finalCost;
	delete DI ;
	// Comment the following two lines to disable waiting on exit.
	cerr << endl << "Press Enter to exit." << endl;
	while( cin.get() != '\n');
	return 0;
}


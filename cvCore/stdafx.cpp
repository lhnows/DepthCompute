// stdafx.cpp : 只包括标准包含文件的源文件
// cvCore.pch 将作为预编译头
// stdafx.obj 将包含预编译类型信息

#include "stdafx.h"

// TODO: 在 STDAFX.H 中
// 引用任何所需的附加头文件，而不是在此文件中引用

/*************************************************
*
*Function:  LoadImgFromFilePath
*Description:   load the reference image from the filepath 
*Input:    char *filepath,int *buffer              
*Output:   int *buffer (grayimage) 
*Return:   none
*Others:         
*************************************************/
void LoadImgFromFilePath(const char *filepath, int *buffer,int rectLeftTopX, int rectLeftTopY, int rectWidth, int rectHeight)
{
	
	IplImage *imgLoad = cvLoadImage(filepath,0);//只处理灰度图
	IplImage *imgRef_OriSize = cvCreateImage(cvSize(rectWidth,rectHeight),IPL_DEPTH_8U,1);
	IplImage *imgRef_NewSize = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
	//标定相机获取的原始数据中将要被处理的区域
	CvRect imgROI = cvRect(rectLeftTopX,rectLeftTopY,rectWidth,rectHeight);

	cvSetImageROI(imgLoad,imgROI);
	cvCopy(imgLoad,imgRef_OriSize);

	cvResize(imgRef_OriSize,imgRef_NewSize);

	for (int i=0;i<height;i++)
	{
		uchar *ptr = (uchar *) (imgRef_NewSize->imageData + i * imgRef_NewSize->widthStep);
		int *ptr1 = (int *) (buffer + (i)*width);
		for (int j=0;j<width;j++)
		{
			ptr1[j] = ptr[j];
		}
	}
	cvReleaseImage(&imgLoad);
	cvReleaseImage(&imgRef_OriSize);
	cvReleaseImage(&imgRef_NewSize);
}

/*************************************************
*
*Function:  PreProcess_Truncation
*Description:   if img[i]>toplimit , img[i]=toplimit
*Input:    int *img,int toplimit         
*Output:   int *img
*Return:   none
*Others:         
*************************************************/
void PreProcess_Truncation(int *img,int toplimit)
{
	for (int i=0;i<WH;i++)
	{
		img[i] = img[i] > toplimit?toplimit:img[i];
	}
}

void CalculateTargetInteg(int *target_img,int *DT,int *DTT,int *target_SI,int *target_SII,int hwin)
{

	int *T = new int[width]; // sum of each column
	int *TT = new int[width]; // sum of each column
	// calculate integral of the first line
	for(int i=0;i<width;i++){
		T[i]=target_img[i];
		DT[i] = target_img[i];
		*(TT+i)=*(target_img+i) * *(target_img+i);
		*(DTT+i)=*(target_img+i) * *(target_img+i);
		if(i>0){
			DT[i] += DT[i-1];
			DTT[i] += DTT[i-1];
		}
	}
	for (int i=1;i<height;i++){
		int offset = i*width;
		// first column of each line
		T[0] += target_img[offset];
		DT[offset] = T[0];
		TT[0] += target_img[offset]*target_img[offset];
		DTT[offset] = TT[0];
		// other columns 
		for(int j=1;j<width;j++){
			T[j] += target_img[offset+j];
			DT[offset+j] = DT[offset+j-1] + T[j]; 
			TT[j] += target_img[offset+j]*target_img[offset+j];
			DTT[offset+j] = DTT[offset+j-1] + TT[j];
		}
	}

	for(int i=0;i<WH;i++)
	{
		target_SI[i] = DT[i];
		target_SII[i] = DTT[i];
	}
	int i,j;
	for (i=hwin+1;i<height-hwin;i++)
	{
		for (j=hwin+1;j<width-hwin;j++)
		{
			target_SI[i*width+j] = DT[(i+hwin)*width+j+hwin] - DT[(i-hwin-1)*width+j+hwin] - DT[(i+hwin)*width+j-hwin-1]
			+DT[(i-hwin-1)*width+j-hwin-1];
			target_SII[i*width+j] = DTT[(i+hwin)*width+j+hwin] - DTT[(i-hwin-1)*width+j+hwin] - DTT[(i+hwin)*width+j-hwin-1]
			+DTT[(i-hwin-1)*width+j-hwin-1];
		}
	}
}

//参考图像滤波
void AdaptiveFilter_refer(int *refer_img, int *refer_MSI,int winPixTotal)
{
	//不理解，为什么refer_img[i]最小值是平移disparity_begin后的像素临域均值？
	for (int i=0;i<WH;i++)
	{
		if (refer_img[i]*winPixTotal < refer_MSI[i])
		{
			refer_img[i] = refer_MSI[i]/winPixTotal;
		}
	}
}

void CalculateReferInteg(int *refer_img,int *DJ,int *DJJ,int *refer_I,int *refer_MSI,int *refer_MSII,int hwin,int disparity_begin,int disparity_range)
{
	int *I = refer_img;
	int *refer_SI = new int[WH];//参考图像像素点临域 一阶和
	int *refer_SII = new int[WH];//参考图像像素点临域 二阶和

	//计算 参考图像一阶积分图DJ 参考图像二阶积分图DJJ
	*DJ=*I;
	*DJJ=(*I) * (*I);
	for (int j=1; j<width; j++)//第一行
	{
		*(DJ+j)=*(DJ+j-1)+*(I+j);
		*(DJJ+j)=*(DJJ+j-1)+*(I+j)* *(I+j);
	}
	for (int i=width; i<WH; i+=width)//第一列
	{
		*(DJ+i)=*(I+i)+ *(DJ+(i-width));
		*(DJJ+i)=*(I+i)* *(I+i)+ *(DJJ+(i-width));
	}
	int i,j;
	for (i=width; i<WH; i+=width)//所有像素
	{
		for(int j_index=1+i; j_index<width+i; j_index++)
		{
			*(DJ+j_index)=*(I+j_index)+ *(DJ+j_index-width)+*(DJ+j_index-1)-*(DJ+j_index-width-1);
			*(DJJ+j_index)=*(I+j_index)* *(I+j_index)+ *(DJJ+j_index-width)+*(DJJ+j_index-1)-*(DJJ+j_index-width-1);
		}
	}


	//计算 参考图像 像素点的 临域（2*hwin+1）*（2*hwin+1）一阶和refer_SI，二阶和refer_SII
	for (i=hwin+1;i<height-hwin;i++)
	{
		for (j=hwin+1;j<width-hwin;j++)
		{
			refer_SI[i*width+j] = DJ[(i+hwin)*width+j+hwin] - DJ[(i-hwin-1)*width+j+hwin] - DJ[(i+hwin)*width+j-hwin-1]
			+DJ[(i-hwin-1)*width+j-hwin-1];
			refer_SII[i*width+j] = DJJ[(i+hwin)*width+j+hwin] - DJJ[(i-hwin-1)*width+j+hwin] - DJJ[(i+hwin)*width+j-hwin-1]
			+DJJ[(i-hwin-1)*width+j-hwin-1];
		}
	}

	//存储refer_I,refer_MSI,refer_MSII
	int t=disparity_begin*2;//为什么乘2
	for (int n=0; n<disparity_range*WH; n+=WH,t+=2)//step=2？ 扩大了搜索范围但是降低了精度？disparity_range=37，37个刻度的精度是不是太小
	{
		for (int i=n; i<n+WH; i+=width)
		{
			for (int j=i; j<i+width;j++)
			{
				if (j-i+t < hwin+1 || j-i+t >= width-hwin)
				{
					refer_I[j]=refer_img[j-n];
					refer_MSI[j]=refer_SI[j-n];
					refer_MSII[j]=refer_SII[j-n];
				}
				else
				{
					refer_I[j]=refer_img[j-n+t];
					refer_MSI[j]=refer_SI[j-n+t];
					refer_MSII[j]=refer_SII[j-n+t];
				}
			}
		}
	}
	delete[] refer_SI;
	delete[] refer_SII;
}

void AdaptiveFilter_target(int *target_img, int *target_SI, int winPixTotal)
{
	int value;
	for (int i=0;i<WH;i++)
	{
		value = target_img[i];
		if (value*winPixTotal < target_SI[i])
		{
			value = 0;
			target_img[i] = target_SI[i]/winPixTotal;
			/*if(target_SI[i]<0)
				target_SI[i] = 0;*/
		}
	}
}

void CalculateReferTargetInteg(int *refer_I, int *target_img,int *DIJ,int *SIJ,int hwin,int disparity_range )
{

	int *IJ = new int[disparity_range*WH]; // sum of each column
	// calculate integral of the first line
	for(int d=0;d<disparity_range;d++)
		for(int i=0;i<width;i++){
			IJ[i+d*WH] = refer_I[i+d*WH] * target_img[i];
			DIJ[i+d*WH] = refer_I[i+d*WH] * target_img[i];
			if(i>0){
				DIJ[i+d*WH] += DIJ[i-1+d*WH];
			}
		}
	for(int d=0;d<disparity_range;d++)
		for (int i=1;i<height;i++){
			int offset = i*width;
		// first column of each line
			IJ[0+d*WH] += refer_I[offset+d*WH] * target_img[offset];
			DIJ[offset+d*WH] = IJ[0+d*WH];
		// other columns 
		for(int j=1;j<width;j++){
			IJ[j+d*WH] += refer_I[offset+d*WH+j] * target_img[offset+j];
			DIJ[offset+j+d*WH] = DIJ[offset+j-1+d*WH] + IJ[j+d*WH]; 
		}
	}

	for(int i=0;i<disparity_range;i++)
		for(int j=0;j<WH;j++)
		{
			*(SIJ+i*WH+j)=*(DIJ+i*WH+j);
		}
	
	for (int i=hwin+1;i<height-hwin;i++)
		for (int j=hwin+1;j<width-hwin;j++)
			for(int d=0;d<disparity_range;d++)
			{
				SIJ[d*WH+i*width+j] = DIJ[d*WH+(i+hwin)*width+j+hwin] - DIJ[d*WH+(i-hwin-1)*width+j+hwin] - DIJ[d*WH+(i+hwin)*width+j-hwin-1]
				+DIJ[d*WH+(i-hwin-1)*width+j-hwin-1];
			}
	delete IJ;
}

void CalculateAllCostMatrix(int *refer_SI,int *refer_SII,int *SJ,int *SJJ,int *SIJ,float *costmatrix,int disparity_range,int winPixTotal)
{
	float *data=new float[WH*disparity_range];
	//float *result=new float[WH*disparity_range];
	for(int d=0;d<disparity_range;d++)
		for(int i=0;i<height;i++)
			for(int j=0;j<width;j++)
			{
				int pos = j+i*width+d*WH;
				//int target_pos = j+i*width;
				data[pos] = abs((float)(winPixTotal*refer_SII[pos]-refer_SI[pos]*refer_SI[pos])*(winPixTotal*SJJ[pos-d*WH]-
					SJ[pos-d*WH]*SJ[pos-d*WH]));
				data[pos] = sqrt(data[pos]);
				costmatrix[pos] = 0;
				if(data[pos]!=0)
				{
					costmatrix[pos]=(winPixTotal*SIJ[pos]-refer_SI[pos]*SJ[pos-d*WH])/data[pos];
				}
			}
			
	delete data;
}

void blockWTA(float *costmatrix,float *finalCost,int disparity_range)
{
	float cost,cost1,cost2;
	for(int d=0;d<disparity_range;d++)
		for(int i=0;i<height;i++)
			for(int j=0;j<width;j++)
			{
				finalCost[j+i*width+d*WH]=costmatrix[j+i*width+d*WH];
			}
	for(int d=0;d<disparity_range;d++)
		for(int i=bwin+10+2;i<height-bwin-10-1;i++)
			for(int j=bwin+10+2;j<width-bwin-10-1;j++)
			{
				cost=costmatrix[j+i*width+d*WH];
				cost1 = costmatrix[j - bwin + i*width + d * WH];
				cost2 = costmatrix[j + bwin + i*width + d * WH];
				if(cost1 < cost2)
					cost += cost2;
				else
					cost += cost1;


				cost1 = costmatrix[j - bwin + (i - bwin)*width + d * WH];
				cost2 = costmatrix[j + bwin + (i + bwin)*width + d * WH];
				if(cost1 < cost2)
					cost += cost2;
				else
					cost += cost1;


				cost1 = costmatrix[j - bwin + (i + bwin)*width + d * WH];
				cost2 = costmatrix[j + bwin + (i - bwin)*width + d * WH];
				if(cost1 < cost2)
					cost += cost2;
				else
					cost += cost1;


				cost1 = costmatrix[j + (i - bwin)*width + d * WH];
				cost2 = costmatrix[j + (i + bwin)*width + d * WH];
				if(cost1 < cost2)
					cost += cost2;
				else
					cost += cost1;
				finalCost[j+i*width+d*WH]=cost;
			}
			
}

void WTA(float *costmatrix,float *DI ,int hwin,int disparity_begin,int disparity_range)
{

	float *cost=new float[WH];
	for(int i=0;i<height;i++)
		for (int j=0;j<width;j++)
		{
			int pos = j+i*width;
			cost[pos]=costmatrix[pos];
			DI[pos] = disparity_begin;
		}
	for(int i=hwin+1;i<height-hwin;i++)
		for(int j=hwin+1;j<width-hwin;j++)
			for(int d=1;d<disparity_range;d++)
			{
				int pos = j+i*width+d*WH;
				//int pos1= j+i*width;
				if(cost[pos-d*WH]<costmatrix[pos])
				{
					cost[pos-d*WH] = costmatrix[pos];
					DI[pos-d*WH] = d+disparity_begin;
				}

			}
}

void TransDepthMap(float *Disparity,float *CostMatrix,int disparity_begin,int disparity_range)
{
	float MCost = 0;
	float RCost = 0;
	float LCost = 0;
	float *data = new float[WH];
	int *depth = new int[WH];
	
	//int flag;
	/*for(int i=WH/2;i<WH/2+100;i++)
	{
		printf("%f ",Disparity[i]);
	}*/

	for(int i=0;i<height;i++)
		for (int j=0;j<width;j++)
		{
			int pos = j+i*width;
			
			data[pos] = Disparity[pos];
			//depth[pos] = data[pos]-disparity_begin;
			depth[pos] = data[pos]-disparity_begin;
			//flag = depth[pos];
			if(depth[pos]<disparity_range && depth[pos]>=0)
				MCost = CostMatrix[depth[pos]*WH+pos];
			/*if(depth[pos]<disparity_range &&depth[pos] > 0)
				LCost = CostMatrix[(depth[pos]-1)*WH+pos];
			if(depth[pos] < disparity_range-1&& depth[pos]>=0)
				RCost = CostMatrix[(depth[pos]+1)*WH+pos];
			LCost = MCost - LCost;
			RCost = RCost - MCost;
			if(LCost!=RCost && MCost != 0)
				data[pos] = data[pos] + (LCost+RCost)/(LCost-RCost)/2;*/

			if(MCost < TFM)
				data[pos] = disparity_range+disparity_begin;
			if(depth[pos]>disparity_range-1 || depth<0)
				data[pos] = disparity_range+disparity_begin;
			if (data[pos] != disparity_begin )
			{
				Disparity[pos] = bxf/(doff-data[pos]);
				if(data[pos] == disparity_end)
					Disparity[pos] = 0;
			}
			/*if(Disparity[pos] = 0)
				Disparity[pos]=100;*/
			/*LCost = MCost - LCost;
			//RCost = MCost - RCost;
			RCost = RCost - MCost;
			/ *if(LCost!=RCost && MCost != 0)
				data[pos]  = data[pos];
			LCost = MCost - LCost;
			RCost = MCost - RCost;* /
			if(LCost > RCost && MCost != 0 && RCost != 0)
				data[pos]  = data[pos] + 0.5 - RCost/(LCost+RCost);
			if(LCost < RCost && MCost != 0 && LCost != 0)
				data[pos]  = data[pos] - 0.5 + LCost/(LCost+RCost);

// 			if(CostMatrix[pos] < TFM)
// 				data[pos] = (disparity_range+disparity_begin)*12;
// 			Disparity[pos] = 12 * abs(Disparity[pos]);
			if(depth[pos]>disparity_range-1 || depth[pos]<0)
				data[pos] = disparity_range+disparity_begin;
			if (data[pos] != disparity_begin )
			{
				Disparity[pos] = 4 * abs(data[pos]);

				if(data[pos] == disparity_end)
					Disparity[pos] = 0;
			}*/

		}
	FILE *fileptr=fopen("D:\\target_teddy.txt","w");
	for(int i=0;i<WH;i++)
		fprintf(fileptr,"%f ",Disparity[i]);
	fclose(fileptr);
	delete data;
	delete depth;
}

void ResizeDepthImage(float *d)
{
	
	IplImage* depthShow=cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);  
	IplImage* depthShowNew=cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);  
	for (int i=0;i<height;i++)
	{
		float *ptr = (float *) (depthShow->imageData + i * depthShow->widthStep);
		float *ptr1 = (float *) (d + i*width);
		for (int j=0;j<width;j++)
		{
			ptr[j] = ptr1[width-1-j];
		}
	}

	//对depth进行裁剪成1024x768

	CvRect imgROI = cvRect(10,10,1024,768);
	//CvRect imgROI = cvRect(10,10,400,350);
	cvSetImageROI(depthShow,imgROI);
	cvResize(depthShow,depthShowNew);
	for (int i=0;i<height;i++)
	{
		float *ptr = (float *) (depthShowNew->imageData + i * depthShowNew->widthStep);
		float *ptr1 = (float *) (d + i*width);
		for (int j=0;j<width;j++)
		{
			ptr1[j] = ptr[j];
		}
	}
	cvReleaseImage(&depthShowNew);
	cvReleaseImage(&depthShow);
}

void SaveDepthMapInTxt(float *D, const char * fileName)
{
	ofstream output(fileName,ios::trunc);
	for (int i=0;i<height;i++)
	{
		float *ptr = (float *) (D+i*width);
		for (int j=0;j<width;j++)
		{
			output << ptr[j] << " ";
		}
		output<<endl;
	}
}
/*
 * fft.c
 *
 *  Created on: 2020年3月26日
 *      Author: Administrator
 */
#include "include.h"
#include <math.h>
#include <string.h>

#include "arm_math.h"
#include "arm_const_structs.h"

#define NUM_FFT 128
//#define PI 3.1415926


/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = 128;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

#define NPT  128  //1024点FFT
#define Fs 6400  //采样频率 5120Hz 频率分辨率 5Hz
#define PI2 6.28318530717959
float32_t  testInput_f32[256];
float32_t  testOutput_f32[256];
float32_t  testOutput[128];


typedef struct
{
//	s16 iRealArray[NUM_FFT];
//	s16 iMageArray[NUM_FFT];
	float iRealArray[NUM_FFT];
	float iMageArray[NUM_FFT];
	uint16_t	FU[3];			//---基波电压---NNN.N6
	uint32_t	FI[4];			//---基波电流NNNN.NNNN
//	uint32_t HarmonicpercentU[3][51];
	float HarmonicpercentU[3][51];
	uint32_t	HarmonicpercentI[3][51];			//---谐波含有率--NNN.N6
}HarmonicData_TypeDef;

typedef struct
{
    uint8_t  Channel;					 // 当前采样通道
    uint16_t  ReadAdress;				 // 读取地址
    uint8_t  StarFlag;					 // 开始进行谐波数据分析
    uint8_t  ADSPIBusy;					 // 进行谐波分析瞬时值数据采样
    uint16_t DataCount;					 // 当前采样点数
    uint16_t ReadAddres;				 // 采样点数据地址
    uint16_t	dwFreq;					 // 采样周期当前频率
    uint16_t TimeOutStamp;				 // 采样延时退出
    uint8_t InstantaneousData[384];      //当前采样128点瞬时数据
}Instantaneous_TypeDef;

typedef enum
{
	UAChannel		= 0x00,
	UBChannel		= 0x01,
	UCChannel		= 0x02,
	IAChannel		= 0x03,
	IBChannel		= 0x04,
	ICChannel		= 0x05
}ChannelFlag_TypeDef;

typedef struct
{
    uint8_t		ChkErrCnt;       //读错误计数1

    int32_t 	Pw[12];   		//{Pa Pb Pc P Qa Qb Qc Q Sa Sb Sc S}   48
    int32_t 	UI[7];       	//Ua Ub Uc Ia Ib Ic Inal   28
    int32_t		VectorU[9];		// 正序、负序、零序电压
    int32_t		VectorI[9];		// 正序、负序、零序电流
    int32_t  	Pf[4];       	//Pf Pfa Pfb Pfc      16
    uint32_t 	Frequency;   	//电网频率，单位：        4
    int32_t  	YUI[3],YUU[2];  //20

    int32_t		Pulse[15];		//前台高频脉冲48
    //---电能脉冲---
    int32_t		Pulse_EgTmp[20];	//高频脉冲{P,Q,Ps},{Pa,Qa,Psa},{Pb,Qb,Psb},{Pc,Qc,Psc}{Fp,Fq}{Fpa,Fqa}{Fpb,Fqb}{Fpc,Fqc}
    uint32_t		Pulse_Eg[20];  //低频脉冲数
	//---需量脉冲---
	int32_t		Pulse_NeedTmp[12];
  	uint16_t		Pulse_Need[12]; //{PNeed,QNeed,PsNeed},{PNeeda,QNeeda,PsNeeda},{PNeedb,QNeedb,PsNeedb},{PNeedc,QNeedc,PsNeedc}48


    uint16_t		Angle[9];
    uint16_t 	PDirect;   //4
    uint32_t 	ChkSum1;   //4
    uint32_t 	ChkSum2;   //4

	uint16_t		Temperature;	//温度4
	uint32_t		ClockBat;		//时钟电池4
	uint32_t		BackupBat;		//后备电池4

    uint16_t   CF1DelayStamp;
    uint16_t   CF2DelayStamp;

    uint16_t   CfIn_P;
    uint16_t   CfIn_q;

    uint16_t   CfTime_P;
    uint16_t   CfTime_q;

} sDl645FrontTmp_TypeDef;

const float Fftcoefficient[50]=					// 谐波计算时补偿系数
{
	1.0005,
	1.0027,
	1.0054,
	1.0087,
	1.0129,
	1.0175,
	1.0229,
	1.029,
	1.0362,
	1.0438,
	1.0524,
	1.0618,
	1.0718,
	1.0827,
	1.0944,
	1.1075,
	1.121,
	1.136,
	1.1517,
	1.1685,
	1.1862,
	1.2051,
	1.2255,
	1.2475,
	1.27,
	1.2942,
	1.3199,
	1.3468,
	1.3758,
	1.407,
	1.4421,
	1.4714,
	1.5107,
	1.5483,
	1.5873,
	1.6312,
	1.6727,
	1.7199,
	1.77,
	1.8231,
	1.8797,
	1.9448,
	2.0074,
	2.074,
	2.1534,
	2.2126,
	2.2919,
	2.3762,
	2.467,
	2.5639,
};


Instantaneous_TypeDef Harmonictemp;
HarmonicData_TypeDef HarmonicData;
sDl645FrontTmp_TypeDef Dl645FrontTmp;

void InitForFFT();

uint8_t tempdata[384]={0XD6,0X4E,0XED,0Xe6,0XF8,0XEC,0XC4,0XAA,0XEC,0Xe5,0X6B,0XEC,0Xe2,0Xe7,0XEC,0XE1,
		0XeE,0XEC,0X69,0XF2,0XEB,0XF4,0XE2,0XEB,0XeE,0XE1,0XEB,0XB5,0XE9,0XEB,0X8F,0X01,0XEC,0X0C,0X22,
		0XEC,0X30,0X50,0XEC,0X94,0X8B,0XEC,0X99,0XD3,0XEC,0X54,0X26,0XED,0XB4,0Xe4,0XED,0X9D,0XED,0XED,
		0X62,0Xe2,0XEE,0XED,0XE1,0XEE,0X17,0XeC,0XEF,0X04,0Xe1,0XF0,0XB2,0XeE,0XF0,0X8A,0Xe6,0XF1,0XF8,
		0XF7,0XF1,0X66,0XB1,0XF2,0XD0,0Xe2,0XF3,0XD5,0XeB,0XF4,0X92,0X0D,0XF5,0XD7,0XE3,0XF5,0X5A,0XC2,
		0XF6,0X23,0XA6,0XF7,0XA4,0X8D,0XF8,0XE4,0X7A,0XF9,0XA4,0X6C,0XFA,0XDC,0X61,0XFB,0X9E,0X57,0XFC,
		0X2A,0X52,0XFD,0XFA,0X4D,0,0X28,0X4A,0XFF,0XCC,0X46,0X00,0XEA,0X42,0X01,0X37,0X3D,0X02,0X16,0X38,
		0X03,0X3C,0X30,0X04,0X44,0X26,0X05,0X72,0X1A,0X06,0XFE,0X08,0X07,0X49,0XF3,0X07,
		0X4A,0XD9,0X08,0XFA,0XB7,0X09,0XD4,0X90,0X0A,0X02,0X65,0X0B,0XDD,0X31,0X0C,0X38,0XF8,0X0C,0X9A,
		0XB6,0X0D,0XAB,0X6B,0X0E,0XCC,0X17,0X0F,0X4A,0XBA,0X0F,0XB7,0X52,0X10,0X34,0XE1,0X10,0X4C,0X66,
		0X11,0X55,0XDE,0X11,0X73,0X4D,0X12,0XD8,0XAF,0X12,0X66,0X07,0X13,0X54,0X54,0X13,0XDB,0X93,0X13,
		0X9E,0XC8,0X13,0XDA,0XF0,0X13,0XF5,0X0B,0X14,0X26,0X1C,0X14,0X09,0X1E,0X14,0X90,0X14,0X14,0X6F,
		0XFF,0Xe3,0Xe5,0XeC,0Xe3,0Xe4,0XeE,0Xe3,0Xe0,0X74,0X13,0XC3,0X2C,0X13,0X0E,0XD9,0X12,0XB4,0X7A,
		0X12,0X4D,0X11,0X12,0X63,0X9C,0X11,0X74,0X1B,0X11,0XCD,0X92,0X10,0X10,0XFE,0X0F,0X53,0X5F,0X0F,
		0X87,0XB8,0X0E,0X75,0X08,0X0E,0X57,0X4D,0X0D,0X0F,0X8B,0X0C,0X60,0XC2,0X0B,0XB6,0XF1,0X0A,0XDB,
		0X1A,0X0A,0X57,0X3C,0X09,0X6C,0X57,0X08,0X5E,0X6E,0X07,0X38,0X83,0X06,0X2F,0X93,0X05,0X27,0X9E,
		0X04,0X5F,0XA7,0X03,0X31,0XAC,0X02,0XF2,0XAF,0X01,0XC4,0XB3,0X00,0X51,0XB8,0XFF,0XC9,0XBB,0XFE,
		0XFC,0XBE,0XFD,0X92,0XC5,0XFC,0X5B,0XCF,0XFB,0X77,0XD9,0XFA,0X9F,0XE5,0XF9,0XC3,0XF6,0XF8,0XFE,
		0X0B,0XF8,0X7D,0X27,0XF7,0XA9,0X46,0XF6,0X43,0X6D,0XF5,0XBE,0X98,0XF4,0XD8,0XCA,0XF3,0X3B,0X06,
		0XF3,0X47,0X49,0XF2,0X79,0X94,0XF1,0X32,0XE9,0XF0,0XA6,0X45,0XF0,0Xe8,0XAD,0XEF,0XeD,0X1D,0XEF,
		0XeA,0X99,0XEE,0XE3,0X1E,0XEE,0Xe8,0XB0,0XED};


//uint8_t tempdata[384];

float sin_tab[NUM_FFT];
float cos_tab[NUM_FFT];


void fnDl645Fft_init(void)
{
	Harmonictemp.Channel=0x00;
	Harmonictemp.DataCount=0;
    Harmonictemp.StarFlag=0;
    Harmonictemp.ADSPIBusy = 0;
    memset(&Harmonictemp,0,sizeof(Harmonictemp));
    memset(&HarmonicData,0,sizeof(HarmonicData));
    InitForFFT();
    SampleDataModifyF(&HarmonicData.iRealArray[0]);	//采样数据修正
}

void InitForFFT()
{
	int i;

	for ( i=0;i<NUM_FFT;i++ )
	{
//		sin_tab[i]=sin(PI*2*i/NUM_FFT);
//		cos_tab[i]=cos(PI*2*i/NUM_FFT);
		sin_tab[i]=arm_sin_f32(PI*2*i/NUM_FFT);
		cos_tab[i]=arm_cos_f32(PI*2*i/NUM_FFT);
	}
}



uint32_t	fnHexToBcd_u32(uint32_t Dat)
{
	uint32_t result = 0;

	Dat = Dat % 100000000;
	result += (Dat / 10000000) * 0x10000000;
	Dat = Dat % 10000000;
	result += (Dat / 1000000) * 0x1000000;
	Dat = Dat % 1000000;
	result += (Dat / 100000) * 0x100000;
	Dat = Dat % 100000;
	result += (Dat / 10000) * 0x10000;
	Dat = Dat % 10000;
	result += (Dat / 1000) * 0x1000;
	Dat = Dat % 1000;
	result += (Dat / 100) * 0x100;
	Dat = Dat % 100;
	result += (Dat / 10) * 0x10;
	Dat = Dat % 10;
	result += Dat;

	return(result);
}


void FFT(float dataR[NUM_FFT],float dataI[NUM_FFT])
{
	int x0,x1,x2,x3,x4,x5,x6,xx;
	int i,j,k,b,p,L;
	float TR,TI,temp;
	int temp1[128];
	/********** following code invert sequence ************/
	for ( i=0;i<NUM_FFT;i++ )
	{
		x0=x1=x2=x3=x4=x5=x6=0;
		x0=i&0x01; x1=(i/2)&0x01; x2=(i/4)&0x01; x3=(i/8)&0x01;x4=(i/16)&0x01; x5=(i/32)&0x01; x6=(i/64)&0x01;
		xx=x0*64+x1*32+x2*16+x3*8+x4*4+x5*2+x6;
		dataI[xx]=dataR[i];
		temp1[i]=xx;
	}
	for ( i=0;i<NUM_FFT;i++ )
	{
		dataR[i]=dataI[i]; dataI[i]=0;
	}

//	for ( i=0;i<NUM_FFT;i++ )
//	{
//		temp1[i]=dataR[i];
//	}

	/************** following code FFT *******************/
	for ( L=1;L<=7;L++ )
	{ /* for(1) */
		b=1; i=L-1;
		while ( i>0 )
		{
			b=b*2; i--;
		} /* b= 2^(L-1) */
		for ( j=0;j<=b-1;j++ ) /* for (2) */
		{
			p=1; i=7-L;
			while ( i>0 ) /* p=pow(2,7-L)*j; */
			{
				p=p*2; i--;
			}
			p=p*j;
			for ( k=j;k<128;k=k+2*b ) /* for (3) */
			{
				TR=dataR[k]; TI=dataI[k]; temp=dataR[k+b];
				dataR[k]=dataR[k]+dataR[k+b]*cos_tab[p]+dataI[k+b]*sin_tab[p];
				dataI[k]=dataI[k]-dataR[k+b]*sin_tab[p]+dataI[k+b]*cos_tab[p];
				dataR[k+b]=TR-dataR[k+b]*cos_tab[p]-dataI[k+b]*sin_tab[p];
				dataI[k+b]=TI+temp*sin_tab[p]-dataI[k+b]*cos_tab[p];
			} /* END for (3) */
		} /* END for (2) */
	} /* END for (1) */

//	printf("排序打印\r\n");
//   	for(i=0;i<128;i++)
//   	{
//    	printf("%d\r\n",temp1[i]);
//   	}
} /* END FFT */

/*******  取样值转浮点数  ********/
void SampleDataModifyF(float *piRetValue)
{
	uint8_t i;
	uint32_t	Temp;
	int32_t	Temp1[128];
   	for(i=0; i<NUM_FFT; i++)
   	{
	   	piRetValue[i] = 0;
	   	Temp = 0;
	   	memcpy((uint8_t *)&Temp , &(Harmonictemp.InstantaneousData[i*3]) , 3);
	   	Temp1[i]=Temp;
   		if(Temp &0x800000)
   		{
   			Temp &=0x00ffffff;
   			Temp = (0x00ffffff - Temp) + 1;  //取补码
//   			piRetValue[i] = -(((float)Temp)/8388608);  //去掉符号位
//   			piRetValue[i] = -(Temp/8388608.0);  //归一化处理
   			piRetValue[i] = -(Temp/8388608.0);  //归一化处理
//   			printf("%.3f\r\n",&piRetValue[i]);
   		}
   		else {
//   			piRetValue[i] = ((float)Temp)/8388608;
//   			piRetValue[i] = Temp/8388608.0;
   			piRetValue[i] = Temp/8388608.0;
   		}
   	}
//   	for(i=0;i<128;i++)
//   	{
//    	printf("%d\r\n",Temp1[i]);
//   	}
}


void Fft_Harmonic_Exec(void)
{
	float ftemp_N,ftemp_P;
	float ftemp_P1;
	float fBase;  //基波的平方

//	Harmonictemp.StarFlag = 0x02;
//	Harmonictemp.Channel=0;
//
//	for(int i=0;i<384;i++)
//	{
//		if(i%2==0)
//		{
//			for(int j=0;j<20;j++)
//			{
//		      Harmonictemp.InstantaneousData[i+j]=0xd6;
//			}
//		}else{
//			for(int j=0;j<20;j++)
//			{
//			  Harmonictemp.InstantaneousData[i+j]=0x11;
//			}
//		}
//		Harmonictemp.InstantaneousData[i]=tempdata[i];
//	}
//
//	if(Harmonictemp.StarFlag == 0x02)
//	{
//		SampleDataModifyF(&HarmonicData.iRealArray[0]);	//采样数据修正


//		for(int i=0;i<128;i++)
//		{
//		 printf("第%d点：%f，",i,HarmonicData.iRealArray[i]); 	//	归一化值
//		}

	    /* 按照实部，虚部，实部，虚部..... 的顺序存储数据 */
//		int k=0;
//	  for(int i=0; i<256; i++)
//	  {
//		  if(i%2==0)
//		  {
//		   testInput_f32[i] =HarmonicData.iRealArray[k];
//		   k++;
//		  }else{
//		   testInput_f32[i] =0;
//		  }
//	  }

//	  		for(int i=0;i<256;i++)
//	  		{
//	  		 printf("%f\r\n",testInput_f32[i]); 	//	归一化值
//	  		}


//		for(i=0;i<127;i++)
//		{
//	 	 printf("%.6f\r\n",HarmonicData.iRealArray[i]);
//		 printf("%f\r\n",HarmonicData.iRealArray[i]); 	//	归一化值
//		}

//		arm_rfft_fast_init_f32(&arm_cfft_sR_f32_len128,128);
	  /* Process the data through the CFFT/CIFFT module */
//		arm_rfft_fast_f32(&arm_cfft_sR_f32_len128, testInput_f32,testOutput_f32, ifftFlag);
		arm_cfft_f32(&arm_cfft_sR_f32_len128, testInput_f32, ifftFlag, doBitReverse);  //用的这个函数
//		arm_cmplx_mag_f32(testInput_f32, testOutput, fftSize);
	  /* Process the data through the Complex Magnitude Module for
	  calculating the magnitude at each bin */
//	  arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);

	  /* Calculates maxValue and returns corresponding BIN value */
//	  arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);



//		ftemp_N = (float)((testInput_f32[2] * testInput_f32[2]) + (testInput_f32[3] * testInput_f32[3]));
//		ftemp_N = sqrt(ftemp_N);
//		fBase	= ftemp_N;
//
//		int m=0;
//		ftemp_P = 0;
//		for(int i=0;i<50;i++)
//		{
//			ftemp_N = (float)((testInput_f32[m+4] * testInput_f32[m+4]) + (testInput_f32[m+5] * testInput_f32[m+5]));
//			ftemp_N = sqrt(ftemp_N);
//			if(Harmonictemp.Channel<0x03)
//			{
//				HarmonicData.HarmonicpercentU[Harmonictemp.Channel][i+2] = ftemp_N  /fBase;
//				ftemp_P += (((float)HarmonicData.HarmonicpercentU[0][i+2]) * ((float)HarmonicData.HarmonicpercentU[0][i+2]));
//			}
//			else
//			{
//				HarmonicData.HarmonicpercentI[Harmonictemp.Channel-3][i+1] = ftemp_N  /fBase ;
//			}
//			m+=2;
//		}

//		FFT(HarmonicData.iRealArray, HarmonicData.iMageArray);

//

//        int s=0;
//		for(int i=0;i<60;i++)
//		{
//		 printf("第%d实:%f\r\n",i,HarmonicData.iRealArray[i]); 	//	归一化值
//		 printf("第%d实:%f\r\n",i,testInput_f32[s]); 	//	归一化值
//		 printf("第%d虚:%f\r\n",i,HarmonicData.iMageArray[i]); 	//	归一化值
//		 printf("第%d虚:%f\r\n",i,testInput_f32[s+1]); 	//	归一化值
//		 s+=2;
//		}
//		int s=0;
//		for(int i=0;i<60;i++)
//		{
//		 printf("第%d实:%f,",i,HarmonicData.iRealArray[i]); 	//	归一化值
//		 printf("第%d实:%f,",i,testInput_f32[s]); 	//	归一化值
//		 printf("第%d虚:%f,",i,HarmonicData.iMageArray[i]); 	//	归一化值
//		 printf("第%d虚:%f,",i,testInput_f32[s+1]); 	//	归一化值
//		 s+=2;
//		}

//
//		ftemp_N = (float)(((float)HarmonicData.iRealArray[1] *(float)HarmonicData.iRealArray[1]) + ((float)HarmonicData.iMageArray[1] * (float)HarmonicData.iMageArray[1]));
//		ftemp_N = sqrt(ftemp_N);
//		fBase	= ftemp_N;
//
//
//		ftemp_N = (float)(((float)HarmonicData.iRealArray[0] *(float)HarmonicData.iRealArray[0]) + ((float)HarmonicData.iMageArray[0] * (float)HarmonicData.iMageArray[0]));
//		HarmonicData.HarmonicpercentU[1][0]=sqrt(ftemp_N)/fBase;
////
//		ftemp_P1 = 0;
//		for(int i=0;i<50;i++)
//	    {
//	    	ftemp_N = (float)(((float)HarmonicData.iRealArray[i+2] *(float)HarmonicData.iRealArray[i+2]) + ((float)HarmonicData.iMageArray[i+2] * (float)HarmonicData.iMageArray[i+2]));
//	    	ftemp_N = sqrt(ftemp_N);
//	    	if(Harmonictemp.Channel<0x03)
//	    	{
////	    		HarmonicData.HarmonicpercentU[1][i+2] = ( ftemp_N *Fftcoefficient[i]) /fBase;
//	    		HarmonicData.HarmonicpercentU[1][i+2] = ftemp_N/fBase;
//	    		ftemp_P1 += (((float)HarmonicData.HarmonicpercentU[1][i+2]) * ((float)HarmonicData.HarmonicpercentU[1][i+2]));
//	    	}
//	    	else
//	    	{
//	    		HarmonicData.HarmonicpercentI[Harmonictemp.Channel-3][i+1] = ftemp_N  /fBase ;
//	    	}
//	    }

//		for(int i=0;i<51;i++)
//		{
//		 printf("第%d次谐波畸形率,%f\r\n",i,HarmonicData.HarmonicpercentU[0][i]); 	//	归一化值
//		 printf("第%d次谐波畸形率,%f\r\n",i,HarmonicData.HarmonicpercentU[1][i]); 	//	归一化值
//		}
//		for(int i=0;i<51;i++)
//		{
//		 printf("第%d次谐波含有率：%f,",i,HarmonicData.HarmonicpercentU[0][i]); 	//	归一化值
//		 printf("第%d次谐波含有率：%f,",i,HarmonicData.HarmonicpercentU[1][i]); 	//	归一化值
//		}
//		if(Harmonictemp.Channel<0x03)
//		{
//			ftemp_P = sqrt(ftemp_P);
//			HarmonicData.HarmonicpercentU[Harmonictemp.Channel][0] = ftemp_P;
//		}
//		 printf("总谐波含量%f\r\n",HarmonicData.HarmonicpercentU[0][0] );

//		if(Harmonictemp.Channel<0x03)
//		{
//			ftemp_P1 = sqrt(ftemp_P1);
//			HarmonicData.HarmonicpercentU[1][0] = ftemp_P1;
//		}
//		 printf("总谐波含量%f\r\n",HarmonicData.HarmonicpercentU[1][0] );






//		for(int i=Harmonictemp.Channel;i<6;i++)
//		{
//			Harmonictemp.Channel++;
//			Harmonictemp.ReadAdress+=128;
//		}
//
//		if(Harmonictemp.Channel > 5 )
//		{
//			Harmonictemp.StarFlag = 0;
//			return;
//		}
//
//	}
}

void Three_Phase_Unbalance(void)
{
	int32_t temp1[3];
	int32_t temp2[3];
	float average_U;
	float Unbalance_U;
	float Unbalance_I;

	for(int i=0;i<6;i++)
	{
		Dl645FrontTmp.UI[i] = 0 ;
//		Rn8302Read( 0x0007+i , (uint32_t *)&Dl645FrontTmp.UI[i] , 4 ) ;  //基波电压有效值
	}

	if(Dl645FrontTmp.UI[0]>Dl645FrontTmp.UI[1])
	{
		temp1[1]=Dl645FrontTmp.UI[0];
		temp1[2]=Dl645FrontTmp.UI[1];
	}else{
		temp1[1]=Dl645FrontTmp.UI[1];
		temp1[2]=Dl645FrontTmp.UI[0];
	}

	if(Dl645FrontTmp.UI[2]>temp1[1])
	{
		temp1[0]=Dl645FrontTmp.UI[2];
	}else if(Dl645FrontTmp.UI[2]>temp1[2]){
		temp1[0]=temp1[1];
		temp1[1]=Dl645FrontTmp.UI[2];
	}else{
		temp1[0]=temp1[1];
		temp1[1]=temp1[2];
		temp1[2]=Dl645FrontTmp.UI[2];
	}

	if(Dl645FrontTmp.UI[3]>Dl645FrontTmp.UI[4])
	{
		temp2[1]=Dl645FrontTmp.UI[3];
		temp2[2]=Dl645FrontTmp.UI[4];
	}else{
		temp2[1]=Dl645FrontTmp.UI[4];
		temp2[2]=Dl645FrontTmp.UI[3];
	}

	if(Dl645FrontTmp.UI[5]>temp2[4])
	{
		temp2[0]=Dl645FrontTmp.UI[5];
	}else if(Dl645FrontTmp.UI[5]>temp2[2]){
		temp2[0]=temp2[1];
		temp2[1]=Dl645FrontTmp.UI[5];
	}else{
		temp2[0]=temp2[1];
		temp2[1]=temp2[2];
		temp2[2]=Dl645FrontTmp.UI[5];
	}

	average_U=(temp1[0]+temp1[1]+temp1[2])/3;
	Unbalance_U=(temp1[0]-average_U)/average_U;
	Unbalance_I=(temp2[0]-temp2[2])/temp2[0];

	return;
}

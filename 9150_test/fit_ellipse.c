#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include "9150_test.h"

CONFIG_TEMP_VALUE equation_temp_value(double *ax,double *ay,CONFIG_TEMP_VALUE g_equation_temp_value,unsigned int m)
{
	double D[m][5],S[5][m],sum[5];
	
	for(int i=0;i<m;i++)
		for(int j=0;j<5;j++)
			D[i][j]=0.0;

	g_equation_temp_value.V = (double **)malloc(sizeof (double*)* 5);
	for (int i=0;i<5;i++)
	{
		g_equation_temp_value.V[i] = (double *)malloc(sizeof (double)* 5);
	}

	for(int i=0;i<m;i++)
	{
		D[i][0]=ax[i]*ax[i];
		D[i][1]=ax[i]*ay[i];
		D[i][2]=ay[i]*ay[i];
		D[i][3]=ax[i];
		D[i][4]=ay[i];
	}

	for(int i=0;i<m;i++)
		for(int j=0;j<5;j++)
			S[j][i]=D[i][j];//矩阵转置

	for(int i=0;i<5;i++)
		for(int j=0;j<5;j++)	
			{
				g_equation_temp_value.V[i][j]=0.0;
				for(int k=0;k<m;k++)
				{
					g_equation_temp_value.V[i][j]+=S[i][k]*D[k][j];
				}
			}

	for(int i=0;i<5;i++)
	{
		sum[i]=0.0;
		for(int j=0;j<m;j++)
			sum[i]+=D[j][i];
		g_equation_temp_value.bb[i] *= sum[i];
	}//每一列的和
	return g_equation_temp_value;
}


ELLIPSE_PARAMETER ellipse_parameter_calculate(CONFIG_TEMP_VALUE g_equation_temp_value,ELLIPSE_PARAMETER g_ellipse_parameter){
	double cos_phi,sin_phi,pm_test,long_axis,short_axis;
	double A0,B0,C0,D0,E0,X0,Y0;
	g_ellipse_parameter.A0 = g_equation_temp_value.bb[0];
	g_ellipse_parameter.B0 = g_equation_temp_value.bb[1];
	g_ellipse_parameter.C0 = g_equation_temp_value.bb[2];
	g_ellipse_parameter.D0 = g_equation_temp_value.bb[3];
	g_ellipse_parameter.E0 = g_equation_temp_value.bb[4];
	A0 = g_equation_temp_value.bb[0];
	B0 = g_equation_temp_value.bb[1];
	C0 = g_equation_temp_value.bb[2];
	D0 = g_equation_temp_value.bb[3];
	E0 = g_equation_temp_value.bb[4];
	//printf("A0=%lf,B0=%lf,C0=%lf,D0=%lf,E0=%lf\n",A0,B0,C0,D0,E0);

	if(min(abs(B0/A0),abs(B0/C0))>ot)//判断是否为椭圆
	{
		g_ellipse_parameter.orientation_rad=0.5*atan(B0/(A0-C0));//calculate the tilt value
		cos_phi=cos(g_ellipse_parameter.orientation_rad);
		sin_phi=sin(g_ellipse_parameter.orientation_rad);
	}
	else
	{
		g_ellipse_parameter.orientation_rad=0;
		cos_phi=cos(g_ellipse_parameter.orientation_rad);
		sin_phi=sin(g_ellipse_parameter.orientation_rad);
	}
	//if we found an ellipse return it's data
	pm_test=A0*C0;
	if(pm_test>0)
	{
		//make sure coefficients are positive as required
		if(A0<0)
		{
			A0=-A0;
			C0=-C0;
			D0=-D0;
			E0=-E0;

		}
		
		g_ellipse_parameter.X0 = (B0*E0 - 2*C0*D0)/(4*A0*C0 - B0*B0);
		g_ellipse_parameter.Y0 = (B0*D0 - 2*A0*E0)/(4*A0*C0 - B0*B0);
		X0 = g_ellipse_parameter.X0;
		Y0 = g_ellipse_parameter.Y0;
	
		/*a_radius = 2*sqrt(abs(-2.0*F/(A0 + C0 - sqrt(B0*B0 + ((A0 - C0)/F)*((A0 - C0)/F)))));
		b_radius = 2*sqrt(abs(-2.0*F/(A0 + C0 + sqrt(B0*B0 + ((A0 - C0)/F)*((A0 - C0)/F)))));*/
		g_ellipse_parameter.a_radius = sqrt(abs(-(g_ellipse_parameter.F + A0*X0*X0 + B0*X0*Y0 +C0*Y0*Y0 +D0*X0 + E0*Y0)/(A0*cos_phi*cos_phi + B0*cos_phi*sin_phi +C0*sin_phi*sin_phi)));
		g_ellipse_parameter.b_radius = sqrt(abs(-(g_ellipse_parameter.F + A0*X0*X0 + B0*X0*Y0 +C0*Y0*Y0 +D0*X0 + E0*Y0)/(A0*sin_phi*sin_phi - B0*cos_phi*sin_phi +C0*cos_phi*cos_phi)));
		long_axis=2*max(g_ellipse_parameter.a_radius,g_ellipse_parameter.b_radius);
		short_axis=2*min(g_ellipse_parameter.a_radius,g_ellipse_parameter.b_radius);
	}
	

	//printf("a=%lf,b=%lf,phi=%lf,X0=%lf,Y0=%lf,long_axis=%lf,short_axis=%lf\n",a_radius,b_radius,orientation_rad*57.3,X0,Y0,long_axis,short_axis);
	return g_ellipse_parameter;
}
	
void Gauss(double **pCoff,double *pConst,int n)
{
	double t,d;
	int i,j,k,row,flag,*pCol;

	// 临时缓冲区，存放列数
	pCol = (int *)malloc(sizeof (int)*n);
	if(pCol != NULL)
	{
		//消元
		flag = 1;

		for (k=0;k<n-1;k++)
		{
			d = 0.0;
			for (i=k;i<n;i++)
			{
				for(j=k;j<n;j++)
				{
					t = fabs(pCoff[i][j]);
					if (t>d)
					{
						d = t;
						pCol[k] = j;
						row = i;
					}
				}
			}

			if (d==0.0)
				flag = 0;

			//列交换
			else 
			{
				if (pCol[k]!=k)
				{
					for (i=0;i<n;i++)
					{
						t = pCoff[i][k];
						pCoff[i][k] = pCoff[i][pCol[k]];
						pCoff[i][pCol[k]] = t;
					}
				}
				//行交换
				if (row != k)
				{
					for (j=k;j<n;j++)
					{
						t = pCoff[k][j];
						pCoff[k][j] = pCoff[row][j];
						pCoff[row][j] = t;
					}
					
				   t=pConst[k]; 
				   pConst[k]=pConst[row]; 
				   pConst[row]=t;
				}
			}
		

			//没有解
			if(flag == 0)
			{
				free(pCol);
				pCol = NULL;
				//delete [] pCol;
				//cout << "没有解！?" <<endl;
				printf("The question is no solution");
			}

			d = pCoff[k][k];
			for (j=k;j<n;j++)
			{
				pCoff[k][j] = pCoff[k][j]/d;
			}
			
			pConst[k] = pConst[k]/d;

			for (i=k+1;i<n;i++)
			{
				for(j=k+1;j<n;j++)
				{
					pCoff[i][j] = pCoff[i][j]-pCoff[i][k]*pCoff[k][j];
				}
				pConst[i] = pConst[i]-pCoff[i][k]*pConst[k];
			}
		}

			//求解失败
			d = pCoff[n-1][n-1];
			if(d==0)
			{
				//delete [] pCol;
				//cout << "没有解！" <<endl;
				free(pCol);
				pCol = NULL;
				printf("The question is no solution");
			}

			//代入求解
			pConst[n-1] = pConst[n-1]/d;
			for (i=n-2;i>=0;i--)
			{
				t = 0.0;
				for (j=i+1;j<n;j++)
				{
					t = t+pCoff[i][j]*pConst[j];
				}
				pConst[i] =  pConst[i]-t;
			}

			//调整解的位置
			pCol[n-1]=n-1;
			for (k=n-1;k>=0;k--)
			{
				if (pCol[k]!=k)
				{ 
					t=pConst[k]; 
					pConst[k]=pConst[pCol[k]]; 
					pConst[pCol[k]]=t;
				}
			}

			//回收内存
			free(pCol);
			pCol = NULL;					
	}
	else
 
	{	 
		printf("mallocerror!\n");				 
		exit(-1);	 
	}

}

double MAX(double *a,unsigned int m)
{
	double max_val;
	max_val = a[0];
	for(int i=1;i<m;i++)
	{
		if(a[i] > max_val)
			max_val = a[i];
	}
	return max_val;
}

double MIN(double *a,unsigned int m)
{
	double min_val;
	min_val = a[0];
	for(int i=1;i<m;i++)
	{
		if(a[i] < min_val)
			min_val = a[i];
	}
	return min_val;
}
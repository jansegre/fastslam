//**ESTRATEGIA DE EXPLORACAO + SLAM + CAMERA**
//**           AMBIENTE COMPLEXO            **
//************(02 VEICULOS)********************
#include "stdafx.h"
#include <highgui.h>
#include <cv.h>
#include <cxcore.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <string>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/random.hpp>
#include <ctime>

//Outras Bibliotecas
#include <playerc++.h>
#include "R2D2.h"
#include "cholesky.hpp"
#include "inverse.hpp"
#include "stlastar1.h"
#include "veiculos.h"
#include "fsaida.h"
#include "SIFT.h"

//TEM QUE SER O VALOR EXATO SENÃO PROBLEMA DE ASSOCIAÇÃO DE DADOS
#define nLm     38	 	     //IECON
//#define nLm     16	    //ICCA

#define tamB   128

using namespace std;
using namespace PlayerCc;

CvMat* matO;
CvMat* matM;
CvMat* cov;

int NrDsc;
struct{
	float   FV[tamB]; //Buffer de descritores
	double  d;
}TabDsc[nLm];

R2D2 *r00;
R2D2 *r11;



int cl=0; //NUMERO DE CELULAS LIVRES
boost::numeric::ublas::matrix<double> R(3,3);
double Lm[3][nLm];
long double Zcam2[5][nLm];
long double Zcam1[5][nLm];
std::vector<int> tagCam1;
std::vector<int> tagCam2;;



double E0=0;
double P0=0.9;
class fastslams
{
public:
	int irb;

	/*************************************/
	/*            FILTRO                 */
	/*************************************/
	//Matrizes de Covari�ncia

	double xv,yv,tv;   //POSE DO VEICULO
    fastslams(int ir);
	/*************************************/
	/*      FUNCOES DE EXPLORACAO       */
	/*************************************/
	void   PosV(double ddx, double ddy, double yaw);
	void   PosO(double   x, double   y, double   t);
	void   CDir(int i, double* dscx, double* dscy, double* yaw , int TAMCAMINHO);
	/*************************************/
	/*      FUNCOES DO FILTRO            */
	/*************************************/
	//void   ObsCamera(void);
	void   IniciaPart(int irb);
	void   PosP(int irb,int ip, double dscx, double dscy, double yaw, double giro);
	void   PosOdom(double dscx, double dscy, double yaw, double giro);
	void   AdicMarco (int irb,int ip,  int obs);
	void   AtzMarco (int irb,int ip,int obs);
	void   PesoAss(int irb, int ip, int obs,  double * wmax); //Peso da associação dos marcadores
	void   ReamostreC(int irb);

};

fastslams::fastslams(int ir){
	irb=ir;
	
	//IECON

	Lm[0][0]=  -3; Lm[1][0]=  4.5; Lm[2][0]= 1;
	Lm[0][1]=  -3; Lm[1][1]= -4.5; Lm[2][1]= 1;
	Lm[0][2]=   3; Lm[1][2]=  4.5; Lm[2][2]= 1;
	Lm[0][3]=   3; Lm[1][3]= -4.5; Lm[2][3]= 1;
	Lm[0][4]=  -5; Lm[1][4]=  4.5; Lm[2][4]= 1;
	Lm[0][5]=  -5; Lm[1][5]= -4.5; Lm[2][5]= 1;
	Lm[0][6]=   5; Lm[1][6]=  4.5; Lm[2][6]= 1;
	Lm[0][7]=   5; Lm[1][7]= -4.5; Lm[2][7]= 1;
	Lm[0][8]=  -6; Lm[1][8]=  1;   Lm[2][8]= 1;
	Lm[0][9]=  -6; Lm[1][9]= -1;   Lm[2][9]=  1;
	Lm[0][10]=  6; Lm[1][10]= 1; Lm[2][10]= 1;
	Lm[0][11]=  6; Lm[1][11]=-1; Lm[2][11]= 1;
	Lm[0][12]=  2; Lm[1][12]= 1; Lm[2][12]= 1;
	Lm[0][13]=  2; Lm[1][13]=-1; Lm[2][13]= 1;
	Lm[0][14]= -2; Lm[1][14]= 1; Lm[2][14]= 1;
	Lm[0][15]= -2; Lm[1][15]=-1; Lm[2][15]= 1;
	Lm[0][16]= -8; Lm[1][16]= 1; Lm[2][16]= 1;
	Lm[0][17]= -8; Lm[1][17]=-1; Lm[2][17]= 1;
	Lm[0][18]=  8; Lm[1][18]= 1; Lm[2][18]= 1;
	Lm[0][19]=  8; Lm[1][19]=-1; Lm[2][19]= 1;
	Lm[0][20]=  0; Lm[1][20]= -1; Lm[2][20]= 1;
	Lm[0][21]=  0; Lm[1][21]=  1; Lm[2][21]= 1;
	Lm[0][22]= -3; Lm[1][22]=  3; Lm[2][22]= 1;
	Lm[0][23]= -3; Lm[1][23]= -3; Lm[2][23]= 1;
	Lm[0][24]=  3; Lm[1][24]=  3; Lm[2][24]= 1;
	Lm[0][25]=  3; Lm[1][25]= -3; Lm[2][25]= 1;
	Lm[0][26]= -5; Lm[1][26]=  3; Lm[2][26]= 1;
	Lm[0][27]= -5; Lm[1][27]= -3; Lm[2][27]= 1;
	Lm[0][28]=  5; Lm[1][28]=  3; Lm[2][28]= 1;
	Lm[0][29]=  5; Lm[1][29]= -3; Lm[2][29]= 1;
	Lm[0][30]=-3; Lm[1][30]= 1.5; Lm[2][30]= 1;
	Lm[0][31]=-3; Lm[1][31]=-1.5; Lm[2][31]= 1;
	Lm[0][32]= 3; Lm[1][32]= 1.5; Lm[2][32]= 1;
	Lm[0][33]= 3; Lm[1][33]=-1.5; Lm[2][33]= 1;
	Lm[0][34]=-5; Lm[1][34]= 1.5; Lm[2][34]= 1;
	Lm[0][35]=-5; Lm[1][35]=-1.5; Lm[2][35]= 1;
	Lm[0][36]= 5; Lm[1][36]= 1.5; Lm[2][36]= 1;
	Lm[0][37]= 5; Lm[1][37]=-1.5; Lm[2][37]= 1;



	/*
	//ICCA
	Lm[0][0]= -14; Lm[1][0]=   1.5; Lm[2][0]= 2;
	Lm[0][1]= -10; Lm[1][1]=   1.5; Lm[2][1]= 2;
	Lm[0][2]=  -6; Lm[1][2]=   1.5; Lm[2][2]= 2;
	Lm[0][3]=  -2; Lm[1][3]=   1.5; Lm[2][3]= 2;
	Lm[0][4]=   2; Lm[1][4]=   1.5; Lm[2][4]= 2;
	Lm[0][5]=   6; Lm[1][5]=   1.5; Lm[2][5]= 2;
	Lm[0][6]=  10; Lm[1][6]=   1.5; Lm[2][6]= 2;
	Lm[0][7]=  14; Lm[1][7]=   1.5; Lm[2][7]= 2;
	Lm[0][8]= -14; Lm[1][8]=  -1.5; Lm[2][8]= 2;
	Lm[0][9]= -10; Lm[1][9]=  -1.5; Lm[2][9]= 2;
	Lm[0][10]= -6; Lm[1][10]= -1.5; Lm[2][10]= 2;
	Lm[0][11]= -2; Lm[1][11]= -1.5; Lm[2][11]= 2;
	Lm[0][12]=  2; Lm[1][12]= -1.5; Lm[2][12]= 2;
	Lm[0][13]=  6; Lm[1][13]= -1.5; Lm[2][13]= 2;
	Lm[0][14]= 10; Lm[1][14]= -1.5; Lm[2][14]= 2;
	Lm[0][15]= 14; Lm[1][15]= -1.5; Lm[2][15]= 2;
	*/
	for(int r=0; r<=nLm;r++)
	{
		TabDsc[r].d =1000;
	}
	//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
	//Matrizes de Covari�ncia
	R(0,0)=pow(sR,2); R(0,1)= 0;          R(0,2)= 0;
	R(1,0)= 0;        R(1,1)=pow(sB1,2);   R(1,2)= 0;
	R(2,0)= 0;        R(2,1)=0;           R(2,2)= pow(sB2,2);

	NrDsc=-1;
	/*************************************/
	/*            EXPLORA��O             */
	/*************************************/
	//Matrizes e Vetores do OpenCV
	matO = cvCreateMat( 128, 1,   CV_32FC1 );
	matM = cvCreateMat( 128, 1,   CV_32FC1 );
	cov  = cvCreateMat( 128, 128, CV_32FC1 );
	for(int m=0;m<128;m++)
	{
		for(int n=0;n<128;n++){
			*((float*)CV_MAT_ELEM_PTR( *cov, m, n ) )= 0;
			if (m==n){*((float*)CV_MAT_ELEM_PTR( *cov, m, n ) )= 1;}
		}
	}
	tagCam1.clear();tagCam2.clear();
}

void fastslams::IniciaPart(int irb){
	//Inicializar Partículas
	double np = NP;
	for (int i=0;i<NP;i++)
	{
		part[irb] [i].n = -1;

		part[irb] [i].w = 1/np;
		part[irb][i].x=xv; part[irb][i].y=yv; part[irb][i].t=tv;
		for (int j=0;j<=NF;j++)
		{
			part[irb][i].p[j]=0;

			part[irb] [i].xf [0][j]=0; part[irb][i].xf [1][j]=0; part[irb][i].xf [2][j]=0;

			part[irb] [i].pf [0][j]=0; part[irb][i].pf [1][j]=0; part[irb] [i].pf [2][j]=0;
			part[irb] [i].pf [3][j]=0; part[irb][i].pf [4][j]=0; part [irb][i].pf [5][j]=0;
			part[irb] [i].pf [6][j]=0; part[irb][i].pf [7][j]=0; part [irb][i].pf [8][j]=0;

		}
	}
}

void ObsLaser(double xpos1, double ypos1, double yaw1, double xpos2, double ypos2, double yaw2)
{
	r00->ReadSensors();
	//sleep(8);
	r11->ReadSensors();
	//DISTANCIA, PONTOS, ORIENTACAO
	AtzGrid(r00->laser, r00->laserConf,r00->bear,r00->pos,r11->laser, r11->laserConf,r11->bear,r11->pos);
	AtzGridBP(r00->laser, r00->laserConf,r00->bear,r11->laser, r11->laserConf,r11->bear, xpos1, ypos1, yaw1, xpos2, ypos2, yaw2);
	AtzMap(r00->laser, r00->laserConf,r00->bear,r11->laser, r11->laserConf,r11->bear, xpos1, ypos1, yaw1, xpos2, ypos2, yaw2);
	SaveGrid();SaveMap();SaveGridBP();
}


void fastslams::CDir(int i, double* dscx, double* dscy, double* yaw, int TAMCAMINHO){
		double xv1,yv1;
		int i0,j0,i1,j1,i2,j2,i12,j12,i01,j01;

		i0=Path[irb].x[i-1]; j0=Path[irb].y[i-1]; //ONDE ESTA
		i1=Path[irb].x[i];   j1=Path[irb].y[i];   //PARA ONDE IR
		i2=Path[irb].x[i+1]; j2=Path[irb].y[i+1]; //PROXIMO PASSO
		i12=i2-i1;
		j12=j2-j1;
		i01=i1-i0;
		j01=j1-j0;

		if (i<TAMCAMINHO)
		{
			if((i12==0)&&(j12== 1)) *yaw= pi/2;
			if((i12==0)&&(j12==-1)) *yaw=-pi/2;
			if((j12==0)&&(i12== 1)) *yaw=    0;
			if((j12==0)&&(i12==-1)) *yaw=  -pi;

			if((i12==1)&&(j12== 1)) *yaw= pi/4;
			if((i12==1)&&(j12==-1)) *yaw=-pi/4;
			if((i12==-1)&&(j12== 1)) *yaw= (3*pi)/4;
			if((i12==-1)&&(j12==-1)) *yaw=-(3*pi)/4;

		}
		else
		{
			if((i01==0)&&(j01== 1)) *yaw= pi/2;
			if((i01==0)&&(j01==-1)) *yaw=-pi/2;
			if((j01==0)&&(i01== 1)) *yaw=    0;
			if((j01==0)&&(i01==-1)) *yaw=  -pi;

			if((i01==1)&&(j01== 1)) *yaw= pi/4;
			if((i01==1)&&(j01==-1)) *yaw=-pi/4;
			if((i01==-1)&&(j01== 1)) *yaw= (3*pi)/4;
			if((i01==-1)&&(j01==-1)) *yaw=-(3*pi)/4;

		}

		xv1 = (i1-0.5-TAM/2)/ESCALA;
		yv1 = (j1-0.5-TAM/2)/ESCALA;

		*dscx=xv1-xv;
		*dscy=yv1-yv;
}
void fastslams::PosV(double ddx, double ddy, double yaw){
	xv=ddx;
	yv=ddy;
	tv=pi_to_pi(yaw);
}



void fastslams::PosP(int irb, int ip, double dscx, double dscy, double yaw, double giro)
{
	//DESTA FORMA É MELHOR!!!!!
	double dx,dy,dyaw;

	double sample1 = SampleNormal(0,0.5); if (sample1>1) sample1=1;  if (sample1<-1) sample1=-1;
	double sample2 = SampleNormal(0,0.5); if (sample2>1) sample2=1;  if (sample2<-1) sample2=-1;
	double sample3 = SampleNormal(0,0.5); if (sample3>1) sample3=1;  if (sample3<-1) sample3=-1;

	dx =    (0.025)*sample1;
	dy =    (0.025)*sample2;

	part[irb][ip].x=part[irb][ip].x + dscx + dx;
	part[irb][ip].y=part[irb][ip].y + dscy + dy;

	dyaw =  (3*pi/180)*sample3;
	part[irb][ip].t=pi_to_pi(part[irb][ip].t + giro +dyaw);

}

void fastslams::PosOdom(double dscx, double dscy, double yaw, double giro){
	//DESTA FORMA É MELHOR!!!!!
	xodom[irb]=xodom[irb] + 0.5 + 0.025;
	yodom[irb]=yodom[irb] + 0.5 + 0.025;
	todom[irb]=pi_to_pi(todom[irb] + giro +3.0*pi/180);
}
int selecao (int i, int j){

	int ne=0,oc=0;

	if (Grid.occ[i][j]<0)
	{
		for(int m=i-REGIAOF;m<=i+REGIAOF;m++)
		{
			for(int n=j-REGIAOF;n<=j+REGIAOF;n++)
			{
				if((m>=0)&&(m<TAM)&&(n>=0)&&(n<TAM))
				{
					if(Grid.occ[m][n]==0)
					{
						ne++;
					}
					if(Grid.occ[m][n]==1)
					{
						oc++;
					}

				}
			}
		}
		if ((ne>=1)&&(oc==0))
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else return 0;
}

int destino (int i, int j){

	int oc=0;

	for(int m=i-REGIAOF;m<i+REGIAOF;m++)
	{
		for(int n=j-REGIAOF;n<j+REGIAOF;n++)
		{
			if((m>=0)&&(m<TAM)&&(n>=0)&&(n<TAM))
			{

				if(Grid.occ[m][n]==1)
				{
					oc++;
				}
			}

		}
	}
	if (oc>=1)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

int FrontGrid(int iv1, int jv1, int iv2, int jv2){
	CustosNav(iv1,jv1,iv2,jv2);
	//Encontrar celulas fronteiras
	cl=0;
	Cfr.n=0;
	for(int i=0;i<TAM;i++)
	{
		for(int j=0;j<TAM;j++)
		{
			if (Grid.occ[i][j]<0){cl++;}
			if(selecao(i,j)==1)
			{
				if (((i==iv1)&&(j==jv1))||((i==iv2)&&(j==jv2))){}
				else
				{
					Cfr.x [Cfr.n]=i;
					Cfr.y [Cfr.n]=j;
					Cfr.up[Cfr.n]=1; 
					Cfr.n = Cfr.n+1;
				}
			}
			
		}//Fim For j
	}//Fim For i
	//ARMAZENANDO CUSTO DE NAVEGACAO
	for (int i=0 ; i < Cfr.n; i++)
	{
		Cfr.gn1[i]=Grid.cn1[Cfr.x[i]][Cfr.y[i]];
		Cfr.gn2[i]=Grid.cn2[Cfr.x[i]][Cfr.y[i]];
		Cfr.u1[i]= Cfr.up[i]-Cfr.gn1[i];
		Cfr.u2[i]= Cfr.up[i]-Cfr.gn2[i];
		if (Cfr.u1[i]>=Cfr.u2[i])
		{
			Cfr.uf[i]=Cfr.u1[i];
		}
		else
		{
			Cfr.uf[i]=Cfr.u2[i];
		}
	}
	if (Cfr.n >= 1){cout<<"CELULAS FRONTEIRAS: "<< Cfr.n<<"\n"; return 1;}else{cout<<"NAO EXISTEM CELULAS FRONTEIRAS\n";return 0;}
}



//Igual o Artigo....
int DstGrid(fastslams f, fastslams g)
{
	int     epath=0;
	int     idst, jdst;
	int     iv1=0;
	int     jv1=0;
	int     iv2=0;
	int     jv2=0;

	iv1 = (int) floor (f.xv*ESCALA + 0.5)+TAM/2;
	jv1 = (int) floor (f.yv*ESCALA + 0.5)+TAM/2;
	iv2 = (int) floor (g.xv*ESCALA + 0.5)+TAM/2;
	jv2 = (int) floor (g.yv*ESCALA + 0.5)+TAM/2;

	//ENCONTRA AS CELULAS FRONTEIRAS
	if (FrontGrid(iv1, jv1, iv2, jv2))
	{		
			double   best;
			int     ibest;
			//DESTINO PRIMEIRO VEICULO
			do
			{
				best=-1000;
				ibest= 0;

				for(int k=0;k<Cfr.n;k++)
				{
					if (Cfr.uf[k]> best){best=Cfr.uf[k]; ibest=k;}
				}			
				idst = Cfr.x[ibest];
				jdst = Cfr.y[ibest];

				if (Cfr.u1[ibest]>=Cfr.u2[ibest])
				{
					epath = AStarGrid(iv1,jv1,idst,jdst,f.irb);
				}
				else
				{
					epath = AStarGrid(iv2,jv2,idst,jdst,g.irb);
				}
				if (epath == 0) 
				{
					cout<<"1 => OUTRO => EXISTE CAMINHO ?:" << epath <<"\n";
					Cfr.uf[ibest]=-1000;
				}
			}
			while(epath==0);

			//ENCONTRANDO OS OBSTACULOS AO REDOR DO DESTINO
			//DO PRIMEIRO VEICULO...
			/*
			double obstac[2][80];
			double xdst =  (idst+beta-TAM/2)/ESCALA;
			double ydst =  (jdst+beta-TAM/2)/ESCALA;
			int    no=0;
			for(int m=0;m<TAM;m++)
			{
				for(int n=0;n<TAM;n++)
				{
					double xnv =  (m+beta-TAM/2)/ESCALA;
					double ynv =  (n+beta-TAM/2)/ESCALA;
					double dd =  sqrt(pow(xnv-xdst,2)+pow(ynv-ydst,2));
					if(dd < ALASER)
					{
						if (Grid.occ[m][n]>0)
						{
							obstac[0][no]=m;
							obstac[1][no]=n;
							no++;
						}
					}
				}
			}
			//REDUCAO DA UTILIDADE NO ENTORNO DA CELULA SELECIONADA COMO DESTINO---1 (C0MPLEXA)
			Cfr.up[ibest]=0;
			for(int k=0;k<Cfr.n;k++)
			{
				int i=Cfr.x[k];
				int j=Cfr.y[k];
				double xnv =  (i+beta-TAM/2)/ESCALA;
				double ynv =  (j+beta-TAM/2)/ESCALA;
				double dd =  sqrt(pow(xnv-xdst,2)+pow(ynv-ydst,2));

				if(dd < ALASER)
				{
					int existeo=0;
					int io,jo;
					if((xdst> xnv)&&(ydst> ynv))
					{
						for(int z=0;z<no;z++)
						{
							io = obstac[0][z];
							jo = obstac[1][z];
							double xo =  (io+beta-TAM/2)/ESCALA;
							double yo =  (jo+beta-TAM/2)/ESCALA;
							if((xo > xnv)&&(yo>ynv)){existeo=1;break;}
						}
					}

					if((xdst< xnv)&&(ydst> ynv))
					{
						for(int z=0;z<no;z++)
						{
							io = obstac[0][z];
							jo = obstac[1][z];
							double xo =  (io+beta-TAM/2)/ESCALA;
							double yo =  (jo+beta-TAM/2)/ESCALA;
							if((xo < xnv)&&(yo > ynv)){existeo=1;break;}
						}
					}


					if((xdst< xnv)&&(ydst< ynv))
					{
						for(int z=0;z<no;z++)
						{
							io = obstac[0][z];
							jo = obstac[1][z];
							double xo =  (io+beta-TAM/2)/ESCALA;
							double yo =  (jo+beta-TAM/2)/ESCALA;
							if((xo < xnv)&&(yo < ynv)){existeo=1;break;}
						}
					}

					if((xdst> xnv)&&(ydst< ynv))
					{
						for(int z=0;z<no;z++)
						{
							io = obstac[0][z];
							jo = obstac[1][z];
							double xo =  (io+beta-TAM/2)/ESCALA;
							double yo =  (jo+beta-TAM/2)/ESCALA;
							if((xo > xnv)&&(yo < ynv)){existeo=1;break;}
						}
					}

					if (existeo==0){Cfr.up[k]=dd/ALASER;}
				}
			}
			*/

			//REDUCAO DA UTILIDADE NO ENTORNO DA CELULA SELECIONADA COMO DESTINO---2 (SIMPLES)
			double xdst =  (idst+beta-TAM/2)/ESCALA;
			double ydst =  (jdst+beta-TAM/2)/ESCALA;
			for(int k=0;k<Cfr.n;k++)
			{
				double xx =  (Cfr.x[k]+beta-TAM/2)/ESCALA;
				double yy =  (Cfr.y[k]+beta-TAM/2)/ESCALA;
				double dd =  sqrt(pow(xx-xdst,2)+pow(yy-ydst,2));
				if (dd <= ALASER)  {Cfr.up[k]=dd/ALASER;}

			}
			//DESTINO SEGUNDO VEICULO
			if (Cfr.u1[ibest]>=Cfr.u2[ibest])
			{
					
				for(int k=0;k<Cfr.n;k++)
				{
					Cfr.uf[k]= Cfr.up[k]-Cfr.gn2[k];
				}
				epath=0;
				do
				{
					best=-1000;
					ibest= 0;
					for(int k=0;k<Cfr.n;k++)
					{
						if (Cfr.uf[k]> best){best=Cfr.uf[k]; ibest=k;}
					}
					idst = Cfr.x[ibest];
					jdst = Cfr.y[ibest];
					epath= AStarGrid(iv2,jv2,idst,jdst,g.irb);
					if (epath == 0)
					{
						cout<<"2 => OUTRO => EXISTE CAMINHO ?:" << epath <<"\n";
						Cfr.uf[ibest]=-1000;
					}
				}
				while(epath==0);
			}
			else
			{
				for(int k=0;k<Cfr.n;k++)
				{
					Cfr.uf[k]= Cfr.up[k]-Cfr.gn1[k];
				}
				epath=0;
				do
				{
					best=-1000;
					ibest= 0;
					for(int k=0;k<Cfr.n;k++)
					{
						if (Cfr.uf[k]> best){best=Cfr.uf[k]; ibest=k;}
					}
					idst = Cfr.x[ibest];
					jdst = Cfr.y[ibest];
					epath=AStarGrid(iv1,jv1,idst,jdst,f.irb);
					if (epath == 0)
					{
						cout<<"2 => OUTRO => EXISTE CAMINHO ?:" << epath <<"\n";
						Cfr.uf[ibest]=-1000;
					}
				}
				while(epath==0);
			}
			
			return 1;
	}
	else 
	{		
		return 0;	
	}
}



void ObsCameraf(fastslams f){
	double dx,dy,dz;
	double phi;
	char nome[10];
	tagCam1.clear();
	phi=f.tv;
	for(int i=0;i<nLm;i++)
	{
		
		dx=Lm[0][i]-f.xv;
		dy=Lm[1][i]-f.yv;
		dz=Lm[2][i];
		if  (( (fabs(dx)< MaxRangeC)  && (fabs(dy) < MaxRangeC) ) && ( dx*cos(phi) + dy*sin(phi) > 0 )  && ( pow(dx,(double)2) + pow(dy,(double)2)  < pow(MaxRangeC,(double)2))  && ( pow(dx,(double)2) + pow(dy,(double)2)  >  pow(MinRangeC,(double)2)))
		{

			tagCam1.push_back(i);

			double sample1 = SampleNormal(0,sig); if (sample1>1) sample1=1;  if (sample1<-1) sample1=-1;
			double sample2 = SampleNormal(0,sig); if (sample2>1) sample2=1;  if (sample2<-1) sample2=-1;
			double sample3 = SampleNormal(0,sig); if (sample3>1) sample3=1;  if (sample3<-1) sample3=-1;


			Zcam1[0][i]=sqrt(pow(dx,(double)2)+pow(dy,(double)2)+pow(dz,(double)2));
			Zcam1[0][i]=Zcam1[0][i]+sqrt(R(0,0))*sample1;
			Zcam1[1][i]=pi_to_pi(atan2(dy,dx) - phi);
			Zcam1[1][i]=Zcam1[1][i]+sqrt(R(1,1))*sample2;
			Zcam1[2][i]=pi_to_pi(atan2(dz,sqrt(pow(dx,(double)2) + pow(dy,(double)2))));
			Zcam1[2][i]=Zcam1[2][i]+sqrt(R(2,2))*sample3;


			//Descritor SIFT
			switch (i){
				case 0:
					strcpy(nome,"1.JPG");
				break;
				case 1:
					strcpy(nome,"2.JPG");
				break;
				case 2:
					strcpy(nome,"3.JPG");
				break;
				case 3:
					strcpy(nome,"4.JPG");
				break;
				case 4:
					strcpy(nome,"5.JPG");
				break;
				case 5:
					strcpy(nome,"6.JPG");
				break;
				case 6:
					strcpy(nome,"7.JPG");
				break;
				case 7:
					strcpy(nome,"8.JPG");
				break;
				case 8:
					strcpy(nome,"9.JPG");
				break;
				case 9:
					strcpy(nome,"10.JPG");
				break;
				case 10:
					strcpy(nome,"11.JPG");
				break;
				case 11:
					strcpy(nome,"12.JPG");
				break;
				case 12:
					strcpy(nome,"13.JPG");
				break;
				case 13:
					strcpy(nome,"14.JPG");
				break;
				case 14:
					strcpy(nome,"15.JPG");
				break;
				case 15:
					strcpy(nome,"16.JPG");
				break;
				case 16:
					strcpy(nome,"17.JPG");
				break;
				case 17:
					strcpy(nome,"18.JPG");
				break;
				case 18:
					strcpy(nome,"19.JPG");
				break;
				case 19:
					strcpy(nome,"20.JPG");
				break;
				case 20:
					strcpy(nome,"21.JPG");
				break;
				case 21:
					strcpy(nome,"22.JPG");
				break;
				case 22:
					strcpy(nome,"23.JPG");
				break;
				case 23:
					strcpy(nome,"24.JPG");
				break;
				case 24:
					strcpy(nome,"25.JPG");
				break;
				case 25:
					strcpy(nome,"26.JPG");
				break;
				case 26:
					strcpy(nome,"27.JPG");
				break;
				case 27:
					strcpy(nome,"28.JPG");
				break;
				case 28:
					strcpy(nome,"29.JPG");
				break;
				case 29:
					strcpy(nome,"30.JPG");
				break;
				case 30:
					strcpy(nome,"31.JPG");
				break;
				case 31:
					strcpy(nome,"32.JPG");
				break;
				case 32:
					strcpy(nome,"33.JPG");
				break;
				case 33:
					strcpy(nome,"34.JPG");
				break;
				case 34:
					strcpy(nome,"35.JPG");
				break;
				case 35:
					strcpy(nome,"36.JPG");
				break;
				case 36:
					strcpy(nome,"37.JPG");
				break;
				case 37:
					strcpy(nome,"38.JPG");
				break;
				case 38:
					strcpy(nome,"39.JPG");
				break;
				case 39:
					strcpy(nome,"40.JPG");
				break;
				case 40:
					strcpy(nome,"41.JPG");
				break;
				case 41:
					strcpy(nome,"42.JPG");
				break;
				case 42:
					strcpy(nome,"43.JPG");
				break;
				case 43:
					strcpy(nome,"44.JPG");
				break;

			}
			//std::cout<<nome<<"\n";
			//ARMAZENANDO Descritor SIFT DA OBSERVAÇÃO//
			double BuffF[tamB];
			SIFT *sift = new SIFT(nome, 4, 2);
			sift->DoSift();
			sift->GetDescritor(BuffF);

			//DISTANCIA DE MAHALANOBIS....
			for(int r=0; r<=NrDsc;r++)
			{
				for(int k=0;k<128;k++){
					*( (float*)CV_MAT_ELEM_PTR( *matO, k, 0 ) ) = TabDsc[r].FV[k];
					*( (float*)CV_MAT_ELEM_PTR( *matM, k, 0 ) ) = BuffF[k];
				}
				TabDsc[r].d= cvMahalanobis(matO,matM,cov);
			}

			double best  = 1000;
			int   jbest =   -1;
			for(int r=0; r<=NrDsc;r++)
			{
				if (TabDsc[r].d < best)
				{
					best =  TabDsc[r].d;
					jbest=r;
				}
			}
			if ((jbest!=-1)&&(best<=E0))
			{
				Zcam1[3][i]=(int) jbest;
				Zcam1[4][i]=0;
				cout<< f.irb << "->" << jbest<< " -> "<< best<<"\n";
				fresult<< f.irb << "->" << jbest<< " -> "<< best<<"\n";

			}
			else
			{
				NrDsc++;
				int k=0;
				while(k<128)
				{
					TabDsc[NrDsc].FV[k]=BuffF[k];
					k++;
				}
				Zcam1[3][i]=(int) NrDsc;
				Zcam1[4][i]=1;//NOVO MARCO
				ferro <<"rb: " << f.irb << " i:" << i << " nome: " << nome << " NrDsc: " << NrDsc <<  "\n";
				cout  <<"INSERI: " << "rb: " << f.irb << " i:" << i << " nome: " << nome << " NrDsc: " << NrDsc <<  "\n";

			}
			delete sift;
		}//Fim If
	}//Fim For Int
}//Fim Function

void ObsCamerag(fastslams g){
	double dx,dy,dz;
	double phi;
	char nome[10];
	tagCam2.clear();
	phi=g.tv;
	for(int i=0;i<nLm;i++)
	{

		dx=Lm[0][i]-g.xv;
		dy=Lm[1][i]-g.yv;
		dz=Lm[2][i];
		if  (( (fabs(dx)< MaxRangeC)  && (fabs(dy) < MaxRangeC) ) && ( dx*cos(phi) + dy*sin(phi) > 0 )  && ( pow(dx,(double)2) + pow(dy,(double)2)  < pow(MaxRangeC,(double)2))&& ( pow(dx,(double)2) + pow(dy,(double)2)  > pow(MinRangeC,(double)2)) )
		{

			tagCam2.push_back(i);
			double sample1 = SampleNormal(0,sig); if (sample1>1) sample1=1;  if (sample1<-1) sample1=-1;
			double sample2 = SampleNormal(0,sig); if (sample2>1) sample2=1;  if (sample2<-1) sample2=-1;
			double sample3 = SampleNormal(0,sig); if (sample3>1) sample3=1;  if (sample3<-1) sample3=-1;

			Zcam2[0][i]=sqrt(pow(dx,(double)2)+pow(dy,(double)2)+pow(dz,(double)2));
			Zcam2[0][i]=Zcam2[0][i]+sqrt(R(0,0))*sample1;
			Zcam2[1][i]=pi_to_pi(atan2(dy,dx) - phi);
			Zcam2[1][i]=Zcam2[1][i]+sqrt(R(1,1))*sample2;
			Zcam2[2][i]=pi_to_pi(atan2(dz,sqrt(pow(dx,(double)2) + pow(dy,(double)2))));
			Zcam2[2][i]=Zcam2[2][i]+sqrt(R(2,2))*sample3;


			//Descritor SIFT
			switch (i){
				case 0:
					strcpy(nome,"1.JPG");
				break;
				case 1:
					strcpy(nome,"2.JPG");
				break;
				case 2:
					strcpy(nome,"3.JPG");
				break;
				case 3:
					strcpy(nome,"4.JPG");
				break;
				case 4:
					strcpy(nome,"5.JPG");
				break;
				case 5:
					strcpy(nome,"6.JPG");
				break;
				case 6:
					strcpy(nome,"7.JPG");
				break;
				case 7:
					strcpy(nome,"8.JPG");
				break;
				case 8:
					strcpy(nome,"9.JPG");
				break;
				case 9:
					strcpy(nome,"10.JPG");
				break;
				case 10:
					strcpy(nome,"11.JPG");
				break;
				case 11:
					strcpy(nome,"12.JPG");
				break;
				case 12:
					strcpy(nome,"13.JPG");
				break;
				case 13:
					strcpy(nome,"14.JPG");
				break;
				case 14:
					strcpy(nome,"15.JPG");
				break;
				case 15:
					strcpy(nome,"16.JPG");
				break;
				case 16:
					strcpy(nome,"17.JPG");
				break;
				case 17:
					strcpy(nome,"18.JPG");
				break;
				case 18:
					strcpy(nome,"19.JPG");
				break;	
				case 19:
					strcpy(nome,"20.JPG");
				break;	
				case 20:
					strcpy(nome,"21.JPG");
				break;
				case 21:
					strcpy(nome,"22.JPG");
				break;
				case 22:
					strcpy(nome,"23.JPG");
				break;
				case 23:
					strcpy(nome,"24.JPG");
				break;
				case 24:
					strcpy(nome,"25.JPG");
				break;
				case 25:
					strcpy(nome,"26.JPG");
				break;
				case 26:
					strcpy(nome,"27.JPG");
				break;
				case 27:
					strcpy(nome,"28.JPG");
				break;
				case 28:
					strcpy(nome,"29.JPG");
				break;
				case 29:
					strcpy(nome,"30.JPG");
				break;
				case 30:
					strcpy(nome,"31.JPG");
				break;
				case 31:
					strcpy(nome,"32.JPG");
				break;
				case 32:
					strcpy(nome,"33.JPG");
				break;
				case 33:
					strcpy(nome,"34.JPG");
				break;
				case 34:
					strcpy(nome,"35.JPG");
				break;
				case 35:
					strcpy(nome,"36.JPG");
				break;
				case 36:
					strcpy(nome,"37.JPG");
				break;
				case 37:
					strcpy(nome,"38.JPG");
				break;
				case 38:
					strcpy(nome,"39.JPG");
				break;
				case 39:
					strcpy(nome,"40.JPG");
				break;
				case 40:
					strcpy(nome,"41.JPG");
				break;
				case 41:
					strcpy(nome,"42.JPG");
				break;
				case 42:
					strcpy(nome,"43.JPG");
				break;
				case 43:
					strcpy(nome,"44.JPG");
				break;

			}
			//std::cout<<nome<<"\n";
			//ARMAZENANDO Descritor SIFT DA OBSERVAÇÃO//
			double BuffF[tamB];
			SIFT *sift = new SIFT(nome, 4, 2);
			sift->DoSift();
			sift->GetDescritor(BuffF);
			//DISTANCIA DE MAHALANOBIS....
			for(int r=0; r<=NrDsc;r++)
			{
				for(int k=0;k<128;k++){
					*( (float*)CV_MAT_ELEM_PTR( *matO, k, 0 ) ) = TabDsc[r].FV[k];
					*( (float*)CV_MAT_ELEM_PTR( *matM, k, 0 ) ) = BuffF[k];
				}
				TabDsc[r].d= cvMahalanobis(matO,matM,cov);
			}

			double best  = 1000;
			int   jbest =   -1;
			for(int r=0; r<=NrDsc;r++)
			{
				if (TabDsc[r].d < best)
				{
					best =  TabDsc[r].d;
					jbest=r;
				}
			}
			if ((jbest!=-1)&&(best<=E0))
			{
				Zcam2[3][i]=(int) jbest;
				Zcam2[4][i]=0;
				cout<< g.irb << "->" <<jbest<< " -> "<< best<<"\n";
				fresult<< g.irb << "->" <<jbest<< " -> "<< best<<"\n";
			}
			else
			{
				NrDsc++;
				int k=0;
				while(k<128)
				{
					TabDsc[NrDsc].FV[k]=BuffF[k];
					k++;
				}
				Zcam2[3][i]=(int) NrDsc;
				Zcam2[4][i]=1;//NOVO MARCO
				ferro <<"rb: " << g.irb << " i:" << i << " nome: " << nome << " NrDsc: " << NrDsc <<  "\n";
				cout  <<"INSERI" << "rb: " << g.irb << " i:" << i << " nome: " << nome << " NrDsc: " << NrDsc <<  "\n";

			}
			delete sift;
		}//Fim If	
	}//Fim For Int
}//Fim Function

//INICIALIZAÇÃO DOS MARCOS
void fastslams::AdicMarco (int irb, int ip, int obs){

	long double r,r2,d, b1,b2, s,c,sb2,cb2, t,num;
	int ass;
	boost::numeric::ublas::matrix<long double> Gza(3,3),Gz(3,3), pf(3,3), xf(3,1), GzR(3,3);

	if (irb ==0 ){ass= Zcam1[3][obs];}
	if (irb ==1 ){ass= Zcam2[3][obs];}

	//DISTÂNCIAS E ORIENTACOES....
	if (irb ==0 ){
		r  =  Zcam1[0][obs];
		r2 =  pow(r,(double)2);
		b1 = Zcam1[1][obs];
		b2 = Zcam1[2][obs];
		d  = r*cos(b2);
	}
	if (irb ==1 ){
		r  =  Zcam2[0][obs];
		r2 =  pow(r,(double)2);
		b1 = Zcam2[1][obs];
		b2 = Zcam2[2][obs];
		d  = r*cos(b2);
	}

	//SENO E COSSENO DAS ORIENTACOES
	s = sin(part[irb][ip].t + b1);
	c = cos(part[irb][ip].t + b1);

	sb2=sin(b2);
	cb2=cos(b2);

	//COORDENADAS ESTIMADAS DOS MARCOS...
	xf(0,0)= part[irb][ip].x + d*c; //aq
	xf(1,0)= part[irb][ip].y + d*s; //aq
	xf(2,0)= r*sb2;
	part[irb][ip].xf[0][ass] = xf(0,0);
	part[irb][ip].xf[1][ass] = xf(1,0);
	part[irb][ip].xf[2][ass] = xf(2,0);

	//JACOBIANO ESTIMADO
	Gza(0,0)=      c*cb2;     Gza(0,1)= s*cb2;                   Gza(0,2)= sb2;
	Gza(1,0)=     -(s/d);     Gza(1,1)= c/d;                     Gza(1,2)= 0;
	Gza(2,0)= -sb2*cb2*c*d;   Gza(2,1)=-sb2*cb2*s*d;             Gza(2,2)= d*pow(cb2,(double)2);

	//invert(Gza,Gz); // ESTOU USANDO FÓRMULA

	Gz(0,0)=     c*cb2;     Gz(0,1)= -s*d;            Gz(0,2)= -(sb2*c)/(d*cb2);
	Gz(1,0)=     s*cb2;     Gz(1,1)=  c*d;            Gz(1,2)= -(sb2*s)/(d*cb2);
	Gz(2,0)=       sb2;     Gz(2,1)=    0;            Gz(2,2)=   1/d;



	GzR = prod(Gz,R);
	//COVARIANCIA ESTIMADA
	pf = prod(GzR,trans(Gz));
	part[irb][ip].pf[0][ass]=pf(0,0);
	part[irb][ip].pf[1][ass]=pf(0,1);
	part[irb][ip].pf[2][ass]=pf(0,2);
	part[irb][ip].pf[3][ass]=pf(1,0);
	part[irb][ip].pf[4][ass]=pf(1,1);
	part[irb][ip].pf[5][ass]=pf(1,2);
	part[irb][ip].pf[6][ass]=pf(2,0);
	part[irb][ip].pf[7][ass]=pf(2,1);
	part[irb][ip].pf[8][ass]=pf(2,2);

}
//ATUALIZAÇÃO DOS MARCO
void fastslams::AtzMarco (int irb, int ip,int obs){

	long double d,d2,r,r2,dx,dy,dz;
	boost::numeric::ublas::matrix<long double> Gm(3,3),pf(3,3),PGmt(3,3),Z(3,3),Zi(3,3),K(3,3),W(3,3),S(3,3),Si(3,3),W1(3,3);
	boost::numeric::ublas::matrix<long double> za(3,1),v(3,1),zp(3,1),xf(3,1);
	int flag;
	int ass;
	if (irb ==0 ){ass= Zcam1[3][obs];}
	if (irb ==1 ){ass= Zcam2[3][obs];}

	//COORDENADAS DO MARCO NO MAPA
	xf(0,0) = part[irb][ip].xf[0][ass];
	xf(1,0) = part[irb][ip].xf[1][ass];
	xf(2,0) = part[irb][ip].xf[2][ass];

	//MATRIZ DE COVARIÂNCIA - ESTIMAÇÃO DOS MARCOS...
	pf(0,0)=part[irb][ip].pf[0][ass];
	pf(0,1)=part[irb][ip].pf[1][ass];
	pf(0,2)=part[irb][ip].pf[2][ass];
	pf(1,0)=part[irb][ip].pf[3][ass];
	pf(1,1)=part[irb][ip].pf[4][ass];
	pf(1,2)=part[irb][ip].pf[5][ass];
	pf(2,0)=part[irb][ip].pf[6][ass];
	pf(2,1)=part[irb][ip].pf[7][ass];
	pf(2,2)=part[irb][ip].pf[8][ass];


	dx = xf(0,0) - part[irb][ip].x;
	dy = xf(1,0) - part[irb][ip].y;
	dz = xf(2,0);
	d2 = pow(dx,(double)2)  +  pow(dy,(double)2);
	d  = sqrt(d2);
	r2 = d2 + pow(dz,(double)2);
	r  = sqrt(r2);
//Aqui começa o Filtro de Kalman
	//JACOBIANO
	Gm(0,0) =  dx/r;                  Gm(0,1) = dy/r;             Gm(0,2) = dz/r;
	Gm(1,0) = -dy/d2;                 Gm(1,1) = dx/d2;            Gm(1,2) = 0;
	Gm(2,0) = - (dz*dx*d)/r2;         Gm(2,1) = -(dz*dy*d)/r2;    Gm(2,2) = pow(d,(double)3)/r2;

	//OBSERVAÇÃO PREDITA... Zp
	zp(0,0) = sqrt(r2) ;
	zp(1,0) = atan2 (dy,dx) - part[irb][ip].t;
	zp(2,0) = atan2 (dz,d);
	
	//V=Z-Zp

	if (irb==0){
		za(0,0)= Zcam1[0][obs];
		za(1,0)= Zcam1[1][obs];
		za(2,0)= Zcam1[2][obs];
	}
	if (irb==1){
		za(0,0)= Zcam2[0][obs];
		za(1,0)= Zcam2[1][obs];
		za(2,0)= Zcam2[2][obs];
	}

	v(0,0) = za(0,0)-zp(0,0);     v(1,0) = za(1,0)-zp(1,0);     v(2,0) = za(2,0)-zp(2,0);
	v(1,0) = pi_to_pi(v(1,0));
	v(2,0) = pi_to_pi(v(2,0));

	//COVARIÂNCIA DA INOVAÇÃO....Z
	PGmt = prod(pf,trans(Gm));
	Z=prod(Gm,PGmt) + R;
	/*
	//********************** IMPLEMENTAÇÃO NUMERICAMENTE ESTÁVEL (TIM BAYLEI)....
	S=(Z+trans(Z))*0.5;
	//Fatorização de Cholesky
	flag = cholesky_decompose(S);
	if (flag!=0) 
	{
    	ferro <<"ip: "<< ip <<  " => Erro cholesky_decompos \n";
	}
	else
	{
		//Atualização
		invert(S,Si);         // ou -> invert(Schol,ScholInv); //ZZinv
		W1=prod(PGmt,Si);      // K=W1=....Pf(G)^{T}ZZinv
		W=prod(W1,trans(Si));  // Pf(G)^{T}ZZinv ZZinv^{T}
		xf = xf + prod(W,v); //
		pf = pf-prod(W1,trans(W1));

		part[irb][ip].xf[0][ass] = xf(0,0);
		part[irb][ip].xf[1][ass] = xf(1,0);
		part[irb][ip].xf[2][ass] = xf(2,0);

		part[irb][ip].pf[0][ass] = pf(0,0);
		part[irb][ip].pf[1][ass] = pf(0,1);
		part[irb][ip].pf[2][ass] = pf(0,2);
		part[irb][ip].pf[3][ass] = pf(1,0);
		part[irb][ip].pf[4][ass] = pf(1,1);
		part[irb][ip].pf[5][ass] = pf(1,2);
		part[irb][ip].pf[6][ass] = pf(2,0);
		part[irb][ip].pf[7][ass] = pf(2,1);
		part[irb][ip].pf[8][ass] = pf(2,2);
	}
	*/
	if (invert(Z,Zi)==false){ferro<<"erro atz\n"; cout<<"erro atz\n";}
	else
	{
	//GANHO DE KALMAN K
	K=prod(PGmt,Zi);
	//ATUALIZACAO...
	xf = xf + prod(K,v);
	W = prod(K,Gm);
	pf = pf-prod(W,pf);

	part[irb][ip].xf[0][ass] = xf(0,0);
	part[irb][ip].xf[1][ass] = xf(1,0);
	part[irb][ip].xf[2][ass] = xf(2,0);

	part[irb][ip].pf[0][ass] = pf(0,0);
	part[irb][ip].pf[1][ass] = pf(0,1);
	part[irb][ip].pf[2][ass] = pf(0,2);
	part[irb][ip].pf[3][ass] = pf(1,0);
	part[irb][ip].pf[4][ass] = pf(1,1);
	part[irb][ip].pf[5][ass] = pf(1,2);
	part[irb][ip].pf[6][ass] = pf(2,0);
	part[irb][ip].pf[7][ass] = pf(2,1);
	part[irb][ip].pf[8][ass] = pf(2,2);


	}
	
}


void fastslams::PesoAss(int irb, int ip, int obs,  double * wmax){
	boost::numeric::ublas::matrix<long double> vnum(1,1);
	long double num,den;
	long double p,d2,r,r2,dx,dy,dz,d,det;
	boost::numeric::ublas::matrix<long double> pf(3,3), GmPf(3,3), ZZi(3,3);
	//JACOBIANO...
	boost::numeric::ublas::matrix<long double> Gm(3,3);
	boost::numeric::ublas::matrix<long double> ZZ(3,3);
	boost::numeric::ublas::matrix<long double> za(3,1), v(3,1),zp(3,1);
	boost::numeric::ublas::matrix<long double> vtZZi(1,3);
	long double nis;	// [ v'*ZZi*v ]
	
	int ift;

	if (irb==0){ift = Zcam1[3][obs];}
	if (irb==1){ift = Zcam2[3][obs];}
	//****************************************/
	//           PESO DA PARTÍCULA           //
	//****************************************/

	dx = part[irb][ip].xf[0][ift] - part[irb][ip].x;
	dy = part[irb][ip].xf[1][ift] - part[irb][ip].y;
	dz = part[irb][ip].xf[2][ift];
	d2 = pow(dx,(double) 2)  +  pow(dy,(double)2);
	d  = sqrt(d2);
	r2 = d2 + pow(dz,(double)2);
	r  = sqrt(r2);
		
	//OBSERVAÇÃO PREDITA...Zp
	zp(0,0) = r ;
	zp(1,0) = atan2 (dy,dx) - part[irb][ip].t;
	zp(2,0) = atan2 (dz,d);

	//JACOBIANO
		Gm(0,0) =    dx/r;                      Gm(0,1) = dy/r;              Gm(0,2) = dz/r;
		Gm(1,0) =  -dy/d2;                      Gm(1,1) = dx/d2;             Gm(1,2) = 0;
		Gm(2,0) = -(dz*dx*d)/r2;                Gm(2,1) = - (dz*dy*d)/r2;    Gm(2,2) = pow(d,(double)3)/r2;

	//MATRIZ DE COVARIÂNCIA - ESTIMAÇÃO DOS MARCOS...
	pf(0,0)=part[irb][ip].pf[0][ift];
	pf(0,1)=part[irb][ip].pf[1][ift];
	pf(0,2)=part[irb][ip].pf[2][ift];
	pf(1,0)=part[irb][ip].pf[3][ift];
	pf(1,1)=part[irb][ip].pf[4][ift];
	pf(1,2)=part[irb][ip].pf[5][ift];
	pf(2,0)=part[irb][ip].pf[6][ift];
	pf(2,1)=part[irb][ip].pf[7][ift];
	pf(2,2)=part[irb][ip].pf[8][ift];
	//COVARIÂNCIA DA INOVAÇÃO....ZZ
	GmPf = prod(Gm, pf);
	ZZ=prod(GmPf,trans(Gm))+R;
	//COVARIÂNCIA DA INOVACAO INVERSA ...ZZ^{-1}
	if (invert(ZZ,ZZi)==false){cout<<"erro ass d\n";};
	//V= Z - Zp
	if (irb==0){
		za(0,0)=Zcam1[0][obs];
		za(1,0)=Zcam1[1][obs];
		za(2,0)=Zcam1[2][obs];
	}
	if (irb==1){
		za(0,0)=Zcam2[0][obs];
		za(1,0)=Zcam2[1][obs];
		za(2,0)=Zcam2[2][obs];
	}

	v(0,0)=za(0,0)-zp(0,0);  v(1,0)=za(1,0)-zp(1,0);        v(2,0)=za(2,0)-zp(2,0);
	v(1,0)=pi_to_pi(v(1,0));
	v(2,0)=pi_to_pi(v(2,0));

	//v'*ZZi*v
	vtZZi = prod(trans(v),ZZi);
	vnum  = prod(vtZZi,v);
	//NIS...
	nis = vnum(0,0);
	det = determinant(ZZ);
	if (det==0){
			cout<<"erro det\n";
			p=10;
			part[irb][ip].p[ift] = p;
	}
	else
	{
			//p=num/den
			num = exp(-0.5* nis );
			den = sqrt(fabs(2*pi*det));
			p   = num/den;
			part[irb][ip].p[ift] = p;
	}
	
	*wmax = part[irb][ip].p[ift];
	if(*wmax==*wmax){}else{*wmax=0;}
	if((*wmax<=P0)||(*wmax>1))
	{
		ferro<<"Ip: " << ip <<" jbest: " << ift << " wmax: "<< *wmax <<"\n";
		*wmax=0;
	}
	else
	{
			cout<<"irb:" << irb << "=> "<< ip << " " <<  *wmax<< part[irb][ip].x << " " <<  part[irb][ip].y << " "<< 180*part[irb][ip].t/pi<< "\n"<<"\n";
			if(irb==0){nreobs1++; reobs1[dt]=reobs1[dt]+1;}
			if(irb==1){nreobs2++; reobs2[dt]=reobs2[dt]+1;}
	}
}

int BestPart(int irb){
	int idc=0;
	long double wmax=-10;
	for (int i=0; i<NP; i++)
	{
		long double ff = part[irb][i].w ;
		if (ff> wmax)
		{
			wmax = ff;
			idc = i;
		}
	}
	long double xfx,xfy,xfz;
	fmpart << "P: " << irb << " => " << idc  << " =>  w: " << part[irb][idc].w;
		for (int j = 0; j <= NF; j++)
		{
			xfx = part[irb][idc].xf[0][j];
			xfy = part[irb][idc].xf[1][j];
			xfz = part[irb][idc].xf[2][j];
			fmpart << "  ("<< xfx << " , " << xfy << "," << xfz <<") ";
		}
		fmpart << "\n";

	return idc;
}



void fastslams::ReamostreC(int irb ){
	long double w[NP];		//Vetor com pesos das partículas
	long double select[NP];
	long double Neff;		//Número de partículas efetivas (medida da variância do peso)
	int    manter[NP];
	long double di[NP];
	long double k,q;
	int    ctr;
	long double NEFFECTIVE;
	long double np;
	np = NP;
	NEFFECTIVE = 0.75*NP;
	long double ws=0;		//Soma do elementos do vetor w
	long double ws2=0;		//Soma do quadrado dos elementos do vetor w
	//Soma dos Pesos
	for(int i=0; i<NP;i++)
	{
		w[i]=part[irb][i].w;
		ws =  ws + w[i];
	}
	//Pesos Normalizados
	for(int i=0; i<NP;i++){
		w[i]=w[i]/ws;	
		//ISTO ESTAVA CAUSANDO NAN
		//part[i].w=(part[i].w)/ws;
	}
	//********************INICIO STRATIFIED RESAMPLE*********************//
	ws=0;
	for(int i=0; i<NP;i++)
	{
		ws =  ws + w[i];
	}
	for(int i=0; i<NP;i++)
	{
		w[i]=w[i]/ws;
	}
	ws2=0;
	for(int i=0; i<NP;i++)
	{
		ws2=ws2+pow(w[i],2);
		manter[i]=-1;                 //Inicializando
	}
	Neff = 1/ws2;
	//********************Início Stratified Random***************************
	
	k =  1/np;
	//Intervalos Determinísticos
	q = k/2;
	for(int i =0; i<NP;i++)
	{
		di[i] = q;
		q = q + k;
	}
	for(int r=0;r<NP;r++)
	{
		select[r]=di[r]+k*uniform()-k/2;
	}
	//********************Fim Stratified Random***************************
	//Soma Cumulativa
	for(int i=1; i<NP;i++)
	{
		w[i]=w[i]+w[i-1];
	}
	ctr=0;
	for(int i=0; i < NP; i++)
	{
		while((ctr < NP) && (select[ctr] < w[i]))
		{
			manter[ctr] = i;
			ctr = ctr + 1;
		}
	}
	//********************FIM STRATIFIED RESAMPLE*********************//
	if (Neff < NEFFECTIVE)
	{
		std::cout <<"\n REAMOSTRANDO**************************************\n"<<std::endl;

		for (int i=0; i<NP; i++)
		{
			//Robô 01
			part[irb][i].x = part[irb][manter[i]].x;
			part[irb][i].y = part[irb][manter[i]].y;
			part[irb][i].t = part[irb][manter[i]].t;
			//Número de Marcos
			part[irb][i].n = part[irb][manter[i]].n;
			for (int j =0; j <= NF; j++)
			{
				part[irb][i].xf[0][j]= part[irb][manter[i]].xf[0][j];
				part[irb][i].xf[1][j]= part[irb][manter[i]].xf[1][j];
				part[irb][i].xf[2][j]= part[irb][manter[i]].xf[2][j];

				part[irb][i].pf[0][j]= part[irb][manter[i]].pf[0][j];
				part[irb][i].pf[1][j]= part[irb][manter[i]].pf[1][j];
				part[irb][i].pf[2][j]= part[irb][manter[i]].pf[2][j];
				part[irb][i].pf[3][j]= part[irb][manter[i]].pf[3][j];
				part[irb][i].pf[4][j]= part[irb][manter[i]].pf[4][j];
				part[irb][i].pf[5][j]= part[irb][manter[i]].pf[5][j];
				part[irb][i].pf[6][j]= part[irb][manter[i]].pf[6][j];
				part[irb][i].pf[7][j]= part[irb][manter[i]].pf[7][j];
				part[irb][i].pf[8][j]= part[irb][manter[i]].pf[8][j];
			}
			part[irb][i].w = 1/np;
		}//Fim for
	}//Fim if 
}

int  main(void)
{
	r00 = new R2D2(6665);
	r11 = new R2D2(6666);
	int	Dst=1;
	int atz_peso1=0,atz_peso2=0;;
	int obs,dec;
	double wmax;
	double dscx1,dscy1,dscx2,dscy2;
	double xpreva1,ypreva1,tpreva1,xpreva2,ypreva2,tpreva2;
	int  TAMCAMINHO=0;
	int idc=0,idc1=0,idc2=0;
	OpenFiles();
	InitVar();
	IniciaGrid();IniciaMap();
	fastslams f(0);
	fastslams g(1);
	//POSES INICIAIS...
	pose p00 = r00->GetPosition();
	f.xv=p00.xpos;f.yv=p00.ypos;f.tv=pi_to_pi(p00.yaw);
	double yaw1a;yaw1=f.tv;
	xodom[f.irb]=f.xv;yodom[f.irb]=f.yv;todom[f.irb]=f.tv;
	pose p11 = r11->GetPosition();
	g.xv=p11.xpos;g.yv=p11.ypos;g.tv=pi_to_pi(p11.yaw);
	xodom[g.irb]=g.xv;yodom[g.irb]=g.yv;todom[g.irb]=g.tv;
	double yaw2a; yaw2=g.tv;
	cout<<"POSICOES INICIAIS: ["<<f.xv<<","<<f.yv<<"] - "<<"["<<g.xv<<","<<g.yv<<"]"<<"\n";
	f.IniciaPart(0);
	g.IniciaPart(1);
	xpreva1=f.xv;ypreva1=f.yv;tpreva1=f.tv;
	xpreva2=g.xv;ypreva2=g.yv;tpreva2=g.tv;
	//PRIMEIRA OBSERVACAO
	ObsLaser(xpreva1,ypreva1,tpreva1,xpreva2,ypreva2,tpreva2);
	Dst=DstGrid(f,g);
	while ((Dst == 1)&&(cl<4000))
	{
		cout<<"DESTINOS =>  "<< "1: "<<"["<< Path[0].x[Path[0].n]<<","<< Path[0].y[Path[0].n] << "] " <<  "2: " <<"["<< Path[1].x[Path[1].n] << "," << Path[1].y[Path[1].n] << "]:" << "\n";
		int i=1;//ONDE QUER IR

		//PEGO O MENOR DOS CAMINHOS
		if (Path[0].n>=Path[1].n)
		{
			TAMCAMINHO=Path[1].n;
		}
		else
		{
			TAMCAMINHO=Path[0].n;
		}
		//if (TAMCAMINHO>ESCALA+1){TAMCAMINHO=TAMCAMINHO-(ESCALA+1);}
		cout<<" => TAM CAM: " << TAMCAMINHO <<"\n";
		while (i<=TAMCAMINHO)
		{
			dt++;
			//*****************************FASTSLAM**********************************//
			double xdstp1,ydstp1,xdstp2,ydstp2;
			if (i<=TAMCAMINHO)
			{
				dt1++;

				yaw1a=yaw1;
				f.CDir(i,&dscx1,&dscy1,&yaw1,TAMCAMINHO);
				double giro1= yaw1-yaw1a;
				cout<<"=>Giro1: "<< 180*giro1/pi << "=>";
				//DESTINO PARCIAL
				xdstp1 = (Path[0].x[i]-0.5-TAM/2)/ESCALA;
				ydstp1 = (Path[0].y[i]-0.5-TAM/2)/ESCALA;

				f.PosV(xdstp1,ydstp1,yaw1);
				cout<< "REAL1:"<<xdstp1<< ","<<ydstp1<<","<< 180*yaw1/pi;
				fresult<< "REAL1:"<<xdstp1<< ","<<ydstp1<<","<< 180*yaw1/pi;

				yaw2a=yaw2;
				g.CDir(i,&dscx2,&dscy2,&yaw2,TAMCAMINHO);
				double giro2=yaw2-yaw2a;
				cout<<" Giro2: "<< 180*giro2/pi << "=>";
				//DESTINO PARCIAL
				xdstp2 = (Path[1].x[i]-0.5-TAM/2)/ESCALA;
				ydstp2 = (Path[1].y[i]-0.5-TAM/2)/ESCALA;

				g.PosV(xdstp2,ydstp2,yaw2);
				cout<< "   REAL2:"<<xdstp2<< ","<<ydstp2<<","<< 180*yaw2/pi<<"\n";
				fresult<< "REAL2:"<<xdstp2<< ","<<ydstp2<<","<< 180*yaw2/pi<<"\n";

				r00->Ir(xdstp1,ydstp1,yaw1);
				r11->Ir(xdstp2,ydstp2,yaw2);

				if ((yaw1==yaw1a)&&(yaw2==yaw2a)) {sleep(8);} else {sleep(12);}


				for(int ip=0;ip<NP;ip++)
				{
					f.PosP(0,ip,dscx1,dscy1,yaw1,giro1);
				}
				f.PosOdom(dscx1,dscy1,yaw1,giro1);

				for(int ip=0;ip<NP;ip++)
				{
					g.PosP(1,ip,dscx2,dscy2,yaw2,giro2);
				}
				g.PosOdom(dscx2,dscy2,yaw2,giro2);
			}
			if (dt%dtobs==0)
			{
				nrobs++;

				ObsCameraf(f); ObsCamerag(g);
				for(int ip =0; ip < NP; ip++)
				{
					long double wp=1;
					int flag1=0;
					for (int io = 0; io < tagCam1.size(); io++)
					{	
						obs = (int) tagCam1.at(io);
						dec = (int) Zcam1[4][obs];
						if (dec==1)
						{
							f.AdicMarco (0, ip , obs );
						}
						else 
						if (dec==0)
						{
							f.PesoAss(0, ip, obs, &wmax);
							//DESTE JEITO FUNCIONA!!!
							if (wmax!=0)
							{
								f.AtzMarco (0, ip, obs );
								wp=wp*wmax;
								if(flag1!=0){wp=sqrt(wp);}
								if (wp==wp){}else{wp=0;}
								part[0][ip].w = wp;
								flag1++;
								atz_peso1=1;
							}
						}//Fim
					}//Fim

					wp=1;
					int flag2=0;
					for (int io = 0; io < tagCam2.size(); io++)
					{	
						obs = tagCam2.at(io);
						dec = (int) Zcam2[4][obs];
						if (dec==1)
						{
							g.AdicMarco (1, ip , obs );
						}
						else 
						if (dec==0)
						{
							g.PesoAss(1, ip, obs, &wmax);
							//DESTE JEITO FUNCIONA!!!
							if (wmax!=0)
							{
								g.AtzMarco (1, ip ,obs );
								wp=wp*wmax;
								if(flag2!=0){wp=sqrt(wp);}
								if (wp==wp){}else{wp=0;}
								part[1][ip].w = wp;
								flag2++;
								atz_peso2=1;
							}
						}//Fim
					}//Fim
				}//Fim For ip

			}//Fim

			if (atz_peso1==1)
			{
				idc1 = BestPart(0);
			}

			if (atz_peso2==1)
			{
				idc2 = BestPart(1);
			}

			atz_peso1=0;
			atz_peso2=0;
			cout<< " PART r0 => " << idc1 << " =>"<< part[0][idc1].w << " =>  " << part[0][idc1].x<<","<<part[0][idc1].y<<","<<180*(part[0][idc1].t)/pi <<"=>";
			cout<< " PART r1 => " << idc2 << " =>"<< part[1][idc2].w << " =>  " << part[1][idc2].x<<","<<part[1][idc2].y<<","<<180*(part[1][idc2].t)/pi <<"\n";

			fresult << " PART r0 => " << idc1 << " =>"<< part[0][idc1].w << " =>  " << part[0][idc1].x<<","<<part[0][idc1].y<<","<<180*(part[0][idc1].t)/pi <<"=>";
			fresult << " PART r1 => " << idc2 << " =>"<< part[1][idc2].w << " =>  " << part[1][idc2].x<<","<<part[1][idc2].y<<","<<180*(part[1][idc2].t)/pi <<"\n";

			sleep(5);
			ObsLaser(part[0][idc1].x,part[0][idc1].y,part[0][idc1].t,part[1][idc2].x,part[1][idc2].y,part[1][idc2].t);


			//VEICULO 01
			//ERRO SLAM
			e1x = part[0][idc1].x-f.xv;
			e1y = part[0][idc1].y-f.yv;
			e1t = pi_to_pi(part[0][idc1].t-f.tv);

			e1x2= e1x2+pow(e1x,(double)2);
			e1y2= e1y2+pow(e1y,(double)2);
			e1t2= e1t2+pow(e1t,(double)2);

			//ERRO ODOM
			e1ox = xodom[0]-f.xv;
			e1oy = yodom[0]-f.yv;
			e1ot = pi_to_pi(todom[0]-f.tv);

			e1ox2= e1ox2+pow(e1ox,(double)2);
			e1oy2= e1oy2+pow(e1oy,(double)2);
			e1ot2= e1ot2+pow(e1ot,(double)2);

			f1x << e1x << "\n"; f1y << e1y << "\n"; f1t << e1t << "\n";
			f1xod << xodom[0] << "\n"; f1yod << yodom[0] << "\n"; f1tod << todom[0] << "\n";
			f1ex << part[0][idc1].x << "\n"; f1ey << part[0][idc1].y << "\n"; f1et << part[0][idc1].t << "\n";
			f1px << f.xv << "\n"; f1py << f.yv << "\n"; f1pt << f.tv << "\n";

			//VEICULO 02
			//ERRO SLAM
			e2x = part[1][idc2].x-g.xv;
			e2y = part[1][idc2].y-g.yv;
			e2t = pi_to_pi(part[1][idc2].t-g.tv);

			e2x2= e2x2+pow(e2x,(double)2);
			e2y2= e2y2+pow(e2y,(double)2);
			e2t2= e2t2+pow(e2t,(double)2);

			//ERRO ODOMETRIA
			e2ox = xodom[1]-g.xv;
			e2oy = yodom[1]-g.yv;
			e2ot = pi_to_pi(todom[1]-g.tv);

			e2ox2= e2ox2+pow(e2ox,(double)2);
			e2oy2= e2oy2+pow(e2oy,(double)2);
			e2ot2= e2ot2+pow(e2ot,(double)2);


			f2x << e2x << "\n"; f2y << e2y << "\n"; f2t << e2t << "\n";
			f2xod << xodom[1] << "\n"; f2yod << yodom[1] << "\n"; f2tod << todom[1] << "\n";

			f2ex << part[1][idc2].x << "\n"; f2ey << part[1][idc2].y << "\n"; f2et << part[1][idc2].t << "\n";
			f2px << g.xv << "\n"; f2py << g.yv << "\n"; f2pt << g.tv << "\n";


			fcam1 << "Path[0].x[" << dt << "]" << "=" << Path[0].x[i] <<";" << "Path[0].y[" << dt << "]" << "=" << Path[0].y[i] <<"; \n";
			fcam2 << "Path[1].x[" << dt << "]" << "=" << Path[1].x[i] <<";" << "Path[1].y[" << dt << "]" << "=" << Path[1].y[i] <<"; \n";
			//REAMOSTRAGEM
			f.ReamostreC(0);
			g.ReamostreC(1);
			cout<< " PART REr0 => " << idc1 << " =>"<< part[0][idc1].w << " =>  " << part[0][idc1].x<<","<<part[0][idc1].y<<","<<180*(part[0][idc1].t)/pi <<"=>";
			cout<< " PART REr1 => " << idc2 << " =>"<< part[1][idc2].w << " =>  " << part[1][idc2].x<<","<<part[1][idc2].y<<","<<180*(part[1][idc2].t)/pi <<"\n";


			//VER SE REGIAO DO DESTINO FOI VERIFICADA...
			if((selecao(Path[0].x[Path[0].n],Path[0].y[Path[0].n])==0)&&(selecao(Path[1].x[Path[1].n],Path[1].y[Path[1].n])==0))
			{
				cout<<"\n DESTINOS VERIFICADOS ...\n";
				break;
			}
			int bloq1=0;
			for(int t=i;t<=Path[0].n;t++)
			{
				if (Grid.n[Path[0].x[t]][Path[0].y[t]]==9)
				{
					bloq1=1;
				}
			}
			int bloq2=0;
			for(int t=i;t<=Path[1].n;t++)
			{
				if (Grid.n[Path[1].x[t]][Path[1].y[t]]==9)
				{
					bloq2=1;
				}
			}
			if ((bloq1==1)||(bloq2==1))
			{
				cout<<"\n TRAJETORIA BLOQUEADA ...\n";break;
			}
			i++;
		}//Fim while i
		Dst = DstGrid(f,g);
	}//Fim While Ndst
	//veiculo 01
	f1x << "Erro RMS X: " << sqrt((double) (e1x2/dt1)) << "\n";
	f1y << "Erro RMS Y: " << sqrt((double) (e1y2/dt1)) << "\n";
	f1t << "Erro RMS T: " << sqrt((double) (e1t2/dt1)) << "\n";

	f1x << "Erro RMS ODOM X: " << sqrt((double) (e1ox2/dt1)) << "\n";
	f1y << "Erro RMS ODOM Y: " << sqrt((double) (e1oy2/dt1)) << "\n";
	f1t << "Erro RMS ODOM T: " << sqrt((double) (e1ot2/dt1)) << "\n";

	//veiculo 02
	f2x << "Erro RMS X: " << sqrt((double) (e2x2/dt1)) << "\n";
	f2y << "Erro RMS Y: " << sqrt((double) (e2y2/dt1)) << "\n";
	f2t << "Erro RMS T: " << sqrt((double) (e2t2/dt1)) << "\n";

	f2x << "Erro RMS ODOM X: " << sqrt((double) (e2ox2/dt1)) << "\n";
	f2y << "Erro RMS ODOM Y: " << sqrt((double) (e2oy2/dt1)) << "\n";
	f2t << "Erro RMS ODOM T: " << sqrt((double) (e2ot2/dt1)) << "\n";

	for (int j = 0; j <= NF; j++)
	{
		double xfx = part[0][idc1].xf[0][j];
		double xfy = part[0][idc1].xf[1][j];
		double xfz = part[0][idc1].xf[2][j];
		if ((xfx!=0)&&(xfy!=0)&&(xfz!=0)){
			fmapax << xfx << "\n";
			fmapay << xfy << "\n";
			fmapaz << xfz << "\n";
		}
	}

	for (int j = 0; j <= NF; j++)
	{
		double xfx = part[1][idc2].xf[0][j];
		double xfy = part[1][idc2].xf[1][j];
		double xfz = part[1][idc2].xf[2][j];
		if ((xfx!=0)&&(xfy!=0)&&(xfz!=0)){
			fmapax << xfx << "\n";
			fmapay << xfy << "\n";
			fmapaz << xfz << "\n";
		}
	}

	ImpRes();
	CloseFiles();
	std::cout<<"FIM\n";

	return 0;
}


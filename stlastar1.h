#include "stdafx.h"
#include "stlastar.h" // See header for copyright and usage information
#include <iostream>
#include <math.h>
#include <playerc++.h>
#include "R2D2.h"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;


#define FROMAX 1000   // Fronteira
#define CAMMAX  400   // Path (caminho)


#define NP  4000    					//N�mero M�ximo de Part�culas
#define NF    50						//N�mero m�ximo de Features
#define TAM  300
#define MAP_SIZE  5*TAM

int ESCALA=2;
int REGIAOF = 1;
int REGIAOP = 2;

double beta= (double)1/(2*ESCALA);

int ALASER = 5;
int MAP_SCALE = 5*ESCALA;

int MAP[MAP_SIZE][MAP_SIZE];



struct {
	double w;		    //peso
	int    n;			//n�mero de marcos detectados
	double x,y,t;		//coordenadas do ve�culo
	double xf [3][NF];	//Coordenadas dos marcos no mapa
	double pf [9][NF];	//Covari�ncia dos marcos no mapa
	double p[NF];       //Potencial Associa��o de dados ==> quando encontra ass => "peso = p"
}part[2][NP];




double xodom[2],yodom[2],todom[2];

//CELULAS Fronteiras
struct {
	int    n;
	int x[FROMAX];
	int y[FROMAX];
	double gn1[FROMAX];  // Ganho de navega��o
	double gn2[FROMAX];  // Ganho de navega��o
	double up[FROMAX];  // Utilidade auxiliar
	double u1 [FROMAX];  // Utilidade
	double u2 [FROMAX];  // Utilidade
	double uf [FROMAX];  // Utilidade
	
}Cfr;

//Path (caminho)
struct {
	int    n;
	int x[CAMMAX];
	int y[CAMMAX];
}Path[2];

//Grid
struct {
	int    occ[TAM][TAM]; // Ocupa��o Num�rica
	int    n[TAM][TAM]; // Ocupa��o Num�rica Normalizada
	double p[TAM][TAM]; // Probabilidade de Ocupa��o
	double cn1[TAM][TAM]; // Custo para alcan�ar as c�lulas do Grid
	double cn2[TAM][TAM]; // Custo para alcan�ar as c�lulas do Grid
} Grid;

int GridBP[TAM][TAM];


struct
{
	int    occ;
	int    alt;
}GAUX0[TAM][TAM];

struct
{
	int    occ;
	int    alt;
}GAUX1[TAM][TAM];

struct
{
	int    occ;
	int    alt;
}GBPAUX0[TAM][TAM];

struct
{
	int    occ;
	int    alt;
}GBPAUX1[TAM][TAM];

struct
{
	int    occ;
	int    alt;
}MAPAUX0[MAP_SIZE][MAP_SIZE];

struct
{
	int    occ;
	int    alt;
}MAPAUX1[MAP_SIZE][MAP_SIZE];




void IniciaGrid (void){
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j< TAM;j=j+1)
		{
			Grid.occ[i][j]    = 0.0;
			Grid.n[i][j]      = 9.0;
			Grid.p[i][j]      = 1;
			Grid.cn1[i][j]    = 1000;
			Grid.cn2[i][j]    = 1000;
			GridBP[i][j]=0;
			GAUX0[i][j].occ=0;
			GAUX0[i][j].alt=0;
			GAUX1[i][j].occ=0;
			GAUX1[i][j].alt=0;
			GBPAUX0[i][j].occ=0;
			GBPAUX0[i][j].alt=0;
			GBPAUX1[i][j].occ=0;
			GBPAUX1[i][j].alt=0;
		} 
	}
}

void IniciaMap(void)
{
	for (int i = 0; i < MAP_SIZE; i++)
	{
		for (int j= 0; j < MAP_SIZE; j++)
		{
			MAP[i][j] = 0;
			MAPAUX1[i][j].occ=0;
			MAPAUX1[i][j].alt=0;
		}
	}
}
double maximo(double x, double y){

	if (x>=y){return x;}else{return y;}
}
int sinal (double x, double y){
	if ((x>0)&&(y<0))return -1;
	if ((x<0)&&(y>0))return -1;
	return 1;
}
void  AtzGrid(double *laser0, player_point_2d_t *laserPoints0, double *orientacao0, position pos0,double *laser1, player_point_2d_t *laserPoints1, double *orientacao1, position pos1)  {
	//LIVRO ROBOTICA PROBABILISTICA
	double x  = pos0.xpos;
	double y  = pos0.ypos;
	double xi;
	double yi;
	double r;
	double fi;
	double alfa =beta;
	double min;
	double  pi  = 3.14159265359;
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
		{
			xi = (i-0.5-TAM/2)/ESCALA;
			yi = (j-0.5-TAM/2)/ESCALA;
			r  =  sqrt(pow(xi-x,2)+pow(yi-y,2));
			fi =  atan2(yi-y,xi-x)-pos0.yaw;
			if ((xi-x)*(cos(pos0.yaw)) + (yi-y)*(sin(pos0.yaw)) > 0 )
			{
				double ref=1000;
				int k=0;
				for (int m = 0; m < 361; m=m+1)
				{
					if (fabs(fi-orientacao0[m])<ref)
					{
						ref=fabs(fi-orientacao0[m]);
						k=m;
					}
				}
				if (laser0[k]+alfa>= ALASER){min=ALASER;}else{min=laser0[k]+alfa;}
				if((r>min)||(fabs(fi-orientacao0[k])>pi/2))
				{

				}
				else
				{
					if (r<= laser0[k])
					{
						if (Grid.occ[i][j]<=0){Grid.occ[i][j]= -1;}
					}
					if ((laser0[k]<ALASER)&&(fabs(r-laser0[k])<alfa))
					{
						Grid.occ[i][j]= 1;
					}
				}
			}

		}

	}
	x = pos1.xpos;
	y = pos1.ypos;
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
			{
				xi =  (i-0.5-TAM/2)/ESCALA;
				yi =  (j-0.5-TAM/2)/ESCALA;
				r  =  sqrt(pow(xi-x,2)+pow(yi-y,2));
				fi =  atan2(yi-y,xi-x)-pos1.yaw;
				if ((xi-x)*(cos(pos1.yaw)) + (yi-y)*(sin(pos1.yaw)) > 0 )
				{
					double ref=1000;
					int k=0;
					for (int m = 0; m < 361; m=m+1)
					{
						if (fabs(fi-orientacao1[m])<ref)
						{
							ref=fabs(fi-orientacao1[m]);
							k=m;
						}
					}
					if (laser1[k]+alfa>= ALASER){min=ALASER;}else{min=laser1[k]+alfa;}
					if((r>min)||(fabs(fi-orientacao1[k])>pi/2))
					{

					}
					else
					{
						if (r<= laser1[k])
						{
							if (Grid.occ[i][j]<=0){Grid.occ[i][j]= -1;}
						}
						if ((laser1[k]<ALASER)&&(fabs(r-laser1[k])<alfa))
						{
							Grid.occ[i][j]= 1;
						}
					}
				}

		}
	}

	//DESTINOS E PLANEJAMENTO TRAJETORIA
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
		{
				//CELULAS LIVRES....
				if (Grid.occ[i][j] < 0 )
				{
					Grid.n[i][j]=0.0;
					Grid.p[i][j]=0.1;
				}
		}
	}
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
		{
			//CELULAS NO ENTORNO DE OCUṔADAS E DESCONHECIDAS...
			if (Grid.occ[i][j] >= 0)
			{
				Grid.n[i][j]=9.0;
				Grid.p[i][j]=1.0;
				for(int m=i-REGIAOP;m<=i+REGIAOP;m++)
				{
					for(int n=j-REGIAOP;n<=j+REGIAOP;n++)
					{
						Grid.n[m][n]=5.0;
					}
				}
			}
		}
	}
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
		{
			//CELULAS OCUPADAS E DESCONHECIDAS
			if (Grid.occ[i][j] >= 0)
			{
				Grid.n[i][j]=9.0;
				Grid.p[i][j]=1.0;
			}
		}
	}

}

void  AtzGridBP(double *laser0, player_point_2d_t *laserPoints0, double *orientacao0, double *laser1, player_point_2d_t *laserPoints1, double *orientacao1,double xpos0, double ypos0, double yaw0, double xpos1, double ypos1, double yaw1)  {
	//LIVRO ROBOTICA PROBABILISTICA
	double x = xpos0;
	double y = ypos0;
	double xi;
	double yi;
	double r;
	double fi;
	double alfa =beta;
	double min;
	double  pi  = 3.14159265359;
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
		{
			xi =  (i-0.5-TAM/2)/ESCALA;
			yi =  (j-0.5-TAM/2)/ESCALA;
			r  =  sqrt(pow(xi-x,2)+pow(yi-y,2));
			fi =  atan2(yi-y,xi-x)-yaw0;
			if ( (xi-x)*cos(yaw0) + (yi-y)*sin(yaw0) > 0 )
			{
				double ref=1000;
				int k=0;
				for (int m = 0; m < 361; m=m+1)
				{
					if (fabs(fi-orientacao0[m])<ref)
					{
						ref=fabs(fi-orientacao0[m]);
						k=m;
					}
				}
				if (laser0[k]+alfa>= ALASER){min=ALASER;}else{min=laser0[k]+alfa;}
				if((r>min)||(fabs(fi-orientacao0[k])>pi/2))
				{

				}
				else
				{

					if (r<= laser0[k])
					{
						if (GridBP[i][j]<=0){GridBP[i][j]= -1;}
					}

					if ((laser0[k]<ALASER)&&(fabs(r-laser0[k])<alfa))
					{
						GridBP[i][j]= 1;
					}

				}
			}

		}
	}
	x = xpos1;
	y = ypos1;
	for(int i=0;i<TAM;i=i+1)
	{
			for(int j=0;j<TAM;j=j+1)
			{
				xi =  (i-0.5-TAM/2)/ESCALA;
				yi =  (j-0.5-TAM/2)/ESCALA;
				r  =  sqrt(pow(xi-x,2)+pow(yi-y,2));
				fi =  atan2(yi-y,xi-x)-yaw1;
				if ( (xi-x)*cos(yaw1) + (yi-y)*sin(yaw1) > 0 )
				{
					double ref=1000;
					int k=0;
					for (int m = 0; m < 361; m=m+1)
					{
						if (fabs(fi-orientacao1[m])<ref)
						{
							ref=fabs(fi-orientacao1[m]);
							k=m;
						}
					}
					if (laser1[k]+alfa>= ALASER){min=ALASER;}else{min=laser1[k]+alfa;}
					if((r>min)||(fabs(fi-orientacao1[k])>pi/2))
					{

					}
					else
					{

						if (r<= laser1[k])
						{
							if (GridBP[i][j]<=0){GridBP[i][j]= -1;}
						}

						if ((laser1[k]<ALASER)&&(fabs(r-laser1[k])<alfa))
						{
							GridBP[i][j]= 1;
						}

					}
				}

		}
	}

}

void  AtzMap(double *laser0, player_point_2d_t *laserPoints0, double *orientacao0, double *laser1, player_point_2d_t *laserPoints1, double *orientacao1,double xpos0, double ypos0, double yaw0, double xpos1, double ypos1, double yaw1) {
		//LIVRO ROBOTICA PROBABILISTICA
		double x = xpos0;
		double y = ypos0;
		double xi;
		double yi;
		double r;
		double fi;
		double alfa =beta;
		double min;
		double  pi  = 3.14159265359;
		for(int i=0;i<MAP_SIZE;i=i+1)
		{
			for(int j=0;j<MAP_SIZE;j=j+1)
			{
				xi =  (i+beta-MAP_SIZE/2)/MAP_SCALE;
				yi =  (j+beta-MAP_SIZE/2)/MAP_SCALE;
				r  =  sqrt(pow(xi-x,2)+pow(yi-y,2));
				fi =  atan2(yi-y,xi-x)-yaw0;
				if ( (xi-x)*cos(yaw0) + (yi-y)*sin(yaw0) > 0 )
				{
					double ref=1000;
					int k=0;
					for (int m = 0; m < 361; m=m+1)
					{
						if (fabs(fi-orientacao0[m])<ref)
						{
							ref=fabs(fi-orientacao0[m]);
							k=m;
						}
					}
					if (laser0[k]+alfa>= ALASER){min=ALASER;}else{min=laser0[k]+alfa;}
					if((r>min)||(fabs(fi-orientacao0[k])>pi/2))
					{

					}
					else
					{

						if (r<= laser0[k])
						{
							if (MAP[i][j]<=0){MAP[i][j]= -1;}
						}
						if ((laser0[k]<ALASER)&&(fabs(r-laser0[k])<alfa))
						{
							MAP[i][j]=1; //MUDOU ARTIGO
						}

					}
				}

			}
		}
		//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
		x = xpos1;
		y = ypos1;
		for(int i=0;i<MAP_SIZE;i=i+1)
		{
			for(int j=0;j<MAP_SIZE;j=j+1)
			{
				xi =  (i+beta-MAP_SIZE/2)/MAP_SCALE;
				yi =  (j+beta-MAP_SIZE/2)/MAP_SCALE;
				r  =  sqrt(pow(xi-x,2)+pow(yi-y,2));
				fi =  atan2(yi-y,xi-x)-yaw1;
				if ( (xi-x)*cos(yaw1) + (yi-y)*sin(yaw1) > 0 )
				{
					double ref=1000;
					int k=0;
					for (int m = 0; m < 361; m=m+1)
					{
								if (fabs(fi-orientacao1[m])<ref)
								{
									ref=fabs(fi-orientacao1[m]);
									k=m;
								}
					}
					if (laser1[k]+alfa>= ALASER){min=ALASER;}else{min=laser1[k]+alfa;}
					if((r>min)||(fabs(fi-orientacao1[k])>pi/2))
					{

					}
					else
					{
						if (r<= laser1[k])
						{
							if (MAP[i][j]<=0){MAP[i][j]= -1;}
						}
						if ((laser1[k]<ALASER)&&(fabs(r-laser1[k])<alfa))
						{
							MAP[i][j]=1; //MUDOU ARTIGO
						}
					}
					}
			}
		}
}

void SaveGrid(void) {
	FILE *output;
	int aux=0;
	output = fopen("Grid.pgm", "wt");
	fprintf(output, "P2\n%d %d\n255\n", TAM, TAM);
	for (int i = 0; i < TAM; i++) {
		for (int j = 0; j < TAM; j++) {

			if (Grid.occ[i][j] < 0)   {aux=65500;}
			if (Grid.occ[i][j] > 0)   {aux=0;}
			if (Grid.occ[i][j]== 0)   {aux=32750;}
			fprintf(output, "%d ", (int) (aux) >> 8);
		}
	}
	fclose(output);
}

void SaveGridBP(void) {
	FILE *output;
	int aux=0;
	output = fopen("GridBP.pgm", "wt");
	fprintf(output, "P2\n%d %d\n255\n", TAM, TAM);
	for (int i = 0; i < TAM; i++) {
		for (int j = 0; j < TAM; j++) {

			if (GridBP[i][j] < 0)   {aux=65500;}
			if (GridBP[i][j] > 0)   {aux=0;}
			if (GridBP[i][j]== 0)   {aux=32750;}
			fprintf(output, "%d ", (int) (aux) >> 8);
		}
	}
	fclose(output);
}

void SaveMap(void) {
	FILE *output;
	int aux=0;
	output = fopen("Map.pgm", "wt");
	fprintf(output, "P2\n%d %d\n255\n", MAP_SIZE, MAP_SIZE);
	for (int i = 0; i < MAP_SIZE; i++) {
		for (int j = 0; j < MAP_SIZE; j++) {

			if (MAP[i][j] < 0)   {aux=65500;}
			if (MAP[i][j] > 0)   {aux=0;}
			if (MAP[i][j]== 0)   {aux=32750;}
			fprintf(output, "%d ", (int) (aux) >> 8);
		}
	}
	fclose(output);
}
void CustosNav(int iv1, int jv1,int iv2, int jv2){
	double  min,vd,ve,vs,vi,vsd,vse,vid,vie,raiz2,custo[8];

	//*********************************VEICULO 01**********************************************
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
		{
			if ((i==iv1)&&(j==jv1)){
				Grid.cn1[i][j]=0;
			}
			else{
				Grid.cn1[i][j]=1000;//Infinito
			}
		}
	}
	//VEICULO 01
	raiz2=sqrt(2.0);
	for(int loop=0;loop<TAM;loop++){
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j< TAM;j=j+1)
		{
			
			if((Grid.occ[i][j]>=0)||((i==iv1)&&(j==jv1)))
			{}
			else
			{
			//Zerando os custos
			vd=0; ve=0;vs=0;vi=0;vsd=0;vse=0;vid=0;vie=0;
			for(int k=0;k<8;k++){custo[k]=0;}
			//Parte Interna
			if((i!=0)&&(i!=TAM-1)&&(j!=0)&&(j!=TAM-1))
			{
				vd  = Grid.cn1[i+1][j]   + Grid.p[i+1][j];
				ve  = Grid.cn1[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn1[i][j-1]   + Grid.p[i][j-1];
				vi  = Grid.cn1[i][j+1]   + Grid.p[i][j+1];
				vsd = Grid.cn1[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
				vse = Grid.cn1[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
				vid = Grid.cn1[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
				vie = Grid.cn1[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
			}
			//Cantos
			if((i==0)&&(j==0)){
				vd  = Grid.cn1[i+1][j]   + Grid.p[i+1][j];
				vi  = Grid.cn1[i][j+1]   + Grid.p[i][j+1];
				vid = Grid.cn1[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
			}
			if((i==0)&&(j==TAM-1)){
				vd  = Grid.cn1[i+1][j]   + Grid.p[i+1][j];
				vs  = Grid.cn1[i][j-1]   + Grid.p[i][j-1];
				vsd = Grid.cn1[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
			}
			if((i==TAM-1)&&(j==0)){
				ve  = Grid.cn1[i-1][j]   + Grid.p[i-1][j];
				vi  = Grid.cn1[i][j+1]   + Grid.p[i][j+1];
				vie = Grid.cn1[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
			}
			if((i==TAM-1)&&(j==TAM-1)){
				ve  = Grid.cn1[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn1[i][j-1]   + Grid.p[i][j-1];
				vse = Grid.cn1[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
			}
			//Linhas Externas
			if(i==0){
				vd  = Grid.cn1[i+1][j]   + Grid.p[i+1][j];
				vs  = Grid.cn1[i][j-1]   + Grid.p[i][j-1];
				vi  = Grid.cn1[i][j+1]   + Grid.p[i][j+1];
				vsd = Grid.cn1[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
				vid = Grid.cn1[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
			}
			if(j==0){
				vd  = Grid.cn1[i+1][j]   + Grid.p[i+1][j];
				ve  = Grid.cn1[i-1][j]   + Grid.p[i-1][j];
				vi  = Grid.cn1[i][j+1]   + Grid.p[i][j+1];
				vid = Grid.cn1[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
				vie = Grid.cn1[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
				
			}			
			if(i==TAM-1){
				ve  = Grid.cn1[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn1[i][j-1]   + Grid.p[i][j-1];
				vi  = Grid.cn1[i][j+1]   + Grid.p[i][j+1];
				vse = Grid.cn1[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
				vie = Grid.cn1[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
			}
			if(j==TAM-1){
				vd  = Grid.cn1[i+1][j]   + Grid.p[i+1][j];
				ve  = Grid.cn1[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn1[i][j-1]   + Grid.p[i][j-1];
				vsd = Grid.cn1[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
				vse = Grid.cn1[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
			}
			custo[0]=vd; custo[1]=ve; custo[2]=vs; custo[3]=vi; custo[4]=vsd; custo[5]=vse; custo[6]=vid; custo[7]=vie;
			min=custo[0];
			for(int k=1;k<8;k++)
			{
				if (custo[k]< min)
				{
					min = custo[k];
				}
			}
			Grid.cn1[i][j]=min;
			}
		} //Fim j
	}//Fim i
	}//Fim if

	//*********************************VEICULO 02**********************************************
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j<TAM;j=j+1)
		{
			if ((i==iv2)&&(j==jv2))
			{
				Grid.cn2[i][j]=0;
			}
			else{
				Grid.cn2[i][j]=1000;//Infinito
			}
		}
	}
	raiz2=sqrt(2.0);
	for(int loop=0;loop<TAM;loop++)
	{
	for(int i=0;i<TAM;i=i+1)
	{
		for(int j=0;j< TAM;j=j+1)
		{
			
			if((Grid.occ[i][j]>=0)||((i==iv2)&&(j==jv2)))
			{}
			else
			{
			//Zerando os custos
			vd=0; ve=0;vs=0;vi=0;vsd=0;vse=0;vid=0;vie=0;
			for(int k=0;k<8;k++){custo[k]=0;}
			//Parte Interna
			if((i!=0)&&(i!=TAM-1)&&(j!=0)&&(j!=TAM-1)){
				vd  = Grid.cn2[i+1][j]   + Grid.p[i+1][j];
				ve  = Grid.cn2[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn2[i][j-1]   + Grid.p[i][j-1];
				vi  = Grid.cn2[i][j+1]   + Grid.p[i][j+1];
				vsd = Grid.cn2[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
				vse = Grid.cn2[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
				vid = Grid.cn2[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
				vie = Grid.cn2[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
			}
			//Cantos
			if((i==0)&&(j==0)){
				vd  = Grid.cn2[i+1][j]   + Grid.p[i+1][j];
				vi  = Grid.cn2[i][j+1]   + Grid.p[i][j+1];
				vid = Grid.cn2[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
			}
			if((i==0)&&(j==TAM-1)){
				vd  = Grid.cn2[i+1][j]   + Grid.p[i+1][j];
				vs  = Grid.cn2[i][j-1]   + Grid.p[i][j-1];
				vsd = Grid.cn2[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
			}
			if((i==TAM-1)&&(j==0)){
				ve  = Grid.cn2[i-1][j]   + Grid.p[i-1][j];
				vi  = Grid.cn2[i][j+1]   + Grid.p[i][j+1];
				vie = Grid.cn2[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
			}
			if((i==TAM-1)&&(j==TAM-1)){
				ve  = Grid.cn2[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn2[i][j-1]   + Grid.p[i][j-1];
				vse = Grid.cn2[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
			}
			//Linhas Externas
			if(i==0){
				vd  = Grid.cn2[i+1][j]   + Grid.p[i+1][j];
				vs  = Grid.cn2[i][j-1]   + Grid.p[i][j-1];
				vi  = Grid.cn2[i][j+1]   + Grid.p[i][j+1];
				vsd = Grid.cn2[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
				vid = Grid.cn2[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
			}
			if(j==0){
				vd  = Grid.cn2[i+1][j]   + Grid.p[i+1][j];
				ve  = Grid.cn2[i-1][j]   + Grid.p[i-1][j];
				vi  = Grid.cn2[i][j+1]   + Grid.p[i][j+1];
				vid = Grid.cn2[i+1][j+1] + Grid.p[i+1][j+1]*raiz2;
				vie = Grid.cn2[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
				
			}			
			if(i==TAM-1){
				ve  = Grid.cn2[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn2[i][j-1]   + Grid.p[i][j-1];
				vi  = Grid.cn2[i][j+1]   + Grid.p[i][j+1];
				vse = Grid.cn2[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
				vie = Grid.cn2[i-1][j+1] + Grid.p[i-1][j+1]*raiz2;
			}
			if(j==TAM-1){
				vd  = Grid.cn2[i+1][j]   + Grid.p[i+1][j];
				ve  = Grid.cn2[i-1][j]   + Grid.p[i-1][j];
				vs  = Grid.cn2[i][j-1]   + Grid.p[i][j-1];
				vsd = Grid.cn2[i+1][j-1] + Grid.p[i+1][j-1]*raiz2;
				vse = Grid.cn2[i-1][j-1] + Grid.p[i-1][j-1]*raiz2;
			}
			custo[0]=vd; custo[1]=ve; custo[2]=vs; custo[3]=vi; custo[4]=vsd; custo[5]=vse; custo[6]=vid; custo[7]=vie;
			min = custo[0];
			for(int k=1;k<8;k++)
			{
				if (custo[k]< min)
				{
					min = custo[k];
				}
			}
			Grid.cn2[i][j]=min;
			}
		} //Fim j
	}//Fim i
	}//Fim if

}


// map helper functions
int GetMap( int x, int y )
{
	if( x < 0 || x >= TAM || y < 0 || y >= TAM )
	{
		return 9;	 
	}
	return Grid.n[x][y];
}
// Definitions
class MapSearchNode
{
public:
	unsigned int x;	 // the (x,y) positions of the node
	unsigned int y;	
	
	MapSearchNode() { x = y = 0; }
	MapSearchNode( unsigned int px, unsigned int py ) { x=px; y=py; }
	//Dist�ncia estimada para a Meta
	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	//� o destino ?
	bool IsGoal( MapSearchNode &nodeGoal ); 
	//Gera os sucessores de um dado N�
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	//Custo para mover para o sucessor
	float GetCost( MapSearchNode &successor );
	//S�o posi��es id�nticas?
	bool IsSameState( MapSearchNode &rhs );
	//Salva a Posi��o no vetor Path (caminho)
	void SaveNodeInfo(int irb); 
};
//S�o posi��es id�nticas?
bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
	// same state in a maze search is simply when (x,y) are the same
	if( (x == rhs.x) &&
		(y == rhs.y) )
	{
		return true;
	}
	else
	{
		return false;
	}
}
//Salva a Posi��o no vetor Path (caminho)
void MapSearchNode::SaveNodeInfo(int irb)
{
	
	Path[irb].n=Path[irb].n+1;
	Path[irb].x[Path[irb].n]=x; Path[irb].y[Path[irb].n]=y;
	
	
}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 
//Dist�ncia estimada para a Meta
float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	float xd = fabs(float(((float)x - (float)nodeGoal.x)));
	float yd = fabs(float(((float)y - (float)nodeGoal.y)));
	return xd + yd;
}
//� o destino ?
bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{
	if( (x == nodeGoal.x) &&
		(y == nodeGoal.y) )
	{
		return true;
	}
	return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
//Gera os sucessores de um dado N�
//Pode ser modificado para permitir movimentos na diagonal!!
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{
	int parent_x = -1; 
	int parent_y = -1; 
	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}
	MapSearchNode NewNode;
	// push each possible move except allowing the search to go backwards
	if( (GetMap( x-1, y ) < 9)&& !((parent_x == x-1) && (parent_y == y))) 
	{
		NewNode = MapSearchNode( x-1, y );
		astarsearch->AddSuccessor( NewNode );
	}	
	if( (GetMap( x, y-1 ) < 9)&& !((parent_x == x) && (parent_y == y-1))) 
	{
		NewNode = MapSearchNode( x, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap( x+1, y ) < 9)&& !((parent_x == x+1) && (parent_y == y))) 
	{
		NewNode = MapSearchNode( x+1, y );
		astarsearch->AddSuccessor( NewNode );
	}	
	if( (GetMap( x, y+1 ) < 9)&& !((parent_x == x) && (parent_y == y+1)))
	{
		NewNode = MapSearchNode( x, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}	
	//*********************ADICIONADOS POR MIM******************************

	if( (GetMap( x-1, y-1 ) < 9)&& !((parent_x == x-1) && (parent_y == y-1)))
	{
		NewNode = MapSearchNode( x-1, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}
	if( (GetMap( x+1, y+1 ) < 9)&& !((parent_x == x+1) && (parent_y == y+1)))
	{
		NewNode = MapSearchNode( x+1, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}
	if( (GetMap( x-1, y+1 ) < 9)&& !((parent_x == x-1) && (parent_y == y+1)))
	{
		NewNode = MapSearchNode( x-1, y+1 );
		astarsearch->AddSuccessor( NewNode );
	}
	if( (GetMap( x+1, y-1 ) < 9)&& !((parent_x == x+1) && (parent_y == y-1)))
	{
		NewNode = MapSearchNode( x+1, y-1 );
		astarsearch->AddSuccessor( NewNode );
	}

	return true;
}
// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving
//Custo para mover para o sucessor
float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return (float) GetMap( x, y );
}
//Obtencaoo de um caminho no Grid
int AStarGrid(int inix, int iniy, int endx, int endy, int irb)
{

	// Our sample problem defines the world as a 2d array representing a terrain
	// Each element contains an integer from 0 to 5 which indicates the cost 
	// of travel across the terrain. Zero means the least possible difficulty 
	// in travelling (think ice rink if you can skate) whilst 5 represents the 
	// most difficult. 9 indicates that we cannot pass.
	Path[irb].n=-1;
	// Create an instance of the search class..
	AStarSearch<MapSearchNode> astarsearch;
	unsigned int SearchCount = 0;       // Contagem das pesquisas
	const unsigned int NumSearches = 1; // N�mero de pesquisa
	while(SearchCount < NumSearches)
	{
		// Create a start state
		MapSearchNode nodeStart;
		nodeStart.x = inix;
		nodeStart.y = iniy; 
		// Define the goal state
		MapSearchNode nodeEnd;
		nodeEnd.x = endx;						
		nodeEnd.y = endy; 
		// Set Start and goal states
		astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
		unsigned int SearchState;
		unsigned int SearchSteps = 0;
		do
		{
			SearchState = astarsearch.SearchStep();
			SearchSteps++;
			#if DEBUG_LISTS // Verificando listas (aberta e fechada)
				cout << "Steps:" << SearchSteps << "\n";
				int len = 0;
				cout << "Open:\n";
				MapSearchNode *p = astarsearch.GetOpenListStart();
				while( p )
				{
					len++;
					#if !DEBUG_LIST_LENGTHS_ONLY			
						((MapSearchNode *)p)->PrintNodeInfo();
					#endif
					p = astarsearch.GetOpenListNext();				
				}
				cout << "Open list has " << len << " nodes\n";
				len = 0;
				cout << "Closed:\n";
				p = astarsearch.GetClosedListStart();
				while( p )
				{
					len++;
					#if !DEBUG_LIST_LENGTHS_ONLY			
						p->PrintNodeInfo();
					#endif			
					p = astarsearch.GetClosedListNext();
				}
				cout << "Closed list has " << len << " nodes\n";
			#endif
		}while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
		if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
		{
			//+cout << "Search found goal state\n";
			MapSearchNode *node = astarsearch.GetSolutionStart();
			#if DISPLAY_SOLUTION
				cout << "Displaying solution\n";
			#endif				
			int steps = 0;
			node->SaveNodeInfo(irb);
			for( ;; )
			{
				node = astarsearch.GetSolutionNext();
				if( !node )
				{
					break;
				}
				node->SaveNodeInfo(irb);				
				steps ++;	
			};
			//+cout << "Solution steps " << steps << endl;
			// Once you're done with the solution you can free the nodes up
			astarsearch.FreeSolutionNodes();	
		}
		else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
		{
			//+cout << "Search terminated. Did not find goal state\n";
		}
		// Display the number of loops the search went through
		//+cout << "SearchSteps : " << SearchSteps << "\n";
		SearchCount ++;
		astarsearch.EnsureMemoryFreed();
	}
	/*
	//IECON
	//robo 1
	Path[0].x[1]=132;Path[0].y[1]=150;
	Path[0].x[2]=132;Path[0].y[2]=151;
	Path[0].x[3]=131;Path[0].y[3]=151;
	Path[0].x[4]=132;Path[0].y[4]=152;
	Path[0].x[5]=133;Path[0].y[5]=152;
	Path[0].x[6]=134;Path[0].y[6]=153;
	Path[0].x[7]=134;Path[0].y[7]=154;
	Path[0].x[8]=134;Path[0].y[8]=155;
	Path[0].x[9]=135;Path[0].y[9]=154;
	Path[0].x[10]=136;Path[0].y[10]=153;
	Path[0].x[11]=137;Path[0].y[11]=152;
	Path[0].x[12]=138;Path[0].y[12]=151;
	Path[0].x[13]=139;Path[0].y[13]=151;
	Path[0].x[14]=140;Path[0].y[14]=152;
	Path[0].x[15]=141;Path[0].y[15]=152;
	Path[0].x[16]=142;Path[0].y[16]=152;
	Path[0].x[17]=143;Path[0].y[17]=152;
	Path[0].x[18]=144;Path[0].y[18]=152;
	Path[0].x[19]=145;Path[0].y[19]=152;
	Path[0].x[20]=146;Path[0].y[20]=152;
	Path[0].x[21]=147;Path[0].y[21]=152;
	Path[0].x[22]=148;Path[0].y[22]=152;
	Path[0].x[23]=149;Path[0].y[23]=153;
	Path[0].x[24]=149;Path[0].y[24]=154;
	Path[0].x[25]=149;Path[0].y[25]=155;
	Path[0].x[26]=149;Path[0].y[26]=156;
	Path[0].x[27]=149;Path[0].y[27]=157;
	Path[0].x[28]=150;Path[0].y[28]=156;
	Path[0].x[29]=151;Path[0].y[29]=155;
	Path[0].x[30]=152;Path[0].y[30]=154;
	Path[0].x[31]=152;Path[0].y[31]=153;
	Path[0].x[32]=153;Path[0].y[32]=152;
	Path[0].x[33]=154;Path[0].y[33]=151;
	Path[0].x[34]=155;Path[0].y[34]=150;
	Path[0].x[35]=156;Path[0].y[35]=149;
	Path[0].x[36]=157;Path[0].y[36]=149;
	Path[0].x[37]=158;Path[0].y[37]=149;
	Path[0].x[38]=159;Path[0].y[38]=149;
	Path[0].x[39]=160;Path[0].y[39]=149;
	Path[0].x[40]=161;Path[0].y[40]=149;
	Path[0].x[41]=162;Path[0].y[41]=149;
	Path[0].x[42]=163;Path[0].y[42]=149;
	Path[0].x[43]=164;Path[0].y[43]=148;
	Path[0].x[44]=164;Path[0].y[44]=147;
	Path[0].x[45]=164;Path[0].y[45]=146;
	Path[0].x[46]=165;Path[0].y[46]=147;

	//robo 2

	Path[1].x[1]=136;Path[1].y[1]=142;
	Path[1].x[2]=136;Path[1].y[2]=143;
	Path[1].x[3]=136;Path[1].y[3]=142;
	Path[1].x[4]=137;Path[1].y[4]=143;
	Path[1].x[5]=137;Path[1].y[5]=144;
	Path[1].x[6]=137;Path[1].y[6]=145;
	Path[1].x[7]=137;Path[1].y[7]=146;
	Path[1].x[8]=137;Path[1].y[8]=147;
	Path[1].x[9]=137;Path[1].y[9]=148;
	Path[1].x[10]=138;Path[1].y[10]=149;
	Path[1].x[11]=139;Path[1].y[11]=149;
	Path[1].x[12]=140;Path[1].y[12]=150;
	Path[1].x[13]=141;Path[1].y[13]=150;
	Path[1].x[14]=142;Path[1].y[14]=150;
	Path[1].x[15]=143;Path[1].y[15]=150;
	Path[1].x[16]=144;Path[1].y[16]=150;
	Path[1].x[17]=145;Path[1].y[17]=150;
	Path[1].x[18]=146;Path[1].y[18]=150;
	Path[1].x[19]=147;Path[1].y[19]=150;
	Path[1].x[20]=148;Path[1].y[20]=149;
	Path[1].x[21]=149;Path[1].y[21]=148;
	Path[1].x[22]=149;Path[1].y[22]=147;
	Path[1].x[23]=149;Path[1].y[23]=146;
	Path[1].x[24]=149;Path[1].y[24]=145;
	Path[1].x[25]=150;Path[1].y[25]=146;
	Path[1].x[26]=151;Path[1].y[26]=147;
	Path[1].x[27]=152;Path[1].y[27]=148;
	Path[1].x[28]=153;Path[1].y[28]=149;
	Path[1].x[29]=154;Path[1].y[29]=150;
	Path[1].x[30]=155;Path[1].y[30]=150;
	Path[1].x[31]=156;Path[1].y[31]=150;
	Path[1].x[32]=157;Path[1].y[32]=151;
	Path[1].x[33]=158;Path[1].y[33]=152;
	Path[1].x[34]=159;Path[1].y[34]=152;
	Path[1].x[35]=160;Path[1].y[35]=152;
	Path[1].x[36]=161;Path[1].y[36]=152;
	Path[1].x[37]=162;Path[1].y[37]=152;
	Path[1].x[38]=163;Path[1].y[38]=152;
	Path[1].x[39]=164;Path[1].y[39]=153;
	Path[1].x[40]=165;Path[1].y[40]=153;
	Path[1].x[41]=164;Path[1].y[41]=154;
	Path[1].x[42]=164;Path[1].y[42]=155;
	Path[1].x[43]=165;Path[1].y[43]=154;
	Path[1].x[44]=166;Path[1].y[44]=153;
	Path[1].x[45]=167;Path[1].y[45]=152;
	Path[1].x[46]=168;Path[1].y[46]=152;

	Path[0].n=46;
	Path[1].n=46;
	*/
	if (Path[irb].n>=0) {return 1;}else{return 0;}
}





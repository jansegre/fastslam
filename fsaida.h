using namespace std;
ofstream f1x, f1y,  f1t;
ofstream f1ex,f1ey, f1et;
ofstream f1px,f1py, f1pt;
ofstream f1res;
ofstream f1xod,f1yod,f1tod;
ofstream f2x, f2y,  f2t;
ofstream f2ex,f2ey, f2et;
ofstream f2px,f2py, f2pt;
ofstream f2res;
ofstream f2xod,f2yod,f2tod;
ofstream fmapax,fmapay,fmapaz;


//Outros
ofstream fmpart;
ofstream fgiro;
ofstream ferro;
ofstream fcam1;
ofstream fcam2;
ofstream fresult;


double e1x,e1y,e1t;
double e1x2,e1y2,e1t2;

double e1ox,e1oy,e1ot;
double e1ox2,e1oy2,e1ot2;

double e2x,e2y,e2t;
double e2x2,e2y2,e2t2;

double e2ox,e2oy,e2ot;
double e2ox2,e2oy2,e2ot2;


int    dt,dt1;
int    dtobs;
int    nrobs;
int    nreobs1;
int    nreobs2;
int    reobs1[100];
int    reobs2[100];

void InitVar(void){
	nreobs1=0;
	nreobs2=0;
	dt=0;dt1=0;
	nrobs=0;
	dtobs=1;
	e1x =0,e1y =0,e1t =0;
	e1x2=0,e1y2=0,e1t2=0;
	e1ox =0,e1oy =0,e1ot =0;
	e1ox2=0,e1oy2=0,e1ot2=0;

	e2x =0,e2y =0,e2t =0;
	e2x2=0,e2y2=0,e2t2=0;

	e2ox =0,e2oy =0,e2ot =0;
	e2ox2=0,e2oy2=0,e2ot2=0;


	for(int i=0; i<100;i++){
		reobs1[i]=0;
		reobs2[i]=0;
	}
}

void OpenFiles(void){

	fmpart.open ("fmpart.txt"); fgiro.open ("fgiro.txt"); ferro.open ("ferro.txt");
	f1res.open ("f1res.txt"); 
	f1x.open  ("f1x.txt"); f1y.open   ("f1y.txt"); f1t.open ("f1t.txt");
	f1xod.open  ("f1xod.txt"); f1yod.open   ("f1yod.txt"); f1tod.open ("f1tod.txt");

	f1ex.open ("f1ex.txt");f1ey.open  ("f1ey.txt");f1et.open("f1et.txt");
	f1px.open ("f1px.txt");f1py.open  ("f1py.txt");f1pt.open("f1pt.txt");

	f2res.open ("f2res.txt"); 
	f2x.open  ("f2x.txt"); f2y.open   ("f2y.txt"); f2t.open ("f2t.txt");
	f2xod.open  ("f2xod.txt"); f2yod.open   ("f2yod.txt"); f2tod.open ("f2tod.txt");

	f2ex.open ("f2ex.txt");f2ey.open  ("f2ey.txt");f2et.open("f2et.txt");
	f2px.open ("f2px.txt");f2py.open  ("f2py.txt");f2pt.open("f2pt.txt");
	fcam1.open("fcam1.txt");
	fcam2.open("fcam2.txt");

	fresult.open("fresult.txt");

	fmapax.open("fmapax.txt");
	fmapay.open("fmapay.txt");
	fmapaz.open("fmapaz.txt");

}
void CloseFiles(void){
	// Fechando Arquivos
	f1x.close();  f1y.close();  f1t.close(); 
	f1ex.close(); f1ey.close(); f1et.close();
	f1px.close(); f1py.close(); f1pt.close();
	f1xod.close();f1yod.close();f1tod.close();

	f1res.close();

	f2x.close();  f2y.close();  f2t.close(); 
	f2ex.close(); f2ey.close(); f2et.close();
	f2px.close(); f2py.close(); f2pt.close();
	f2xod.close();f2yod.close();f2tod.close();

	f2res.close();

	fmpart.close();
	fgiro.close();
	ferro.close();
	fresult.close();
	fcam1.close();
	fcam2.close();

	fmapax.close();
	fmapay.close();
	fmapaz.close();
}

void ImpRes(void){

	f1res << "Nr Part: "<< NP << "\n";
	f1res << "Erro RMS em X, Y e T\n";
	f1res << sqrt((double) (e1x2/dt1))<< "\n" << sqrt((double) (e1y2/dt1))<< "\n" << sqrt((double) (e1t2/dt1)) << "\n";
	f1res << "Nr Obs: " << nrobs << "\n";
	f1res << "Nr Reobs': " << nreobs1 << "\n";
	f1res << "Nr Passos: " << dt1 << "\n";
	f1res << "Max Range: " << MaxRangeC << "\n";

	f2res << "Nr Part: "<< NP << "\n";
	f2res << "Erro RMS em X, Y e T\n";
	f2res << sqrt((double) (e2x2/dt1))<< "\n" << sqrt((double) (e2y2/dt1))<< "\n" << sqrt((double) (e2t2/dt1)) << "\n";
	f2res << "Nr Obs: " << nrobs << "\n";
	f2res << "Nr Reobs': " << nreobs2 << "\n";
	f2res << "Nr Passos: " << dt1 << "\n";
	f2res << "Max Range: " << MaxRangeC << "\n";

	for(int i=0; i<dt1;i++)
	{
		f1res << reobs1[i] << "\n";
		f2res << reobs2[i] << "\n";
	}

}

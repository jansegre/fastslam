/*
 * R2D2.h
 *
 *  Created on: Apr 2, 2011
 *      Author: darknerd
 */

#ifndef R2D2_H_
#define R2D2_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <playerc++.h>
#include <cstdlib>
#include <unistd.h>

using namespace std;
using namespace PlayerCc;

#define LASER_SCAN_SIZE 361
#define SONAR_SCAN_SIZE 16

typedef struct velocity{
	double linearv;
	double angularv;
}veloc;

typedef struct position{
	double xpos;
	double ypos;
	double yaw;
}pose;

class R2D2 {
public:

	//obstacle avoidance
	double min_value;
	double average;
	int min_num;
	int max_num;
	double obsTol;
	double minfrontdistance;
	int avoidcount;
	int randcount;

	double cruisespeed;
	double avoidspeed;
	double avoidturn;
	double stopdist;
	int avoidduration;

	//position
	pose pos;
	veloc vel;
	double dist_Trav;

	//sonar - laser
	double sonar[16];
	double laser[361];                //this->laserProxy->GetRange(i)
	double bear[361];                 //this->laserProxy->GetBearing(i);
	double laserC[8];
	player_point_2d_t laserConf[361]; //this->laserProxy->GetPoint(i);

	//mission
	bool mission1, mission2;
	uint colorObj1;
	uint colorObj2;

	//moving
	void Wander3(double v, double giro);
	void Ir(double x, double y, double Yaw);
	void AvoidObstacles();
	void SetVelocityArgs(bool min_method);


	int SearchBlob(); //return the blobIndex of object searched
	void GetLaser();
	void GetSonar();
	void ReadSensors();
	veloc GetSpeed();
	pose GetPosition();
	void SetSpeed(double lspeed, double aspeed);
	R2D2(int porta);
	virtual ~R2D2();

private:
	PlayerClient *robotlink;

	Position2dProxy *p2dProxy; //Finished
	SonarProxy *sonarProxy;
	LaserProxy *laserProxy;
	SimulationProxy *simProxy;

	PlayerClient *Connect(int porta);
	Position2dProxy *InitPosition(PlayerClient *rb);
	SonarProxy *InitSonar(PlayerClient *rb);
	LaserProxy *InitLaser(PlayerClient *rb);
	SimulationProxy *InitSimulation(PlayerClient *rb);
};

#endif /* R2D2_H_ */

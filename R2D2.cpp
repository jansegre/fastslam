/*
 * R2D2.cpp
 *
 *  Created on: Apr 2, 2011
 *      Author: darknerd
 */

#include "R2D2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <playerc++.h>
#include <cstdlib>
#include <unistd.h>

#define pi 3.1415926
using namespace std;
using namespace PlayerCc;

R2D2::R2D2(int porta) {

	this->robotlink = Connect(porta);
	this->p2dProxy = InitPosition(this->robotlink);
	this->sonarProxy = InitSonar(this->robotlink);
	this->laserProxy = InitLaser(this->robotlink);

   //this->simProxy = InitSimulation(this->robotlink);

	if (porta==6665){this->p2dProxy->SetOdometry(-8,-0.25,0);}
	if (porta==6666){this->p2dProxy->SetOdometry(-8, 0.25,0);}
	this->pos.xpos = this->pos.ypos = this->pos.yaw = 0.0;

	for (uint i = 0; i < 8; ++i) {
		sonar[i] = 0.0;
	}

	//wander3
	this->minfrontdistance = 1.0; //0.6;
	this->avoidcount = 0;
	this->randcount = 0;
	this->cruisespeed = 0.5;
	this->avoidspeed = 0.03;
	this->avoidturn = 0.5;
	this->stopdist = 0.3;
	this->avoidduration = 10;

	//wander2
	this->min_num = 0;

	mission1 = false;
	mission2 = false;
	colorObj1 = 139;
	colorObj2 = 16753920;

	//gripperProxy->RequestGeometry();
	p2dProxy->RequestGeom();
	sonarProxy->RequestGeom();
	laserProxy->RequestGeom();

}

R2D2::~R2D2() {
	// TODO Auto-generated destructor stub
}

PlayerClient *R2D2::Connect(int porta) {
	PlayerClient *rb;
	rb = new PlayerClient("localhost", porta);
	return rb;
}

Position2dProxy *R2D2::InitPosition(PlayerClient *rb) {
	Position2dProxy *pt = new Position2dProxy(rb, 0);

	return pt;
}

SonarProxy *R2D2::InitSonar(PlayerClient *rb) {
	SonarProxy *sr = new SonarProxy(rb, 0);
	return sr;
}

LaserProxy *R2D2::InitLaser(PlayerClient *rb) {
	LaserProxy *lr = new LaserProxy(rb, 0);
	return lr;
}

SimulationProxy *R2D2::InitSimulation(PlayerClient *rb) {
	SimulationProxy *st = new SimulationProxy(rb, 0);
	return st;
}

void R2D2::ReadSensors() {
	robotlink->Read();
	this->GetPosition();
	this->GetLaser();
}

void R2D2::SetSpeed(double lspeed, double aspeed) {
	p2dProxy->SetSpeed(lspeed, aspeed);
}

veloc R2D2::GetSpeed() {
	vel.linearv = p2dProxy->GetXSpeed();
	vel.angularv = p2dProxy->GetYawSpeed();
	return vel;
}

pose R2D2::GetPosition() {
	robotlink->Read();
	pos.xpos = p2dProxy->GetXPos();
	pos.ypos = p2dProxy->GetYPos();
	pos.yaw =  p2dProxy->GetYaw();
	return pos;
}

void R2D2::GetSonar() {
	for (uint i = 8; i < 16; i++) {
		sonar[i] = sonarProxy->GetScan(i);
	}
}

void R2D2::GetLaser() {
	for (uint i = 0; i < 361; i++) {
		this->laser[i] = this->laserProxy->GetRange(i);// Distância
		this->laserConf[i]=this->laserProxy->GetPoint(i); // Array Pontos
		this->bear[i] = this->laserProxy->GetBearing(i); //Orientação

	}

}
void R2D2::Ir(double x, double y, double Yaw)
{
	//player_pose2d_t pos={x,y,Yaw};
	//player_pose2d_t vel={0.5,0.0,Yaw};

	this->p2dProxy->GoTo(x,y,Yaw);
	//this->p2dProxy->GoTo(pos,vel);

}
void R2D2::Wander3(double v, double giro) {

}

void R2D2::AvoidObstacles() {

}

void R2D2::SetVelocityArgs(bool min_method) {
	double turn_rate = 0.0; // Angular velocity
	double sd = 0.0; // Linear velocity

	if (min_method == true) // Turn away from minimum reading
	{
		switch (this->min_num) // Set robot to turn away from minimum position and set speeds
		{
		case 1:
			turn_rate = -20;
			sd = 0.1;
			break;
		case 2:
			turn_rate = -30;
			sd = 0.05;
			break;
		case 3:
			turn_rate = -45;
			sd = -0.1;
			break;
		case 4:
			turn_rate = 45;
			sd = -0.1;
			break;
		case 5:
			turn_rate = 30;
			sd = 0.05;
			break;
		case 6:
			turn_rate = 20;
			sd = 0.1;
			break;
		}
	}

	if (min_method == false) {
		switch (this->max_num) {
		case 0:
			turn_rate = -30;
			sd = 0.05;
			break;
		case 1:
			turn_rate = -20;
			sd = 0.05;
			break;
		case 2:
			turn_rate = -10;
			sd = 0.1;
			break;
		case 3:
			turn_rate = 0;
			sd = 0.1;
			break;
		case 4:
			turn_rate = 0;
			sd = 0.1;
			break;
		case 5:
			turn_rate = 10;
			sd = 0.1;
			break;
		case 6:
			turn_rate = 20;
			sd = 0.05;
			break;
		case 7:
			turn_rate = 30;
			sd = 0.05;
			break;
		}
	}

	this->vel.angularv = DTOR(turn_rate);
	this->vel.linearv = sd;
}

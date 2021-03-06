// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2016 INRIA for CODYCO Project
* Author: Serena Ivaldi <serena.ivaldi@inria.fr>
* website: www.codyco.eu
*
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/


#include <stdio.h>
#include <iostream>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <string>
#include <sstream>
#include <fstream>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

int nJointsArm=7;
int nJointsTorso=3;
int nbIter;

//---------------------------------------------------------
// open drivers with compliance (real robot)
//---------------------------------------------------------
bool openDriversArm(Property &options, string robot, string part, PolyDriver *&pd, IPositionControl *&ipos, IPositionDirect *&iposd, IEncoders *&ienc, IControlMode2 *&imode, IImpedanceControl *&iimp, ITorqueControl *&itrq)
{
	// open the device drivers
	options.put("device","remote_controlboard");
	options.put("local",string("/upperBodyPlayer/"+part).c_str());
	options.put("remote",string("/"+robot+"/"+part).c_str());
		 
	if(!pd->open(options))
	{
		cout<<"Problems connecting to the remote driver of "<<part<<endl;
		return false;
	}
	if(!pd->isValid())
	{
	    printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return false;	
	}
	if(!pd->view(imode) || !pd->view(ienc) || !pd->view(ipos) || !(pd->view(iposd)) || !pd->view(iimp) || !pd->view(itrq))
	{
		cout<<"Problems acquiring interfaces for "<<part<<endl;
		return false;
	}
	return true;
}

//---------------------------------------------------------
// open drivers no compliance (simulation)
//---------------------------------------------------------
bool openDriversArm_noImpedance(Property &options, string robot, string part, PolyDriver *&pd, IPositionControl *&ipos, IPositionDirect *&iposd, IEncoders *&ienc, IControlMode2 *&imode)
{
	// open the device drivers
	options.put("device","remote_controlboard");
	options.put("local",string("/upperBodyPlayer/"+part).c_str());
	options.put("remote",string("/"+robot+"/"+part).c_str());

	if(!(pd->open(options)))
	{
		cout<<"Problems connecting to the remote driver of "<<part<<endl;
		return false;
	}
	if(!(pd->isValid()))
	{
	    printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return false;	
	}
	if(!(pd->view(imode)) || !(pd->view(ienc)) || !(pd->view(ipos)) || !(pd->view(iposd)))
	{
		cout<<"Problems acquiring interfaces for "<<part<<endl;
		return false;
	}
	return true;  
}

//---------------------------------------------------------
// read the trajectory from a file
//---------------------------------------------------------
bool loadFile (string &filename, Matrix &q_RA, Matrix &q_LA, Matrix &q_T, Matrix &timestamps)
{
	cout<<"Reading trajectories from file: "<<filename<<endl;
	
	// open the file
	ifstream inputFile;

	inputFile.open(filename.c_str());
	if (!inputFile.is_open ())
	{
		cout << "ERROR: Can't open file: " << filename << endl;
		return false;
	}
		
	// get the number of lines in the file
	nbIter = 0; string l;
	while (getline (inputFile, l))
	{
		nbIter++;
	}
	cout << "INFO: "<< filename << " is a record of " << nbIter << " iterations" << endl;
		
	inputFile.clear();
	inputFile.seekg(0, ios::beg);
	
	// resizing matrix to get the correct values of the trajectories
	q_RA.resize(nbIter,nJointsArm); q_RA.zero();
	q_LA.resize(nbIter,nJointsArm); q_LA.zero();
	q_T.resize(nbIter,nJointsTorso); q_T.zero();
	timestamps.resize(nbIter,1); timestamps.zero();
	
	Vector base(7);

	// reading the trajectory from the file
	for (int c=0; c<nbIter; c++)
	{
		printf ("Load file %s : \r%d / %d", filename.c_str (), c + 1, nbIter);
		getline (inputFile, l);
		stringstream line;
		line << l;
		//line >> timestamps[c][0];
		
		// the floating base
		line>>base[0];
		line>>base[1];
		line>>base[2];
		line>>base[3];
		line>>base[4];
		line>>base[5];
		line>>base[6];
		
		
		//torso_yaw
		line >> q_T[c][0];
		//l_elbow
		line >> q_LA[c][3];
		//l_wrist_prosup
		line >> q_LA[c][4];
		//l_wrist_yaw
		line >> q_LA[c][6];
		//l_shoulder_pitch
		line >> q_LA[c][0];
		//l_shoulder_roll
		line >> q_LA[c][1];
		//l_shoulder_yaw
		line >> q_LA[c][2];
		//l_wrist_pitch
		line >> q_LA[c][5];
		//r_elbow
		line >> q_RA[c][3];
		//r_wrist_prosup
		line >> q_RA[c][4];
		//r_wrist_yaw
		line >> q_RA[c][6];
		//r_shoulder_pitch
		line >> q_RA[c][0];
		//r_shoulder_roll
		line >> q_RA[c][1];
		//r_shoulder_yaw
		line >> q_RA[c][2];
		//r_wrist_pitch
		line >> q_RA[c][5];
		//torso_pitch
		line >> q_T[c][2];
		//torso_roll
		line >> q_T[c][1];	
	}
	
	cout<<"File is read! "<<endl;
	return true;

}


//---------------------------------------------------------
// read the trajectory from a file
//---------------------------------------------------------
int safety_check(Vector &command_RA, Vector &command_LA, Vector &command_T)
{
	int violations=0;
	
	Vector max_RA(7);
	Vector max_LA(7);
	Vector max_T(3);
	Vector min_RA(7);
	Vector min_LA(7);
	Vector min_T(3);
	
	//max and min values for all the joints of the arms
	max_RA[0]=6; 	min_RA[0]=-85;
	max_RA[1]=80; 	min_RA[1]=15;
	max_RA[2]=78; 	min_RA[2]=-15;
	max_RA[3]=85; 	min_RA[3]=15;
	max_RA[4]=60; 	min_RA[4]=-70;
	max_RA[5]=0; 	min_RA[5]=-70;
	max_RA[6]=30; 	min_RA[6]=-10;
	
	max_LA[0]=6; 	min_LA[0]=-85;
	max_LA[1]=80; 	min_LA[1]=15;
	max_LA[2]=78; 	min_LA[2]=-15;
	max_LA[3]=85; 	min_LA[3]=15;
	max_LA[4]=60; 	min_LA[4]=-70;
	max_LA[5]=0; 	min_LA[5]=-70;
	max_LA[6]=30; 	min_LA[6]=-10;
	
	max_T[0]=25; 	min_T[0]=-25;
	max_T[1]=8; 	min_T[1]=-8;
	max_T[2]=20; 	min_T[2]=-10;
	
	// arms
	for(int i=0; i<7;i++)
	{
		if(command_RA[i]>max_RA[i]) {  command_RA[i]=max_RA[i];	cout<<"#### max RIGHT_ARM "<<i<<endl; violations++;}
		if(command_RA[i]<min_RA[i]) {  command_RA[i]=min_RA[i];	cout<<"#### min RIGHT_ARM "<<i<<endl; violations++;}
	
		if(command_LA[i]>max_LA[i]) {  command_LA[i]=max_LA[i];	cout<<"#### max LEFT_ARM "<<i<<endl; violations++;}
		if(command_LA[i]<min_LA[i]) {  command_LA[i]=min_LA[i];	cout<<"#### min LEFT_ARM "<<i<<endl; violations++;}
	
	}
	
	// torso
	for(int i=0; i<3;i++)
	{
		if(command_T[i]>max_T[i]) {  command_T[i]=max_T[i];	cout<<"#### max TORSO "<<i<<endl; violations++;}
		if(command_T[i]<min_T[i]) {  command_T[i]=min_T[i];	cout<<"#### min TORSO "<<i<<endl; violations++;}
	
	}

	return violations;
}

//==============================================================
//
//		MAIN
//
//==============================================================
int main(int argc, char *argv[]) 
{
		
	//--------------- VARIABLES --------------
	
	int verbosity=1;
	string robotName;
    string fileName;
    int startingPoint=0;
    
    int jointLimitsViolations=0;
    int totalJointsLimitsViolations=0;
    
    //--------------- CONFIG  --------------
    
	Property params;
    params.fromCommand(argc, argv);
    
    if (params.check("help"))
    {
        cout<<"This module plays a given joints trajectory for the upper body, the trajectory being stored on a file."<<endl
			<<" Usage:   upperBodyPlayer --robot ROBOTNAME --file FILENAME --verbosity LEVEL --start STARTPOINT"<<endl
			<<" Default values: robot=icubSim  file=ugo_trajectory.txt verbosity=2 startpoint=0"<<endl;
        return 1;
    }

    if (!params.check("robot"))
    {
        cout<<"==> Missing robot name, setting default"<<endl;
        robotName="icubSim";
    }
    else
    {
		robotName=params.find("robot").asString().c_str();
	}
    
     if (!params.check("file"))
    {
        cout<<"==> Missing file name, setting default"<<endl;
        fileName="trajectory.txt";
    } 
    else
    {
		fileName=params.find("file").asString().c_str();
	}  
	
	if (!params.check("verbosity"))
    {
        cout<<"==> Missing verbosity, setting default"<<endl;
        verbosity=2;
    } 
    else
    {
		verbosity=params.find("verbosity").asInt();
	}  
	
	if (!params.check("start"))
    {
        cout<<"==> Missing start, setting default"<<endl;
        startingPoint=0;
    } 
    else
    {
		startingPoint=params.find("start").asInt();
		
		if(startingPoint<0) 
		{
			cout<<"Warning: Starting point must be >=0"<<endl;
			startingPoint=0;
		}
	}
    
	if(verbosity>=1)
    cout<<"Robot = "<<robotName<<endl
		<<"File	= "<<fileName<<endl
		<<"Verbosity = "<<verbosity<<endl
		<<"Starting point = "<<startingPoint<<endl;
		
	//--------------- CONFIG  --------------
	
	Network yarp;
    if (!yarp.checkNetwork())
	{
		cout<<"YARP network not available. Aborting."<<endl;
		return -1;
	}
    
    //--------------- READING TRAJECTORY  --------------
	// trajectories for the joints
	Matrix q_RA, q_LA, q_T, timestamps;

	if(!loadFile(fileName, q_RA, q_LA, q_T, timestamps))
	{
		cout<<"Errors in loading the trajectory file. Closing."<<endl;
		return -1;
	}
	
	if( startingPoint >= nbIter )
	{
		cout<<"Starting point is after the end of the trajectory. Please choose a starting point smaller than "<<nbIter<<endl;
		return -1;
	}
    
	//--------------- OPENING DRIVERS  --------------
	
	// left arm, right arm, torso
	Property options_LA, options_RA, options_T;
	PolyDriver *dd_LA, *dd_RA, *dd_T;	
	IPositionControl *pos_LA, *pos_RA, *pos_T;
	IPositionDirect *posd_LA, *posd_RA, *posd_T;
    IEncoders *encs_LA, *encs_RA, *encs_T;
    IControlMode2 *ictrl_LA, *ictrl_RA, *ictrl_T;
    IInteractionMode *iint_LA, *iint_RA, *iint_T;
    IImpedanceControl *iimp_LA, *iimp_RA, *iimp_T;
    ITorqueControl *itrq_LA, *itrq_RA, *itrq_T; 
	
	if(robotName=="icub")
	{	
		dd_LA=new PolyDriver;
		dd_RA=new PolyDriver;
		dd_T=new PolyDriver;
		if(verbosity>=1)  cout<<"drivers created"<<endl;
	
		if(verbosity>=1) cout<<"** Opening left arm drivers"<<endl;
		if(!openDriversArm(options_LA, robotName, "left_arm", dd_LA, pos_LA, posd_LA, encs_LA, ictrl_LA, iimp_LA, itrq_LA))
		{
			cout<<"Error opening left arm"<<endl;
			return -1;
		}
		
		if(verbosity>=1) cout<<"** Opening right arm drivers"<<endl;
		if(!openDriversArm(options_RA, robotName, "right_arm", dd_RA, pos_RA, posd_RA, encs_RA, ictrl_RA, iimp_RA, itrq_RA))
		{
			cout<<"Error opening right arm"<<endl;
			return -1;
		}
		
		if(verbosity>=1) cout<<"** Opening torso drivers"<<endl;
		if(!openDriversArm(options_T, robotName, "torso", dd_T, pos_T, posd_T, encs_T, ictrl_T, iimp_T, itrq_T))
		{
			cout<<"Error opening left arm"<<endl;
			return -1;
		}
	}
	else
	{

		dd_LA=new PolyDriver;
		dd_RA=new PolyDriver;
		dd_T=new PolyDriver;
		if(verbosity>=1)  cout<<"drivers created"<<endl;
	
		if(verbosity>=1) cout<<"** Opening left arm drivers"<<endl;
		if(!openDriversArm_noImpedance(options_LA, robotName, "left_arm", dd_LA, pos_LA, posd_LA, encs_LA, ictrl_LA))
		{
			cout<<"Error opening left arm"<<endl;
			return -1;
		}
		
		if(verbosity>=1) cout<<"** Opening right arm drivers"<<endl;
		if(!openDriversArm_noImpedance(options_RA, robotName, "right_arm", dd_RA, pos_RA, posd_RA, encs_RA, ictrl_RA))
		{
			cout<<"Error opening right arm"<<endl;
			return -1;
		}
		
		if(verbosity>=1) cout<<"** Opening torso drivers"<<endl;
		if(!openDriversArm_noImpedance(options_T, robotName, "torso", dd_T, pos_T, posd_T, encs_T, ictrl_T))
		{
			cout<<"Error opening left arm"<<endl;
			return -1;
		}
	}
	
	//---------------  NOW WE CONTROL !! --------------
	
	if(verbosity>=1) cout<< " ***** EVERYTHING IS CREATED ****** "<<endl;
	
	
	//---------------  1) bring to initial position --------------
	
	int i=0;
	int nj_arms=0;
	int nj_torso=0; 
	
    pos_RA->getAxes(&nj_arms);
    pos_T->getAxes(&nj_torso);    
    if(verbosity>=1) cout<<"nj arms / torso "<<nj_arms<<" / "<<nj_torso<<endl;
    
    Vector encoders_RA, encoders_LA, encoders_T;
    Vector command_RA, command_LA, command_T;
    Vector tmp;
    
    encoders_RA.resize(nj_arms);
    encoders_LA.resize(nj_arms);
    encoders_T.resize(nj_torso);
    
    command_RA.resize(nj_arms);
    command_LA.resize(nj_arms);
    command_T.resize(nj_torso);
    
    // setting accelerations
    for (i=0; i<nj_arms; i++) {command_RA[i]= 50.0; command_LA[i]= 50.0;}
    for (i=0; i<nj_torso; i++) {command_T[i]= 50.0;}   
    pos_RA->setRefAccelerations(command_RA.data());
    pos_LA->setRefAccelerations(command_LA.data());
    pos_T->setRefAccelerations(command_T.data());

	// setting velocities
    for (i = 0; i < nj_arms; i++) 
    {
        command_RA[i] = 5.0;
        command_LA[i] = 5.0;
        pos_RA->setRefSpeed(i, command_RA[i]);
        pos_LA->setRefSpeed(i, command_LA[i]);
    }   
    for(i=0; i<nj_torso; i++)
    {
		command_T[i]=5.0;	
		pos_T->setRefSpeed(i, command_T[i]);
	}
	
	// getting the initial configuration of the limbs
	
	if(verbosity>=1) cout<<"Encoders right arm "<<std::endl;
    while(!encs_RA->getEncoders(encoders_RA.data()))
    {
        Time::delay(0.1);
		printf(".");
    }
    if(verbosity>=1) cout<<encoders_RA.toString()<<endl;
    if(verbosity>=1) cout<<"Encoders left arm ";
    while(!encs_LA->getEncoders(encoders_LA.data()))
    {
        Time::delay(0.1);
		printf(".");
    }
    if(verbosity>=1) cout<<encoders_LA.toString()<<endl;
    if(verbosity>=1) cout<<"Encoders torso "<<std::endl;
    while(!encs_T->getEncoders(encoders_T.data()))
    {
        Time::delay(0.1);
		printf(".");
    }
    if(verbosity>=1) cout<<encoders_T.toString()<<endl;
    

    command_RA=encoders_RA;
    command_LA=encoders_LA;
    command_T=encoders_T;
    
    //now set the limbs to the starting point level
    for(i=0; i<nJointsArm; i++) command_RA[i] = q_RA[startingPoint][i];
    for(i=0; i<nJointsArm; i++) command_LA[i] = q_LA[startingPoint][i];
    for(i=0; i<nJointsTorso; i++) command_T[i] = q_T[startingPoint][i];
    
    jointLimitsViolations = safety_check(command_RA, command_LA, command_T);
    
    if(jointLimitsViolations==0)
		cout<<" *** FEASIBLE STARTING POSITION *** "<<endl;
	else
	{
		cout<<" *** INFEASIBLE STARTING POSITION *** "<<endl
			<<"\nThe initial position violates the joint limits x"<<jointLimitsViolations<<" times"<<endl
			<<"CANNOT PROCEED"<<endl;
			
		cout << "Closing drivers" << endl;
		if(dd_RA) {delete dd_RA; dd_RA=0; }
		if(dd_LA) {delete dd_LA; dd_LA=0; }
		if(dd_T) {delete dd_T; dd_T=0;}
		return 0;
	}
    
	cout<<"Move the robot to the initial position: "<<endl
		<<" right arm : "<<command_RA.toString()<<endl
		<<" left arm : "<<command_LA.toString()<<endl
		<<" torso : "<<command_T.toString()<<endl
		<<endl
		<<" ==> at starting time = "<<startingPoint<<endl
		<<" ok? (y/n) ";
		
	string chinput;	
	cin >> chinput;
	cout<<endl;
	 
	if(chinput != "y")
	{
		cout << "Closing drivers" << endl;
		if(dd_RA) {delete dd_RA; dd_RA=0; }
		if(dd_LA) {delete dd_LA; dd_LA=0; }
		if(dd_T) {delete dd_T; dd_T=0;}
		return 0;		
	}
	
	// set the normal position mode
	for(int j=0; j<nJointsArm; j++) 
	{
		ictrl_LA->setControlMode(j,VOCAB_CM_POSITION);
		ictrl_RA->setControlMode(j,VOCAB_CM_POSITION);
	}
	for(int j=0; j<nJointsTorso; j++) 
		ictrl_T->setControlMode(j,VOCAB_CM_POSITION);
		
    cout<<" Moving torso "<<endl;
    pos_T->positionMove(command_T.data());
    Time::delay(1.0);
    cout<<" Moving right arm "<<endl;
    pos_RA->positionMove(command_RA.data());
    Time::delay(1.0);
    cout<<" Moving left arm "<<endl;
    pos_LA->positionMove(command_LA.data());
    Time::delay(1.0);
    
    cout<<" Starting movement ? (y/n) ";
	cin >> chinput; 
	cout<<endl;
	
	if(chinput != "y")
	{
		cout << "Closing drivers" << endl;
		if(dd_RA) {delete dd_RA; dd_RA=0; }
		if(dd_LA) {delete dd_LA; dd_LA=0; }
		if(dd_T) {delete dd_T; dd_T=0;}
		return 0;		
	}
	
	//FIX increase elbow velocity (to check)
	//command_RA[3] = 10.0;
	//command_LA[3] = 10.0;
	
	//---------------  2) play trajectory --------------
	
	cout<<"******  MOVING! ****** "<<endl;
	
	Time::delay(1.0);
	bool cmode=true;
	bool notpossible=false;

/*	//debug
	//first set the direct position mode
	for(int j=0; j<nJointsArm; j++) 
	{
		cmode=ictrl_LA->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
		
		if(cmode==false)
		{
			ictrl_LA->setControlMode(j,VOCAB_CM_POSITION);
			cout<<"Left arm: joint "<<j<<" cannot change direct position"<<endl;
			notpossible=true;
		}
				
		cmode=ictrl_RA->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
		
		if(cmode==false)
		{
			ictrl_RA->setControlMode(j,VOCAB_CM_POSITION);
			cout<<"Right arm: joint "<<j<<" cannot change direct position"<<endl;
			notpossible=true;
		}
			
	}
	for(int j=0; j<nJointsTorso; j++) 
	{
			cmode=ictrl_T->setControlMode(j,VOCAB_CM_POSITION_DIRECT);	
			
			if(cmode==false)
			{
				ictrl_T->setControlMode(j,VOCAB_CM_POSITION);
				cout<<"Torso: joint "<<j<<" cannot change direct position"<<endl;
				notpossible=true;
			}
			
	}
		
		//// if there is errors in the direct mode, do not play the trajectory
		
		if(notpossible == true)
		{
			cout << "Closing drivers" << endl;
			if(dd_RA) {delete dd_RA; dd_RA=0; }
			if(dd_LA) {delete dd_LA; dd_LA=0; }
			if(dd_T) {delete dd_T; dd_T=0;}
			return 0;		
		}
		else
		{
			cout<<"**** direct position possible! ****"<<endl;
		}
	*/
		
		//// ok you can play
	/* //debug
	for(int t=startingPoint; t<nbIter; t++)
	{
		for(i=0; i<nJointsArm; i++) command_RA[i] = q_RA[t][i];
		for(i=0; i<nJointsArm; i++) command_LA[i] = q_LA[t][i];
		for(i=0; i<nJointsTorso; i++) command_T[i] = q_T[t][i];
		
		jointLimitsViolations = safety_check(command_RA, command_LA, command_T);
		totalJointsLimitsViolations += jointLimitsViolations;
		
		if(verbosity>=1)   printf ("Moving : \r%d / %d  - violating %d", t, nbIter, jointLimitsViolations);
    	
		pos_T->positionMove(command_T.data());
		pos_RA->positionMove(command_RA.data());
		pos_LA->positionMove(command_LA.data());
    
		//posd_T->setPositions(command_T.data());
		//posd_RA->setPositions(command_RA.data());
		//posd_LA->setPositions(command_LA.data());
	
		//10ms
		Time::delay(0.01);
		//1ms 
		//Time::delay(0.001);
		
	}*/
	
		for(int t=startingPoint; t<nbIter; t+=6)
	{
		for(i=0; i<nJointsArm; i++) command_RA[i] = q_RA[t][i];
		for(i=0; i<nJointsArm; i++) command_LA[i] = q_LA[t][i];
		for(i=0; i<nJointsTorso; i++) command_T[i] = q_T[t][i];
		
		jointLimitsViolations = safety_check(command_RA, command_LA, command_T);
		totalJointsLimitsViolations += jointLimitsViolations;
		
		if(verbosity>=1)   printf ("Moving : \r%d / %d  - violating %d", t, nbIter, jointLimitsViolations);
    	
		pos_T->positionMove(command_T.data());
		pos_RA->positionMove(command_RA.data());
		pos_LA->positionMove(command_LA.data());
    
		//debug
		//posd_T->setPositions(command_T.data());
		//posd_RA->setPositions(command_RA.data());
		//posd_LA->setPositions(command_LA.data());
	
		//10ms
		Time::delay(0.01);
		//1ms 
		//Time::delay(0.001);
		
	}
	
	Time::delay(1.0);
	
	cout<<"\n******  FINISHED! ****** "<<endl
		<<"\nYou violated the joints limits x"<<totalJointsLimitsViolations<<" times"<<endl;;

/*	//debug
	//go back to a normal position mode
	for(int j=0; j<nJointsArm; j++) 
	{
		ictrl_LA->setControlMode(j,VOCAB_CM_POSITION);
		ictrl_RA->setControlMode(j,VOCAB_CM_POSITION);
	}
	for(int j=0; j<nJointsTorso; j++) 
		ictrl_T->setControlMode(j,VOCAB_CM_POSITION);	
*/
	
	
	//---------------  CLOSING --------------


	if(verbosity>=1) cout << "Closing drivers" << endl;

	if(dd_RA) {delete dd_RA; dd_RA=0; }
	if(dd_LA) {delete dd_LA; dd_LA=0; }
	if(dd_T) {delete dd_T; dd_T=0;}

	return 0;
}
	

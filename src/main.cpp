
/************************************************************************
* \brief: NeoPhysIx ultra fast 3D-physics simulation           *
*																		*
* (c) copyright by Jörn Fischer											*
*                                                                       *
*        ***  ITS PROHIBITED TO DISTRIBUTE THIS SOURCE!  ***            *
*        ***************************************************            *
*																		* 
* @autor: Prof.Dr.Jörn Fischer											*
* @email: j.fischer@hs-mannheim.de										*
*                                                                       *
* @file : main.cpp                                                      *
*************************************************************************/
#include <stdio.h>
#include <time.h>

#include <windows.h>
#include <gl/gl.h>
#include "rigidBody.h"

/************************************************************************
TODO:
Implement ODE_graphical interface
Implement ODE_box__cappedCylinder__Joints etc.

Implement Infinite World (generation and draw Function)

Optional: Menue


************************************************************************/
// graphics painted all GFX_STEP frames
int GFX_STEP = 1;

// --- Physics simulation files --- //
#include "GlobalDefs.h"

#include "ObjectDefsWheeledRobot.h"
#include "ObjectDefsHumanoid.h"
#include "ObjectDefs6LeggedRobot.h"
#include "ObjectDefs4LeggedRobot.h"
#include "ObjectDefsQuadruped.h"

#include "Math.h"

double landscape[MAX_WORLD+1][MAX_WORLD+1];

#include "Environment.h"
environment world;

#include "PhysicsEngine.h"

#if (ROBOT==LEGGED6)
   SixLeggedRobot robot[NUM_OF_ROBOTS];
#endif
#if (ROBOT==WHEELED) 
	WheeledRobot robot[NUM_OF_ROBOTS]; 
#endif
#if (ROBOT==LEGGED)
   FourLeggedRobot robot[NUM_OF_ROBOTS]; 
#endif
#if (ROBOT==HUMANOID)
   Humanoid robot[NUM_OF_ROBOTS]; 
#endif
#if (ROBOT==QUADRUPED)
   FourLeggedComparison robot[NUM_OF_ROBOTS]; 
#endif

physics NeoPhysIx;

#include "NeuralNetwork.h"

extern int LIFETIME;
#include "Evolution.h"

FLOAT_32 clkTime;

unsigned char inkey;
int mousex,mousey,mousekey;
float colorR,colorG,colorB;

extern char letter[256][15][11];
#include "Graphics.h"

void evolve(int &geneNum, FLOAT_32 &bestfitness, evolution evo, int &generation, physics NeoPhysIx)
{
	/*
	evo.setFitness(geneNum,sqrt(NeoPhysIx.CM[0]*NeoPhysIx.CM[0]+NeoPhysIx.CM[2]*NeoPhysIx.CM[2]));
	if (bestfitness < NeoPhysIx.CM[1]*sqrt(NeoPhysIx.CM[0]*NeoPhysIx.CM[0]+NeoPhysIx.CM[2]*NeoPhysIx.CM[2])){
		bestfitness = NeoPhysIx.CM[1]*sqrt(NeoPhysIx.CM[0]*NeoPhysIx.CM[0]+NeoPhysIx.CM[2]*NeoPhysIx.CM[2]);
	}
	geneNum++;
	if (geneNum == NUM_OF_INDIVIDUUMS){
		evo.sort();
		evo.copyBest();
		evo.Mutate();
		generation++;
	}
	geneNum = geneNum % NUM_OF_INDIVIDUUMS;

	evo.net.initNeurons();
	evo.createNetwork(geneNum); // first gene -> network
	*/
}
// ---------------------------------------------------------------------
// --- Main ------------------------------------------------------------
// ---------------------------------------------------------------------
int myMain()
{
	//static SixLeggedRobot robot[NUM_OF_ROBOTS];
	//static FourLeggedComparison robot[NUM_OF_ROBOTS];
	robot[0].CM[0]=0;
	robot[0].CM[1]=0;
	robot[0].CM[2]=0;
	MSG      msg; // Windows messages
	int counter = 0;
	int globalCounter=0;
	int geneNum=0;
	int generation=0;
	FLOAT_32 bestfitness=0;
	clock_t beginTimer, endTimer;             //initialize Begin and End for the timer
//	FLOAT_32 input[100];
	char text[1000];

	evolution evo;
	evo.net.initNeurons();
	evo.createNetwork(0); // first gene -> 
	initViewPoint();

	//drawEnvironment();

    // Event loop
	while(1){
        if (PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE) == TRUE){
            if (!GetMessage(&msg, NULL, 0, 0)) 
				return TRUE;

			evaluateMessages(msg);
			mouseAction();
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

		sprintf(text," RobotHours = %f\nCPU time   = %f s for %d steps\nIndividuum = %d\nGeneration = %d\nbestFitness = %f\nx=%d\ny=%d pos=%f",globalCounter/360000.0,clkTime,GFX_STEP,geneNum,generation,bestfitness,mousex,mousey,sqrt(robot[0].CM[0]*robot[0].CM[0]+robot[0].CM[1]*robot[0].CM[1]+robot[0].CM[2]*robot[0].CM[2]));
		Render(robot,NUM_OF_ROBOTS,text);	
	
#if ROBOT==HUMANOID || ROBOT==LEGGED
		if (counter > LIFETIME){
			counter=0;
//			evolve(geneNum,bestfitness,evo,generation,NeoPhysIx);
//			NeoPhysIx.init();
		}
#endif
		beginTimer = clock() * CLK_TCK;      // start the timer 

		if (inkey == 'X'){
			initViewPoint();
		}
		if (inkey == 187 /*&& GFX_STEP<10000*/){ //+
			GFX_STEP *=10;
			inkey=0;
		}
		if (inkey == 189 && GFX_STEP>9.999){ //-
			GFX_STEP/=10;
			inkey=0;
		}

		for (int t=0; t<GFX_STEP; t++){
			NeoPhysIx.simulateStep(robot, NUM_OF_ROBOTS);
			counter++;
			globalCounter++;
		}

		endTimer = clock() * CLK_TCK;        // stop the timer
		
		clkTime= (double)(endTimer - beginTimer)/1000000.0;
	//	cout << "clockTime:" << (double)(endTimer - beginTimer)/1000000.0 <<"\nSimTime  :"<<time << "\nFactor:"<<time/((double)(endTimer - beginTimer)/1000000.0)<< endl;
	}

	return 0;
}

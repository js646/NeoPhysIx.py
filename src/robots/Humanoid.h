/************************************************************************
* \brief: NeoPhysIx - ultra fast 3D-physics simulation                  *
*																		*
* (c) copyright by Jörn Fischer											*
*                                                                       *
*        ***  ITS PROHIBITED TO DISTRIBUTE THIS SOURCE!  ***            *
*        ***************************************************            *
*																		* 
* @autor: Prof.Dr.Jörn Fischer											*
* @email: j.fischer@hs-mannheim.de										*
*                                                                       *
* @file : ObjectDefsHumanoid.h                                      *
*************************************************************************/
#include <cmath>
#include <ctime>
#include "../GlobalDefs.h"
#include "../PhysicsEngine.h"
#include "../Robot.h"

#include <stdio.h>

class Humanoid : public Robot
{
public:
	int numOfAngles;

	bodyID idBackMotor;
	bodyID idFootL;
	bodyID idFootR;
	bodyID idFootMotor1L;
	bodyID idFootMotor1R;
	bodyID idFootMotor2L;
	bodyID idFootMotor2R;
	bodyID idKneeMotorL;
	bodyID idKneeMotorR;
	bodyID idLowerHippMotor1L;
	bodyID idLowerHippMotor1R;
	bodyID idLowerHippMotor2L;
	bodyID idLowerHippMotor2R;
	bodyID idUpperHippMotorL;
	bodyID idUpperHippMotorR;
	bodyID idLowerHeadMotor;
	bodyID idUpperHeadMotor;
	bodyID idHead;
	bodyID idShoulder1L;
	bodyID idShoulder1R;
	bodyID idShoulder2L;
	bodyID idShoulder2R;
	bodyID idEllbowL;
	bodyID idEllbowR;
	bodyID idLowerArmL;
	bodyID idLowerArmR;


	Humanoid(){// --- init robot

#define SCALE 0.1

		const FLOAT_32 motorHeight = 0.5*SCALE
			          ,motorWidth  = 0.3*SCALE
	  		          ,motorDepth  = 0.35*SCALE
					  ,motorWeight = 0.8*SCALE
					  ,headMeasure = 0.5*SCALE
					  ,footHeight  = 0.1*SCALE
					  ,footWidth   = 0.9*SCALE
					  ,footDepth   = 1.6*SCALE
					  ,footWeight  = 0.8*SCALE;
		FLOAT_32 x,y,z;
		FLOAT_32 lr = 0.4*SCALE;
		FLOAT_32 footMototrHeight = 0.45*SCALE;
		FLOAT_32 kneeHeight = 1.45*SCALE;
		FLOAT_32 lowerHippHeight = 2.4*SCALE;
		FLOAT_32 upperHippHeight = 3.0*SCALE;
		FLOAT_32 backHeight = 3.5*SCALE;
		FLOAT_32 lowerHeadMotorHeight = 4.45*SCALE;
		FLOAT_32 upperHeadMotorHeight = 4.95*SCALE;
		FLOAT_32 headHeight = 5.7*SCALE;
		FLOAT_32 shoulderHeight = 4.3*SCALE;
		FLOAT_32 ellBowHeight = 3.4*SCALE;

		// --- idBackMotor
		x=0*SCALE; y=backHeight; z=0*SCALE;              idBackMotor        = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);

		// --- idFoot
		x=(lr+0.2*SCALE); y=0*SCALE; z=0.25*SCALE;   idFootL            = createBox(footWidth, footHeight, footDepth,x,y,z, footWeight, 0.1,0.4,0.1);
		x=(-lr-0.2*SCALE); y=0*SCALE; z=0.25*SCALE;	 idFootR            = createBox(footWidth, footHeight, footDepth,x,y,z, footWeight, 0.1,0.4,0.1);
		// --- idFootMotor1
		x=+(lr); y=footMototrHeight; z=0;      idFootMotor1L      = createBox(motorDepth, motorHeight, motorWidth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=(-lr); y=footMototrHeight; z=0;	     idFootMotor1R      = createBox(motorDepth, motorHeight, motorWidth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idFootMotor2
		x=+(lr*2); y=footMototrHeight; z=0;    idFootMotor2L      = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=(-lr*2); y=footMototrHeight; z=0;	 idFootMotor2R      = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idKneeMotor
		x=+lr; y=kneeHeight; z=0;            idKneeMotorL       = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=-lr; y=kneeHeight; z=0;	         idKneeMotorR       = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idLowerHippMotor1
		x=+lr; y=lowerHippHeight; z=0;       idLowerHippMotor1L = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=-lr; y=lowerHippHeight; z=0;       idLowerHippMotor1R = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idLowerHippMotor2
		x=+lr; y=lowerHippHeight; z=-0.35*SCALE;   idLowerHippMotor2L = createBox(motorDepth, motorHeight, motorWidth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=-lr; y=lowerHippHeight; z=-0.35*SCALE;	 idLowerHippMotor2R = createBox(motorDepth, motorHeight, motorWidth,x,y,z, motorWeight, 0.1,0.4,0.1);

		// --- idUpperHippMotor
		x=+lr; y=upperHippHeight; z=0;       idUpperHippMotorL  = createBox(motorHeight, motorWidth, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=-lr; y=upperHippHeight; z=0;	     idUpperHippMotorR  = createBox(motorHeight, motorWidth, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idLowerHeadMotor
		x=0; y=lowerHeadMotorHeight; z=0;	 idLowerHeadMotor   = createBox(motorDepth, motorHeight, motorWidth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idUpperHeadMotor
		x=0; y=upperHeadMotorHeight; z=0;	 idUpperHeadMotor   = createBox(motorWidth, motorDepth, motorHeight,x,y,z, motorWeight, 0.1,0.4,0.1);
		
		// --- idHead
		x=0; y=headHeight; z=0;	             idHead             = createBox(headMeasure, headMeasure, headMeasure,x,y,z, motorWeight, 0.1,0.4,0.1);

		// --- idShoulder1
		x=+lr; y=shoulderHeight; z=0;        idShoulder1L       = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=-lr; y=shoulderHeight; z=0;	     idShoulder1R       = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idShoulder2
		x=+lr*2; y=shoulderHeight; z=0;      idShoulder2L       = createBox(motorDepth, motorHeight, motorWidth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=-lr*2; y=shoulderHeight; z=0;	     idShoulder2R       = createBox(motorDepth, motorHeight, motorWidth,x,y,z, motorWeight, 0.1,0.4,0.1);
		// --- idEllbow
		x=+lr*2; y=ellBowHeight; z=0;        idEllbowL          = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=-lr*2; y=ellBowHeight; z=0;	     idEllbowR          = createBox(motorWidth, motorHeight, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);

		x=(+lr*2+motorWidth*SCALE/2.0); y=2.65*SCALE; z=0; idLowerArmL        = createBox(0.1*SCALE, 1.3*SCALE, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);
		x=(-lr*2-motorWidth*SCALE/2.0); y=2.65*SCALE; z=0; idLowerArmR        = createBox(0.1*SCALE, 1.4*SCALE, motorDepth,x,y,z, motorWeight, 0.1,0.4,0.1);


		// Joint Definitionen
		// ------------------
		jointID j[22];
		int i=0;
		// left leg
		j[i++] = createJoint(idFootL, idFootMotor1L,                +lr, 0.35*SCALE,0, 0,0,1);
		j[i++] = createJoint(idFootMotor1L, idFootMotor2L,                   0,0.35*SCALE,0, 1,0,0);	
		j[i++] = createJoint(idFootMotor2L, idKneeMotorL,                    0,1.35*SCALE,0, 1,0,0);
		j[i++] = createJoint(idKneeMotorL, idLowerHippMotor1L,                0,2.3*SCALE,0, 1,0,0);
		j[i++] = createJoint(idLowerHippMotor1L, idLowerHippMotor2L, +lr, 2.3*SCALE,0, 0,0,1);
		j[i++] = createJoint(idLowerHippMotor2L, idUpperHippMotorL,  +lr, 2.3*SCALE,0, 0,1,0);
		connectBodies(idUpperHippMotorL,idBackMotor);

		// right leg
		j[i++] = createJoint(idFootR, idFootMotor1R,                -lr, 0.35*SCALE,0, 0,0,1);
		j[i++] = createJoint(idFootMotor1R, idFootMotor2R,             0,0.35*SCALE,0, 1,0,0);	
		j[i++] = createJoint(idFootMotor2R, idKneeMotorR,              0,1.35*SCALE,0, 1,0,0);
		j[i++] = createJoint(idKneeMotorR, idLowerHippMotor1R,          0,2.3*SCALE,0, 1,0,0);
		j[i++] = createJoint(idLowerHippMotor1R, idLowerHippMotor2R, -lr, 2.3*SCALE,0, 0,0,1);
		j[i++] = createJoint(idLowerHippMotor2R, idUpperHippMotorR,  -lr, 2.3*SCALE,0, 0,1,0);
		connectBodies(idUpperHippMotorR,idBackMotor);

		// back
		j[i++] = createJoint(idBackMotor, idLowerHeadMotor,  0, 3.6*SCALE,0, 1,0,0);
		j[i++] = createJoint(idLowerHeadMotor, idUpperHeadMotor,  0, 4.9*SCALE,0, 0,1,0);
		j[i++] = createJoint(idUpperHeadMotor, idHead,  0, 4.9*SCALE,0, 1,0,0);

		connectBodies(idLowerHeadMotor,idShoulder1L);
		j[i++] = createJoint(idShoulder1L, idShoulder2L,  0, 4.4*SCALE,0, 1,0,0);
		j[i++] = createJoint(idShoulder2L, idEllbowL,  2*lr*SCALE, 4.4*SCALE,0, 0,0,1);
		j[i++] = createJoint(idEllbowL, idLowerArmL,   2*lr*SCALE, 3.3*SCALE,0, 1,0,0);

		connectBodies(idLowerHeadMotor,idShoulder1R);
		j[i++] = createJoint(idShoulder1R, idShoulder2R,  0, 4.4*SCALE,0, 1,0,0);
		j[i++] = createJoint(idShoulder2R, idEllbowR,  -2*lr*SCALE, 4.4*SCALE,0, 0,0,1);
		j[i++] = createJoint(idEllbowR, idLowerArmR,  -2*lr*SCALE, 3.3*SCALE,0,  1,0,0);
		numOfAngles = i;

		for ( int id=0; id<getMaxBodyID(); id++ ){
			// --- lifting robot up that it falls down on the ground
			moveBody(id,0,VERTICAL_OFFSET,0); 
		}	
		finalizeConstruction();
		// --- end robot definition --
	}
	/**************************************************************************************
	 @brief move function to change the angle values and read out sensors
	**************************************************************************************/
	void move(int counter)
	{
		
		for (int i=0; i<numOfAngles; i++){
			//angle[i] = 0;//(float)(cos(counter/33.0)/2.0);
		}
		int i = (int)(counter/500.0) % numOfAngles;
//		angle[i] = (float)(cos(counter/33.0)/2.0);

		angle[3] = (float)(cos(counter/33.0)/2.0);
		angle[9] = (float)(cos(counter/33.0)/2.0);

	//	angle[15] = (float)(cos(counter/33.0)/2.0);

		for (int id=0; id<=idLowerArmR; id++){
			if (groundContact(id)){                // each body might be contact sensor
				changeObjectColor(id, 1.0, 0, 0);
			}
			else{
				changeObjectColor(id,0.1, 0.4, 0.1);
			}
		}
		// Vector3D getCMorientation(); // sensor for Orientation of the center of mass
		// Vector3D getCMcoordinates(); // sensor for Coordinates of the center of mass

	}
};
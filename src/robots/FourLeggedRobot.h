/************************************************************************
* \brief: NeoPhysIx - ultra fast 3D-physics simulation                  *
*																		*
* (c) copyright by J�rn Fischer											*
*                                                                       *
*        ***  ITS PROHIBITED TO DISTRIBUTE THIS SOURCE!  ***            *
*        ***************************************************            *
*																		* 
* @autor: Prof.Dr.J�rn Fischer											*
* @email: j.fischer@hs-mannheim.de										*
*                                                                       *
* @file : ObjectDefs4LeggedRobot.h                                      *
*************************************************************************/
#include <Math.h>
#include <time.h>
#include "../GlobalDefs.h"
#include "../PhysicsEngine.h"
#include "../Robot.h"

class FourLeggedRobot : public Robot
{
public:
	FourLeggedRobot(){// --- init robot

		simplifyMode(0);

		// --- robot body
		bodyID idBox = createBox(0.5,0.1,0.3,0,0,0,0.8, 0.1,0.4,0.1);
		// --- robot legs
		bodyID idFrontLeft  = createCylinder(-0.25f,-0.05f,-0.15f,   
				              -0.25f,-0.3f ,-0.25f, 0.03f  ,0.1f, 0.8f,0.3f,0.3f);
		bodyID idFrontRight = createCylinder(-0.25f,-0.05f,+0.15f,   
			                  -0.25f,-0.3f ,+0.25f, 0.03f  ,0.1f, 0.8f,0.3f,0.3f);
		bodyID idBackLeft   = createCylinder(+0.25f,-0.05f,-0.15f,   
			                  +0.85f,-0.05f,-0.45f, 0.03f  ,0.1f, 0.8f,0.3f,0.3f);
		bodyID idBackRight  = createCylinder(+0.25f,-0.05f,+0.15f,   
			                  +0.85f,-0.05f,+0.45f, 0.03f  ,0.1f, 0.8f,0.3f,0.3f);
		// Joint Definitionen
		// ------------------
		jointID j1,j2,j3,j4;
		j1 = createJoint(idBox, idFrontLeft,  -0.25,-0.05,-0.15, 1,0,0);
		j2 = createJoint(idBox, idFrontRight, -0.25,-0.05,+0.15, 1,0,0);
		j3 = createJoint(idBox, idBackLeft,   +0.25,-0.05,-0.15, 0,1,0);
		j4 = createJoint(idBox, idBackRight,  +0.25,-0.05,+0.15, 0,1,0);

		for ( int id=0; id<getMaxBodyID(); id++ ){
			// --- lifting robot up that it falls down on the ground
			moveBody(id,0,VERTICAL_OFFSET,0); 
		}	
		finalizeConstruction();
		// --- end robot definition ---
	}
	void move(int counter)
	{	int i = 0;
		angle[i++] = (float)(sin(counter/33.0)/2.0);
		angle[i++] = (float)(sin(counter/33.0)/2.0);
		angle[i++] = (float)(cos(counter/33.0)/2.0);
		angle[i++] = (float)(cos(counter/33.0)/2.0);
	}
};
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
* @file : ObjectDefsWheeledRobot.h                                      *
*************************************************************************/
#include <Math.h>
#include <time.h>

#include "../GlobalDefs.h"
#include "../PhysicsEngine.h"
#include "../PhysicsAPI.h"
#include "../Robot.h"
#include "../RigidBody.h"

class WheeledRobot : public Robot
{
	public:
	// --- init robot
	WheeledRobot(){
		FLOAT_32 h=0.01;
		// --- robot platform base
		bodyID idPlatform = createCylinder(0,-h,0,  
			                               0,+h,0, 0.12, 0.2, 0.9,0.5,0.2);
		// --- robot wheels
		bodyID idLeftWheel = 
			createCylinder(-0.13,0,0,  -0.12,0,0,  0.05, 0.2, 0.9,0.0,0.0);
		bodyID idRightWheel = 
			createCylinder(+0.12,0,0,  +0.13,0,0, 0.05, 0.2, 0.9,0.0,0.0);

		// --- Joint Definitionen
		jointID j1,j2;
		j1 = createJoint(idPlatform, idLeftWheel,  -0.12,0,0, 1,0,0);
		j2 = createJoint(idPlatform, idRightWheel, -0.12,0,0, 1,0,0);

		for ( int id=0; id<getMaxBodyID(); id++ ){
			// --- lifting robot up that it falls down on the ground
			moveBody(id,0,VERTICAL_OFFSET,0); 
		}	
	
		setContactHeight(0.004);
		finalizeConstruction();
		// --- end robot definition ---
	}
	
	void move(int counter)
	{
		angle[0] = (float)(counter/100.0);
		angle[1] = (float)(counter/100.0);
	}
};

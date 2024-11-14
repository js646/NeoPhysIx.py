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

class FourLeggedComparison : public Robot
{
	public:

	bodyID idUpConnec[6],idUpper[6],idMiddle[6],idConnect[6],idLower[6],idFoot[6],idBump[6];
	// --- init robot 
	FourLeggedComparison(){

	#define SCALE 1

		// --- robot body
		bodyID idBox = createBox(0.2*SCALE, 0.2*SCALE, 0.2*SCALE,  0,0,0, 0.8, 0.7,0.3,0.3);
		FLOAT_32 radius = 0.08*SCALE;
		
		simplifyMode(1);

		idUpper[0]   = createCylinder(0.1*SCALE,0*SCALE,0*SCALE,           0.4*SCALE,0.4*SCALE,0*SCALE,        radius, 0.2, 0.3,0.3,0.7);
		idLower[0]   = createCylinder(0.4*SCALE,0.4*SCALE,0*SCALE,         0.9*SCALE,-0.7*SCALE,0*SCALE,        radius, 0.2, 0.3,0.3,0.7);

		idUpper[1]   = createCylinder(-0.1*SCALE,0*SCALE,0*SCALE,          -0.4*SCALE,0.4*SCALE,0*SCALE,        radius, 0.2, 0.3,0.3,0.7);
		idLower[1]   = createCylinder(-0.4*SCALE,0.4*SCALE,0*SCALE,        -0.9*SCALE,-0.7*SCALE,0*SCALE,        radius, 0.2, 0.3,0.3,0.7);

		idUpper[2]   = createCylinder(0*SCALE,0*SCALE,0.1*SCALE,            0*SCALE,0.4*SCALE,0.4*SCALE,        radius, 0.2, 0.3,0.3,0.7);
		idLower[2]   = createCylinder(0*SCALE,0.4*SCALE,0.4*SCALE,          0*SCALE,-0.7*SCALE,0.9*SCALE,        radius, 0.2, 0.3,0.3,0.7);

		idUpper[3]   = createCylinder(0*SCALE,0*SCALE,-0.1*SCALE,           0*SCALE,0.4*SCALE,-0.4*SCALE,        radius, 0.2, 0.3,0.3,0.7);
		idLower[3]   = createCylinder(0*SCALE,0.4*SCALE,-0.4*SCALE,         0*SCALE,-0.7*SCALE,-0.9*SCALE,        radius, 0.2, 0.3,0.3,0.7);


		// --- Joint Definitionen
		jointID j[8];
		j[0] = createJoint(idBox, idUpper[0],          0.1*SCALE,0,0, 0,0,1);
		j[1] = createJoint(idUpper[0], idLower[0],   0.4*SCALE,0.4*SCALE,0, 0,0,1);

		j[2] = createJoint(idBox, idUpper[1],          -0.1*SCALE,0,0, 0,0,1);
		j[3] = createJoint(idUpper[1], idLower[1],   -0.4*SCALE,0.4*SCALE,0, 0,0,1);

		j[4] = createJoint(idBox, idUpper[2],          0,0,0.1*SCALE, 1,0,0);
		j[5] = createJoint(idUpper[2], idLower[2],   0,0.4*SCALE,0.4*SCALE, 1,0,0);

		j[6] = createJoint(idBox, idUpper[3],          0,0,-0.1*SCALE, 1,0,0);
		j[7] = createJoint(idUpper[3], idLower[3],   0,0.4*SCALE,-0.4*SCALE, 1,0,0);

		for ( int id=0; id<getMaxBodyID(); id++ ){
			// --- lifting robot up that it falls down on the ground
			moveBody(id,0,VERTICAL_OFFSET,0); 
		}	

		finalizeConstruction();
		// --- end robot definition ---
	}

	void move(int counter)
	{
		int i = 0;
		angle[i++] = (float)(sin(counter/33.0)/4.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);

		angle[i++] = (float)(-sin(counter/33.0)/2.0);//(float)(bodyNum+1.0);
		angle[i++] = (float)(-cos(counter/33.0)/4.0);//(float)(bodyNum+1.0);
		angle[i++] = (float)(-cos(counter/33.0)/4.0);//(float)(bodyNum+1.0);
		angle[i++] = (float)(-cos(counter/33.0)/4.0);//(float)(bodyNum+1.0);

		// --- foot sensor colors the
		for (int t=0;t<4;t++){
			if (groundContact(idLower[t])){
				changeObjectColor(idLower[t], 1.0, 0, 0);
			}
			else{
				changeObjectColor(idLower[t],0.2,0.7,0.2);
			}
		}
	}
};

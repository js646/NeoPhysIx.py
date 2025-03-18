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
* @file : ObjectDefs4LeggedRobot.h                                      *
*************************************************************************/
#include <Math.h>
#include <time.h>
#include "../GlobalDefs.h"
#include "../PhysicsEngine.h"
#include "../Robot.h"

class SixLeggedRobot : public Robot
{
	public:

	bodyID idUpConnec[6],idUpper[6],idMiddle[6],idConnect[6],idLower[6],idFoot[6],idBump[6];
	// --- init robot 
	SixLeggedRobot(){
#define SCALE 0.4
		// --- robot body
		bodyID idBox = createBox(1.0*SCALE, 0.2*SCALE, 0.2*SCALE,  0,0,0, 0.8, 0.1,0.8,0.1);
		FLOAT_32 posX=-0.4*SCALE,posY=0,posZ=0.225*SCALE;
		FLOAT_32 length = 0.2*SCALE;
		FLOAT_32 radius = 0.025*SCALE;
		FLOAT_32 posXX[]={-0.4*SCALE,0*SCALE,0.4*SCALE};
		
		simplifyMode(0);
		posZ = 0.225*SCALE;
		for (int t=0;t<3;t++){
			posX = posXX[t];
			// --- robot legs
			idUpConnec[t]= createCylinder(posX,posY,posZ-length,               posX,posY,posZ,                   radius, 0.2, 0.1,0.8,0.1);
			idUpper[t]   = createCylinder(posX,posY-length/2.0,posZ,           posX,posY+length/2.0,posZ,        radius, 0.2, 0.1,0.8,0.1);
			idMiddle[t]  = createCylinder(posX-length/2.0,posY,posZ,           posX+length/2.0,posY,posZ,        radius, 0.2, 0.1,0.8,0.1);
			idConnect[t] = createCylinder(posX,posY,posZ,                      posX,posY,posZ+length,            radius, 0.2, 0.1,0.8,0.1);
			idLower[t]   = createCylinder(posX-length/2.0,posY,posZ+length,    posX+length/2.0,posY,posZ+length, radius, 0.2,0.1,0.8,0.1);
			idFoot[t]    = createCylinder(posX,posY,posZ+length,               posX,posY-length,posZ+length*2.0, radius/3.0, 0.05 , 0.1,0.8,0.1);
			idBump[t]    = createSphere(posX,posY-length,posZ+length*2.0,radius,0.05 ,0.2,0.7,0.2);
		}
		posZ = -0.225*SCALE;
		for (int t=0;t<3;t++){
			posX = posXX[t];
			// --- robot legs
			idUpConnec[t+3]= createCylinder(posX,posY,posZ+length,               posX,posY,posZ,                   radius, 0.2, 0.1,0.8,0.1);
			idUpper[t+3]   = createCylinder(posX,posY+length/2.0,posZ,           posX,posY-length/2.0,posZ,        radius, 0.2, 0.1,0.8,0.1);
			idMiddle[t+3]  = createCylinder(posX+length/2.0,posY,posZ,           posX-length/2.0,posY,posZ,        radius, 0.2, 0.1,0.8,0.1);
			idConnect[t+3] = createCylinder(posX,posY,posZ,                      posX,posY,posZ-length,            radius, 0.2, 0.1,0.8,0.1);
			idLower[t+3]   = createCylinder(posX+length/2.0,posY,posZ-length,    posX-length/2.0,posY,posZ-length, radius, 0.2, 0.1,0.8,0.1);
			idFoot[t+3]    = createCylinder(posX,posY,posZ-length,               posX,posY-length,posZ-length*2.0, radius/3.0, 0.05 , 0.1,0.8,0.1);
			idBump[t+3]    = createSphere(posX,posY-length,posZ-length*2.0,radius,0.05, 0.2,0.7,0.2);
		}

	
		// --- Joint Definitionen
		jointID j[18];
		posZ = 0.225*SCALE;
		for (int t=0;t<3;t++){
			posX = posXX[t];
			//connectBody();
			connectBodies(idBox,idUpConnec[t]);
			connectBodies(idBox,idUpper[t]);
			j[t*3+0] = createJoint(idBox, idMiddle[t],          posX,posY,posZ, 0,1,0);
			j[t*3+1] = createJoint(idMiddle[t], idConnect[t],   posX,posY,posZ, 1,0,0);
			connectBodies(idConnect[t],idLower[t]);
			j[t*3+2] = createJoint(idLower[t], idFoot[t],       posX,posY,posZ+length, 1,0,0);
			connectBodies(idFoot[t],idBump[t]);
		}
		posZ = -0.225*SCALE;
		for (int t=0;t<3;t++){
			posX = posXX[t];
			//connectBody();
			connectBodies(idBox,idUpConnec[t+3]);
			connectBodies(idBox,idUpper[t+3]);
			j[t*3+9] = createJoint(idBox, idMiddle[t+3],             posX,posY,posZ, 0,1,0);
			j[t*3+10] = createJoint(idMiddle[t+3], idConnect[t+3],   posX,posY,posZ, 1,0,0);
			connectBodies(idConnect[t+3],idLower[t+3]);
			j[t*3+11] = createJoint(idLower[t+3], idFoot[t+3],       posX,posY,posZ-length, 1,0,0);
			connectBodies(idFoot[t+3],idBump[t+3]);
		}	
	

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
		angle[i++] = (float)(sin(counter/33.0)/2.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
								
		angle[i++] = (float)(-sin(counter/33.0)/2.0);//(float)(bodyNum+1.0);
		angle[i++] = (float)(-cos(counter/33.0)/4.0);//(float)(bodyNum+1.0);
		angle[i++] = (float)(-cos(counter/33.0)/4.0);//(float)(bodyNum+1.0);
	
		angle[i++] = (float)(sin(counter/33.0)/2.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
	
	
		angle[i++] = (float)(sin(counter/33.0)/2.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
	
		angle[i++] = (float)(-sin(counter/33.0)/2.0);
		angle[i++] = (float)(-cos(counter/33.0)/4.0);
		angle[i++] = (float)(-cos(counter/33.0)/4.0);
	
		angle[i++] = (float)(sin(counter/33.0)/2.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);
		angle[i++] = (float)(cos(counter/33.0)/4.0);

		// --- foot sensor colors the 
		for (int t=0;t<6;t++){
			if (groundContact(idBump[t])){
				changeObjectColor(idBump[t], 1.0, 0, 0);
			}
			else{
				changeObjectColor(idBump[t],0.2,0.7,0.2);
			}
		}
	}
};

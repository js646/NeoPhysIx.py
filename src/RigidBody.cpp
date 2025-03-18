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
* @file : RigidBody.h                                                   *
*************************************************************************/
#include <cmath>
#include "RigidBody.h"
#include "Math.h"

RigidBody::RigidBody()
{
	maxJoints = 0;
	maxPoints = 0;
	
	MeshCounter = 0;
	
	bodyIDcounter = 0;
	bodyIDcounter = 0;
	jointIDcounter = 0;
	connectionIDcounter = 0;
	simplify_Mode = 0;

	for (int t=0;t<MAX_JOINTS;t++){
		joint[t].angle=0;
		joint[t].axisPoint1=0;
		joint[t].axisPoint2=0;
		for (int i=0; i<MAX_BODIES; i++){
			joint[t].bodyList[i] = 0;
		}
		body[t].endPointNum=0;
		body[t].startPointNum=0;
		body[t].endMeshNum=0;
		body[t].startMeshNum=0;
	}

	for (int i=0; i<MAX_BODIES; i++){
		massPoint[i].bump = false;
	}
	CMVeloVec[DIM_Y] = 0;
	friction = 1.0;
	calculateCenterOfMass();
	bounce=-1;
	alpha=0;

	for (int t=0;t<3;t++){
		CM[t]=0;                           // Center Of Mass
		CM_Old[t]=0;
		CMVeloVec[t] = 0; // Center Of Mass Velocity
		omega[t] = 0;     // Winkel
		for (int i=0;i<3;i++){
			InertiaTensor[t][i]=0;
		}
	}
	ANGLE_STEP = 7;
	contactHeight = 0.005;
	for (int t=0;t<MAX_MESH;t++){
		triMesh[t].color[0]=200;
		triMesh[t].color[1]=0;
		triMesh[t].color[2]=0;
	}
	for (int t=0;t<3;t++){
		CMforce[t]=0;
	}
}
void RigidBody::simplifyMode(int simp)
{
	simplify_Mode = simp;
}
	
// calculates center of mass
void RigidBody::calculateCenterOfMass(){
	int points=0;
	CM[DIM_X] = 0.0;
	CM[DIM_Y] = 0.0;
	CM[DIM_Z] = 0.0;
	for(int i = 0; i < maxPoints; i++){
		if (massPoint[i].colidable){
			CM[DIM_X]+= massPoint[i].x;
			CM[DIM_Y]+= massPoint[i].y;
			CM[DIM_Z]+= massPoint[i].z;
			points++;
		}
	}
	CM[DIM_X] /= points;
	CM[DIM_Y] /= points;
	CM[DIM_Z] /= points;
}


void RigidBody::calculateMomentumOfInertia()
{
	mInertia = 0;
	for (int t=0 ; t<maxPoints ;t++){
		mInertia += massPoint[t].mass * ((massPoint[t].x-CM[DIM_X]) * (massPoint[t].x-CM[DIM_X]) +
								         (massPoint[t].y-CM[DIM_Y]) * (massPoint[t].y-CM[DIM_Y]) +
								         (massPoint[t].z-CM[DIM_Z]) * (massPoint[t].z-CM[DIM_Z])   );
	}
	for(int i = 0; i < maxPoints; i++){
		CMmass += massPoint[i].mass;
	}
	/*
	// --- 3D tensor calculation:
	FLOAT_32 rMatrix[3][3];

	for (int i=0;i<3;i++){
		for (int t=0;t<3;t++){
			InertiaTensor[t][i]=0;
		}
	}
	for (int t=0 ; t<maxPoints ;t++){
		rMatrix[0][0]=0;
		rMatrix[0][1]=-(massPoint[t].z-CM[DIM_Z]);
		rMatrix[0][2]=(massPoint[t].y-CM[DIM_Y]);
		rMatrix[1][0]=(massPoint[t].z-CM[DIM_Z]);
		rMatrix[1][1]=0;
		rMatrix[1][2]=-(massPoint[t].x-CM[DIM_X]);
		rMatrix[2][0]=-(massPoint[t].y-CM[DIM_Y]);
		rMatrix[2][1]=(massPoint[t].x-CM[DIM_X]);
		rMatrix[2][2]=0;

		InertiaTensor[0][0] += -massPoint[t].mass * (rMatrix[0][0]*rMatrix[0][0] + rMatrix[0][1]*rMatrix[1][0] + rMatrix[0][2]*rMatrix[2][0]);
		InertiaTensor[0][1] += -massPoint[t].mass * (rMatrix[0][0]*rMatrix[0][1] + rMatrix[0][1]*rMatrix[1][1] + rMatrix[0][2]*rMatrix[2][1]);
		InertiaTensor[0][2] += -massPoint[t].mass * (rMatrix[0][0]*rMatrix[0][2] + rMatrix[0][1]*rMatrix[1][2] + rMatrix[0][2]*rMatrix[2][2]);

		InertiaTensor[1][0] += -massPoint[t].mass * (rMatrix[1][0]*rMatrix[0][0] + rMatrix[1][1]*rMatrix[1][0] + rMatrix[1][2]*rMatrix[2][0]);
		InertiaTensor[1][1] += -massPoint[t].mass * (rMatrix[1][0]*rMatrix[0][1] + rMatrix[1][1]*rMatrix[1][1] + rMatrix[1][2]*rMatrix[2][1]);
		InertiaTensor[1][2] += -massPoint[t].mass * (rMatrix[1][0]*rMatrix[0][2] + rMatrix[1][1]*rMatrix[1][2] + rMatrix[1][2]*rMatrix[2][2]);

		InertiaTensor[2][0] += -massPoint[t].mass * (rMatrix[2][0]*rMatrix[0][0] + rMatrix[2][1]*rMatrix[1][0] + rMatrix[2][2]*rMatrix[2][0]);
		InertiaTensor[2][1] += -massPoint[t].mass * (rMatrix[2][0]*rMatrix[0][1] + rMatrix[2][1]*rMatrix[1][1] + rMatrix[2][2]*rMatrix[2][1]);
		InertiaTensor[2][2] += -massPoint[t].mass * (rMatrix[2][0]*rMatrix[0][2] + rMatrix[2][1]*rMatrix[1][2] + rMatrix[2][2]*rMatrix[2][2]);
	}
	// now calculate inverse:
	// solve 3 linear equations!!!
	


	
*/



}

// ### START API functions ######################################

int RigidBody::createVirtualPoint(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z)
{
	 massPoint[maxPoints].x = x;
	 massPoint[maxPoints].y = y;
	 massPoint[maxPoints].z = z;
	 massPoint[maxPoints].xCopy = x;
	 massPoint[maxPoints].yCopy = y;
	 massPoint[maxPoints].zCopy = z;
	 massPoint[maxPoints].mass = 0;
	 massPoint[maxPoints].colidable = false;
	 maxPoints++;
	 return maxPoints-1;
}

void RigidBody::createMassPoint(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 mass)
{
	 massPoint[maxPoints].x = x;
	 massPoint[maxPoints].y = y;
	 massPoint[maxPoints].z = z;
	 massPoint[maxPoints].xCopy = x;
	 massPoint[maxPoints].yCopy = y;
	 massPoint[maxPoints].zCopy = z;
	 massPoint[maxPoints].mass = mass;
	 massPoint[maxPoints].colidable = true;
	 
	 maxPoints++;

}
bodyID RigidBody::createPoint(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 mass, FLOAT_32 red, FLOAT_32 green, FLOAT_32 blue)
{
	 int MeshStart = MeshCounter;
	 body[bodyIDcounter].startMeshNum = MeshCounter;
	 triMesh[MeshCounter].pointNum[0] = maxPoints;
	 triMesh[MeshCounter].pointNum[1] = maxPoints;
	 triMesh[MeshCounter].pointNum[2] = maxPoints;
	 MeshCounter++;
	 body[bodyIDcounter].endMeshNum = MeshCounter;

	 for (int t=MeshStart;t<MeshCounter;t++){
		triMesh[t].color[0]=red;
		triMesh[t].color[1]=green;
		triMesh[t].color[2]=blue;
	 }

	 body[bodyIDcounter].startPointNum = maxPoints;

	 indexFirstFromID[bodyIDcounter]=maxPoints;
	 createMassPoint(x,y,z,mass);
	 indexLastFromID[bodyIDcounter]=maxPoints-1;

	 body[bodyIDcounter].endPointNum = maxPoints-1;
	 return bodyIDcounter++;
}
bodyID RigidBody::createCylinder(FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1,FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2, FLOAT_32 radius, FLOAT_32 mass, FLOAT_32 red, FLOAT_32 green, FLOAT_32 blue)
{
	 Vector3D cylVec, randomVec, orthogVec1, orthogVec2, interVec1, interVec2;
	 FLOAT_32 norm;
	int MeshStart=MeshCounter;

	if (simplify_Mode==0){ // cylinder with simlifyMode 0 = real cylinder
		body[bodyIDcounter].startMeshNum = MeshCounter;
		 // plate front
		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 3;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 4;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 5;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 5;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 6;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 6;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 7;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 7;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 8;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 8;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 9;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 9;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
		MeshCounter++;

		// --- plate back
		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 10;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 11;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 11;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 12;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 12;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 13;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 13;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 14;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 14;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 15;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 15;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 16;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 16;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 17;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 17;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 10;
		MeshCounter++;

		// --- cylinder itself
		triMesh[MeshCounter].pointNum[0] = maxPoints + 10;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 11;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 3;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 4;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 12;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 5;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 13;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 5;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 6;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 14;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 6;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 7;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 15;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 7;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 8;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 16;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 8;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 9;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 17;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 9;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
		MeshCounter++;
		// ---
		triMesh[MeshCounter].pointNum[0] = maxPoints + 3;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 10;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 11;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 4;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 11;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 12;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 5;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 12;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 13;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 6;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 13;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 14;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 7;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 14;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 15;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 8;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 15;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 16;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 9;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 16;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 17;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 2;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 17;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 10;
		MeshCounter++;
		body[bodyIDcounter].endMeshNum = MeshCounter;

		for (int t=MeshStart;t<MeshCounter;t++){
			triMesh[t].color[0]=red;
			triMesh[t].color[1]=green;
			triMesh[t].color[2]=blue;
		 }

		 body[bodyIDcounter].startPointNum = maxPoints;

		 indexFirstFromID[bodyIDcounter]=maxPoints;
	 
		 cylVec.x = x2-x1;
		 cylVec.y = y2-y1;
		 cylVec.z = z2-z1;
		 norm = sqrt(cylVec.x*cylVec.x + cylVec.y*cylVec.y + cylVec.z*cylVec.z);
		 cylVec.x/=norm;
		 cylVec.y/=norm;
		 cylVec.z/=norm;

		 randomVec.x = 0.832174;
		 randomVec.y = 0.432134;
		 randomVec.z = 1.243665;
		 norm = sqrt(randomVec.x*randomVec.x + randomVec.y*randomVec.y + randomVec.z*randomVec.z);
		 randomVec.x/=norm;
		 randomVec.y/=norm;
		 randomVec.z/=norm;

		 orthogVec1 = crossProduct(cylVec, randomVec);
		 orthogVec2 = crossProduct(cylVec, orthogVec1);

		 interVec1.x = (orthogVec1.x + orthogVec2.x)/sqrt(2.0)*radius; 
		 interVec1.y = (orthogVec1.y + orthogVec2.y)/sqrt(2.0)*radius;
		 interVec1.z = (orthogVec1.z + orthogVec2.z)/sqrt(2.0)*radius;

		 interVec2.x = (orthogVec1.x - orthogVec2.x)/sqrt(2.0)*radius; 
		 interVec2.y = (orthogVec1.y - orthogVec2.y)/sqrt(2.0)*radius;
		 interVec2.z = (orthogVec1.z - orthogVec2.z)/sqrt(2.0)*radius;

		 orthogVec1.x*=radius;
		 orthogVec1.y*=radius;
		 orthogVec1.z*=radius;

		 orthogVec2.x*=radius;
		 orthogVec2.y*=radius;
		 orthogVec2.z*=radius;

		 createMassPoint(x1, y1, z1, mass/16.0);
		 createMassPoint(x2, y2, z2, mass/16.0);

		 createMassPoint(x1+orthogVec1.x, y1+orthogVec1.y, z1+orthogVec1.z, mass/16.0);
		 createMassPoint(x1+interVec1.x, y1+interVec1.y, z1+interVec1.z, mass/16.0);	
		 createMassPoint(x1+orthogVec2.x, y1+orthogVec2.y, z1+orthogVec2.z, mass/16.0);
		 createMassPoint(x1-interVec2.x, y1-interVec2.y, z1-interVec2.z, mass/16.0);
		 createMassPoint(x1-orthogVec1.x, y1-orthogVec1.y, z1-orthogVec1.z, mass/16.0);
		 createMassPoint(x1-interVec1.x, y1-interVec1.y, z1-interVec1.z, mass/16.0);
		 createMassPoint(x1-orthogVec2.x, y1-orthogVec2.y, z1-orthogVec2.z, mass/16.0);	 
		 createMassPoint(x1+interVec2.x, y1+interVec2.y, z1+interVec2.z, mass/16.0);
	 
		 createMassPoint(x2+orthogVec1.x, y2+orthogVec1.y, z2+orthogVec1.z, mass/16.0);
		 createMassPoint(x2+interVec1.x, y2+interVec1.y, z2+interVec1.z, mass/16.0);
		 createMassPoint(x2+orthogVec2.x, y2+orthogVec2.y, z2+orthogVec2.z, mass/16.0);
		 createMassPoint(x2-interVec2.x, y2-interVec2.y, z2-interVec2.z, mass/16.0);
		 createMassPoint(x2-orthogVec1.x, y2-orthogVec1.y, z2-orthogVec1.z, mass/16.0);
		 createMassPoint(x2-interVec1.x, y2-interVec1.y, z2-interVec1.z, mass/16.0);
		 createMassPoint(x2-orthogVec2.x, y2-orthogVec2.y, z2-orthogVec2.z, mass/16.0);	 
		 createMassPoint(x2+interVec2.x, y2+interVec2.y, z2+interVec2.z, mass/16.0);

		 indexLastFromID[bodyIDcounter]  = maxPoints-1;
		 body[bodyIDcounter].endPointNum = maxPoints-1;
	 	 return bodyIDcounter++;
	}
	if (simplify_Mode == 1){	// cylinder with simlifyMode 1	
		
		body[bodyIDcounter].startMeshNum = MeshCounter;
				
		// top and bottom plane
		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
		MeshCounter++;
		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0+4;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 1+4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2+4;
		MeshCounter++;
		triMesh[MeshCounter].pointNum[0] = maxPoints + 0+4;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2+4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3+4;
		MeshCounter++;
			
		// side planes

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 0+4;
		MeshCounter++;
				
		triMesh[MeshCounter].pointNum[0] = maxPoints + 0+4;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 1+4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 1;
		MeshCounter++;		

		triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 1+4;
		MeshCounter++;
				
		triMesh[MeshCounter].pointNum[0] = maxPoints + 1+4;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2+4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
		MeshCounter++;				
			
		// ---
			
		triMesh[MeshCounter].pointNum[0] = maxPoints + 2;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 3;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2+4;
		MeshCounter++;
				
		triMesh[MeshCounter].pointNum[0] = maxPoints + 2+4;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 3+4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3;
		MeshCounter++;				

		// ---
			
		triMesh[MeshCounter].pointNum[0] = maxPoints + 3;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3+4;
		MeshCounter++;
				
		triMesh[MeshCounter].pointNum[0] = maxPoints + 3+4;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 0+4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 0;
		MeshCounter++;				
		body[bodyIDcounter].endMeshNum = MeshCounter;

		for (int t=MeshStart;t<MeshCounter;t++){
			triMesh[t].color[0]=red;
			triMesh[t].color[1]=green;
			triMesh[t].color[2]=blue;
			}

			body[bodyIDcounter].startPointNum = maxPoints;

			indexFirstFromID[bodyIDcounter]=maxPoints;
	 
			cylVec.x = x2-x1;
			cylVec.y = y2-y1;
			cylVec.z = z2-z1;
			norm = sqrt(cylVec.x*cylVec.x + cylVec.y*cylVec.y + cylVec.z*cylVec.z);
			cylVec.x/=norm;
			cylVec.y/=norm;
			cylVec.z/=norm;

			randomVec.x = 0.832174;
			randomVec.y = 0.432134;
			randomVec.z = 1.243665;
			norm = sqrt(randomVec.x*randomVec.x + randomVec.y*randomVec.y + randomVec.z*randomVec.z);
			randomVec.x/=norm;
			randomVec.y/=norm;
			randomVec.z/=norm;

			orthogVec1 = crossProduct(cylVec, randomVec);
			orthogVec2 = crossProduct(cylVec, orthogVec1);

			interVec1.x = (orthogVec1.x + orthogVec2.x)/sqrt(2.0)*radius; 
			interVec1.y = (orthogVec1.y + orthogVec2.y)/sqrt(2.0)*radius;
			interVec1.z = (orthogVec1.z + orthogVec2.z)/sqrt(2.0)*radius;

			interVec2.x = (orthogVec1.x - orthogVec2.x)/sqrt(2.0)*radius; 
			interVec2.y = (orthogVec1.y - orthogVec2.y)/sqrt(2.0)*radius;
			interVec2.z = (orthogVec1.z - orthogVec2.z)/sqrt(2.0)*radius;

			orthogVec1.x*=radius;
			orthogVec1.y*=radius;
			orthogVec1.z*=radius;

			orthogVec2.x*=radius;
			orthogVec2.y*=radius;
			orthogVec2.z*=radius;
		createMassPoint(x1 + orthogVec1.x, y1 + orthogVec1.y, z1 + orthogVec1.z, mass / 16.0f);
		createMassPoint(x1 + orthogVec2.x, y1 + orthogVec2.y, z1 + orthogVec2.z, mass / 16.0f);
		createMassPoint(x1 - orthogVec1.x, y1 - orthogVec1.y, z1 - orthogVec1.z, mass / 16.0f);
		createMassPoint(x1 - orthogVec2.x, y1 - orthogVec2.y, z1 - orthogVec2.z, mass / 16.0f);
	
		createMassPoint(x2 + orthogVec1.x, y2 + orthogVec1.y, z2 + orthogVec1.z, mass / 16.0f);
		createMassPoint(x2 + orthogVec2.x, y2 + orthogVec2.y, z2 + orthogVec2.z, mass / 16.0f);
		createMassPoint(x2 - orthogVec1.x, y2 - orthogVec1.y, z2 - orthogVec1.z, mass / 16.0f);
		createMassPoint(x2 - orthogVec2.x, y2 - orthogVec2.y, z2 - orthogVec2.z, mass / 16.0f);
	
		indexLastFromID[bodyIDcounter]  = maxPoints-1;
		body[bodyIDcounter].endPointNum = maxPoints-1;
	
		return bodyIDcounter++;
			
	}
	if (simplify_Mode == 2){ // cylinder with simlifyMode 2
		body[bodyIDcounter].startMeshNum = MeshCounter;
		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 1;
		MeshCounter++;
		body[bodyIDcounter].endMeshNum = MeshCounter;

		for (int t=MeshStart;t<MeshCounter;t++){
			triMesh[t].color[0]=red;
			triMesh[t].color[1]=green;
			triMesh[t].color[2]=blue;
		}
		body[bodyIDcounter].startPointNum = maxPoints;
		indexFirstFromID[bodyIDcounter]   = maxPoints;

		createMassPoint(x1, y1, z1, mass/2.0);
		createMassPoint(x2, y2, z2, mass/2.0);

		indexLastFromID[bodyIDcounter]  = maxPoints-1;
		body[bodyIDcounter].endPointNum = maxPoints-1;
	
		return bodyIDcounter++;
	}
	if (simplify_Mode >= 3){ // cylinder with simlifyMode 3
		return createPoint((x1+x2)/2.0, (y1+x2)/2.0, (z1+z2)/2.0, mass/2.0);
	}
}

bodyID RigidBody::createBox(FLOAT_32 length, FLOAT_32 width, FLOAT_32 height, FLOAT_32 x, FLOAT_32 y, FLOAT_32 z, FLOAT_32 mass, FLOAT_32 red, FLOAT_32 green, FLOAT_32 blue)
{
	int MeshStart=MeshCounter;
	body[bodyIDcounter].startMeshNum = MeshCounter;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 4;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
	MeshCounter++;

	//top box
	triMesh[MeshCounter].pointNum[0] = maxPoints + 3;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 6;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 7;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 6;
	MeshCounter++;

	//front box
	triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 3;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 6;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 3;
	MeshCounter++;

	// back box
	triMesh[MeshCounter].pointNum[0] = maxPoints + 2;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 4;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 7;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 4;
	MeshCounter++;

	// left box
	triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 3;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 5;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 3;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
	MeshCounter++;

	// right box
	triMesh[MeshCounter].pointNum[0] = maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 4;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 6;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 7;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 4;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 6;
	MeshCounter++;
	body[bodyIDcounter].endMeshNum = MeshCounter;
     for (int t=MeshStart;t<MeshCounter;t++){
		triMesh[t].color[0]=red;
		triMesh[t].color[1]=green;
		triMesh[t].color[2]=blue;
	 }


	body[bodyIDcounter].startPointNum = maxPoints;
	
	indexFirstFromID[bodyIDcounter]=maxPoints;
	createMassPoint(x-length/2.0f,y-width/2.0f,z-height/2.0,mass/8.0f);
	createMassPoint(x+length/2.0f,y-width/2.0f,z-height/2.0,mass/8.0f);
	createMassPoint(x-length/2.0f,y+width/2.0f,z-height/2.0,mass/8.0f);
	createMassPoint(x-length/2.0f,y-width/2.0f,z+height/2.0,mass/8.0f);

	createMassPoint(x+length/2.0f,y+width/2.0f,z-height/2.0,mass/8.0f);
	createMassPoint(x-length/2.0f,y+width/2.0f,z+height/2.0,mass/8.0f);
	createMassPoint(x+length/2.0f,y-width/2.0f,z+height/2.0,mass/8.0f);
	createMassPoint(x+length/2.0f,y+width/2.0f,z+height/2.0,mass/8.0f);
	indexLastFromID[bodyIDcounter]=maxPoints-1;

	body[bodyIDcounter].endPointNum = maxPoints-1;
	return bodyIDcounter++;
}
	
bodyID RigidBody::createSphere(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 radius, FLOAT_32 mass, FLOAT_32 red, FLOAT_32 green, FLOAT_32 blue)
{
	FLOAT_32 xx,yy,zz;
    int MeshStart=MeshCounter;

	if (simplify_Mode == 0){
		body[bodyIDcounter].startMeshNum = MeshCounter;
		 // plate front
		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 3;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 4;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 5;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 5;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 6;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 6;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 7;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 7;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 8;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 8;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 1;
		MeshCounter++;


		 // plate ground
		triMesh[MeshCounter].pointNum[0] = maxPoints + 25;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-2;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-2;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-3;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-3;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-4;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-4;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-5;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-5;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-6;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-6;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-7;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-7;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-8;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-0;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-8;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-1;
		MeshCounter++;


		// --- cylinder itself
		triMesh[MeshCounter].pointNum[0] = maxPoints + 10-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 2-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 3-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 11-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 3-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 4-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 12-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 4-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 5-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 13-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 5-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 6-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 14-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 6-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 7-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 15-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 7-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 8-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 16-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 8-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 9-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 17-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 9-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 2-1;
		MeshCounter++;
		// ---
		triMesh[MeshCounter].pointNum[0] = maxPoints + 3-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 10-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 11-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 4-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 11-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 12-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 5-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 12-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 13-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 6-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 13-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 14-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 7-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 14-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 15-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 8-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 15-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 16-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 9-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 16-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 17-1;
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 2-1;
		triMesh[MeshCounter].pointNum[1] = maxPoints + 17-1;
		triMesh[MeshCounter].pointNum[2] = maxPoints + 10-1;
		MeshCounter++;

		// --- cylinder itself
		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(10-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(2-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(3-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(11-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(3-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(4-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(12-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(4-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(5-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(13-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(5-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(6-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(14-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(6-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(7-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(15-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(7-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(8-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(16-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(8-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(9-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(17-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(9-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(2-1);
		MeshCounter++;
		// ---
		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(3-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(10-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(11-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(4-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(11-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(12-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(5-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(12-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(13-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(6-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(13-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(14-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(7-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(14-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(15-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(8-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(15-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(16-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(9-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(16-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(17-1);
		MeshCounter++;

		triMesh[MeshCounter].pointNum[0] = maxPoints + 25-(2-1);
		triMesh[MeshCounter].pointNum[1] = maxPoints + 25-(17-1);
		triMesh[MeshCounter].pointNum[2] = maxPoints + 25-(10-1);
		MeshCounter++;

		body[bodyIDcounter].endMeshNum = MeshCounter;
		 for (int t=MeshStart;t<MeshCounter;t++){
			triMesh[t].color[0]=red;
			triMesh[t].color[1]=green;
			triMesh[t].color[2]=blue;
		 }


		 body[bodyIDcounter].startPointNum = maxPoints;

		 indexFirstFromID[bodyIDcounter]=maxPoints;

		 createMassPoint(x, y, z+radius, mass);
		 for (int i=1;i<4;i++){
			 for (int t=0;t<8;t++){
				 xx = cos(t*3.1415927/4.0)*sin(i*3.1415927/4.0)*radius;
				 yy = sin(t*3.1415927/4.0)*sin(i*3.1415927/4.0)*radius;
				 zz = cos(i*3.1415927/4.0)*radius;
				 createMassPoint(x+xx, y+yy, z+zz, mass/18.0);
			 }
		 }
		 createMassPoint(x, y, z-radius, mass);

		 indexLastFromID[bodyIDcounter]=maxPoints-1;

		 body[bodyIDcounter].endPointNum = maxPoints-1;
		return bodyIDcounter++;
	}
	if(simplify_Mode == 1){
		return createBox(radius*2.0f,radius*2.0f,radius*2.0f,x,y,z,mass, red, green, blue);
	}
	if(simplify_Mode >= 2){
		return createPoint(x,y,z, mass, red, green, blue);
	}
}
bodyID RigidBody::createRay(FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1, FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2, FLOAT_32 red, FLOAT_32 green, FLOAT_32 blue)
{
	int MeshStart=MeshCounter;
	body[bodyIDcounter].startMeshNum = MeshCounter;
	triMesh[MeshCounter].pointNum[0] = maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = maxPoints + 0;
	triMesh[MeshCounter].pointNum[2] = maxPoints + 1;
	MeshCounter++;
	body[bodyIDcounter].endMeshNum = MeshCounter;

	for (int t=MeshStart;t<MeshCounter;t++){
		triMesh[t].color[0]=red;
		triMesh[t].color[1]=green;
		triMesh[t].color[2]=blue;
	}

	body[bodyIDcounter].startPointNum = maxPoints;
	indexFirstFromID[bodyIDcounter]=maxPoints;
	createVirtualPoint(x1,y1,z1);
	createVirtualPoint(x2,y2,z2);
	indexLastFromID[bodyIDcounter]=maxPoints-1;

	return bodyIDcounter++;
}

// --- to fixate two bodies ---
void RigidBody::connectBodies(bodyID idFrom, bodyID idTo)
{
	connection[connectionIDcounter].body1ID = idFrom;
	connection[connectionIDcounter].body2ID = idTo;
	connectionIDcounter++;
}
// --- this is a hinge joint ---
jointID RigidBody::createJoint(bodyID id1, bodyID id2, FLOAT_32 anchorX, FLOAT_32 anchorY, FLOAT_32 anchorZ, FLOAT_32 axisX, FLOAT_32 axisY, FLOAT_32 axisZ)
{
	// optimizing possible by looking if any points of the connected bodies lie in the axis
	int found = 0;
	bodyID bothIDs[2];
	bothIDs[0]=id1;
	bothIDs[1]=id2;
	
	for (int i=0;i<2;i++){
		for (int t = body[bothIDs[i]].startPointNum; t<=body[bothIDs[i]].endPointNum;t++){
			FLOAT_32 xx = (massPoint[t].x-anchorX);
			FLOAT_32 yy = (massPoint[t].y-anchorY);
			FLOAT_32 zz = (massPoint[t].z-anchorZ);
			FLOAT_32 length = sqrt(xx*xx + yy*yy + zz*zz);
			FLOAT_32 scalarprod;
			if (length>0){
				xx/=length;
				yy/=length;
				zz/=length;
			    scalarprod = (xx*axisX + yy*axisY + zz*axisZ); // normed vector
			}

			if (  (scalarprod > 0.99999) && (scalarprod < 1.000001) || 
				  (scalarprod < -0.99999) && (scalarprod > -1.000001)|| length==0 ){
				found++;
				if (found==1){
					joint[jointIDcounter].axisPoint1 = t;
				}else{
					joint[jointIDcounter].axisPoint2 = t;
				}

			}// endif

		}// endfor t
	}// endfor i
	
//	found=0; //generate points even if there exist some on the axis
	
	if (found==0){
		body[bodyIDcounter].startPointNum = maxPoints;
		indexFirstFromID[bodyIDcounter]=maxPoints;
		joint[jointIDcounter].axisPoint1 = createVirtualPoint(anchorX, anchorY, anchorZ);
		indexLastFromID[bodyIDcounter]=maxPoints-1;
		body[bodyIDcounter++].endPointNum = maxPoints-1;
		connectBodies(joint[jointIDcounter].axisPoint1, id1); // connect jointsAxispoints to one of the bodies 
	}
	if (found<=1){
		body[bodyIDcounter].startPointNum = maxPoints;
		indexFirstFromID[bodyIDcounter] = maxPoints;
		joint[jointIDcounter].axisPoint2 = createVirtualPoint(anchorX + axisX, anchorY + axisY, anchorZ+axisZ);
		indexLastFromID[bodyIDcounter] = maxPoints-1;
		body[bodyIDcounter++].endPointNum = maxPoints-1;
		connectBodies(joint[jointIDcounter].axisPoint2, id1); // connect jointsAxispoints to one of the bodies 
	}
	// vector points in the same direction? Than swap axisPoints
	// axisX, axisY, axisZ compared to axisPoint2Coordinates - axisPoint1Coordinates

	FLOAT_32 xx1 = massPoint[joint[jointIDcounter].axisPoint1].x;
	FLOAT_32 yy1 = massPoint[joint[jointIDcounter].axisPoint1].y;
	FLOAT_32 zz1 = massPoint[joint[jointIDcounter].axisPoint1].z;
	FLOAT_32 xx2 = massPoint[joint[jointIDcounter].axisPoint2].x;
	FLOAT_32 yy2 = massPoint[joint[jointIDcounter].axisPoint2].y;
	FLOAT_32 zz2 = massPoint[joint[jointIDcounter].axisPoint2].z;
	
	if (((xx2-xx1)>0.001 && axisX<-0.001) || ((xx2-xx1)<-0.001 && axisX>0.001) ||
		((yy2-yy1)>0.001 && axisY<-0.001) || ((yy2-xx1)<-0.001 && axisY>0.001) ||
		((zz2-zz1)>0.001 && axisZ<-0.001) || ((zz2-zz1)<-0.001 && axisZ>0.001) ){ 
		//swap joint IDs
		int help = joint[jointIDcounter].axisPoint1;
		joint[jointIDcounter].axisPoint1 = joint[jointIDcounter].axisPoint2;
		joint[jointIDcounter].axisPoint2 = help;
	}

	joint[jointIDcounter].body1ID = id1;
	joint[jointIDcounter].body2ID = id2;

	maxJoints++;
	return jointIDcounter++;
}

 bodyID RigidBody::getMaxBodyID()
 {
	 return bodyIDcounter;
 }
 int RigidBody::finalizeConstruction()
 {
	 // here the joints and connections between bodies are transfered in a bodylist
	 int i,t,j;
	 calculateMomentumOfInertia();

	 for (t=0; t<connectionIDcounter + jointIDcounter; t++){
		 for (i=0; i<jointIDcounter; i++){
			 joint[i].bodyList[joint[i].body2ID]=1; // connected to body2ID
			 // is the body t connected to joint[i}.body1ID or to joint[i].body2ID? --> joint[i].bodyList[t] = 0 or 1;

			 for (j=0; j<connectionIDcounter; j++){
				 if (joint[i].bodyList[connection[j].body1ID] == 1){
					 joint[i].bodyList[connection[j].body2ID] = 1;
				 }
				 if (joint[i].bodyList[connection[j].body2ID] == 1){
					 joint[i].bodyList[connection[j].body1ID] = 1;
				 }
			 }
			 for (j=0; j<jointIDcounter; j++){
				 if (j!=i){
					 if (joint[i].bodyList[joint[j].body1ID] == 1){
						 joint[i].bodyList[joint[j].body2ID] = 1;
					 }
					 if (joint[i].bodyList[joint[j].body2ID] == 1){
						 joint[i].bodyList[joint[j].body1ID] = 1;
					 }
				 }
			 }

		 }// endfor i
	 }// endor t

	// to accelerate joint movements more ones than zeroes are put into joint[t].bodyList[i]
	for (t=0;t<maxJoints;t++){
		int sum=0;
		for (i=0;i<bodyIDcounter;i++){
			sum += joint[t].bodyList[i];
		}
		if (sum<bodyIDcounter/2){
			for (i=0;i<bodyIDcounter;i++){
				joint[t].bodyList[i] = 1-joint[t].bodyList[i];
			}
		}
	}
	 return 0;
 }


 bool RigidBody::groundContact(bodyID id)
 {
	for (int t=body[id].startPointNum; t<body[id].endPointNum; t++){
		if (massPoint[t].collFlag){
			return true; // collision
		}
	}
	return false;
 }
 void RigidBody::changeObjectColor(bodyID id, FLOAT_32 red, FLOAT_32 green, FLOAT_32 blue)
 {
	 for(int i = body[id].startMeshNum; i < body[id].endMeshNum; i++){
		triMesh[i].color[0]=red;
		triMesh[i].color[1]=green;
		triMesh[i].color[2]=blue;
	 }
 }


// ### END API functions #########################################################

 
// --- these Functions move and rotate the body (to be used before simulating)
void RigidBody::moveBody(bodyID id, FLOAT_32 xShift, FLOAT_32 yShift,FLOAT_32 zShift)
{
	int i;
	for(i=body[id].startPointNum; i<=body[id].endPointNum; i++){
		 massPoint[i].x += xShift;
		 massPoint[i].y += yShift;
		 massPoint[i].z += zShift;
		 massPoint[i].xCopy += xShift;
		 massPoint[i].yCopy += yShift;
		 massPoint[i].zCopy += zShift;
	}
}
void RigidBody::rotateBody(bodyID id, FLOAT_32 alpha,FLOAT_32 beta, FLOAT_32 gamma)
{

}	
Vector3D RigidBody::getCMorientation(){
	Vector3D vec;

	vec.x = omega[0];
	vec.y = omega[1];
	vec.z = omega[2];

	return vec;
}
Vector3D RigidBody::getCMcoordinates(){
	Vector3D vec;
	vec.x = CM[0];
	vec.y = CM[1];
	vec.z = CM[2];
		
	return vec;
}
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
* @file : PhysicsAPI.cpp                                                *
*************************************************************************/ 
/*
#include <math.h>
#include <time.h>

#include "PhysicsAPI.h"
#include "PhysicsEngine.h"
extern physics NeoPhysIx;
extern _triMesh triMesh[MAX_MESH];

int MeshCounter;
RigidBody entity;

void startConstruction()
{
	MeshCounter = 0;
}

 // --- sets the time which passes during one simulation step ---
 void setTimeStep(FLOAT_32 timeStep)  // default is 0.01
 {
	 NeoPhysIx.DELTA_TIME = timeStep;
 }
 // --- sets the gravity acceleration ---
 void setGravity(FLOAT_32 gravity)    // default is 9.81
 {
	 NeoPhysIx.G_ACCELLERATION = gravity;
 }
 // --- sets the friction coefficient 1.0 means highest possible friction, 0.0 means slippery
 void setFriction(FLOAT_32 friction)  // default is 1.0
 {
	 NeoPhysIx.friction = friction;
 }
 // --- sets the number of simulation steps until the angles of all joins are updated ---
 void setAngleStep(int angleStep) // default 8
 {
	 NeoPhysIx.ANGLE_STEP = angleStep;
 }

 // --- these functions generate bodies --- (to be used before simulating)
 bodyID createPoint( FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 mass)
 { 
	triMesh[MeshCounter].pointNum[0] = maxPoints;
	triMesh[MeshCounter].pointNum[1] = maxPoints;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints;
	MeshCounter++;

	return entity.createPoint(x,y,z, mass);	
 }
 bodyID createCylinder( FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1,
	                                     FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2, FLOAT_32 radius, FLOAT_32 mass)
 {
	 // plate front
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 2;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 3;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 3;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 4;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 4;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 5;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 6;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 7;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 7;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 8;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 8;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 9;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 9;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2;
	MeshCounter++;

	// --- plate back
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 10;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 11;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 11;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 12;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 12;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 13;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 13;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 14;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 14;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 15;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 15;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 16;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 16;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 17;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 17;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 10;
	MeshCounter++;

	// --- cylinder itself
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 10;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 2;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 3;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 11;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 3;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 4;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 12;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 4;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 5;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 13;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 14;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 6;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 7;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 15;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 7;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 8;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 16;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 8;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 9;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 17;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 9;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2;
	MeshCounter++;
	// ---
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 3;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 10;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 11;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 4;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 11;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 12;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 12;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 13;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 6;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 13;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 14;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 7;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 14;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 15;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 8;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 15;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 16;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 9;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 16;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 17;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 2;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 17;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 10;
	MeshCounter++;

	 return entity.createCylinder(x1, y1, z1, x2, y2, z2, radius, mass);
 }
 bodyID createBox( FLOAT_32 length, FLOAT_32 width, FLOAT_32 height, FLOAT_32 x, FLOAT_32 y, FLOAT_32 z, FLOAT_32 mass)
 {

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 4;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2;
	MeshCounter++;

	//top box
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 3;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 7;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6;
	MeshCounter++;

	//front box
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 3;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 6;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 3;
	MeshCounter++;

	// back box
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 2;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 4;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 7;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 4;
	MeshCounter++;

	// left box
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 3;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 3;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2;
	MeshCounter++;

	// right box
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 4;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6;
	MeshCounter++;
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 7;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 4;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6;
	MeshCounter++;

	 return entity.createBox(length, width, height, x, y, z, mass);
 }
 bodyID createSphere( FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 radius, FLOAT_32 mass)
 {
	 // plate front
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 2;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 3;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 3;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 4;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 4;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 5;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 6;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 7;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 7;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 8;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 8;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 1;
	MeshCounter++;


	 // plate ground
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-2;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-2;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-3;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-3;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-4;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-4;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-5;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-5;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-6;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-6;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-7;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-7;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-8;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-8;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-1;
	MeshCounter++;


	// --- cylinder itself
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 10-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 2-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 3-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 11-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 3-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 4-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 12-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 4-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 5-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 13-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 5-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 6-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 14-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 6-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 7-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 15-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 7-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 8-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 16-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 8-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 9-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 17-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 9-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 2-1;
	MeshCounter++;
	// ---
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 3-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 10-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 11-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 4-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 11-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 12-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 5-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 12-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 13-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 6-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 13-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 14-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 7-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 14-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 15-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 8-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 15-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 16-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 9-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 16-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 17-1;
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 2-1;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 17-1;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 10-1;
	MeshCounter++;

	// --- cylinder itself
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(10-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(2-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(3-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(11-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(3-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(4-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(12-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(4-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(5-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(13-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(5-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(6-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(14-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(6-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(7-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(15-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(7-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(8-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(16-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(8-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(9-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(17-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(9-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(2-1);
	MeshCounter++;
	// ---
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(3-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(10-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(11-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(4-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(11-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(12-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(5-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(12-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(13-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(6-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(13-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(14-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(7-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(14-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(15-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(8-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(15-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(16-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(9-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(16-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(17-1);
	MeshCounter++;

	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 25-(2-1);
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 25-(17-1);
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 25-(10-1);
	MeshCounter++;

	 return entity.createSphere(x, y, z, radius, mass);
 }
 //
 bodyID createRay( FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1, FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2)
 {
	triMesh[MeshCounter].pointNum[0] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[1] = entity.maxPoints + 0;
	triMesh[MeshCounter].pointNum[2] = entity.maxPoints + 1;
	MeshCounter++;
	return entity.createRay(x1, y1,z1, x2, y2,z2);
 }


 // --- these Functions move and rotate the body (to be used before simulating)
 void moveBody(bodyID id, FLOAT_32 xShift, FLOAT_32 yShift,FLOAT_32 zShift)
 {
    entity.moveBody(id, xShift, yShift, zShift);
 
 }
 void rotateBody( bodyID id, FLOAT_32 alpha,FLOAT_32 beta, FLOAT_32 gamma)
 {
	 entity.rotateBody(id, alpha, beta, gamma);
 }

 // --- function to fixate two bodies ---
 void connectBodies( bodyID idFrom, bodyID idTo)
 {
     entity.connectBodies(idFrom, idTo);
 }
 // --- this is functions creates a hinge joint ---
 jointID createJoint( bodyID id1, bodyID id2,  FLOAT_32 anchorX, FLOAT_32 anchorY, FLOAT_32 anchorZ, FLOAT_32 axisX, FLOAT_32 axisY, FLOAT_32 axisZ)
 {
	 return entity.createJoint(id1, id2, anchorX, anchorY, anchorZ, axisX, axisY, axisZ);
 }
 bodyID getMaxBodyID(RigidBody entity )
 {
	 return entity.getMaxBodyID();
 }
 void setContactHeight(FLOAT_32 cheight){
	 NeoPhysIx.setContactHeight(cheight);
 }
 // --- here the joints and connections between bodies are transfered in a bodylist
 int finalizeConstruction()
 {	 
	 return entity.finalizeConstruction();
 }
 */
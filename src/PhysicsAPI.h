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
* @file : PhysicsAPI.h                                                  *
*************************************************************************/  
/*
#include "GlobalDefs.h"
#include "RigidBody.h"

#ifndef PHYSICSAPI_H
 #define PHYSICSAPI_H

 void startConstruction(RigidBody myRigidbody);
 // --- sets the time which passes during one simulation step ---
 void setTimeStep(FLOAT_32 timeStep);  // default is 0.01
 // --- sets the gravity acceleration ---
 void setGravity(FLOAT_32 gravity);    // default is 9.81
 // --- sets the friction coefficient 1.0 means highest possible friction, 0.0 means slippery
 void setFriction(FLOAT_32 friction);  // default is 1.0
 // --- sets the number of simulation steps until the angles of all joins are updated ---
 void setAngleStep(int angleStep); // default 7

 // --- these functions generate bodies --- (to be used before simulating)
 bodyID createPoint(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 mass);
 bodyID createCylinder(FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1,
									    FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2, FLOAT_32 radius, FLOAT_32 mass);
 bodyID createBox(FLOAT_32 length, FLOAT_32 width, FLOAT_32 height, FLOAT_32 x, FLOAT_32 y, FLOAT_32 z, FLOAT_32 mass);
 bodyID createSphere(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 radius, FLOAT_32 mass);

 bodyID createRay(FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1, FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2);


 // --- these Functions move and rotate the body (to be used before simulating)
 void moveBody(bodyID id, FLOAT_32 xShift, FLOAT_32 yShift,FLOAT_32 zShift);
 void rotateBody(bodyID id, FLOAT_32 alpha,FLOAT_32 beta, FLOAT_32 gamma);

 // --- to fixate two bodies ---
 void connectBodies(bodyID idFrom, bodyID idTo);
 // --- this is a hinge joint ---
 jointID createJoint(bodyID id1, bodyID id2, FLOAT_32 anchorX, FLOAT_32 anchorY, FLOAT_32 anchorZ, FLOAT_32 axisX, FLOAT_32 axisY, FLOAT_32 axisZ);
 // getter for Maximum of bodyIDs
 bodyID getMaxBodyID();
 // --- finalize the construction ---
 int finalizeConstruction();
 // --- makro to set the angle of the joint during simulation ---
 #define setJointAngle(id,angl) NeoPhysIx.angle[id]=angl
 #define getJointAngle(id) NeoPhysIx.angle[id]

 void setContactHeight(FLOAT_32 cheight);

 // --- simulate one step ---
 void simulateStep(RigidBody* entity, int maxEntities);

#endif // PHYSICSAPI_H
 */
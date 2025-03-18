
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "GlobalDefs.h"

class RigidBody
{
public:
	_connection connection[MAX_CONNECTIONS];
	_joint joint[MAX_JOINTS];
	_body  body[MAX_BODIES];

	FLOAT_32 mInertia;
	FLOAT_32 CM[3];                           // Center Of Mass
	FLOAT_32 CM_Old[3];
    FLOAT_32 CMVeloVec[3]; // Center Of Mass Velocity
	FLOAT_32 CMforce[3];
	FLOAT_32 CMmass;
	FLOAT_32 omega[3] ;     // Winkel
	FLOAT_32 friction;

	FLOAT_32 alpha;
	int bounce;

	massPointProp massPoint[MAX_POINTS];        // Mass MAX_POINTS

	int indexFirstFromID[MAX_BODIES];
	int indexLastFromID[MAX_BODIES];

	int bodyIDcounter;
	int jointIDcounter;
	int connectionIDcounter;

	int maxJoints;
	int maxPoints;
	
	FLOAT_32 contactHeight;

	int MeshCounter;
	_triMesh triMesh[MAX_MESH];

	FLOAT_32 angle[MAX_JOINTS];
	int ANGLE_STEP;

	FLOAT_32 InertiaTensor[3][3];
	FLOAT_32 InverseInertiaTensor[3][3];
	int simplify_Mode;
	
	RigidBody();
	void simplifyMode(int simp);
	void setContactHeight(FLOAT_32 cheight){
		contactHeight = cheight;
	}
	// calculates center of mass
	void calculateCenterOfMass();
	void calculateMomentumOfInertia();
	int createVirtualPoint(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z);
	void createMassPoint(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 mass);
	bodyID createPoint(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 mass, FLOAT_32 red = 0.5f, FLOAT_32 green=0.5, FLOAT_32 blue=0.5);
	bodyID createCylinder(FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1,FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2, FLOAT_32 radius, FLOAT_32 mass, FLOAT_32 red = 0.5f, FLOAT_32 green=0.5, FLOAT_32 blue=0.5);
	bodyID createBox(FLOAT_32 length, FLOAT_32 width, FLOAT_32 height, FLOAT_32 x, FLOAT_32 y, FLOAT_32 z, FLOAT_32 mass, FLOAT_32 red = 0.5f, FLOAT_32 green=0.5, FLOAT_32 blue=0.5);
	bodyID createSphere(FLOAT_32 x, FLOAT_32 y,FLOAT_32 z, FLOAT_32 radius, FLOAT_32 mass, FLOAT_32 red = 0.5f, FLOAT_32 green=0.5, FLOAT_32 blue=0.5);
	bodyID createRay(FLOAT_32 x1, FLOAT_32 y1,FLOAT_32 z1, FLOAT_32 x2, FLOAT_32 y2,FLOAT_32 z2, FLOAT_32 red = 0.5f, FLOAT_32 green=0.5, FLOAT_32 blue=0.5);
	// --- to fixate two bodies ---
	void connectBodies(bodyID idFrom, bodyID idTo);
	// --- this is a hinge joint ---
	jointID createJoint(bodyID id1, bodyID id2, FLOAT_32 anchorX, FLOAT_32 anchorY, FLOAT_32 anchorZ, FLOAT_32 axisX, FLOAT_32 axisY, FLOAT_32 axisZ);
	bodyID getMaxBodyID();
	int finalizeConstruction();
	
	// sensor functions
	bool groundContact(bodyID id);
	void changeObjectColor(bodyID id, FLOAT_32 red, FLOAT_32 green, FLOAT_32 blue);
	Vector3D getCMorientation();
	Vector3D getCMcoordinates();

	// --- these Functions move and rotate the body (to be used before simulating)
	void moveBody(bodyID id, FLOAT_32 xShift, FLOAT_32 yShift,FLOAT_32 zShift);
	void rotateBody(bodyID id, FLOAT_32 alpha,FLOAT_32 beta, FLOAT_32 gamma);

	virtual void move(int count){};

};
#endif

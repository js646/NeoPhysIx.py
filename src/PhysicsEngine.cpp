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
* @file : PhysicsEngine.h                                               *
*************************************************************************/
#include <cmath>
#include <ctime>
#include <vector>
#include "Math.h"
#include "GlobalDefs.h"
#include "Environment.h"
#include "PhysicsEngine.h"
/*
ToDo:
 sBody  = dSimpleSpaceCreate(sMorpheus);

  bObj[ObjID] = dBodyCreate(w);
 dMassSetSphere (&mObj[ObjID], 1, diameter/2.0);
  dMassAdjust  (&mObj[ObjID], mass);
  dBodySetMass (bObj[ObjID], &mObj[ObjID]);
  
 gObj[ObjID] = dCreateSphere (s, diameter/2.0);
  dGeomSetBody       (gObj[ObjID], bObj[ObjID]);
  dBodySetPosition   (bObj[ObjID], x, y, z);
  bObj[ObjID] = dBodyCreate(w);

 dMassSetCappedCylinder (&mObj[ObjID], 1, 1,diameter/2.0, length);
 gObj[ObjID] = dCreateCCylinder (s, diameter/2.0, length);
  dBodySetPosition   (bObj[ObjID], x,y,z);

 dMatrix3 Rot;
 dRFromEulerAngles (Rot,  alpha,beta,gamma);
 dBodySetRotation   (bObj[ObjID],Rot);

  connect[HinID]  = dJointCreateHinge(w,0);
  dJointAttach(connect[HinID],getBodyID(part1ID),getBodyID(part2ID));
  dJointSetHingeAnchor(connect[HinID],x, y, z );
  dJointSetHingeAxis (connect[HinID],axesX,axesY,axesZ);
  dJointSetHingeParam(connect[HinID],dParamFMax,100);
  dJointSetHingeParam(connect[HinID],dParamStopERP,0.9);
  dJointSetHingeParam(connect[HinID],dParamStopCFM,0.8);




 BodyID=OdeExt.createBox(sBody,BODY_LENGTH,BODY_WIDTH,BODY_HEIGHT, BODY_MASS,
									0,0,0+BODY_HEIGHT/2.0+FOOT_HEIGHT,
									0,0,0);
 OdeExt.connectHinge(HeadID, BodyID  , -BODY_LENGTH/2.0+HEAD_LENGTH/2.0,0,HEAD_HEIGHT/2.0+BODY_HEIGHT+FOOT_HEIGHT,
						            0,0,1, 0,0, 1);
 
*/

physics::physics()
{
	DELTA_TIME = 0.01f;
	G_ACCELLERATION = -9.81;

	time = 0.0f;
	timeCount = 0.0f;

	counter=0;
		
	beginTimer = clock() * CLK_TCK;      //start the timer 

}


void physics::moveJoints(RigidBody* entity)
{
	FLOAT_32 a[3][3];    // rotation matrix;
	FLOAT_32 help[4][3]; // four points needed to push body back into real position
	FLOAT_32 shift[3];
	FLOAT_32 matrix[3][4];
	FLOAT_32 solution[3];


	int pointNum=0;
	int t;

	pointNum = 0;
	// 4 points are stored, to which the robot is reprojected
	for (t=0;t<4;t++){
			help[t][0] = entity->massPoint[pointNum+t].x;
			help[t][1] = entity->massPoint[pointNum+t].y;
			help[t][2] = entity->massPoint[pointNum+t].z;
	}

	// -------------------------------------------------------------------------------------------------------

	// translation
	for (t=0;t<entity->maxPoints;t++){
		entity->massPoint[t].x = entity->massPoint[t].xCopy;
		entity->massPoint[t].y = entity->massPoint[t].yCopy;
		entity->massPoint[t].z = entity->massPoint[t].zCopy;
	}

	//for each joint
	for (int jointNum=0; jointNum<entity->maxJoints; jointNum++){

		if (entity->joint[jointNum].angle != 0){

			generateBasis(entity->massPoint,entity->joint[jointNum].basis,entity->joint[jointNum].axisPoint1,entity->joint[jointNum].axisPoint2);

			// translation
			for (t=0;t<entity->maxPoints;t++){
				entity->massPoint[t].x -= entity->joint[jointNum].basis[0][DIM_X];
				entity->massPoint[t].y -= entity->joint[jointNum].basis[0][DIM_Y];
				entity->massPoint[t].z -= entity->joint[jointNum].basis[0][DIM_Z];
			}

			// calculate rotation matrix
			//
			// (a00  a10  a20) (basis0) = 1
			// (a01  a11  a21) (basis1) = 0
			// (a02  a12  a22) (basis2) = 0

			for (t=0;t<3;t++){
				matrix[0][0] = entity->joint[jointNum].basis[1][DIM_X];
				matrix[1][0] = entity->joint[jointNum].basis[1][DIM_Y];
				matrix[2][0] = entity->joint[jointNum].basis[1][DIM_Z];
				matrix[0][3] = 0;
				matrix[0][1] = entity->joint[jointNum].basis[2][DIM_X];
				matrix[1][1] = entity->joint[jointNum].basis[2][DIM_Y];
				matrix[2][1] = entity->joint[jointNum].basis[2][DIM_Z];
				matrix[1][3] = 0;
				matrix[0][2] = entity->joint[jointNum].basis[3][DIM_X];
				matrix[1][2] = entity->joint[jointNum].basis[3][DIM_Y];
				matrix[2][2] = entity->joint[jointNum].basis[3][DIM_Z];
				matrix[2][3] = 0;
				matrix[t][3] = 1;
				solve(3,matrix,solution);
				a[t][0]=solution[0];
				a[t][1]=solution[1];
				a[t][2]=solution[2];
			}

			//############################################################################################################################################


			// --- rotate object so that joint is lying on the x-axes

			for (t=0;t<entity->maxPoints;t++){
				shift[0]=entity->massPoint[t].x;
				shift[1]=entity->massPoint[t].y;
				shift[2]=entity->massPoint[t].z;
				entity->massPoint[t].x = a[0][0]*shift[0] + a[1][0]*shift[1] + a[2][0]*shift[2];
				entity->massPoint[t].y = a[0][1]*shift[0] + a[1][1]*shift[1] + a[2][1]*shift[2];
				entity->massPoint[t].z = a[0][2]*shift[0] + a[1][2]*shift[1] + a[2][2]*shift[2];
			}

			// --- turn joint
			for (int i=0;i<entity->bodyIDcounter;i++){
				if (entity->joint[jointNum].bodyList[i]==0){ // I Like to moveIt moveIt
					//	turn(0,massPoint[t].x, massPoint[t].y, massPoint[t].z, joint[jointNum].angle);
					FLOAT_32 cosWi = cos(entity->joint[jointNum].angle);
					FLOAT_32 sinWi = sin(entity->joint[jointNum].angle);
					// move points of body
					for (t = entity->body[i].startPointNum; t<=entity->body[i].endPointNum; t++){
						FLOAT_32 help1  = cosWi*entity->massPoint[t].y  + sinWi*entity->massPoint[t].z;
						entity->massPoint[t].z  = -sinWi*entity->massPoint[t].y + cosWi*entity->massPoint[t].z;
						entity->massPoint[t].y  = help1;
					}
					// move virtual points (of joints)
					for (t=0; t<entity->jointIDcounter; t++){ // for all joints
						if (entity->joint[t].body1ID == i){ // if body to move has a joint connected
							if (entity->massPoint[entity->joint[t].axisPoint1].mass == 0){ // is it a virtual point then turn it
								int tt = entity->joint[t].axisPoint1;
								FLOAT_32 help1  = cosWi*entity->massPoint[tt].y  + sinWi*entity->massPoint[tt].z;
								entity->massPoint[tt].z  = -sinWi*entity->massPoint[tt].y + cosWi*entity->massPoint[tt].z;
								entity->massPoint[tt].y  = help1;
							}
						}
						if (entity->joint[t].body2ID == i){ // if body to move has a joint connected
							if (entity->massPoint[entity->joint[t].axisPoint2].mass == 0){ // is it a virtual point then turn it
								int tt = entity->joint[t].axisPoint2;
								FLOAT_32 help1  = cosWi*entity->massPoint[tt].y  + sinWi*entity->massPoint[tt].z;
								entity->massPoint[tt].z  = -sinWi*entity->massPoint[tt].y + cosWi*entity->massPoint[tt].z;
								entity->massPoint[tt].y  = help1;
							}
						}
					}

				}//endif
			}//endfor i
		}
	}
	// endfor Each joint

	

	// backtransform     : shift bounce body or 0 body to origin
	//                     solve system of equations for rotation that shifted helpPoints are identical with bounce or 0 body
	//                     rotate massPoints according to solved equation
	//                     shift massPoints to bounce Body or 0 Body
	
	// --- shift to origin
	shift[DIM_X] = -entity->massPoint[pointNum].x;
	shift[DIM_Y] = -entity->massPoint[pointNum].y;
	shift[DIM_Z] = -entity->massPoint[pointNum].z;

	for (t=0; t<entity->maxPoints; t++){// t is the actual point
		entity->massPoint[t].x += shift[DIM_X];
		entity->massPoint[t].y += shift[DIM_Y];
		entity->massPoint[t].z += shift[DIM_Z];
	}

	// --- solve equations for rotating
	for (t=0; t<3; t++){
		matrix[0][0] = entity->massPoint[pointNum+1].x;
		matrix[0][1] = entity->massPoint[pointNum+1].y;
		matrix[0][2] = entity->massPoint[pointNum+1].z;

		matrix[0][3] = help[1][t]-help[0][t];

		matrix[1][0] = entity->massPoint[pointNum+2].x;
		matrix[1][1] = entity->massPoint[pointNum+2].y;
		matrix[1][2] = entity->massPoint[pointNum+2].z;

		matrix[1][3] = help[2][t]-help[0][t];

		matrix[2][0] = entity->massPoint[pointNum+3].x;
		matrix[2][1] = entity->massPoint[pointNum+3].y;
		matrix[2][2] = entity->massPoint[pointNum+3].z;

		matrix[2][3] = help[3][t]-help[0][t];

		solve(3,matrix,solution);
		
		a[t][0]=solution[0];
		a[t][1]=solution[1];
		a[t][2]=solution[2];
	}


	// --- rotate object
	for (t=0;t<entity->maxPoints;t++){
		shift[0]=entity->massPoint[t].x;
		shift[1]=entity->massPoint[t].y;
		shift[2]=entity->massPoint[t].z;

		entity->massPoint[t].x = a[0][0]*shift[0] + a[0][1]*shift[1] + a[0][2]*shift[2];
		entity->massPoint[t].y = a[1][0]*shift[0] + a[1][1]*shift[1] + a[1][2]*shift[2];
		entity->massPoint[t].z = a[2][0]*shift[0] + a[2][1]*shift[1] + a[2][2]*shift[2];

	}
		
	
	// --- shift to objectCoordinate

	for (t=0;t<entity->maxPoints;t++){// t is the actual point
		entity->massPoint[t].x += (help[0][DIM_X]);
		entity->massPoint[t].y += (help[0][DIM_Y]);
		entity->massPoint[t].z += (help[0][DIM_Z]);
	}//endfor t


}

// moves the object == massPoint
void physics::moveMassPoint(RigidBody* entity){

	//		s0 = 0.5*a* deltaT*deltaT + v0*deltaT + s0;
	for(int i = 0; i < entity->maxPoints; i++){
		entity->massPoint[i].x += 0.5 * entity->CMforce[0]/entity->CMmass * DELTA_TIME * DELTA_TIME + entity->CMVeloVec[0] * DELTA_TIME;
		entity->massPoint[i].y += 0.5 * entity->CMforce[1]/entity->CMmass * DELTA_TIME * DELTA_TIME + entity->CMVeloVec[1] * DELTA_TIME;
		entity->massPoint[i].z += 0.5 * entity->CMforce[2]/entity->CMmass * DELTA_TIME * DELTA_TIME + entity->CMVeloVec[2] * DELTA_TIME;
	}
	entity->CM[0] += entity->CMVeloVec[0] * DELTA_TIME;
	entity->CM[1] += entity->CMVeloVec[1] * DELTA_TIME;
	entity->CM[2] += entity->CMVeloVec[2] * DELTA_TIME;
}

// Dreht das Objekt == massPoint
void physics::spinMassPoint(RigidBody* entity){
	// Den Körper um den Schwerpunkt drehen -- von Fischer und Modifiziert
	// Omega[DIM_X] means turning around X achses
	// Omega[DIM_Y] means turning around y achses
	// Omega[DIM_Z] means turning around z achses

	FLOAT_32 dummy1;
	
	FLOAT_32 cosX,cosY,cosZ,sinX,sinY,sinZ;

	cosX = cos(entity->omega[DIM_X] * DELTA_TIME);
	cosY = cos(entity->omega[DIM_Y] * DELTA_TIME);
	cosZ = cos(entity->omega[DIM_Z] * DELTA_TIME);
	
	sinX = sin(entity->omega[DIM_X] * DELTA_TIME);
	sinY = sin(entity->omega[DIM_Y] * DELTA_TIME);
	sinZ = sin(entity->omega[DIM_Z] * DELTA_TIME);

	for(int i = 0; i<entity->maxPoints; i++){

		entity->massPoint[i].x -= entity->CM[DIM_X];
		entity->massPoint[i].y -= entity->CM[DIM_Y];
		entity->massPoint[i].z -= entity->CM[DIM_Z];

		// Drehen um die X Achse omega[DIM_X]

		dummy1                 = cosX*entity->massPoint[i].y - sinX*entity->massPoint[i].z;
		entity->massPoint[i].z = sinX*entity->massPoint[i].y + cosX*entity->massPoint[i].z;

		entity->massPoint[i].y = dummy1;

		// Drehen um die Y Achse omega[DIM_Y]

		dummy1                 = cosY*entity->massPoint[i].x - sinY*entity->massPoint[i].z;
		entity->massPoint[i].z = sinY*entity->massPoint[i].x + cosY*entity->massPoint[i].z;

		entity->massPoint[i].x = dummy1;
	
		// Drehen um die Z Achse omega[DIM_Z]

		dummy1                 = cosZ*entity->massPoint[i].x - sinZ*entity->massPoint[i].y;
		entity->massPoint[i].y = sinZ*entity->massPoint[i].x + cosZ*entity->massPoint[i].y;

		entity->massPoint[i].x = dummy1;
	
		entity->massPoint[i].x += entity->CM[DIM_X];
		entity->massPoint[i].y += entity->CM[DIM_Y];
		entity->massPoint[i].z += entity->CM[DIM_Z];
		
	}
}

/****************************************************************************
* \brief massPoint collides with the floor                                  *
*                                                                           *
* \param massPoint[][] : array of massPoints                                *
* \param CM[]          : center of mass x,y,z                               *
*****************************************************************************/
int physics::getAndCorrectCollision(RigidBody* entity){
	int bounce = -1;
	FLOAT_32 shiftUp = 0.0;
	int numOfContacts = 0;

	// toggleFlag is used for second array index of worldHeightForPoint
	static int toggleFlag = 0; // is 0 or 1
	static FLOAT_32 worldHeightForPoint[2][MAX_POINTS];

	static int cOldNum[3];
	static FLOAT_32 collisionPoints[3][3];
	
	toggleFlag = (++toggleFlag) & 1; // is 0 or 1
	if (current_environment != EnvironmentType::Plane){
		world.getHeight( entity->massPoint, entity->maxPoints, &worldHeightForPoint[toggleFlag][0] );
	}
	else{
		for(int i = 0; i<entity->maxPoints;i++){
			worldHeightForPoint[toggleFlag][i]=0;
		}
	}
	for(int i = 0; i<entity->maxPoints;i++){
		if (entity->massPoint[i].colidable){ // only for collidable points
			if (entity->massPoint[i].y < worldHeightForPoint[toggleFlag][i]){
				if(bounce==-1){
					bounce=i;
					shiftUp = entity->massPoint[i].y;
				}else{
					if (entity->massPoint[i].y-worldHeightForPoint[toggleFlag][i]
						< entity->massPoint[bounce].y-worldHeightForPoint[toggleFlag][bounce]){
						shiftUp = entity->massPoint[i].y-worldHeightForPoint[toggleFlag][i];
						bounce  = i;
					}//if
				}// else
			}//if
		}//if collidable
	}//For


	
	// --- Wall collision ---
	// If we assume that the robots masspoints change their height (y-koordinate) slowly then
	// we only must check how the height of the hightmap changes to be able to detect a collision
	for(int i = 0; i<entity->maxPoints;i++){
		entity->massPoint[i].bump = false;
		if (entity->massPoint[i].colidable){ // only for collidable points
			if ((worldHeightForPoint[toggleFlag][i] > worldHeightForPoint[(toggleFlag+1)&1][i]) &&
			    (entity->massPoint[i].yOld < worldHeightForPoint[toggleFlag][i]               )){
				entity->massPoint[i].bump = true;	
			}
		}
	}

	// --- end wall collision ---

	if (shiftUp < 0.0){
		entity->CM[DIM_Y] -= shiftUp;
		for(int i = 0; i<entity->maxPoints;i++){
			entity->massPoint[i].y -= shiftUp;
		}
	}


	// new friction routine part I
	for(int i = 0; i<entity->maxPoints;i++){
		if (entity->massPoint[i].colidable){ // only for collidable points
			if ((entity->massPoint[i].y-worldHeightForPoint[toggleFlag][i]) < entity->contactHeight){
				entity->massPoint[i].collFlag = true;
				numOfContacts++;
			}else{
				entity->massPoint[i].collFlag = false;
			}
		}
		else{
			entity->massPoint[i].collFlag = false;
		}
	}
	// end new friction routine part I



	// --- new friction start
	if (numOfContacts>=2){
//	if (numOfContactPoints>=2){
		FLOAT_32 a[3][3];    // rotation matrix;
		FLOAT_32 matrix[3][4];
		FLOAT_32 solution[3];
		
		for (int t=0; t<entity->maxPoints; t++){
			entity->massPoint[t].x-=entity->CM[0];
			entity->massPoint[t].y-=entity->CM[1];
			entity->massPoint[t].z-=entity->CM[2];
		}
		// init matrix with 0
		for (int i=0;i<3;i++){ 
			for (int t=0;t<4;t++){
				matrix[i][t]=0;
			}
		}
		int colFlagCounter=0;
	//	for (int i=0;i<3;i++){ // for all friction points
		for (int i=0;i<entity->maxPoints;i++){
			//if (cNum[i]==cOldNum[i] && cNum[i]!=-1){
			if ((entity->massPoint[i].collFlag == true) && (entity->massPoint[i].collFlagOld == true)){
				colFlagCounter++;
				// factor of a1
				matrix[0][0] += 1;
			//	matrix[1][0] += 0;
				matrix[2][0] += -entity->massPoint[i].z;
				// factor of a2
			//	matrix[0][1] += 0;
				matrix[1][1] += 1;
				matrix[2][1] += entity->massPoint[i].x;
				// factor of a3
				matrix[0][2] += -entity->massPoint[i].z;
				matrix[1][2] += entity->massPoint[i].x;
				matrix[2][2] += entity->massPoint[i].z*entity->massPoint[i].z + entity->massPoint[i].x*entity->massPoint[i].x;
				// solution
				matrix[0][3] += entity->massPoint[i].x-(entity->massPoint[i].xOld-entity->CM_Old[0]);
				matrix[1][3] += entity->massPoint[i].z - (entity->massPoint[i].zOld-entity->CM_Old[2]);
				matrix[2][3] += ((entity->massPoint[i].xOld-entity->CM_Old[0])*entity->massPoint[i].z - entity->massPoint[i].x*entity->massPoint[i].z) +
					            (entity->massPoint[i].z * entity->massPoint[i].x - entity->massPoint[i].x*(entity->massPoint[i].zOld-entity->CM_Old[2]));
			}

		}
		if (colFlagCounter>=2){
			solve(3,matrix,solution);

			FLOAT_32 alpha   = -asin(solution[2]);
			FLOAT_32 shift_x = -solution[0];
			FLOAT_32 shift_z = -solution[1];

			// Drehmatrix
			a[0][0] = cos(alpha);
			a[1][0] = solution[2];//-sin(alpha);
			a[0][1] = -solution[2];//sin(alpha);
			a[1][1] = a[0][0];//cos(alpha);
	
			for (int t=0; t<entity->maxPoints; t++){
				solution[0] = a[0][0] * entity->massPoint[t].x + a[1][0] * entity->massPoint[t].z;  
				solution[1] = a[0][1] * entity->massPoint[t].x + a[1][1] * entity->massPoint[t].z;
				entity->massPoint[t].x = solution[0];
				entity->massPoint[t].z = solution[1];
			}
		
			entity->CM[0] += shift_x;
			entity->CM[2] += shift_z;
		}
		for (int t=0; t<entity->maxPoints; t++){
			entity->massPoint[t].x += entity->CM[0];
			entity->massPoint[t].y += entity->CM[1];
			entity->massPoint[t].z += entity->CM[2];
		}
		//cout << shift_x << "\n" << shift_z << "\n";

	} 
	// --- friction end
	
	for(int i = 0; i<entity->maxPoints; i++){
		entity->massPoint[i].xOld = entity->massPoint[i].x;
		entity->massPoint[i].yOld = entity->massPoint[i].y;
		entity->massPoint[i].zOld = entity->massPoint[i].z;
		entity->massPoint[i].collFlagOld = entity->massPoint[i].collFlag;
	}
	entity->CM_Old[0] = entity->CM[0];
	entity->CM_Old[1] = entity->CM[1];
	entity->CM_Old[2] = entity->CM[2];
	return bounce;
}


/****************************************************************************
* \brief massPoint collision with the floor is calculated                   *
*                                                                           *
* \param massPoint[][] : array of massPoints                                *
* \param CM[]          : center of mass x,y,z                               *
* \param CMVeloVec[]   : center of mass velocity xv,yv,zv                   *
* \param omega[]       : angular velocity                                   *
* \param bounce        : point number, of the colliding point               *
*****************************************************************************/

void physics::calcCollisionResponse(RigidBody* entity){
	FLOAT_32 nrAP, j;

	nrAP = ((entity->CM[DIM_Y]) - (entity->massPoint[entity->bounce].y));
	j = (FLOAT_32) ((1.0F + BUMP_EPSILON) *	(entity->CMVeloVec[DIM_Y]));
	j = j/ (1.0f/((entity->CMmass)) + nrAP*nrAP/(entity->mInertia) );
		
	// Neuer Speed des Schwerpunkt
	entity->CMVeloVec[DIM_Y] -= (j / (FLOAT_32)(entity->CMmass));
	
	entity->omega[DIM_Z]  *= ROLL_DAMPING; // Dämpfung
	entity->omega[DIM_X]  *= ROLL_DAMPING; // Dämpfung
	entity->omega[DIM_Z]  +=  ((entity->CM[DIM_X] - entity->massPoint[entity->bounce].x) * j )/(entity->mInertia);
	entity->omega[DIM_X]  += -((entity->CM[DIM_Z] - entity->massPoint[entity->bounce].z) * j )/(entity->mInertia);

}

/****************************************************************************
* \brief Simulation cycle, simulating the robot in the artificial world     *
*                                                                           *
*****************************************************************************/
void physics::simulateStep(const std::vector<Robot*>& entities){

	time+= DELTA_TIME;

	for (auto& entity : entities){
		entity->CMforce[DIM_Y] = G_ACCELLERATION * entity->CMmass;  // gravitation

		//		s0 = 0.5* (F/mass) * deltaT*deltaT + v0*deltaT + s0;
		moveMassPoint(entity);
		//      v0 = (F/mass)*deltaT+v0;
		entity->CMVeloVec[DIM_X] += entity->CMforce[DIM_X] / entity->CMmass * DELTA_TIME;
        entity->CMVeloVec[DIM_Y] += entity->CMforce[DIM_Y] / entity->CMmass * DELTA_TIME;
        entity->CMVeloVec[DIM_Z] += entity->CMforce[DIM_Z] / entity->CMmass * DELTA_TIME;

		spinMassPoint(entity);
		
		entity->calculateCenterOfMass();

		FLOAT_32 delta=0.1f;

		entity->bounce = getAndCorrectCollision(entity);

		if (entity->bounce != -1) {
            calcCollisionResponse(entity);
        }

		if ((counter & entity->ANGLE_STEP) == 0) {

			entity->move(counter);

			for (int t = 0; t < entity->maxJoints; t++) {
                entity->joint[t].angle = entity->angle[t];
            }

            moveJoints(entity);
		}
	} // endfor body
	counter++;

}


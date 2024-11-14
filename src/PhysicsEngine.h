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

#ifndef PHYSICS_ENGINE_H
#define PHYSICS_ENGINE_H

#include "Robot.h"
#include "time.h"

class physics
{
public: // variables
	FLOAT_32 DELTA_TIME;
	FLOAT_32 G_ACCELLERATION;

	FLOAT_32 time;
	FLOAT_32 timeCount;

	int counter;

	clock_t beginTimer, endTimer;             //initialize Begin and End for the timer
public: // functions

	physics();
	
	 // --- sets the gravity acceleration ---
	void setGravity(FLOAT_32 gravity)    // default is -9.81
	{
		 G_ACCELLERATION = gravity;
	}
	// --- here the joints and connections between bodies are transfered in a bodylist
	int finalizeConstruction();

	// END API functions

	void moveJoints(RigidBody* entity);

	// moves the object == massPoint
	void moveMassPoint(RigidBody* entity);
	// Dreht das Objekt == massPoint
	void spinMassPoint(RigidBody* entity);
	/****************************************************************************
	* \brief massPoint collides with the floor                                  *
	*                                                                           *
	* \param massPoint[][] : array of massPoints                                *
	* \param CM[]          : center of mass x,y,z                               *
	*****************************************************************************/
	int getAndCorrectCollision(RigidBody* entity);
	/****************************************************************************
	* \brief massPoint collision with the floor is calculated                   *
	*                                                                           *
	* \param massPoint[][] : array of massPoints                                *
	* \param CM[]          : center of mass x,y,z                               *
	* \param CMVeloVec[]   : center of mass velocity xv,yv,zv                   *
	* \param omega[]       : angular velocity                                   *
	* \param bounce        : point number, of the colliding point               *
	* \param time          : actual simulated time                              *
	*****************************************************************************/

	void calcCollisionResponse(RigidBody* entity);

	/****************************************************************************
	* \brief Simulation cycle, simulating the robot in the artificial world     *
	*                                                                           *
	*****************************************************************************/
	void simulateStep(const std::vector<Robot*>& entities);

}; // class physics
#endif // PHYSICS_ENGINE_H
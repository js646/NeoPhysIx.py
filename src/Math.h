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
* @file : math.h                                                        *
*************************************************************************/
#include "GlobalDefs.h"

#define MDIMS 3

// -------------------------------------------------------------------------------
// ----------------------------------- Solve -------------------------------------
// -------------------------------------------------------------------------------

void solve(int MDims,FLOAT_32 matrix[MDIMS][MDIMS+1],FLOAT_32 solution[MDIMS]);

// turns point around x,y or z-axes
void turn(int axis, FLOAT_32 &pointX, FLOAT_32 &pointY, FLOAT_32 &pointZ,FLOAT_32 wi);

void generateBasis(massPointProp massPoint[MAX_POINTS], FLOAT_32 coord[4][3],int joint1, int joint2);

Vector3D crossProduct(Vector3D vec1, Vector3D vec2);

/**********************************************************************
* \brief  : pseudo random number generator
* \return : generates a pseudorandom value of type unsigned int
**********************************************************************/
unsigned int pseudoRand();


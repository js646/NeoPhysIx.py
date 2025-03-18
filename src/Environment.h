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
* @file : Environment.h                                                 *
*************************************************************************/

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

struct vec3{
	FLOAT_32 x,y,z;
};

extern double landscape[MAX_WORLD+1][MAX_WORLD+1];

class environment{
	// World - Definitions
	#define X_MAX 50
	#define Y_MAX 50
public:
	environment(); // Constructor
	vec3 getStartPosition();
	
	/***************************************************************
	* \brief gets collision                                        *
	* \param massPoint is the information defined in massPointProp *
	* \return -1  if no collision took place                       *
	*         >=0 the collision point with deepest collision       *
	***************************************************************/
	//int getCollisionPoints(massPointProp* massPoint, int maxPoints); 

	/***************************************************************
	* \brief gets an RGB image array                               *
	* \param viewPoint: Cameraposition                             *                                                             *
	* \param angle: gives the direction, in which the camera looks *
	* \param rgbImage returns the RGB map                          *
	****************************************************************/
	void getCameraImage(vec3 viewPoint, vec3 angle, char* rgbImage[X_MAX][Y_MAX][3]);

	/***************************************************************
	* \brief paints the envirnment and the robot                   *
	*                                                              *
	* \param viewPoint: Cameraposition                             *                                                             *
	* \param angle: gives the direction, in which the camera looks *
	*                                                              *
	***************************************************************/
	//void showWorld(vec3 viewPoint, vec3 angle, massPointProp* massPoint, int maxPoints);
	/***************************************************************
	* \brief                    *
	***************************************************************/
	bool getIntersect(vec3 start, vec3 end, vec3 *intersecPt, int &red, int &green, int &blue);

	// -- extra Functions ---
	// ----------------------------------------------------------------------
	// --- \brief generates a natural landscape                           ---
	// --- \param x1,y1,x2,y2 are the koordinates in which the landscape  ---
	// ---        is generated                                            ---
	// --- \param deviation is the maximum lift of each point according   ---
	// ---        the mean value of its neighbor points                   ---
	// ----------------------------------------------------------------------
	void getHeight(massPointProp *massPoint, int maxPoints, FLOAT_32 *worldHeightForPoint);
	void genLandscape(int x1,int y1, int x2, int y2, int deviation);
};
#endif

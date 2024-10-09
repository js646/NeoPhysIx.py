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
* @file : Environment.cpp                                               *
*************************************************************************/

#include "GlobalDefs.h"
#include "Math.h"
#include "Environment.h"


////////////// TODO: infinite Landscape
class xCoord{
public:
	double *X;

	void setDim(int size){
		X = new double[size];
	}
};
xCoord *landscapeY;
////////////// TODO: infinite Landscape

environment::environment() // Constructor
{
	int deviation;

	////////////// TODO: infinite Landscape
	landscapeY = new xCoord[1000];

	// initialize graphics
	for (int y=0;y<1000;y++){
		landscapeY[y].setDim(1000);
		for (int x=0;x<1000;x++){
			landscapeY[y].X[x]=0;
		}
	}
	////////////// TODO: infinite Landscape

	deviation = 400;
	for (int x=0;x<MAX_WORLD;x++){
		for (int y=0;y<MAX_WORLD;y++){
			landscape[x][y]=-1000;
		}
	}
	landscape[0][0]=0;
	landscape[0][MAX_WORLD]=0;
	landscape[MAX_WORLD][0]=0;
	landscape[MAX_WORLD][MAX_WORLD]=0;
	genLandscape(0,0,MAX_WORLD,MAX_WORLD,deviation);

	for (int x=0;x<MAX_WORLD;x++){
		for (int y=0;y<MAX_WORLD;y++){
			landscape[x][y]/=2.0;
		}
	}

}

vec3 environment::getStartPosition()
{
  vec3 startPos;
  startPos.x=0;
  startPos.y=0;
  startPos.z=0;

  return startPos;
}
	
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
void environment::getCameraImage(vec3 viewPoint, vec3 angle, char* rgbImage[X_MAX][Y_MAX][3])
{
}

/***************************************************************
* \brief paints the envirnment and the robot                   *
*                                                              *
* \param viewPoint: Cameraposition                             *                                                             *
* \param angle: gives the direction, in which the camera looks *
*                                                              *
***************************************************************/
//	void showWorld(vec3 viewPoint, vec3 angle, massPointProp* massPoint, int maxPoints);
/***************************************************************
* \brief                    *
***************************************************************/
bool environment::getIntersect(vec3 start, vec3 end, vec3 *intersecPt, int &red, int &green, int &blue)
{
	// Todo: get intersection between line and floor
 return false;
}

// -- extra Functions ---
// ----------------------------------------------------------------------
// --- \brief generates a natural landscape                           ---
// --- \param x1,y1,x2,y2 are the koordinates in which the landscape  ---
// ---        is generated                                            ---
// --- \param deviation is the maximum lift of each point according   ---
// ---        the mean value of its neighbor points                   ---
// ----------------------------------------------------------------------
void environment::getHeight(massPointProp *massPoint, int maxPoints, FLOAT_32 *worldHeightForPoint)
{
	#define XZ_FACTOR  10.0
	#define Y_FACTOR 0.1

	for (int i=0;i<maxPoints;i++){
		int xx = (int)(massPoint[i].x * XZ_FACTOR + MAX_WORLD/2);
		int zz = (int)(massPoint[i].z * XZ_FACTOR + MAX_WORLD/2);

		if (xx>MAX_WORLD || xx<0 || zz<0 || zz>MAX_WORLD){
			worldHeightForPoint[i] = 0;
		}
		else{
			worldHeightForPoint[i] = landscape[xx][zz]*Y_FACTOR;
		}
	}
}
void environment::genLandscape(int x1, int y1, int x2, int y2, int deviation)
{
    int x, y;

    switch (current_environment) {
        case EnvironmentType::Apartment:
            for (y=y1;y<y2;y++){
                for (x=x1;x<x2;x++){
                    landscape[x][y] = 0;
                    if (x==x1 || x==x2-1){
                        landscape[x][y] = 20;
                    }
                    if (y==y1 || y==y2-1){
                        landscape[x][y] = 20;
                    }
                }
            }
            break;

        case EnvironmentType::Plane:
            for (y=y1;y<y2;y++){
                for (x=x1;x<x2;x++){
                    landscape[x][y] = 0;
                }
            }
            break;

        case EnvironmentType::Ramp:
            for (y=y1;y<y2;y++){
                for (x=x1;x<x2;x++){
                    landscape[x][y] = 0;
                    if (x>MAX_WORLD/2-5 && x<MAX_WORLD/2+5){
                        landscape[x][y]=10-(x-(MAX_WORLD/2-5));
                    }
                }
            }
            break;

        case EnvironmentType::Mountain:
            x = x1+(x2-x1)/2;
            y = y1+(y2-y1)/2;

            deviation = deviation/2;
            if (deviation<1){
                deviation=1;
            }

            if (y2-y1>1 && x2-x1>1){

                if (landscape[x][y]==-1000){

                    double a=landscape[x1][y1];
                    double b=landscape[x2][y2];
                    double c=landscape[x1][y1];


                    landscape[x][y]=(landscape[x1][y1]+(landscape[x2][y2]-landscape[x1][y1])/2)+
                                    (int)(pseudoRand()%deviation-deviation/2);
                    double d=landscape[x][y];
                    double e=0;
                }
                if (landscape[x1][y]==-1000){
                    landscape[x1][y]=(landscape[x1][y1]+(landscape[x1][y2]-landscape[x1][y1])/2)+
                                    (int) (pseudoRand()%deviation-deviation/2);
                }
                if (landscape[x][y1]==-1000){
                    landscape[x][y1]=(landscape[x1][y1]+(landscape[x2][y1]-landscape[x1][y1])/2)+
                                    (int)(pseudoRand()%deviation-deviation/2);
                }
                if (landscape[x][y2]==-1000){
                    landscape[x][y2]=(landscape[x1][y2]+(landscape[x2][y2]-landscape[x1][y2])/2)+
                                    (int)(pseudoRand()%deviation-deviation/2);
                }
                if (landscape[x2][y]==-1000){
                    landscape[x2][y]=(landscape[x2][y1]+(landscape[x2][y2]-landscape[x2][y1])/2)+
                                    (int)(pseudoRand()%deviation-deviation/2);
                }

                genLandscape(x1,y1,x,y,deviation);
                genLandscape(x,y,x2,y2,deviation);
                genLandscape(x1,y,x,y2,deviation);
                genLandscape(x,y1,x2,y,deviation);
            }
            break;
    }
}
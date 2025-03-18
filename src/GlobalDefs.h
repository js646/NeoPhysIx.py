/************************************************************************
* \brief: NeoPhysIx ultra fast 3D-physics simulation                    *
*																		*
* (c) copyright by Jörn Fischer											*
*                                                                       *
*        ***  ITS PROHIBITED TO DISTRIBUTE THIS SOURCE!  ***            *
*        ***************************************************            *
*																		* 
* @autor: Prof.Dr.Jörn Fischer											*
* @email: j.fischer@hs-mannheim.de										*
*                                                                       *
* @file : GlobalDefs.h                                                  *
*************************************************************************/

#include <array>

#ifndef GLOBALDEFS_H
#define GLOBALDEFS_H

enum class EnvironmentType {
    Mountain = 1,
    Ramp = 2,
    Plane = 3,
    Apartment = 4
};

enum class RobotType {
    FourLegged,
    FourLeggedComparison,
    SixLegged,
    Humanoid,
    Wheeled,
    Custom
};

// defines the environment
extern EnvironmentType current_environment;

// constants
#define PI 3.14159265f

#define DIM_X 0
#define DIM_Y 1
#define DIM_Z 2

#define MAX_DRAW_OBJECTS 10


// ------------ Robot specific defs --------------

// simulation

// weight of each point in kg
#define NUM_OF_ROBOTS 1
#define VERTICAL_OFFSET 1
#define HORIZONTAL_OFFSET 5
#define BUMP_EPSILON 0.4f
extern float ROLL_DAMPING;

#define MAX_POINTS 10000
#define MAX_JOINTS 1000
#define MAX_CONNECTIONS 1000
#define MAX_BODIES 1000
#define MAX_MESH 10000

// --- environment defs -------
#define MAX_WORLD 128
class environment;
extern environment world;
extern double landscape[MAX_WORLD+1][MAX_WORLD+1];
// -------------------------------------------------

typedef double FLOAT_32;
typedef int bodyID;
typedef int jointID;

struct massPointProp{
	// coordinates
	FLOAT_32 x,y,z;
	FLOAT_32 xCopy,yCopy,zCopy;
	FLOAT_32 xOld,yOld,zOld;
	// Mass
	FLOAT_32 mass;
	// able to collide
	bool colidable;
	// collision 
	bool collFlagOld;
	bool collFlag;
	bool bump;
};


struct _joint{
	char bodyList[MAX_BODIES]; // 0 is group 0 (e.g. left  of the joint), 
	                          // 1 is group 1 (e.g. right of the joint)
	FLOAT_32 angle;
	bodyID body1ID,body2ID;

	int axisPoint1,axisPoint2; 

    FLOAT_32 basis[4][3]; // to be moved to the origin
};
struct _connection{
	bodyID body1ID,body2ID;
};
struct _body{
	int startPointNum,endPointNum;
	int startMeshNum,endMeshNum;
};

struct _triMesh {
    std::array<int, 3> pointNum;
    std::array<double, 3> color;
};

class Vector3D
{
public:
	FLOAT_32 x,y,z;
	Vector3D()
	{
		x=0;
		y=0;
		z=0;
	}
};

#endif // GLOBALDEFS_H
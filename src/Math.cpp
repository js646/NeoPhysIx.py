/**********************************************************************
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
* @file : math.h                                                        *
*************************************************************************/

#include <math.h>
#include <time.h>
#include "GlobalDefs.h"
#include "Math.h"

#include<iostream>


#define MDIMS 3
static unsigned int randNumber = (unsigned int)8274658764763245825;

// -------------------------------------------------------------------------------
// ----------------------------------- Solve -------------------------------------
// -------------------------------------------------------------------------------

void solve(int MDims,FLOAT_32 matrix[MDIMS][MDIMS+1],FLOAT_32 solution[MDIMS])
{
 int i,j,k,max;
 FLOAT_32 swap,t;

 for (i=0;i<MDims;i++){

	 // look for highest value
	 max=i;
	 for (j=i+1;j<MDims;j++){
		 if ((matrix[j][i]*matrix[j][i])>(matrix[max][i]*matrix[max][i]))
			max=j;
	 }
	 // swap with line of highest value
	 for (k=i;k<=MDims;k++){
		 swap=matrix[i][k];
		 matrix[i][k]=matrix[max][k];
		 matrix[max][k]=swap;
	 }

	 if (matrix[i][i]==0){
		matrix[i][i]=0.0000000000001f;
//		cout << "Division by Zero\n";
	 }

	 // eliminate
	// FLOAT_32 division = 1.0f;
	 for (j=i+1;j<MDims;j++){
		 FLOAT_32 help=matrix[j][i]/matrix[i][i];
		 for (k=MDims;k>=i;k--){
  			 matrix[j][k]-=matrix[i][k]*help;
		 }
	 }

 }
 for (j=0;j<MDims;j++)
	 solution[j]=0;

 for (j=MDims-1;j>=0;j--){

	 // substitute
	 t=0.0;
	 for (k=j+1;k<MDims;k++){
		 t+=matrix[j][k]*solution[k];
		 
	 }
	 if (matrix[j][j]!=0){
		solution[j]=(matrix[j][MDims]-t)/matrix[j][j];
	 }else{
		solution[j]=(matrix[j][MDims]-t)/0.0000000000001f;
//		cout << "division/0\n";
	 }
 }
}

// turns point around x,y or z-axes
void turn(int axis, FLOAT_32 &pointX, FLOAT_32 &pointY, FLOAT_32 &pointZ,FLOAT_32 wi)
{
	FLOAT_32 help1,help2;
	FLOAT_32 cosWi, sinWi;

	cosWi = cos(wi);
	sinWi = sin(wi);
	if (axis == 0){

		help1  = cosWi*pointY  + sinWi*pointZ;
		help2  = -sinWi*pointY + cosWi*pointZ;
		pointY = help1;
		pointZ = help2;
	}
	else if (axis == 1){
		help1  = cosWi*pointZ  + sinWi*pointX;
		help2  = -sinWi*pointZ + cosWi*pointX;
		pointZ = help1;
		pointX = help2;
	}else {
		help1  = cosWi*pointX  + sinWi*pointY;
		help2  = -sinWi*pointX + cosWi*pointY;
		pointX = help1;
		pointY = help2;
	}
}

void generateBasis(massPointProp massPoint[MAX_POINTS], FLOAT_32 coord[4][3],int joint1, int joint2)
{
 FLOAT_32 a[3],b[3],c[3],length;

 // coord[0][] is the translation vector
 // coord[1..4][] are the 3 basis vectors

 coord[0][DIM_X] = massPoint[joint1].x;
 coord[0][DIM_Y] = massPoint[joint1].y;
 coord[0][DIM_Z] = massPoint[joint1].z;


 // first basis vector
 a[DIM_X] = massPoint[joint2].x-coord[0][DIM_X];
 a[DIM_Y] = massPoint[joint2].y-coord[0][DIM_Y];
 a[DIM_Z] = massPoint[joint2].z-coord[0][DIM_Z];
 length = sqrt(a[DIM_X]*a[DIM_X] + a[DIM_Y]*a[DIM_Y] + a[DIM_Z]*a[DIM_Z]);

if (length > 0) {
    a[DIM_X]/=length;
    a[DIM_Y]/=length;
    a[DIM_Z]/=length;
} else {
    std::cerr << "Warning: MassPoints of joint axisPoints are too close or identical, causing zero length vector and division by zero." << std::endl;
    std::cout << "-- aborted generateBasis() Method, simulation results may be invalid --" << std::endl;
    return; // Abbruch, da die Basis nicht korrekt berechnet werden kann
}


 coord[1][DIM_X] = a[DIM_X];
 coord[1][DIM_Y] = a[DIM_Y];
 coord[1][DIM_Z] = a[DIM_Z];


 // second basis vector
 b[0]=a[1]*0.7456-a[2]*0.4444;
 b[1]=a[2]*0.834- a[0]*0.7456;
 b[2]=a[0]*0.4444-a[1]*0.834;
 length=sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
 b[0]/=length; // normalize
 b[1]/=length; // normalize
 b[2]/=length; // normalize

 // Kreuzprodukt für den 3.Vektor
 coord[2][DIM_X] = b[0];
 coord[2][DIM_Y] = b[1];
 coord[2][DIM_Z] = b[2];
 // third basis vector

 c[0]=a[1]*b[2]-a[2]*b[1];
 c[1]=a[2]*b[0]-a[0]*b[2];
 c[2]=a[0]*b[1]-a[1]*b[0];
 length=sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2]);
 c[0]/=length; // normalize
 c[1]/=length; // normalize
 c[2]/=length; // normalize

 // Kreuzprodukt für den 3.Vektor
 coord[3][DIM_X] = c[0];
 coord[3][DIM_Y] = c[1];
 coord[3][DIM_Z] = c[2];
}

/**********************************************************************
* \brief  : calculates cross product of two vectors to get an orthogonal 
*           vector
* \return : returns resulting vector
**********************************************************************/
Vector3D crossProduct(Vector3D vec1,Vector3D vec2)
{
	Vector3D vec3;

	vec3.x = vec1.y*vec2.z - vec1.z*vec2.y; 
	vec3.y = vec1.z*vec2.x - vec1.x*vec2.z;
	vec3.z = vec1.x*vec2.y - vec1.y*vec2.x; 
	return vec3;
}


/**********************************************************************
* \brief  : pseudo random number generator
* \return : generates a pseudorandom value of type unsigned int
**********************************************************************/
unsigned int pseudoRand()
{
    return (((randNumber = randNumber * 214013L + 2531011L) >> 16) & 0x7fff);
}

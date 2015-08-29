//Defines für EulerAngles.hpp

/**** QuatTypes.h - Basic type declarations ****/
#ifndef _H_QuatTypes
#define _H_QuatTypes
/*** Definitions ***/
/*! struct for quaternions */
typedef struct {
	float x;	/**< the x component */
	float y;	/**< the y component */
	float z;	/**< the z component */
	float w;	/**< the w component */
} Quat; 
/*! enum for quaternions */
enum  
{
	X,		/**< the x component */
	Y,		/**< the y component */
	Z,		/**< the z component */
	W		/**< the w component */
} QuatPart;
typedef float HMatrix[4][4]; /* Right-handed, for column vectors */
typedef Quat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */
#endif
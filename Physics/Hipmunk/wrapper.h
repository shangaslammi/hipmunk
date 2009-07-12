#ifndef WRAPPER_H
#define WRAPPER_H
#include "chipmunk.h"

// New functions
int wrConstantCallback(cpShape*, cpShape*, cpContact*, int, cpFloat, void*);

// From cpBody.h
void wrBodyUpdateVelocity(cpBody*, cpVect*, cpFloat, cpFloat);
void wrBodyApplyImpulse(cpBody*, cpVect*, cpVect*);
void wrDampedSpring(cpBody*, cpBody*, cpVect*, cpVect*, cpFloat, cpFloat, cpFloat, cpFloat);
void wrBodyLocal2World(cpBody*, cpVect*);
void wrBodyWorld2Local(cpBody*, cpVect*);
void wrBodyApplyForce(cpBody*, cpVect*, cpVect*);

// From cpShape.h
void wrCircleShapeInit(cpCircleShape*, cpBody*, cpVect*, cpFloat);
void wrSegmentShapeInit(cpSegmentShape*, cpBody*, cpVect*, cpVect*, cpFloat);
int wrShapePointQuery(cpShape*, cpVect*, cpLayers, cpGroup);
int wrShapeSegmentQuery(cpShape*, cpVect*, cpVect*, cpLayers, cpGroup, cpSegmentQueryInfo*);

// From cpPolyShape.h
void wrPolyShapeInit(cpPolyShape*, cpBody*, int, cpVect*, cpVect*);

// From cpJoint.h
void wrPinJointInit(cpPinJoint*, cpBody*, cpBody*, cpVect*, cpVect*);
void wrSlideJointInit(cpSlideJoint*, cpBody*, cpBody*, cpVect*, cpVect*, cpFloat, cpFloat);
void wrPivotJointInit(cpPivotJoint*, cpBody*, cpBody*, cpVect*);
void wrGrooveJointInit(cpGrooveJoint*, cpBody*, cpBody*, cpVect*, cpVect*, cpVect*);

// From cpArbiter.h
void wrContactsSumImpulses(cpContact*, int, cpVect*);
void wrContactsSumImpulsesWithFriction(cpContact*, int, cpVect*);

// From cpSpace.h
void wrSpacePointQuery(cpSpace*, cpVect*, cpLayers, cpGroup, cpSpacePointQueryFunc);

#endif

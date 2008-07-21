#ifndef WRAPPER_H
#define WRAPPER_H
#include "chipmunk.h"

// From cpBody.h
void wrBodyUpdateVelocity(cpBody *b, cpVect *g, cpFloat d, cpFloat dt);
void wrBodyApplyImpulse(cpBody *b, cpVect *j, cpVect *r);
void wrDampedSpring(cpBody *b1, cpBody *b2, cpVect *a1, cpVect *a2,
                    cpFloat rlen, cpFloat k, cpFloat dmp, cpFloat dt);
void wrBodyLocal2World(cpBody *b, cpVect *v);
void wrBodyWorld2Local(cpBody *b, cpVect *v);
void wrBodyApplyForce(cpBody *b, cpVect *f, cpVect *p);

// From cpShape.h
void wrCircleShapeInit(cpCircleShape *circle, cpBody *body,
                       cpVect *offset, cpFloat radius);
void wrSegmentShapeInit(cpSegmentShape *seg, cpBody *body,
                        cpVect *a, cpVect *b, cpFloat r);
int wrShapePointQuery(cpShape *shape, cpVect *p);

// From cpPolyShape.h
void wrPolyShapeInit(cpPolyShape *poly, cpBody *body,
                     int numVerts, cpVect *verts, cpVect *offset);

#endif

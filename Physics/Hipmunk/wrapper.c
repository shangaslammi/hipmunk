#include "wrapper.h"

// From cpBody.h
void wrBodyUpdateVelocity(cpBody *b, cpVect *g, cpFloat d, cpFloat dt) {
    cpBodyUpdateVelocity(b, *g, d, dt);
}
void wrBodyApplyImpulse(cpBody *b, cpVect *j, cpVect *r) {
    cpBodyApplyImpulse(b, *j, *r);
}
void wrDampedSpring(cpBody *b1, cpBody *b2, cpVect *a1, cpVect *a2,
                    cpFloat rlen, cpFloat k, cpFloat dmp, cpFloat dt) {
    cpDampedSpring(b1, b2, *a1, *a2, rlen, k, dmp, dt);
}
void wrBodyLocal2World(cpBody *b, cpVect *v) {
    cpVect ret = cpBodyLocal2World(b, *v);
    v->x = ret.x;
    v->y = ret.y;
}
void wrBodyWorld2Local(cpBody *b, cpVect *v) {
    cpVect ret = cpBodyWorld2Local(b, *v);
    v->x = ret.x;
    v->y = ret.y;
}
void wrBodyApplyForce(cpBody *b, cpVect *f, cpVect *p) {
    cpBodyApplyForce(b, *f, *p);
}


// From cpShape.h
void wrCircleShapeInit(cpCircleShape *circle, cpBody *body,
                       cpVect *offset, cpFloat radius) {
    cpCircleShapeInit(circle, body, radius, *offset);
}
void wrSegmentShapeInit(cpSegmentShape *seg, cpBody *body,
                        cpVect *a, cpVect *b, cpFloat r) {
    cpSegmentShapeInit(seg, body, *a, *b, r);
}
int wrShapePointQuery(cpShape *shape, cpVect *p) {
    return cpShapePointQuery(shape, *p);
}


// From cpPolyShape.h
void wrPolyShapeInit(cpPolyShape *poly, cpBody *body,
                     int numVerts, cpVect *verts, cpVect *offset) {
    cpPolyShapeInit(poly, body, numVerts, verts, *offset);
}

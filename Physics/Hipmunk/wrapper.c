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

// From cpJoint.h
void wrPinJointInit(cpPinJoint *joint, cpBody *a, cpBody *b,
                    cpVect *anchr1, cpVect *anchr2) {
    cpPinJointInit(joint, a, b, *anchr1, *anchr2);
}
void wrSlideJointInit(cpSlideJoint *joint, cpBody *a, cpBody *b,
                      cpVect *anchr1, cpVect *anchr2,
                      cpFloat min, cpFloat max) {
    cpSlideJointInit(joint, a, b, *anchr1, *anchr2, min, max);
}
void wrPivotJointInit(cpPivotJoint *joint, cpBody *a,
                      cpBody *b, cpVect *pivot) {
    cpPivotJointInit(joint, a, b, *pivot);
}
void wrGrooveJointInit(cpGrooveJoint *joint, cpBody *a, cpBody *b,
                       cpVect *groove_a, cpVect *groove_b, cpVect *anchr2) {
    cpGrooveJointInit(joint, a, b, *groove_a, *groove_b, *anchr2);
}

#include <stdlib.h>
#include "wrapper.h"

// New functions
int wrConstantCallback(cpShape *a, cpShape *b, cpContact *contacts,
                       int numContacts, cpFloat normal_coef, void *data) {
    return (data == 0 ? 0 : 1);
}

// From cpBody.h
void wrBodyUpdateVelocity(cpBody *b, cpVect *g, cpFloat d, cpFloat dt) {
    cpBodyUpdateVelocity(b, *g, d, dt);
}
void wrBodyApplyImpulse(cpBody *b, cpVect *j, cpVect *r) {
    cpBodyApplyImpulse(b, *j, *r);
}
void wrApplyDampedSpring(cpBody *b1, cpBody *b2, cpVect *a1, cpVect *a2,
                         cpFloat rlen, cpFloat k, cpFloat dmp, cpFloat dt) {
    cpApplyDampedSpring(b1, b2, *a1, *a2, rlen, k, dmp, dt);
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

int wrShapeSegmentQuery(cpShape *shape, cpVect *a, cpVect *b,
                        cpSegmentQueryInfo *info) {
    return cpShapeSegmentQuery(shape, *a, *b, info);
}


// From cpPolyShape.h
void wrPolyShapeInit(cpPolyShape *poly, cpBody *body,
                     int numVerts, cpVect *verts, cpVect *offset) {
    cpPolyShapeInit(poly, body, numVerts, verts, *offset);
}

// From various constraints
void wrPinJointInit(cpVect *anchr1, cpVect *anchr2,
                    cpPinJoint *joint, cpBody *a, cpBody *b) {
    cpPinJointInit(joint, a, b, *anchr1, *anchr2);
}
void wrSlideJointInit(cpFloat min, cpFloat max, cpVect *anchr1, cpVect *anchr2,
                      cpSlideJoint *joint, cpBody *a, cpBody *b) {
    cpSlideJointInit(joint, a, b, *anchr1, *anchr2, min, max);
}
void wrPivot1JointInit(cpVect *pivot, cpPivotJoint *joint, cpBody *a, cpBody *b) {
    cpPivotJointInit(joint, a, b, cpBodyWorld2Local(a, *pivot),
                     cpBodyWorld2Local(b, *pivot));
}
void wrPivot2JointInit(cpVect *anchr1, cpVect *anchr2,
                       cpPivotJoint *joint, cpBody *a, cpBody *b) {
    cpPivotJointInit(joint, a, b, *anchr1, *anchr2);
}
void wrGrooveJointInit(cpVect *groove_a, cpVect *groove_b, cpVect *anchr2,
                       cpGrooveJoint *joint, cpBody *a, cpBody *b) {
    cpGrooveJointInit(joint, a, b, *groove_a, *groove_b, *anchr2);
}
void wrGearJointInit(cpFloat phase, cpFloat ratio,
                     cpGearJoint *joint, cpBody *a, cpBody *b) {
    cpGearJointInit(joint, a, b, phase, ratio);
}
void wrDampedSpringInit(cpFloat restLength, cpFloat stiffness, cpFloat damping,
                        cpVect *anchr1, cpVect *anchr2,
                        cpDampedSpring *joint, cpBody *a, cpBody *b) {
    cpDampedSpringInit(joint, a, b, *anchr1, *anchr2,
                       restLength, stiffness, damping);
}
void wrDampedRotarySpringInit(cpFloat restAngle, cpFloat stiffness, cpFloat damping,
                              cpDampedRotarySpring *joint, cpBody *a, cpBody *b) {
    cpDampedRotarySpringInit(joint, a, b, restAngle, stiffness, damping);
}
void wrRotaryLimitJointInit(cpFloat min, cpFloat max,
                            cpRotaryLimitJoint *joint, cpBody *a, cpBody *b) {
    cpRotaryLimitJointInit(joint, a, b, min, max);
}
void wrSimpleMotorInit(cpFloat rate, cpSimpleMotor *joint, cpBody *a, cpBody *b) {
    cpSimpleMotorInit(joint, a, b, rate);
}

// From cpArbiter.h
void wrContactsSumImpulses(cpContact *contacts, int numContacts, cpVect *ret) {
    *ret = cpContactsSumImpulses(contacts, numContacts);
}
void wrContactsSumImpulsesWithFriction(cpContact *contacts, int numContacts,
                                       cpVect *ret) {
    *ret = cpContactsSumImpulsesWithFriction(contacts, numContacts);
}

// From cpSpace.h
void wrSpacePointQuery(cpSpace *space, cpVect *point, cpLayers layers,
                       cpGroup group, cpSpacePointQueryFunc func) {
    cpSpacePointQuery(space, *point, layers, group, func, NULL);
}

// From chipmunk_unsafe.h
void wrCircleShapeSetOffset(cpShape *shape, cpVect *offset) {
    cpCircleShapeSetOffset(shape, *offset);
}
void wrSegmentShapeSetEndpoints(cpShape *shape, cpVect *a, cpVect *b) {
    cpSegmentShapeSetEndpoints(shape, *a, *b);
}
void wrPolyShapeSetVerts(cpShape *shape, int numVerts, cpVect *verts, cpVect *offset) {
    cpPolyShapeSetVerts(shape, numVerts, verts, *offset);
}

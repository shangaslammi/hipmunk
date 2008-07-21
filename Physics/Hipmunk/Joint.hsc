module Physics.Hipmunk.Joint
    (-- * Joints
     Joint,
     pinJoint, slideJoint, pivotJoint, grooveJoint
    )
    where

import Foreign
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal


-- | @pinJoint (b1,a1) (b2,a2)@ connects the two bodies @b1@
--   and @b2@ with a solid pin. The anchor points @a1@ and @a2@,
--   in bodies @b1@ and @b2@ coordinates respectively,
--   are kept at a fixed distance.
pinJoint :: (Body, Position) -> (Body, Position) -> IO Joint
pinJoint (body1@(B b1),a1) (body2@(B b2),a2) =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with a1 $ \a1_ptr ->
  with a2 $ \a2_ptr ->
  mallocForeignPtrBytes #{size cpPinJoint} >>= \joint ->
  withForeignPtr joint $ \joint_ptr -> do
    wrPinJointInit joint_ptr b1_ptr b2_ptr a1_ptr a2_ptr
    return (J joint body1 body2)

foreign import ccall unsafe "wrapper.h"
    wrPinJointInit :: JointPtr -> BodyPtr -> BodyPtr
                   -> VectorPtr -> VectorPtr -> IO ()


-- | @slideJoint (b1,a1) (b2,a2) (mn,mx)@ creates a slide joint.
--   It is similar to a pin joint, however it has a minimum @mn@
--   and a maximum @mx@ distance, keepint the bodies from getting
--   too far but also allowing them to get closer.
slideJoint :: (Body, Position) -> (Body, Position)
           -> (CpFloat, CpFloat) -> IO Joint
slideJoint (body1@(B b1),a1) (body2@(B b2),a2) (mn,mx) =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with a1 $ \a1_ptr ->
  with a2 $ \a2_ptr ->
  mallocForeignPtrBytes #{size cpSlideJoint} >>= \joint ->
  withForeignPtr joint $ \joint_ptr -> do
    wrSlideJointInit joint_ptr b1_ptr b2_ptr a1_ptr a2_ptr mn mx
    return (J joint body1 body2)

foreign import ccall unsafe "wrapper.h"
    wrSlideJointInit :: JointPtr -> BodyPtr -> BodyPtr -> VectorPtr
                     -> VectorPtr -> CpFloat -> CpFloat -> IO ()


-- | @pivotJoint b1 b2 pos@ allows the bodies @b1@ and @b2@ to
--   pivot around a single point @pos@ in world's coordinates.
--   Both bodies should be already in place.
pivotJoint :: Body -> Body -> Position -> IO Joint
pivotJoint body1@(B b1) body2@(B b2) pos =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with pos $ \pos_ptr ->
  mallocForeignPtrBytes #{size cpPivotJoint} >>= \joint ->
  withForeignPtr joint $ \joint_ptr -> do
    wrPivotJointInit joint_ptr b1_ptr b2_ptr pos_ptr
    return (J joint body1 body2)

foreign import ccall unsafe "wrapper.h"
    wrPivotJointInit :: JointPtr -> BodyPtr -> BodyPtr
                     -> VectorPtr -> IO ()


-- | @grooveJoint (b1,g1,g2) (b2,anchor)@ attaches a point of
--   body @b2@ to a groove in body @b1@. The groove spans
--   from @g1@ to @g2@ (both in @b1@'s coordinates), while
--   the pivot is at @anchor@ (in @b2@'s coordinates).
grooveJoint :: (Body, Position, Position)
            -> (Body, Position) -> IO Joint
grooveJoint (body1@(B b1),g1,g2) (body2@(B b2),anchor) =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with g1 $ \g1_ptr ->
  with g2 $ \g2_ptr ->
  with anchor $ \anchor_ptr ->
  mallocForeignPtrBytes #{size cpGrooveJoint} >>= \joint ->
  withForeignPtr joint $ \joint_ptr -> do
    wrGrooveJointInit joint_ptr b1_ptr b2_ptr g1_ptr g2_ptr anchor_ptr
    return (J joint body1 body2)

foreign import ccall unsafe "wrapper.h"
    wrGrooveJointInit :: JointPtr -> BodyPtr -> BodyPtr
                      -> VectorPtr -> VectorPtr -> VectorPtr -> IO ()


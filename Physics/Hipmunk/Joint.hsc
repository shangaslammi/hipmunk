module Physics.Hipmunk.Joint
    (-- * Joints
     Joint,
     JointType(..),
     newJoint
    )
    where

import Foreign
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal


-- | There are currently four types of joints. When appending
--   a number to a property, we hint that it refer to one of
--   the bodies that the joint is contraining (e.g. @anchor2@
--   is the position of the anchor on the second body in its
--   coordinates).
data JointType =
    -- | A pin joint connects the bodies with a solid pin.
    --   The anchor points are kept at a fixed distance.
    Pin {anchor1, anchor2 :: Position}

    -- | A slide joint is similar to a pin joint, however
    --   it has a minimum and a maximum distance.
  | Slide {anchor1, anchor2 :: Position,
           minDist, maxDist :: CpFloat}

    -- | A pivot joint allows the bodies to pivot around
    --   a single point in world's coordinates. Both should
    --   be already in place.
  | Pivot {pivot :: Position}

    -- | A groove joint attaches a point on the second body
    --   to a groove in the first one.
  | Groove {groove1 :: (Position, Position),
            pivot2  :: Position}
    deriving (Eq, Ord, Show)


-- | @newJoint b1 b2 type@ connects the two bodies @b1@ and @b2@
--   with a joint of the given type. Note that you should
--   add the 'Joint' to a space.
newJoint :: Body -> Body -> JointType -> IO Joint
newJoint body1@(B b1) body2@(B b2) (Pin a1 a2) =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with a1 $ \a1_ptr ->
  with a2 $ \a2_ptr ->
  mallocForeignPtrBytes #{size cpPinJoint} >>= \joint ->
  withForeignPtr joint $ \joint_ptr -> do
    wrPinJointInit joint_ptr b1_ptr b2_ptr a1_ptr a2_ptr
    return (J joint body1 body2)

newJoint body1@(B b1) body2@(B b2) (Slide a1 a2 mn mx) =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with a1 $ \a1_ptr ->
  with a2 $ \a2_ptr ->
  mallocForeignPtrBytes #{size cpSlideJoint} >>= \joint ->
  withForeignPtr joint $ \joint_ptr -> do
    wrSlideJointInit joint_ptr b1_ptr b2_ptr a1_ptr a2_ptr mn mx
    return (J joint body1 body2)

newJoint body1@(B b1) body2@(B b2) (Pivot pos) =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with pos $ \pos_ptr ->
  mallocForeignPtrBytes #{size cpPivotJoint} >>= \joint ->
  withForeignPtr joint $ \joint_ptr -> do
    wrPivotJointInit joint_ptr b1_ptr b2_ptr pos_ptr
    return (J joint body1 body2)

newJoint body1@(B b1) body2@(B b2) (Groove (g1,g2) anchor) =
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
    wrPinJointInit :: JointPtr -> BodyPtr -> BodyPtr
                   -> VectorPtr -> VectorPtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    wrSlideJointInit :: JointPtr -> BodyPtr -> BodyPtr -> VectorPtr
                     -> VectorPtr -> CpFloat -> CpFloat -> IO ()
foreign import ccall unsafe "wrapper.h"
    wrPivotJointInit :: JointPtr -> BodyPtr -> BodyPtr
                     -> VectorPtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    wrGrooveJointInit :: JointPtr -> BodyPtr -> BodyPtr
                      -> VectorPtr -> VectorPtr -> VectorPtr -> IO ()


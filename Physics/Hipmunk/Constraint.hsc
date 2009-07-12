-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Constraint.hsc
-- Copyright   :  (c) 2008-2009 Felipe A. Lessa
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  provisional
-- Portability :  portable (needs FFI)
--
-- Constraints that restrict the bodies' movement.
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Constraint
    (-- * Constraints
     Constraint,
     ConstraintType(..),
     newConstraint
    )
    where

import Foreign
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal


-- | There are currently nine types of constraints. When
--   appending a number to a property, we hint that it refer to
--   one of the bodies that the constraint is intercting with
--   (e.g. @anchor2@ is the position of the anchor on the second
--   body in its coordinates).
data ConstraintType =
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
  | Pivot1 {pivot :: Position}

    -- | The same pivot joint but specified as two anchors (on
    --   each body's coordinates), removing the need having the
    --   bodies already in place.
  | Pivot2 {anchor1, anchor2 :: Position}

    -- | A groove joint attaches a point on the second body
    --   to a groove in the first one.
  | Groove {groove1 :: (Position, Position),
            pivot2  :: Position}

    -- | A gear joint restricts the bodies movement to be
    --   coordinated as if they were attached like dented gears.
  | Gear {phase :: Angle,
          ratio :: CpFloat}

    -- | A simple damped spring.  Generally this constraint
    --   should be used instead of @applyDampedSpring@.
  | DampedSpring {anchor1, anchor2 :: Position,
                  restLength :: CpFloat,
                  stiffness :: CpFloat,
                  damping :: CpFloat}

    -- | A damped rotary spring constraint.
  | DampedRotarySpring {restAngle :: Angle,
                        stiffness :: CpFloat,
                        damping :: CpFloat}

    -- | A rotary limit constraints the difference of angle
    --   between two bodies.
  | RotaryLimit {minDist, maxDist :: Angle}

    -- | A simple motor that applies opposite impulses to each
    --   body.  The rate is used to compute the torque.
  | SimpleMotor {rate :: CpFloat}
    deriving (Eq, Ord, Show)


-- | @newConstraint b1 b2 type@ connects the two bodies @b1@ and @b2@
--   with a constraint of the given type. Note that you should
--   add the 'Constraint' to a space.
newConstraint :: Body -> Body -> ConstraintType -> IO Constraint
newConstraint body1@(B b1) body2@(B b2) type_ =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  mallocForeignPtrBytes (size type_) >>= \constraint ->
  withForeignPtr constraint $ \constraint_ptr -> do
    init_ type_ constraint_ptr b1_ptr b2_ptr
    return (C constraint body1 body2)

-- | Size of the structure of the constraint.
size :: ConstraintType -> Int
size (Pin {})                = #{size cpPinJoint}
size (Slide {})              = #{size cpSlideJoint}
size (Pivot1 {})             = #{size cpPivotJoint}
size (Pivot2 {})             = #{size cpPivotJoint}
size (Groove {})             = #{size cpGrooveJoint}
size (Gear {})               = #{size cpGearJoint}
size (DampedSpring {})       = #{size cpDampedSpring}
size (DampedRotarySpring {}) = #{size cpDampedRotarySpring}
size (RotaryLimit {})        = #{size cpRotaryLimitJoint}
size (SimpleMotor {})        = #{size cpSimpleMotor}

-- | Type of generic constraint initializar.
type ConstraintInit = ConstraintPtr -> BodyPtr -> BodyPtr -> IO ()

-- | Helper functions similar to 'with'.
with1 :: (Storable a) => a -> (Ptr a -> ConstraintInit) -> ConstraintInit
with1 x f c b1 b2 =
    with x $ \x_ptr ->
    f x_ptr c b1 b2
with2 :: (Storable a, Storable b) => a -> b
      -> (Ptr a -> Ptr b -> ConstraintInit) -> ConstraintInit
with2 x y f c b1 b2 =
    with x $ \x_ptr ->
    with y $ \y_ptr ->
    f x_ptr y_ptr c b1 b2
with3 :: (Storable a, Storable b, Storable c) => a -> b -> c
      -> (Ptr a -> Ptr b -> Ptr c -> ConstraintInit) -> ConstraintInit
with3 x y z f c b1 b2 =
    with x $ \x_ptr ->
    with y $ \y_ptr ->
    with z $ \z_ptr ->
    f x_ptr y_ptr z_ptr c b1 b2

-- | Initializer of each constraint type.
init_ :: ConstraintType -> ConstraintInit
init_ (Gear p r)                 = wrGearJointInit p r
init_ (DampedRotarySpring r s d) = wrDampedRotarySpringInit r s d
init_ (RotaryLimit mn mx)        = wrRotaryLimitJointInit mn mx
init_ (SimpleMotor r)            = wrSimpleMotorInit r
init_ (Pivot1 pos)               = with1 pos $ wrPivot1JointInit
init_ (Pin a1 a2)                = with2 a1 a2 $ wrPinJointInit
init_ (Slide a1 a2 mn mx)        = with2 a1 a2 $ wrSlideJointInit mn mx
init_ (Pivot2 a1 a2)             = with2 a1 a2 $ wrPivot2JointInit
init_ (DampedSpring a1 a2 r s d) = with2 a1 a2 $ wrDampedSpringInit r s d
init_ (Groove (g1,g2) anchor)    = with3 g1 g2 anchor $ wrGrooveJointInit

foreign import ccall unsafe "wrapper.h"
    wrPinJointInit :: VectorPtr -> VectorPtr -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrSlideJointInit :: CpFloat -> CpFloat -> VectorPtr -> VectorPtr -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrPivot1JointInit :: VectorPtr -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrPivot2JointInit :: VectorPtr -> VectorPtr -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrGearJointInit :: CpFloat -> CpFloat -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrGrooveJointInit :: VectorPtr -> VectorPtr -> VectorPtr -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrDampedSpringInit :: CpFloat -> CpFloat -> CpFloat -> VectorPtr -> VectorPtr -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrDampedRotarySpringInit :: CpFloat -> CpFloat -> CpFloat -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrRotaryLimitJointInit :: CpFloat -> CpFloat -> ConstraintInit
foreign import ccall unsafe "wrapper.h"
    wrSimpleMotorInit :: CpFloat -> ConstraintInit

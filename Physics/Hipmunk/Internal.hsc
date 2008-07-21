module Physics.Hipmunk.Internal
    (VectorPtr,

     BodyInternal,
     BodyPtr,
     Body(..),

     ShapeInternal,
     ShapePtr,
     Shape(..),

     JointInternal,
     JointPtr,
     Joint(..)
    )
    where

import Foreign
#include "wrapper.h"

import Physics.Hipmunk.Common


type VectorPtr = Ptr Vector



data BodyInternal = BodyInternal
type BodyPtr = Ptr BodyInternal
newtype Body = B (ForeignPtr BodyInternal)
-- ^ A rigid body representing the physical properties of an object,
--   but without a shape. It may help to think as a particle that
--   is able to rotate.

instance Storable BodyInternal where
    sizeOf _    = #{size cpBody}
    alignment _ = alignment (undefined :: Vector)
    peek _      = fail "Body.Internal peek not implemented"
    poke _ _    = fail "Body.Internal poke not implemented"



data ShapeInternal = ShapeInternal
type ShapePtr = Ptr ShapeInternal
data Shape = S !(ForeignPtr ShapeInternal) !Body
-- ^ A collision shape is attached to a @Body@ to define its
--   shape. Multiple shapes may be attached, including
--   overlapping ones (shapes of a body don't generate collisions
--   with each other).
--
--   Note that to have any effect, a 'Shape' must also be
--   added to a 'Space', even if the body was already added.

-- We'll not implement Storable for ShapeInternal because
-- the size needed depends on the actual shape
-- and we'll not distinguish between them.
--
-- Note also that we have to maintain a reference to the
-- 'Body' to avoid garbage collection in the case that
-- the user doesn't add the body to a space and don't keep
-- a reference (common when adding bodies with infinite mass).
--
-- However, the body doesn't need to keep references to
-- the attached shapes because cpBody do not reference them,
-- so it wouldn't notice at all if they disappeared =).



data JointInternal = JointInternal
type JointPtr = Ptr JointInternal
data Joint = J !(ForeignPtr JointInternal) !Body !Body
-- ^ A joint represents a constrain between two bodies. Don't
--   forget to add the bodies and the joint to the space.

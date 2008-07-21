module Physics.Hipmunk.Internal
    (VectorPtr,

     BodyPtr,
     Body(..),

     ShapePtr,
     Shape(..),

     JointPtr,
     Joint(..)
    )
    where

import Foreign
#include "wrapper.h"

import Physics.Hipmunk.Common


type VectorPtr = Ptr Vector



-- | A rigid body representing the physical properties of an object,
--   but without a shape. It may help to think as a particle that
--   is able to rotate.
newtype Body = B (ForeignPtr Body)
type BodyPtr = Ptr Body

instance Eq Body where
    B b1 == B b2 = b1 == b2

instance Ord Body where
    B b1 `compare` B b2 = b1 `compare` b2



-- | A collision shape is attached to a @Body@ to define its
--   shape. Multiple shapes may be attached, including
--   overlapping ones (shapes of a body don't generate collisions
--   with each other).
--
--   Note that to have any effect, a 'Shape' must also be
--   added to a 'Space', even if the body was already added.
data Shape = S !(ForeignPtr Shape) !Body
type ShapePtr = Ptr Shape

-- Note also that we have to maintain a reference to the
-- 'Body' to avoid garbage collection in the case that
-- the user doesn't add the body to a space and don't keep
-- a reference (common when adding bodies with infinite mass).
--
-- However, the body doesn't need to keep references to
-- the attached shapes because cpBody do not reference them,
-- so it wouldn't notice at all if they disappeared =).
-- A space would notice, but then the space will keep its
-- own reference the the shape.

instance Eq Shape where
    S s1 _ == S s2 _ = s1 == s2

instance Ord Shape where
    S s1 _ `compare` S s2 _ = s1 `compare` s2



-- | A joint represents a constrain between two bodies. Don't
--   forget to add the bodies and the joint to the space.
data Joint = J !(ForeignPtr Joint) !Body !Body
type JointPtr = Ptr Joint

instance Eq Joint where
    J j1 _ _ == J j2 _ _ = j1 == j2

instance Ord Joint where
    J j1 _ _ `compare` J j2 _ _ = j1 `compare` j2



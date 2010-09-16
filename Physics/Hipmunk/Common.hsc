{-# CFILES
      chipmunk-r402/src/chipmunk.c,
      chipmunk-r402/src/constraints/cpConstraint.c,
      chipmunk-r402/src/constraints/cpDampedRotarySpring.c,
      chipmunk-r402/src/constraints/cpDampedSpring.c,
      chipmunk-r402/src/constraints/cpGearJoint.c,
      chipmunk-r402/src/constraints/cpGrooveJoint.c,
      chipmunk-r402/src/constraints/cpPinJoint.c,
      chipmunk-r402/src/constraints/cpPivotJoint.c,
      chipmunk-r402/src/constraints/cpRatchetJoint.c,
      chipmunk-r402/src/constraints/cpRotaryLimitJoint.c,
      chipmunk-r402/src/constraints/cpSimpleMotor.c,
      chipmunk-r402/src/constraints/cpSlideJoint.c,
      chipmunk-r402/src/cpArbiter.c,
      chipmunk-r402/src/cpArray.c,
      chipmunk-r402/src/cpBB.c,
      chipmunk-r402/src/cpBody.c,
      chipmunk-r402/src/cpCollision.c,
      chipmunk-r402/src/cpHashSet.c,
      chipmunk-r402/src/cpPolyShape.c,
      chipmunk-r402/src/cpShape.c,
      chipmunk-r402/src/cpSpace.c,
      chipmunk-r402/src/cpSpaceHash.c,
      chipmunk-r402/src/cpVect.c,
      Physics/Hipmunk/wrapper.c #-}

-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Common.hsc
-- Copyright   :  (c) 2008-2010 Felipe A. Lessa
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  provisional
-- Portability :  portable (needs FFI)
--
-- Functionality used by various modules and routines for
-- initialization and change of global variables.
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Common
    (-- * Initialization
     initChipmunk,

     -- * Basic data types
     CpFloat,
     infinity,
     Time,
     Angle,
     Distance,
     Damping,

     -- * Global variables
     -- $global_vars

     -- ** Shape counter
     -- $shape_counter
     resetShapeCounter,

     -- ** Contact persistence
     -- $contact_persistence
     getContactPersistence,
     setContactPersistence,

     -- ** Collision slop
     -- $collision_slop
     getCollisionSlop,
     setCollisionSlop,

     -- ** Bias coefficient
     -- $bias_coef
     BiasCoef,
     getBiasCoef,
     setBiasCoef,

     -- ** Constraint bias coefficient
     -- $constraint_bias_coef
     getConstraintBiasCoef,
     setConstraintBiasCoef,

     -- * Vectors
     Vector(..),
     Position,
     fromAngle,
     len,
     normalize,
     scale,
     toAngle,
     dot,
     cross,
     perp,
     project,
     rotate,
     unrotate
    )
    where

import Foreign hiding (rotate)
import Foreign.C.Types (CInt)
#include "wrapper.h"

error' :: String -> a
error' = error . ("Physics.Hipmunk.Common: " ++)

-- | Initilizes the Chipmunk library. This should be called
--   once before using any functions of this library.
initChipmunk :: IO ()
initChipmunk = cpInitChipmunk

foreign import ccall unsafe "wrapper.h"
    cpInitChipmunk :: IO ()


-- | The floating point type used internally in Chipmunk.
type CpFloat = #{type cpFloat}

-- | @infinity@ may be used to create bodies with
--   an infinite mass.
infinity :: CpFloat
infinity = 1e1000

-- | Type synonym used to hint that the argument or result
--   represents time.
type Time = CpFloat

-- | Type synonym used to hint that the argument or result
--   represents an angle in radians.
type Angle = CpFloat

-- | Type synonym used to hint that the argument or result
--   represents a distance.
type Distance = CpFloat

-- | Type synonym used to hint that the argument or result
--   represents a damping constant.
type Damping = CpFloat


-- $global_vars
--   Chipmunk tries to maintain a very few number of global
--   variables to allow multiple @Space@s to be used
--   simultaneously, however there are some.

-- $shape_counter
--   The shape counter is a global counter used for creating
--   unique hash identifiers to the shapes.

-- | @resetShapeCounter@ reset the shape counter to its default
--   value.  This is used to add determinism to a simulation.  As
--   the ids created with this counter may affect the order in
--   which the collisions happen, there may be very slight
--   differences in different simulations.  It may be very useful
--   to call @resetShapeCounter@ everytime you start a new
--   simulation.
--
--   However, be careful as you should not use shapes created
--   before a call to @resetCounter@ with shapes created after it
--   as they may have the same id.  This means that you can't add
--   shapes created after the call to a space created before it.
resetShapeCounter :: IO ()
resetShapeCounter = cpResetShapeIdCounter

foreign import ccall unsafe "wrapper.h"
    cpResetShapeIdCounter :: IO ()


-- $contact_persistence
--   This variable determines how long contacts should persist.
--   It should be small as the cached contacts will only be
--   close for a short time. (default is 3)

getContactPersistence :: IO CInt
getContactPersistence = peek cp_contact_persistence

setContactPersistence :: CInt -> IO ()
setContactPersistence = poke cp_contact_persistence

foreign import ccall unsafe "wrapper.h &cp_contact_persistence"
    cp_contact_persistence :: Ptr CInt


-- $collision_slop
--   The collision slop is the amount that shapes are allowed to
--   penetrate. Setting this to zero will work just fine, but using a
--   small positive amount will help prevent oscillating
--   contacts. (default is 0.1)

getCollisionSlop :: IO CpFloat
getCollisionSlop = peek cp_collision_slop

setCollisionSlop :: CpFloat -> IO ()
setCollisionSlop = poke cp_collision_slop

foreign import ccall unsafe "wrapper.h &cp_collision_slop"
    cp_collision_slop :: Ptr CpFloat


-- $bias_coef
--   The amount of penetration to reduce in each step. Values should
--   range from 0 to 1. Using large values will eliminate penetration in
--   fewer steps, but can cause vibration. (default is 0.1)
type BiasCoef = CpFloat

getBiasCoef :: IO BiasCoef
getBiasCoef = peek cp_bias_coef

setBiasCoef :: BiasCoef -> IO ()
setBiasCoef = poke cp_bias_coef

foreign import ccall unsafe "wrapper.h &cp_bias_coef"
    cp_bias_coef :: Ptr CpFloat


-- $constraint_bias_coef
--   Similar to the bias coefficient, but sets the default bias
--   for all constraints. (default is 0.1)

getConstraintBiasCoef :: IO BiasCoef
getConstraintBiasCoef = peek cp_constraint_bias_coef

setConstraintBiasCoef :: BiasCoef -> IO ()
setConstraintBiasCoef = poke cp_constraint_bias_coef

foreign import ccall unsafe "wrapper.h &cp_constraint_bias_coef"
    cp_constraint_bias_coef :: Ptr CpFloat



-- | A two-dimensional vector. It is an instance of 'Num'
--   however the operations 'signum' and @(*)@ are not
--   supported.
data Vector = Vector !CpFloat !CpFloat
              deriving (Eq, Show, Ord)

-- | Type synonym used to hint that the argument or result
--   represents a position.
type Position = Vector


instance Num Vector where
    (Vector x1 y1) + (Vector x2 y2) = Vector (x1+x2) (y1+y2)
    (Vector x1 y1) - (Vector x2 y2) = Vector (x1-x2) (y1-y2)
    negate (Vector x1 y1)           = Vector (-x1) (-y1)
    abs v                           = Vector (len v) 0
    fromInteger n                   = Vector (fromInteger n) 0
    signum _ = error' "signum not supported"
    _ * _    = error' "(*) not supported"

instance Storable Vector where
    sizeOf _    = #{size cpVect}
    alignment _ = alignment (undefined :: CpFloat)
    peek ptr = do
      x <- #{peek cpVect, x} ptr
      y <- #{peek cpVect, y} ptr
      return (Vector x y)
    poke ptr (Vector x y) = do
      #{poke cpVect, x} ptr x
      #{poke cpVect, y} ptr y


-- | Constructs an unitary vector pointing to the given
--   angle (in radians).
fromAngle :: Angle -> Vector
fromAngle theta = Vector (cos theta) (sin theta)
{-# INLINE fromAngle #-}

-- | The length of a vector.
len :: Vector -> CpFloat
len (Vector x y) = sqrt $ x*x + y*y
{-# INLINE len #-}

-- | Normalizes the vector (i.e. divides it by its length).
normalize :: Vector -> Vector
normalize v = v `scale` (recip $ len v)
{-# INLINE normalize #-}

-- | Scales the components of a vector by the same amount.
scale :: Vector -> CpFloat -> Vector
scale (Vector x y) s = Vector (x*s) (y*s)
{-# INLINE scale #-}

-- | @toAngle v@ is the angle that @v@ has
--   with the vector @Vector 1 0@ (modulo @2*pi@).
toAngle :: Vector -> Angle
toAngle (Vector x y) = atan2 y x
{-# INLINE toAngle #-}

-- | @v1 \`dot\` v2@ computes the familiar dot operation.
dot :: Vector -> Vector -> CpFloat
dot (Vector x1 y1) (Vector x2 y2) = x1*x2 + y1*y2
{-# INLINE dot #-}

-- | @v1 \`cross\` v2@ computes the familiar cross operation.
cross :: Vector -> Vector -> CpFloat
cross (Vector x1 y1) (Vector x2 y2) = x1*y2 - y1*x2
{-# INLINE cross #-}

-- | @perp v@ is a vector of same length as @v@ but perpendicular
--   to @v@ (i.e. @toAngle (perp v) - toAngle v@ equals @pi\/2@
--   modulo @2*pi@).
perp :: Vector -> Vector
perp (Vector x y) = Vector (-y) x
{-# INLINE perp #-}

-- | @v1 \`project\` v2@ is the vector projection of @v1@ onto @v2@.
project :: Vector -> Vector -> Vector
project v1 v2 = v2 `scale` s
    where s = (v1 `dot` v2) / (v2 `dot` v2)
{-# INLINE project #-}

-- | @v1 \`rotate\` v2@ uses complex multiplication
--   to rotate (and scale) @v1@ by @v2@.
rotate :: Vector -> Vector -> Vector
rotate (Vector x1 y1) (Vector x2 y2) = Vector x3 y3
    where x3 = x1*x2 - y1*y2
          y3 = x1*y2 + y1*x2
{-# INLINE rotate #-}

-- | The inverse operation of @rotate@, such that
--   @unrotate (rotate v1 v2) v2@ equals @v1@.
unrotate :: Vector -> Vector -> Vector
unrotate (Vector x1 y1) (Vector x2 y2) = Vector x3 y3
    where x3 = x1*x2 + y1*y2
          y3 = y1*x2 - x1*y2
{-# INLINE unrotate #-}

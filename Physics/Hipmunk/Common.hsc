{-# CFILES
      chipmunk/chipmunk.c
      chipmunk/cpArbiter.c
      chipmunk/cpArray.c
      chipmunk/cpBB.c
      chipmunk/cpBody.c
      chipmunk/cpCollision.c
      chipmunk/cpHashSet.c
      chipmunk/cpJoint.c
      chipmunk/cpPolyShape.c
      chipmunk/cpShape.c
      chipmunk/cpSpace.c
      chipmunk/cpSpaceHash.c
      chipmunk/cpVect.c
      Physics/Hipmunk/wrapper.c #-}

-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Common.hsc
-- Copyright   :  (c) Felipe A. Lessa 2008
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  beta
-- Portability :  portable (needs FFI)
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
infinity = 1e100

-- | Type synonym used to hint that the argument or result
--   represents time.
type Time = CpFloat

-- | Type synonym used to hint that the argument or result
--   represents an angle in radians.
type Angle = CpFloat

-- | A two-dimensional vector. It is an instance of 'Num'
--   however the operations 'signum' and '(*)' are not
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

-- | The length of a vector.
len :: Vector -> CpFloat
len (Vector x y) = sqrt $ x*x + y*y

-- | Normalizes the vector (i.e. divides it by its length).
normalize :: Vector -> Vector
normalize v = v `scale` (recip $ len v)

-- | Scales the components of a vector by the same amount.
scale :: Vector -> CpFloat -> Vector
scale (Vector x y) s = Vector (x*s) (y*s)

-- | @toAngle v@ is the angle that @v@ has
--   with the vector @Vector 1 0@ (modulo @2*pi@).
toAngle :: Vector -> Angle
toAngle (Vector x y) = atan2 y x

-- | @v1 \`dot\` v2@ computes the familiar dot operation.
dot :: Vector -> Vector -> CpFloat
dot (Vector x1 y1) (Vector x2 y2) = x1*x2 + y1*y2

-- | @v1 \`cross\` v2@ computes the familiar cross operation.
cross :: Vector -> Vector -> CpFloat
cross (Vector x1 y1) (Vector x2 y2) = x1*y2 - y1*x2

-- | @perp v@ is a vector of same length as @v@ but perpendicular
--   to @v@ (i.e. @toAngle (perp v) - toAngle v@ equals @pi\/2@
--   modulo @2*pi@).
perp :: Vector -> Vector
perp (Vector x y) = Vector (-y) x

-- | @v1 \`project\` v2@ is the vector projection of @v1@ onto @v2@.
project :: Vector -> Vector -> Vector
project v1 v2 = v2 `scale` s
    where s = (v1 `dot` v2) / (v2 `dot` v2)

-- | @v1 \`rotate\` v2@ uses complex multiplication
--   to rotate (and scale) @v1@ by @v2@.
rotate :: Vector -> Vector -> Vector
rotate (Vector x1 y1) (Vector x2 y2) = Vector x3 y3
    where x3 = x1*x2 - y1*y2
          y3 = x1*y2 + y1*x2

-- | The inverse operation of @rotate@, such that
--   @unrotate (rotate v1 v2) v2@ equals @v1@.
unrotate :: Vector -> Vector -> Vector
unrotate (Vector x1 y1) (Vector x2 y2) = Vector x3 y3
    where x3 = x1*x2 + y1*y2
          y3 = y1*x2 - x1*y2

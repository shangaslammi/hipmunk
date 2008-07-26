-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Internal.hsc
-- Copyright   :  (c) Felipe A. Lessa 2008
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  beta
-- Portability :  portable (needs FFI)
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Internal
    (VectorPtr,

     BodyPtr,
     Body(..),
     unB,

     ShapePtr,
     Shape(..),
     unS,

     JointPtr,
     Joint(..),
     unJ,

     SpacePtr,
     Space(..),
     unP,

     Contact(..),
     ContactPtr
    )
    where

import Data.IORef
import Data.Map (Map)
import Foreign
#include "wrapper.h"

import Physics.Hipmunk.Common


type VectorPtr = Ptr Vector



-- | A rigid body representing the physical properties of an object,
--   but without a shape. It may help to think as a particle that
--   is able to rotate.
newtype Body = B (ForeignPtr Body)
type BodyPtr = Ptr Body

unB :: Body -> ForeignPtr Body
unB (B b) = b

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

unS :: Shape -> ForeignPtr Shape
unS (S s _) = s

instance Eq Shape where
    S s1 _ == S s2 _ = s1 == s2

instance Ord Shape where
    S s1 _ `compare` S s2 _ = s1 `compare` s2



-- | A joint represents a constrain between two bodies. Don't
--   forget to add the bodies and the joint to the space.
data Joint = J !(ForeignPtr Joint) !Body !Body
type JointPtr = Ptr Joint

unJ :: Joint -> ForeignPtr Joint
unJ (J j _ _) = j

instance Eq Joint where
    J j1 _ _ == J j2 _ _ = j1 == j2

instance Ord Joint where
    J j1 _ _ `compare` J j2 _ _ = j1 `compare` j2



-- | A space is where the simulation really occurs. You add
--   bodies, shapes and joints to a space and then step it
--   to update it as whole.
data Space = P !(ForeignPtr Space)
               !(IORef Entities)   -- Active and static entities
               !(IORef Callbacks)  -- Added callbacks
type SpacePtr  = Ptr Space
type Entities  = Map (Ptr ()) (Either (ForeignPtr ()) Shape)
type Callbacks = (Maybe (FunPtr ()), -- Default
                  Map (#{type unsigned int}, #{type unsigned int})
                      (FunPtr ()))

unP :: Space -> ForeignPtr Space
unP (P sp _ _) = sp

instance Eq Space where
    P s1 _ _ == P s2 _ _ = s1 == s2

instance Ord Space where
    P s1 _ _ `compare` P s2 _ _ = s1 `compare` s2



-- 'Contact's are an exception to the pattern we've been following
-- as we're going to use StorableArray with them, so we need
-- them to be Storable (like Vector).

-- | A 'Contact' contains information about a collision.
--   It is passed to 'Physics.Hipmunk.Space.Full'.
--
--   The fields 'ctJnAcc' and 'ctJtAcc' do not have any meaningfull
--   value until 'Physics.Hipmunk.Space.step' has returned
--   (i.e. during a call to a callback this information
--   contains garbage), and by extension you can only know
--   the impulse sum after @step@ returns as well.
--
--   /IMPORTANT:/ You may maintain a reference to an array
--   of @Contact@s that was passed to a callback to do any other
--   processing later. However, /a new call to /@step@/ will
--   invalidate any of those arrays/! Be careful.
data Contact = Contact {
      ctPos    :: Position,
      -- ^ Position of the collision in world's coordinates.

      ctNormal :: Vector,
      -- ^ Normal of the collision.

      ctDist   :: CpFloat,
      -- ^ Penetration distance of the collision.

      ctJnAcc  :: CpFloat,
      -- ^ Normal component of final impulse applied.
      --   (Valid only after @step@ finishes.)

      ctJtAcc  :: CpFloat
      -- ^ Tangential component of final impulse applied.
      --   (Valid only after @step@ finishes.)
    }
               deriving (Eq, Ord, Show)

type ContactPtr = Ptr Contact

instance Storable Contact where
    sizeOf _    = #{size cpContact}
    alignment _ = alignment (undefined :: Vector)
    peek ptr    = do
      p     <- #{peek cpContact, p} ptr
      n     <- #{peek cpContact, n} ptr
      dist  <- #{peek cpContact, dist} ptr
      jnAcc <- #{peek cpContact, jnAcc} ptr
      jtAcc <- #{peek cpContact, jtAcc} ptr
      return $ Contact {ctPos    = p
                       ,ctNormal = n
                       ,ctDist   = dist
                       ,ctJnAcc  = jnAcc
                       ,ctJtAcc  = jtAcc}
    poke ptr c = do
      #{poke cpContact, p} ptr (ctPos c)
      #{poke cpContact, n} ptr (ctNormal c)
      #{poke cpContact, dist} ptr (ctDist c)
      #{poke cpContact, jnAcc} ptr (ctJnAcc c)
      #{poke cpContact, jtAcc} ptr (ctJtAcc c)

-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Body.hsc
-- Copyright   :  (c) Felipe A. Lessa 2008
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  provisional
-- Portability :  portable (needs FFI)
--
-- Rigid bodies and their properties.
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Body
    (-- * Creating
     Body,
     newBody,

     -- * Static properties
     -- ** Basic
     -- *** Mass
     Mass,
     getMass,
     setMass,
     -- *** Moment of inertia
     Moment,
     getMoment,
     setMoment,

     -- ** Linear components of motion
     -- *** Position
     getPosition,
     setPosition,
     -- *** Velocity
     Velocity,
     getVelocity,
     setVelocity,
     -- *** Force
     Force,
     getForce,
     setForce,

     -- ** Angular components of motion
     -- *** Angle
     getAngle,
     setAngle,
     -- *** Angular velocity
     AngVel,
     getAngVel,
     setAngVel,
     -- *** Torque
     Torque,
     getTorque,
     setTorque,

     -- * Dynamic properties
     slew,
     updateVelocity,
     updatePosition,
     resetForces,
     applyForce,
     applyOnlyForce,
     applyImpulse,
     applyDampedSpring,

     -- * Utilities
     localToWorld,
     worldToLocal
    )
    where

import Foreign hiding (rotate, new)
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal

-- | @newBody mass inertia@ creates a new 'Body' with
--   the given mass and moment of inertia.
--
--   It is recommended to call 'setPosition' afterwards.
newBody :: CpFloat -> CpFloat -> IO Body
newBody mass inertia = do
  b <- mallocForeignPtrBytes #{size cpBody}
  withForeignPtr b $ \ptr -> do
    cpBodyInit ptr mass inertia
  return (B b)

foreign import ccall unsafe "wrapper.h"
    cpBodyInit :: BodyPtr -> CpFloat -> CpFloat -> IO ()



type Mass = CpFloat

getMass :: Body -> IO Mass
getMass (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, m} ptr

setMass :: Body -> Mass -> IO ()
setMass (B b) m = do
  withForeignPtr b $ \ptr -> do
    cpBodySetMass ptr m

foreign import ccall unsafe "wrapper.h"
    cpBodySetMass :: BodyPtr -> Mass -> IO ()


type Moment = CpFloat

getMoment :: Body -> IO Moment
getMoment (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, i} ptr

setMoment :: Body -> Moment -> IO ()
setMoment (B b) i = do
  withForeignPtr b $ \ptr -> do
    cpBodySetMoment ptr i

foreign import ccall unsafe "wrapper.h"
    cpBodySetMoment :: BodyPtr -> CpFloat -> IO ()



getAngle :: Body -> IO Angle
getAngle (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, a} ptr

setAngle :: Body -> Angle -> IO ()
setAngle (B b) a = do
  withForeignPtr b $ \ptr -> do
    cpBodySetAngle ptr a

foreign import ccall unsafe "wrapper.h"
    cpBodySetAngle :: BodyPtr -> CpFloat -> IO ()



getPosition :: Body -> IO Position
getPosition (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, p} ptr

-- | Note that using this function to change the position
--   on every step is not recommended as it may leave
--   the velocity out of sync.
setPosition :: Body -> Position -> IO ()
setPosition (B b) pos = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, p} ptr pos


type Velocity = Vector

getVelocity :: Body -> IO Velocity
getVelocity (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, v} ptr

setVelocity :: Body -> Velocity -> IO ()
setVelocity (B b) v = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, v} ptr v



type Force = Vector

getForce :: Body -> IO Force
getForce (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, f} ptr

setForce :: Body -> Force -> IO ()
setForce (B b) f = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, f} ptr f


type AngVel = CpFloat

getAngVel :: Body -> IO AngVel
getAngVel (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, w} ptr

setAngVel :: Body -> AngVel -> IO ()
setAngVel (B b) w = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, w} ptr w


type Torque = CpFloat

getTorque :: Body -> IO Torque
getTorque (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, t} ptr

setTorque :: Body -> Torque -> IO ()
setTorque (B b) t = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, t} ptr t


-- | @slew b newpos dt@ changes the body @b@'s velocity
--   so that it reaches @newpos@ in @dt@ time.
--
--   It is usually used to change the position of a
--   static body in the world. In that case, remember
--   to reset the velocity to zero afterwards!
slew :: Body -> Position -> Time -> IO ()
slew (B b) newpos dt = do
  withForeignPtr b $ \ptr -> do
    p <- #{peek cpBody, p} ptr
    #{poke cpBody, v} ptr $ (newpos - p) `scale` (recip dt)


-- | @updateVelocity b gravity damping dt@ redefines body @b@'s
--   linear and angular velocity to account for the force\/torque
--   being applied to it, the gravity and a damping factor
--   during @dt@ time using Euler integration.
--
--   Note that this function only needs to be called if you
--   are not adding the body to a space.
updateVelocity :: Body -> Vector -> CpFloat -> Time -> IO ()
updateVelocity (B b) g d dt =
  withForeignPtr b $ \b_ptr ->
  with g $ \g_ptr -> do
    wrBodyUpdateVelocity b_ptr g_ptr d dt

foreign import ccall unsafe "wrapper.h"
    wrBodyUpdateVelocity :: BodyPtr -> VectorPtr
                         -> CpFloat -> Time -> IO ()


-- | @updatePosition b dt@ redefines the body position like
--   'updateVelocity' (and it also shouldn't be called if you
--   are adding this body to a space).
updatePosition :: Body -> Time -> IO ()
updatePosition (B b) dt = do
  withForeignPtr b $ \ptr -> do
    cpBodyUpdatePosition ptr dt

foreign import ccall unsafe "wrapper.h"
    cpBodyUpdatePosition :: BodyPtr -> Time -> IO ()


-- | @resetForces b@ redefines as zero all forces and torque
--   acting on body @b@.
resetForces :: Body -> IO ()
resetForces b = do
  setForce b 0
  setTorque b 0


-- | @applyForce b f r@ applies to the body @b@ the force
--   @f@ with offset @r@, both vectors in world coordinates.
--   This is the most stable way to change a body's velocity.
--
--   Note that the force is accumulated in the body, so you
--   may need to call 'applyOnlyForce'.
applyForce :: Body -> Vector -> Position -> IO ()
applyForce (B b) f p =
  withForeignPtr b $ \b_ptr ->
  with f $ \f_ptr ->
  with p $ \p_ptr -> do
    wrBodyApplyForce b_ptr f_ptr p_ptr

foreign import ccall unsafe "wrapper.h"
    wrBodyApplyForce :: BodyPtr -> VectorPtr -> VectorPtr -> IO ()


-- | @applyOnlyForce b f r@ applies a force like 'applyForce',
--   but calling 'resetForces' before. Note that using this
--   function is preferable as it is optimized over this common
--   case.
applyOnlyForce :: Body -> Vector -> Position -> IO ()
applyOnlyForce b f p = do
  setForce b f
  setTorque b (p `cross` f)


-- | @applyImpulse b j r@ applies to the body @b@ the impulse
--   @j@ with offset @r@, both vectors in world coordinates.
applyImpulse :: Body -> Vector -> Position -> IO ()
applyImpulse (B b) j r =
  withForeignPtr b $ \b_ptr ->
  with j $ \j_ptr ->
  with r $ \r_ptr -> do
    wrBodyApplyImpulse b_ptr j_ptr r_ptr

foreign import ccall unsafe "wrapper.h"
    wrBodyApplyImpulse :: BodyPtr -> VectorPtr -> VectorPtr -> IO ()


-- | @dampedSpring (b1,a1) (b2,a2) rlen k dmp dt@ applies a damped
--   spring force between bodies @b1@ and @b2@ at anchors
--   @a1@ and @a2@, respectively. @k@ is the spring constant
--   (force\/distance), @rlen@ is the rest length of the spring,
--   @dmp@ is the damping constant (force\/velocity), and @dt@
--   is the time step to apply the force over. Both anchors are
--   in body coordinates.
--
--   Note: large damping values can be unstable, you should use
--   the damped spring constraint instead.
applyDampedSpring :: (Body,Position) -> (Body,Position) -> CpFloat
                  -> CpFloat -> CpFloat -> Time -> IO ()
applyDampedSpring (B b1,a1) (B b2, a2) rlen k dmp dt =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with a1 $ \a1_ptr ->
  with a2 $ \a2_ptr -> do
    wrApplyDampedSpring b1_ptr b2_ptr a1_ptr a2_ptr rlen k dmp dt

foreign import ccall unsafe "wrapper.h"
    wrApplyDampedSpring :: BodyPtr -> BodyPtr -> VectorPtr -> VectorPtr
                        -> CpFloat -> CpFloat -> CpFloat -> Time -> IO ()


-- | For a vector @p@ in body @b@'s coordinates,
--   @localToWorld b p@ returns the corresponding vector
--   in world coordinates.
localToWorld :: Body -> Position -> IO Position
localToWorld (B b) p =
  withForeignPtr b $ \b_ptr ->
  with p $ \p_ptr -> do
    wrBodyLocal2World b_ptr p_ptr
    peek p_ptr

foreign import ccall unsafe "wrapper.h"
    wrBodyLocal2World :: BodyPtr -> VectorPtr -> IO ()


-- | For a vector @p@ in world coordinates,
--   @worldToLocal b p@ returns the corresponding vector
--   in body @b@'s coordinates.
worldToLocal :: Body -> Position -> IO Position
worldToLocal (B b) p =
  withForeignPtr b $ \b_ptr ->
  with p $ \p_ptr -> do
    wrBodyWorld2Local b_ptr p_ptr
    peek p_ptr

foreign import ccall unsafe "wrapper.h"
    wrBodyWorld2Local :: BodyPtr -> VectorPtr -> IO ()

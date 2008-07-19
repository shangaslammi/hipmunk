module Physics.Hipmunk.Body
    (-- * Creating
     Body,
     new,

     -- * Static properties
     getMass, setMass,
     getMoment, setMoment,

     -- ** Linear components of motion
     getPosition, setPosition,
     getVelocity, setVelocity,
     getForce, setForce,

     -- ** Angular components of motion
     getAngle, setAngle,
     getAngVel, setAngVel,
     getTorque, setTorque,

     -- * Dynamic properties
     slew,
     updateVelocity,
     updatePosition,
     resetForces,
     applyForce,
     applyOnlyForce,
     applyImpulse,
     dampedSpring,

     -- * Utilities
     localToWorld,
     worldToLocal
    )
    where

import Foreign hiding (rotate, new)
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal

-- | @new mass inertia@ creates a new 'Body' with
--   the given mass and moment of inertia.
--
--   It is recommended to call 'setPosition' afterwards.
new :: CpFloat -> CpFloat -> IO Body
new mass inertia = do
  b <- mallocForeignPtr
  withForeignPtr b $ \ptr -> do
    cpBodyInit ptr mass inertia
  return (B b)

foreign import ccall unsafe "chipmunk.h"
    cpBodyInit :: BodyPtr -> CpFloat -> CpFloat -> IO ()


-- | @getMass b@ returns the mass of the body @b@.
getMass :: Body -> IO CpFloat
getMass (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, m} ptr


-- | @setMass b m@ redefines the mass of the body @b@ to be @m@.
setMass :: Body -> CpFloat -> IO ()
setMass (B b) m = do
  withForeignPtr b $ \ptr -> do
    cpBodySetMass ptr m

foreign import ccall unsafe "chipmunk.h"
    cpBodySetMass :: BodyPtr -> CpFloat -> IO ()


-- | @getMoment b@ returns the moment of inertia
--   of the body @b@.
getMoment :: Body -> IO CpFloat
getMoment (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, i} ptr


-- | @setMoment b i@ redefines the moment of
--   inertia of the body @b@ to be @i@.
setMoment :: Body -> CpFloat -> IO ()
setMoment (B b) i = do
  withForeignPtr b $ \ptr -> do
    cpBodySetMoment ptr i

foreign import ccall unsafe "chipmunk.h"
    cpBodySetMoment :: BodyPtr -> CpFloat -> IO ()


-- | @getAngle b@ returns the angle of the body @b@
--   (initially zero).
getAngle :: Body -> IO Angle
getAngle (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, a} ptr


-- | @setAngle b a@ defines the angle of the body @b@
--   to be @a@ (in radians).
setAngle :: Body -> Angle -> IO ()
setAngle (B b) a = do
  withForeignPtr b $ \ptr -> do
    cpBodySetAngle ptr a

foreign import ccall unsafe "chipmunk.h"
    cpBodySetAngle :: BodyPtr -> CpFloat -> IO ()


-- | @getPosition b@ returns the current position
--   of the body @b@.
getPosition :: Body -> IO Position
getPosition (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, p} ptr


-- | @setPosition b pos@ defines the position of the
--   body @b@ to be @pos@.
--
--   Note that using this function to change the position
--   on every step is not recommended as it may leave
--   the velocity out of sync.
setPosition :: Body -> Position -> IO ()
setPosition (B b) pos = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, p} ptr pos


-- | @getVelocity b@ returns the current velocity of
--   the body @b@.
getVelocity :: Body -> IO Vector
getVelocity (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, v} ptr


-- | @setVelocity b v@ defines the velocity of the
--   body @b@ to be @v@.
setVelocity :: Body -> Vector -> IO ()
setVelocity (B b) v = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, v} ptr v


-- | @getForce b@ returns the current force being applied
--   to the body @b@.
getForce :: Body -> IO Vector
getForce (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, f} ptr


-- | @setForce b f@ defines @f@ as the force to be
--   applied to the body @b@.
setForce :: Body -> Vector -> IO ()
setForce (B b) f = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, f} ptr f


-- | @getAngVel b@ returns the current angular velocity
--   of the body @b@.
getAngVel :: Body -> IO CpFloat
getAngVel (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, w} ptr


-- | @setAngVel b w@ defines the angular velocity of
--   the body @b@ to be @w@.
setAngVel :: Body -> CpFloat -> IO ()
setAngVel (B b) w = do
  withForeignPtr b $ \ptr -> do
    #{poke cpBody, w} ptr w


-- | @getTorque b@ returns the current torque being
--   applied to the body @b@.
getTorque :: Body -> IO CpFloat
getTorque (B b) = do
  withForeignPtr b $ \ptr -> do
    #{peek cpBody, t} ptr


-- | @setTorque b t@ defines the torque of the body @b@
--   to be @t@.
setTorque :: Body -> CpFloat -> IO ()
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
    #{poke cpBody, v} ptr $ (p - newpos) `scale` (recip dt)


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
--   Note: not solving the damping forces in the impulse solver
--   causes problems with large damping values. This function
--   will eventually be replaced by a new constraint (joint) type.
dampedSpring :: (Body,Position) -> (Body,Position) -> CpFloat
             -> CpFloat -> CpFloat -> Time -> IO ()
dampedSpring (B b1,a1) (B b2, a2) rlen k dmp dt =
  withForeignPtr b1 $ \b1_ptr ->
  withForeignPtr b2 $ \b2_ptr ->
  with a1 $ \a1_ptr ->
  with a2 $ \a2_ptr -> do
    wrDampedSpring b1_ptr b2_ptr a1_ptr a2_ptr rlen k dmp dt

foreign import ccall unsafe "wrapper.h"
    wrDampedSpring :: BodyPtr -> BodyPtr -> VectorPtr -> VectorPtr
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

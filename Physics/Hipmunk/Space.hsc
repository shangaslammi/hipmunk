-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Space.hsc
-- Copyright   :  (c) Felipe A. Lessa 2008
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  beta
-- Portability :  portable (needs FFI)
--
-- The space, where the simulation happens and the various entities
-- interact.
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Space
    (-- * Callbacks problem
     -- $callbacksProblem

     -- * Creating spaces and adding entities
     Space,
     newSpace,
     freeSpace,
     Entity(..),
     StaticShape(..),

     -- * Properties
     -- ** Iterations
     Iterations,
     getIterations,
     setIterations,
     -- ** Elastic iterations
     ElasticIterations,
     getElasticIterations,
     setElasticIterations,
     -- ** Gravity
     Gravity,
     getGravity,
     setGravity,
     -- ** Damping
     Damping,
     getDamping,
     setDamping,
     -- ** Time stamp
     TimeStamp,
     getTimeStamp,

     -- * Spatial hashes
     -- $resizing
     resizeStaticHash,
     resizeActiveHash,
     rehashStatic,
     -- ** Point query
     -- $point_query
     QueryType(..),
     spaceQuery,
     spaceQueryList,

     -- * Stepping
     step,

     -- ** Collision pair functions
     -- $callbacks
     Callback(..),
     setDefaultCallback,
     addCallback,
     removeCallback,

     -- ** Contacts
     Contact(..),
     sumImpulses,
     sumImpulsesWithFriction,
    )
    where

import Control.Exception (bracket)
import Data.Array.Storable
import Data.IORef
import qualified Data.Map as M
import Foreign hiding (new)
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal
import Physics.Hipmunk.Shape


-- $callbacksProblem
--   We have a huge problem for callbacks: we *have* to call
--   'freeHaskellFunPtr' to every Haskell function that was
--   passed via FFI to C code after we don't need them.
--   However, the 'ForeignPtr' that the 'Space' has can
--   portably have finalizers only in the FFI, never in the
--   Haskell land, so we can't run the Haskell function
--   'freeHaskellFunPtr' from a 'ForeignPtr' finalizer.
--
--   There are two options:
--
--     1. Use "Foreign.Concurrent" to add a Haskell finalizer.
--        Under GHC this is great and adds no overhead (maybe there's
--        even less overhead than calling a C function).
--        However "Foreign.Concurrent" is not portable and
--        works only under GHC.
--
--     2. Require that users of the library (you) call
--        a finalizer function when they plan to stop using
--        the space. This adds some burden to the programmer
--        and somehow defeats the purpose of the GC, however
--        it works everywhere.
--
--   As this is a library that intends to be as portable as
--   possible (like Chipmunk itself), of course I chose
--   to follow the second path. This means that your code will
--   run unchanged on every Haskell environment supporting
--   FFI with C99, but also that you have to take care to
--   avoid memory leaks. You've been warned! :)


-- | Creates a new, empty space.
--   Some of the memory resources associated with the space
--   must be manually freed through 'freeSpace' when the
--   'Space' is no longer necessary.
newSpace :: IO Space
newSpace =
  mallocForeignPtrBytes #{size cpSpace} >>= \sp ->
  withForeignPtr sp $ \sp_ptr -> do
    cpSpaceInit sp_ptr
    addForeignPtrFinalizer cpSpaceDestroy sp
    entities  <- newIORef M.empty
    callbacks <- newIORef (Nothing, M.empty)
    return (P sp entities callbacks)

foreign import ccall unsafe "wrapper.h"
    cpSpaceInit :: SpacePtr -> IO ()
foreign import ccall unsafe "wrapper.h &cpSpaceDestroy"
    cpSpaceDestroy :: FunPtr (SpacePtr -> IO ())


-- | @freeSpace sp@ frees some memory resources that can't
--   be automatically deallocated in a portable way.
--   The space @sp@ then becomes invalid and should
--   not be used (passing @sp@ to any other function,
--   including 'freeSpace', results in undefined behavior).
freeSpace :: Space -> IO ()
freeSpace (P _ entities callbacks) = do
  -- The only things we *have* to free are the callbacks,
  -- but we'll release all the IORef contents as well.
  let err :: a
      err = error "Physics.Hipmunk.Space: freeSpace already called here."
  writeIORef entities err
  (def,cbs) <- readIORef callbacks
  writeIORef callbacks err
  maybe (return ()) freeHaskellFunPtr def
  M.fold ((>>) . freeHaskellFunPtr) (return ()) cbs


-- | Type class implemented by entities that can be
--   added to a space.
class Entity a where
    -- | Add an entity to a 'Space'. Don't add the same
    --   entity twice to a space.
    spaceAdd :: Space -> a -> IO ()
    -- | Remove an entity from a 'Space'. Don't remove
    --   an entity that wasn't add.
    spaceRemove :: Space -> a -> IO ()

spaceAddHelper :: (a -> ForeignPtr b)
               -> (SpacePtr -> Ptr b -> IO ())
               -> (a -> Maybe Shape)
               -> (Space -> a -> IO ())
spaceAddHelper get add toShape =
    \(P sp entities _) new_c ->
        let new  = get new_c
            key  = unsafeForeignPtrToPtr $ castForeignPtr new
            val  = case toShape new_c of
                     Just shape -> Right shape
                     Nothing    -> Left (castForeignPtr new)
        in withForeignPtr sp $ \sp_ptr ->
           withForeignPtr new $ \new_ptr -> do
             add sp_ptr new_ptr
             modifyIORef entities (M.insert key val)

spaceRemoveHelper :: (a -> ForeignPtr b)
                  -> (SpacePtr -> Ptr b -> IO ())
                  -> (Space -> a -> IO ())
spaceRemoveHelper get remove =
    \(P sp entities _) old_c -> do
      let old  = get old_c
          key  = unsafeForeignPtrToPtr $ castForeignPtr old
      modifyIORef entities (M.delete key)
      withForeignPtr sp $ \sp_ptr ->
        withForeignPtr old $ \old_ptr ->
          remove sp_ptr old_ptr

instance Entity Body where
    spaceAdd    = spaceAddHelper    unB cpSpaceAddBody (const Nothing)
    spaceRemove = spaceRemoveHelper unB cpSpaceRemoveBody
foreign import ccall unsafe "wrapper.h"
    cpSpaceAddBody :: SpacePtr -> BodyPtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    cpSpaceRemoveBody :: SpacePtr -> BodyPtr -> IO ()

instance Entity Shape where
    spaceAdd    = spaceAddHelper    unS cpSpaceAddShape Just
    spaceRemove = spaceRemoveHelper unS cpSpaceRemoveShape
foreign import ccall unsafe "wrapper.h"
    cpSpaceAddShape :: SpacePtr -> ShapePtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    cpSpaceRemoveShape :: SpacePtr -> ShapePtr -> IO ()

instance Entity Joint where
    spaceAdd    = spaceAddHelper    unJ cpSpaceAddJoint (const Nothing)
    spaceRemove = spaceRemoveHelper unJ cpSpaceRemoveJoint
foreign import ccall unsafe "wrapper.h"
    cpSpaceAddJoint :: SpacePtr -> JointPtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    cpSpaceRemoveJoint :: SpacePtr -> JointPtr -> IO ()


-- | A 'StaticShape' is a 'Shape' container that, when added
--   to a space via 'spaceAdd', is added to the static
--   list of shapes.
--
--   A static shape is one assumed not to move. If you move
--   a static shape after adding it, then you need to 'rehashStatic'.
--
--   You should not add the same shape as active and static,
--   nor should you add as active and try to remove as
--   static or vice versa.
newtype StaticShape = Static {unStatic :: Shape}

instance Entity StaticShape where
    spaceAdd    = spaceAddHelper    (unS . unStatic) cpSpaceAddStaticShape (Just . unStatic)
    spaceRemove = spaceRemoveHelper (unS . unStatic) cpSpaceRemoveStaticShape
foreign import ccall unsafe "wrapper.h"
    cpSpaceAddStaticShape :: SpacePtr -> ShapePtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    cpSpaceRemoveStaticShape :: SpacePtr -> ShapePtr -> IO ()





-- | The number of iterations to use when solving constraints.
--   (default is 10).
type Iterations = #{type int}
getIterations :: Space -> IO Iterations
getIterations (P sp _ _) =
    withForeignPtr sp #{peek cpSpace, iterations}
setIterations :: Space -> Iterations -> IO ()
setIterations (P sp _ _) it =
    withForeignPtr sp $ \sp_ptr -> do
      #{poke cpSpace, iterations} sp_ptr it

-- | The number of elastic iterations to use when solving constraints.
--   (default is 0).
type ElasticIterations = #{type int}
getElasticIterations :: Space -> IO ElasticIterations
getElasticIterations (P sp _ _) =
    withForeignPtr sp #{peek cpSpace, elasticIterations}
setElasticIterations :: Space -> ElasticIterations -> IO ()
setElasticIterations (P sp _ _) it =
    withForeignPtr sp $ \sp_ptr -> do
      #{poke cpSpace, elasticIterations} sp_ptr it

-- | The gravity applied to the system. (default is 0)
type Gravity = Vector
getGravity :: Space -> IO Gravity
getGravity (P sp _ _) =
    withForeignPtr sp #{peek cpSpace, gravity}
setGravity :: Space -> Gravity -> IO ()
setGravity (P sp _ _) g =
    withForeignPtr sp $ \sp_ptr -> do
      #{poke cpSpace, gravity} sp_ptr g

-- | The amount of viscous damping applied to the system.
--   (default is 1)
type Damping = CpFloat
getDamping :: Space -> IO Damping
getDamping (P sp _ _) =
    withForeignPtr sp #{peek cpSpace, damping}
setDamping :: Space -> Damping -> IO ()
setDamping (P sp _ _) dm =
    withForeignPtr sp $ \sp_ptr -> do
      #{poke cpSpace, damping} sp_ptr dm

-- | The time stamp of the simulation, increased in 1
--   every time 'step' is called.
type TimeStamp = #{type int}
getTimeStamp :: Space -> IO TimeStamp
getTimeStamp (P sp _ _) =
    withForeignPtr sp #{peek cpSpace, stamp}





-- $resizing
--   @'resizeStaticHash' sp dim count@ resizes the static
--   hash of space @sp@ to have hash cells of size @dim@
--   and suggested minimum number of cells @count@.
--   @'resizeActiveHash' sp dim count@ works the same way
--   but modifying the active hash of the space.
--
--   Chipmunk's performance is highly sensitive to both
--   parameters, which should be hand-tuned to maximize
--   performance. It is in general recommended to set
--   @dim@ as the average object size and @count@ around
--   10 times the number of objects in the hash. Usually
--   bigger numbers are better to @count@, but only to
--   point. By default dim is @100.0@ and count is @1000@.
--
--   Note that in the case of the static hash you may try
--   larger numbers as the static hash is only rehashed
--   when requested by 'rehashStatic', however that will
--   use more memory.

resizeStaticHash :: Space -> CpFloat -> #{type int} -> IO ()
resizeStaticHash (P sp _ _) dim count =
    withForeignPtr sp $ \sp_ptr -> do
      cpSpaceResizeStaticHash sp_ptr dim count

foreign import ccall unsafe "wrapper.h"
    cpSpaceResizeStaticHash :: SpacePtr -> CpFloat
                            -> #{type int} -> IO ()

resizeActiveHash :: Space -> CpFloat -> #{type int} -> IO ()
resizeActiveHash (P sp _ _) dim count =
  withForeignPtr sp $ \sp_ptr -> do
    cpSpaceResizeActiveHash sp_ptr dim count

foreign import ccall unsafe "wrapper.h"
    cpSpaceResizeActiveHash :: SpacePtr -> CpFloat
                            -> #{type int} -> IO ()

-- | Rehashes the shapes in the static spatial hash.
--   You only need to call this if you move one of the
--   static shapes.
rehashStatic :: Space -> IO ()
rehashStatic (P sp _ _) =
    withForeignPtr sp cpSpaceRehashStatic

foreign import ccall unsafe "wrapper.h"
    cpSpaceRehashStatic :: SpacePtr -> IO ()




-- $point_query
--   Point querying uses the spatial hashes to find out
--   in what shapes a point is contained. It is useful,
--   for example, to know if a shape was clicked by
--   the user.

-- | You may query the static hash, the active hash
--   or both.
data QueryType = ActiveHash | StaticHash | Both

-- | @spaceQuery sp query pos cb@ will call @cb@ for every
--   shape that
--
--   * Contains point @pos@ (in world's coordinates).
--
--   * Is in the hash selected by @query@ (see 'QueryType').
--
--   The order in which the callback is called is unspecified.
--   However it is guaranteed that it will be called once,
--   and only once, for each of the shapes described above
--   (and never for those who aren't).
spaceQuery :: Space -> QueryType -> Position -> (Shape -> IO ()) -> IO ()
spaceQuery spce@(P sp _ _) query pos callback =
  withForeignPtr sp $ \sp_ptr ->
  bracket (makePointQueryFunc cb) freeHaskellFunPtr $ \cb_ptr ->
  with pos $ \pos_ptr ->
    func sp_ptr pos_ptr cb_ptr
 where
   func = case query of
            ActiveHash -> wrSpaceActiveShapePointQuery
            StaticHash -> wrSpaceStaticShapePointQuery
            Both -> wrSpaceBothShapePointQuery
   cb shape_ptr _ = retriveShape spce shape_ptr >>= callback

type PointQueryFunc = ShapePtr -> Ptr () -> IO ()
type PointQueryFuncPtr = FunPtr PointQueryFunc
foreign import ccall "wrapper"
    makePointQueryFunc :: PointQueryFunc -> IO PointQueryFuncPtr
foreign import ccall safe "wrapper.h"
    wrSpaceActiveShapePointQuery
        :: SpacePtr -> VectorPtr -> PointQueryFuncPtr -> IO ()
foreign import ccall safe "wrapper.h"
    wrSpaceStaticShapePointQuery
        :: SpacePtr -> VectorPtr -> PointQueryFuncPtr -> IO ()
foreign import ccall safe "wrapper.h"
    wrSpaceBothShapePointQuery
        :: SpacePtr -> VectorPtr -> PointQueryFuncPtr -> IO ()


-- | @spaceQueryList sp query pos@ acts like 'spaceQuery' but
--   returns a list of 'Shape's instead of calling a callback.
--   This is just a convenience function.
spaceQueryList :: Space -> QueryType -> Position -> IO [Shape]
spaceQueryList spce query pos = do
  var <- newIORef []
  spaceQuery spce query pos $ modifyIORef var . (:)
  readIORef var


-- | @step sp dt@ will update the space @sp@ for a @dt@ time
--   step.
--
--   It is highly recommended to use a fixed @dt@ to increase
--   the efficiency of contact persistence. Some tips may be
--   found in <http://www.gaffer.org/game-physics/fix-your-timestep>.
step :: Space -> Time -> IO ()
step (P sp _ _) dt =
  withForeignPtr sp $ \sp_ptr -> do
    cpSpaceStep sp_ptr dt

-- IMPORTANT! This call can (and probably will) callback into Haskell.
foreign import ccall {- !!! -} safe {- !!! -}
    cpSpaceStep :: SpacePtr -> Time -> IO ()




-- $callbacks
--   A collision pair function is a callback triggered by 'step'
--   in response to certain collision events. Its return value
--   will determine whether or not the collision will be processed.
--   If @False@, then the collision will be ignored.
--
--   The callbacks themselves may execute arbitrary operations
--   with a simple exception: /callbacks cannot add or remove
--   entities from the space/. You can of course create a queue
--   of add\/remove actions and then process it after 'step'
--   returns.
--
--   As for the events that trigger collision pair functions,
--   the rule is simple. All shapes have a 'CollisionType'.
--   When shapes @a@ and @b@ collide, if there was a callback
--   associated with @a@'s and @b@'s collision types, then
--   it is called. Otherwise the default callback is called.
--   By default, the default callback always returns @True@
--   (i.e. all collisions are treated).


-- | A 'Callback' function can be of three types:
--
--   * A 'Full' callback has access to all parameters passed
--     by Chipmunk, but it is common not to need all of them.
--     The two colliding 'Shape's are passed as arguments with
--     a 'Contact' array and a normal coefficient (this coefficient
--     should be multiplied to the contacts' normals as
--     Chipmunk may have reversed the argument order). See 'Contact'
--     for more information.
--
--   * A 'Basic' callback can't access the 'Contact' information,
--     but incurs a lower overhead per call.
--
--   * A 'Constant' callback always accepts or reject the collision.
--     For example, a @Constant False@ will never accept any
--     collision.
--
--   Although 'Basic' and 'Constant' can be implemented
--   in terms of 'Full', they're optimized to incur less overhead.
--   So try to use the simplest callback type
--   (e.g. @Constant False@ instead of @Basic (\_ _ -> return False)@).
data Callback = Full (Shape -> Shape -> StorableArray Int Contact
                      -> CpFloat -> IO Bool)
              | Basic (Shape -> Shape -> IO Bool)
              | Constant !Bool


-- | Internal. Type of callback used by Chipmunk.
type ChipmunkCB = ShapePtr -> ShapePtr -> ContactPtr -> #{type int}
                -> CpFloat -> Ptr () -> IO Int
type ChipmunkCBPtr = FunPtr ChipmunkCB


-- | Internal. Constructs a 'ChipmunkCB' from a 'Callback',
--   returning also the contents of the @data@ pointer.
adaptChipmunkCB :: Space -> Callback
                -> IO (ChipmunkCBPtr, Ptr (), Maybe (FunPtr ()))
adaptChipmunkCB _ (Constant bool) =
  let data_ = intPtrToPtr (if bool then 1 else 0)
  in return (wrConstantCallback, data_, Nothing)
adaptChipmunkCB spce (Basic basic) = makeChipmunkCB' $
  \ptr1 ptr2 _ _ _ _ -> do
    shape1 <- retriveShape spce ptr1
    shape2 <- retriveShape spce ptr2
    okay <- basic shape1 shape2
    return (if okay then 1 else 0)
adaptChipmunkCB spce (Full full) = makeChipmunkCB' $
  \ptr1 ptr2 cont_ptr cont_num normal_coef _ -> do
    shape1 <- retriveShape spce ptr1
    shape2 <- retriveShape spce ptr2

    -- Wrap the pointer in an array. Note that the memory
    -- is managed by Chipmunk, so we don't have finalizers.
    cont_fptr <- newForeignPtr_ cont_ptr
    let bounds = (0, fromIntegral $ cont_num-1)
    array <- unsafeForeignPtrToStorableArray cont_fptr bounds

    okay <- full shape1 shape2 array normal_coef
    return (if okay then 1 else 0)

makeChipmunkCB' :: ChipmunkCB
                -> IO (ChipmunkCBPtr, Ptr (), Maybe (FunPtr ()))
makeChipmunkCB' f = do
  f' <- makeChipmunkCB f
  return (f', nullPtr, Just $ castFunPtr f')

foreign import ccall "wrapper"
    makeChipmunkCB :: ChipmunkCB -> IO ChipmunkCBPtr

foreign import ccall unsafe "wrapper.h &wrConstantCallback"
    wrConstantCallback :: ChipmunkCBPtr

-- | Internal. Retrive a 'Shape' from a 'ShapePtr' and a 'Space'.
retriveShape :: Space -> ShapePtr -> IO Shape
retriveShape (P _ entities _) ptr = do
  ent <- readIORef entities
  Right shape <- M.lookup (castPtr ptr) ent
  return shape


-- | Defines a new default collision pair function.
--   This callback is called whenever two shapes @a@
--   and @b@ collide such that no other collision
--   pair function was defined to @a@'s and @b@'s
--   collision types. The default is @Constant True@.
setDefaultCallback :: Space -> Callback -> IO ()
setDefaultCallback spce@(P sp _ callbacks) func = do
  -- Find out whats our new function details
  -- (NULL for default means @Constant True@, optimize it)
  (cb,data_,hask) <-
      case func of
        Constant True -> return (nullFunPtr, nullPtr, Nothing)
        _             -> adaptChipmunkCB spce func

  -- Free the previous one
  (def,cbs) <- readIORef callbacks
  case def of
    Nothing  -> return ()
    Just ptr -> freeHaskellFunPtr ptr

  -- Define the new
  writeIORef callbacks (hask,cbs)
  withForeignPtr sp $ \sp_ptr -> do
    cpSpaceSetDefaultCollisionPairFunc sp_ptr cb data_

foreign import ccall unsafe "wrapper.h"
    cpSpaceSetDefaultCollisionPairFunc
        :: SpacePtr -> ChipmunkCBPtr -> Ptr () -> IO ()


-- | @addCallback sp (cta,ctb) f@ defines @f@ as the callback
--   to be called whenever a collision occurs between
--   a shape of collision type @cta@ and another of
--   collision type @ctb@ (and vice versa). Any other callback
--   already registered to handle @(cta,ctb)@ will be removed.
--
--   Note that you should /not/ add callbacks to both
--   combinations of @(cta,ctb)@ and @(ctb,cta)@. A good rule
--   of thumb is to always use @cta <= ctb@, although this
--   is not necessary.
addCallback :: Space -> (CollisionType, CollisionType) -> Callback -> IO ()
addCallback spce@(P sp _ callbacks) (cta,ctb) func = do
  -- Find out whats our new function details
  -- (NULL for a specific means @Constant False@, optimize it)
  (cb,data_,hask) <-
      case func of
        Constant False -> return (nullFunPtr, nullPtr, Nothing)
        _              -> adaptChipmunkCB spce func

  -- Free the previous one, using
  --   updateLookupWithKey :: Ord k => (k -> a -> Maybe a) -> k
  --                       -> Map k a -> (Maybe a, Map k a)
  (def,cbs) <- readIORef callbacks
  let (old,cbs') = M.updateLookupWithKey (\_ _ -> hask) (cta,ctb) cbs
  case old of
    Nothing  -> return ()
    Just ptr -> freeHaskellFunPtr ptr

  -- Define the new
  writeIORef callbacks (def,cbs')
  withForeignPtr sp $ \sp_ptr -> do
    cpSpaceAddCollisionPairFunc sp_ptr cta ctb cb data_

foreign import ccall unsafe "wrapper.h"
    cpSpaceAddCollisionPairFunc
        :: SpacePtr -> CollisionType -> CollisionType
        -> ChipmunkCBPtr -> Ptr () -> IO ()

-- | @removeCallback sp (cta,ctb)@ removes any callbacks that
--   were registered to handle @(cta,ctb)@ (see 'addCallback').
--   Any collisions that would be handled by the removed
--   callback will be handled by the default one (see
--   'setDefaultCallback').
--
--   Note that you should /always/ use the same order that
--   was passed to 'addCallback'. In other words, after
--   @addCallback sp (cta,ctb) f@ you should use
--   @removeCallback sp (cta,ctb)@, and /never/
--   @removeCallback sp (ctb,cta)@.
--
--   Although pointless, it is harmless to remove a callback
--   that was not added.
removeCallback :: Space -> (CollisionType, CollisionType) -> IO ()
removeCallback (P sp _ callbacks) (cta,ctb) = do
  -- Free the callback
  (def,cbs) <- readIORef callbacks
  let (old,cbs') = M.updateLookupWithKey (\_ _ -> Nothing) (cta,ctb) cbs
  case old of
    Nothing  -> return ()
    Just ptr -> freeHaskellFunPtr ptr

  -- Remove the callback
  --   Note that we need to call Chipmunk even if old is Nothing
  --   because wrConstantCallback is not added to cbs. And
  --   removing what was not added is harmless here.
  writeIORef callbacks (def,cbs')
  withForeignPtr sp $ \sp_ptr -> do
    cpSpaceRemoveCollisionPairFunc sp_ptr cta ctb

foreign import ccall unsafe "wrapper.h"
    cpSpaceRemoveCollisionPairFunc
      :: SpacePtr -> CollisionType -> CollisionType -> IO ()


-- | Sums the impulses applied to the given contact points.
--   'sumImpulses' sums only the normal components.
--   This function should be called only after 'step'
--   returns.
sumImpulses :: StorableArray Int Contact -> IO Vector
sumImpulses = sumImpulsesInternal wrContactsSumImpulses

foreign import ccall unsafe "wrapper.h"
    wrContactsSumImpulses :: ContactPtr -> #{type int}
                          -> VectorPtr -> IO ()

-- | Sums the impulses applied to the given contact points.
--   This function sums both the normal and tangential components
--   and should be called only after 'step' returns.
sumImpulsesWithFriction :: StorableArray Int Contact -> IO Vector
sumImpulsesWithFriction =
    sumImpulsesInternal wrContactsSumImpulsesWithFriction

foreign import ccall unsafe "wrapper.h"
    wrContactsSumImpulsesWithFriction :: ContactPtr -> #{type int}
                                      -> VectorPtr -> IO ()

sumImpulsesInternal :: (ContactPtr -> #{type int} -> VectorPtr -> IO ())
                    -> StorableArray Int Contact -> IO Vector
sumImpulsesInternal func sa = do
  (i1,i2) <- getBounds sa
  withStorableArray sa $ \sa_ptr ->
    with 0 $ \vec_ptr -> do
      func sa_ptr (fromIntegral $ i2-i1) vec_ptr
      peek vec_ptr


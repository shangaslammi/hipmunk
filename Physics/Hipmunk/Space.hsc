module Physics.Hipmunk.Space
    (-- * Callbacks problem
     -- $callbacksProblem

     -- * Creating spaces and adding entities
     Space,
     space,
     spaceFree,
     SpaceAdd(..),
     StaticShape(..),

     -- * Properties
     Iterations,
     getIterations,
     setIterations,
     Gravity,
     getGravity,
     setGravity,
     Damping,
     getDamping,
     setDamping,
     TimeStamp,
     getTimeStamp,

     -- * Spatial hashes
     -- $resizing
     resizeStaticHash,
     resizeActiveHash,
     rehashStatic,

     -- * Stepping
     step,

     -- * Collision pair functions
     -- $callbacks
     CollisionPairBasic,
     CollisionPairFull,
     CollisionPair
    )
    where

import qualified Data.Array.Storable as A
import qualified Data.Map as M
import Data.IORef
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
--
--   Some of the memory resources associated with the space
--   must be manually freed through 'spaceFree' when the
--   'Space' is no longer necessary.
space :: IO Space
space =
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


-- | @spaceFree sp@ frees some memory resources that can't
--   be automatically deallocated in a portable way.
--   The space @sp@ then becomes invalid and should
--   not be used (passing @sp@ to any other function,
--   including 'spaceFree', results in undefined behavior).
spaceFree :: Space -> IO ()
spaceFree (P _ entities callbacks) = do
  -- The only things we *have* to free are the callbacks,
  -- but we'll release all the IORef contents as well.
  let err :: a
      err = error "Physics.Hipmunk.Space: spaceFree already called here."
  writeIORef entities err
  (def,cbs) <- readIORef callbacks
  writeIORef callbacks err
  maybe (return ()) freeHaskellFunPtr def
  M.fold ((>>) . freeHaskellFunPtr) (return ()) cbs


-- | Type class implemented by entities that can be
--   added to a space.
class SpaceAdd a where
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
                     Just shape -> Right Shape
                     Nothing    -> Left castForeignPtr new
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

instance SpaceAdd Body where
    spaceAdd    = spaceAddHelper    unB cpSpaceAddBody (const Nothing)
    spaceRemove = spaceRemoveHelper unB cpSpaceRemoveBody
foreign import ccall unsafe "wrapper.h"
    cpSpaceAddBody :: SpacePtr -> BodyPtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    cpSpaceRemoveBody :: SpacePtr -> BodyPtr -> IO ()

instance SpaceAdd Shape where
    spaceAdd    = spaceAddHelper    unS cpSpaceAddShape Just
    spaceRemove = spaceRemoveHelper unS cpSpaceRemoveShape
foreign import ccall unsafe "wrapper.h"
    cpSpaceAddShape :: SpacePtr -> ShapePtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    cpSpaceRemoveShape :: SpacePtr -> ShapePtr -> IO ()

instance SpaceAdd Joint where
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

instance SpaceAdd StaticShape where
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
--   (default to 1)
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




-- | Rehashes the shapes in the static spatial hash.
--   You only need to call this if you move one of the
--   static shapes.
rehashStatic :: Space -> IO ()
rehashStatic (P sp _ _) =
    withForeignPtr sp cpSpaceRehashStatic

foreign import ccall unsafe "wrapper.h"
    cpSpaceRehashStatic :: SpacePtr -> IO ()

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



-- | @step sp dt@ will update the space @sp@ for a @dt@ time
--   step.
--
--   It is highly recommended to use a fixed @dt@ to increase
--   the efficiency of contact persistence. Some tips may be
--   found in <http://www.gaffer.org/game-physics/fix-your-timestep>.
step :: Space -> CpFloat -> IO ()
step (P sp _ _) dt =
  withForeignPtr sp $ \sp_ptr -> do
    cpSpaceStep sp_ptr dt

-- IMPORTANT! This call can (and probably will) callback into Haskell.
foreign import ccall {- !!! -} safe {- !!! -}
    cpSpaceStep :: SpacePtr -> CpFloat -> IO ()




-- $callbacks
--   A collision pair function is a callback triggered by 'step'
--   in response to certain collision events. Its return value
--   will determine whether or not the collision will be processed.
--   If @False@, then the collision will be ignored.
--
--   The callbacks themselves may execute arbitrary operations
--   with a simple exception: /callbacks cannot add or remove
--   entities from the space/. You can of course create a queue
--   of add/remove actions and then process it after 'step'
--   returns.
--
--   As for the events that trigger collision pair functions,
--   the rule is simple. All shapes have a 'CollisionType'.
--   When shapes @a@ and @b@ collide, if there was a callback
--   associated with @a@'s and @b@'s collision types, then
--   it is called. Otherwise the default callback is called.
--   By default, the default callback always returns @True@
--   (i.e. all collisions are treated).


-- | A 'CollisionPairBasic' is a callback that given the two
--   shapes, returns @False@ iff the collision is to be ignored.
type CollisionPairBasic = Shape -> Shape -> IO Bool

-- | A 'CollisionPairFull' is similar to 'CollisionPairBasic',
--   however also receives as parameter an array with contact
--   point information. A CpFloat XXX.... See XXX for more info.
type CollisionPairFull =
    (A.MArray a Contact IO, XXX)
 => Shape -> Shape -> a Int Contact -> CpFloat -> IO Bool

-- | A 'CollisionPair' function, after all, can be basic or full.
--   If you don't need access to the extra information
--   'CollisionPairFull' provide, then use 'CollisionPairBasic'
--   that has a lower overhead.
type CollisionPair = Either CollisionPairBasic CollisionPairFull


-- | Internal. Type of callback used by Chipmunk.
type ChipmunkCB = ShapePtr -> ShapePtr -> ContactPtr -> #{type int}
                -> CpFloat -> Ptr () -> IO Int


-- | Internal. Constructs a 'ChipmunkCB' from a 'CollisionPair'.
makeChipmunkCB :: Space -> CollisionPair -> ChipmunkCB
makeChipmunkCB space (Left basic) =
  \ptr1 ptr2 _ _ _ -> do
    (shape1, shape2) <- retriveShape space (ptr1, ptr2)
    okay <- basic shape1 shape2
    return (if okay then 1 else 0)
makeChipmunkCB space (Right full) =
  \ptr1 ptr2 cont_ptr cont_num normal_coef -> do
    (shape1, shape2) <- retriveShape space (ptr1, ptr2)
    array <- XXX cont_ptr cont_num
    okay <- full shape1 shape2 array normal_coef
    return (if okay then 1 else 0)


-- | Internal. Retrive a 'Shape' from a 'ShapePtr' and a 'Space'.
retriveShape :: Space -> (ShapePtr, ShapePtr) -> IO (Shape, Shape)
retriveShape (P _ entities _) (ptr1, ptr2) = do
  ent <- readIORef entities
  shape1 <- lookup (castPtr ptr1) ent
  shape2 <- lookup (castPtr ptr2) ent
  return (shape1, shape2)

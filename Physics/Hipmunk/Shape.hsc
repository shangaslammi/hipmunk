module Physics.Hipmunk.Shape
    (-- * Shapes
     Shape,
     shapeCircle,
     shapeSegment,
     shapePoly,
     resetCounter,

     -- * Properties
     -- ** Collision type
     CollisionType,
     getCollisionType,
     setCollisionType,
     -- ** Group
     Group,
     getGroup,
     setGroup,
     -- ** Layers
     Layers,
     getLayers,
     setLayers,
     -- ** Elasticity
     Elasticity,
     getElasticity,
     setElasticity,
     -- ** Friction
     Friction,
     getFriction,
     setFriction,
     -- ** Surface velocity
     SurfaceVel,
     getSurfaceVel,
     setSurfaceVel,

     -- * Utilities
     getBody,
     momentForCircle,
     momentForPoly,
     shapeQuery,

     -- ** For polygons
     -- $polygon_util
     isLeft,
     isClockwise,
     isConvex,
     polyReduce,
     polyCenter,
     convexHull
    )
    where

import Data.List (foldl', sortBy)
import Foreign hiding (rotate, new)
import Foreign.C
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal

-- | @shapeCircle b off r@ attaches a circle shape to
--   the body @b@ at the offset @off@ from the body's
--   center of gravity in @b@'s coordinates and with
--   a radius of @r@.
--
--   This is the fastest collision type
--   and it also rolls smoothly.
shapeCircle :: Body -> Position -> CpFloat -> IO Shape
shapeCircle body@(B b) off r =
  withForeignPtr b $ \b_ptr ->
  with off $ \off_ptr ->
  mallocForeignPtrBytes #{size cpCircleShape} >>= \shape ->
  withForeignPtr shape $ \shape_ptr -> do
    wrCircleShapeInit shape_ptr b_ptr off_ptr r
    return (S shape body)

foreign import ccall unsafe "wrapper.h"
    wrCircleShapeInit :: ShapePtr -> BodyPtr -> VectorPtr
                      -> CpFloat -> IO ()



-- | @shapeSegment b (p1,p2) r@ attaches a line segment
--   to the body @b@ going from point @p1@ to @p2@ (in
--   @b@'s coordinates) and having @r@ thickness.
--
--   This is meant to be used as a static shape. Although it
--   can be used with moving bodies, line segments don't
--   generate collisions with each other.
shapeSegment :: Body -> (Position,Position) -> CpFloat -> IO Shape
shapeSegment body@(B b) (p1,p2) r =
  withForeignPtr b $ \b_ptr ->
  with p1 $ \p1_ptr ->
  with p2 $ \p2_ptr ->
  mallocForeignPtrBytes #{size cpSegmentShape} >>= \shape ->
  withForeignPtr shape $ \shape_ptr -> do
    wrSegmentShapeInit shape_ptr b_ptr p1_ptr p2_ptr r
    return (S shape body)

foreign import ccall unsafe "wrapper.h"
    wrSegmentShapeInit :: ShapePtr -> BodyPtr -> VectorPtr
                       -> VectorPtr -> CpFloat -> IO ()

-- | @shapePoly b verts off@ attaches a polygon shape to
--   the body @b@. @verts@ contains the list of vertices
--   and they must define a convex hull with a
--   counterclockwise winding, and @off@ is the offset from
--   the body's center of gravity. All positions should be
--   specified in @b@'s coordinates.
--
--   This is the slowest of all shapes but the most flexible.
--   Note that if you want a non-convex polygon you may
--   add several convex polygons to the body.
shapePoly :: Body -> [Position] -> Position -> IO Shape
shapePoly body@(B b) verts off =
  withForeignPtr b $ \b_ptr ->
  with off $ \off_ptr ->
  withArrayLen verts $ \verts_len verts_ptr ->
  mallocForeignPtrBytes #{size cpPolyShape} >>= \shape ->
  withForeignPtr shape $ \shape_ptr -> do
    let verts_len' = fromIntegral verts_len
    wrPolyShapeInit shape_ptr b_ptr verts_len' verts_ptr off_ptr
    addForeignPtrFinalizer cpShapeDestroy shape
    return (S shape body)

foreign import ccall unsafe "wrapper.h"
    wrPolyShapeInit :: ShapePtr -> BodyPtr -> CInt -> VectorPtr
                    -> VectorPtr -> IO ()
foreign import ccall unsafe "wrapper.h &cpShapeDestroy"
    cpShapeDestroy :: FunPtr (ShapePtr -> IO ())

-- | @resetCounter@ resets the global counter used for creating
--   unique hash identifiers to the shapes.
--
--   This is used to add determinism to a simulation. As the ids
--   created with this counter may affect the order in which the
--   collisions happen, there may be very slight differences
--   in different simulations.
--
--   However, be careful as you should not use shapes created
--   before a call to @resetCounter@ with shapes created after
--   it as they may have the same id.
resetCounter :: IO ()
resetCounter = cpResetShapeIdCounter

foreign import ccall unsafe "wrapper.h"
    cpResetShapeIdCounter :: IO ()


-- | @getBody s@ is the body that this shape is associated
--   to. Useful especially in 'Physics.Hipmunk.Space.Callback'.
getBody :: Shape -> Body
getBody (S _ b) = b


-- | The collision type is by collision calback...
--   XXX Improve documentation after writing the rest of the bindings.
type CollisionType = #{type unsigned int}
getCollisionType :: Shape -> IO CollisionType
getCollisionType (S shape _) =
  withForeignPtr shape #{peek cpShape, collision_type}
setCollisionType :: Shape -> CollisionType -> IO ()
setCollisionType (S shape _) col =
  withForeignPtr shape $ \shape_ptr -> do
    #{poke cpShape, collision_type} shape_ptr col

-- | Groups are used to filter collisions between shapes. If
--   the group is zero (default), then it imposes no restriction
--   to the collisions. However, if the group is non-zero then
--   the shape will not collide with other shapes in the same
--   non-zero group.
--
--   This is primarely used to create multi-body, multi-shape
--   objects such as ragdolls. It may be thought as a lightweight
--   alternative to creating a callback that filters the
--   collisions.
type Group = #{type unsigned int}
getGroup :: Shape -> IO Group
getGroup (S shape _) =
  withForeignPtr shape #{peek cpShape, group}
setGroup :: Shape -> Group -> IO ()
setGroup (S shape _) gr =
  withForeignPtr shape $ \shape_ptr -> do
    #{poke cpShape, group} shape_ptr gr

-- | Layers are similar to groups, but use a bitmask. For a collision
--   to occur, two shapes must have at least one layer in common
--   (in other words, @layer1 .&. layer2@ should be non-zero).
--
--   Note that although this type may have more than 32 bits,
--   for portability you should only rely on the lower 32 bits.
--
--   The default value is @0xFFFF@.
type Layers = #{type unsigned int}
getLayers :: Shape -> IO Layers
getLayers (S shape _) =
  withForeignPtr shape #{peek cpShape, layers}
setLayers :: Shape -> Layers -> IO ()
setLayers (S shape _) lay =
  withForeignPtr shape $ \shape_ptr -> do
    #{poke cpShape, layers} shape_ptr lay

-- | The elasticity of the shape is such that @0.0@ gives no bounce
--   while @1.0@ give a "perfect" bounce. Note that due to
--   inaccuracies using @1.0@ or greater is not recommended.
--
--   The amount of elasticity applied during a collision is
--   calculated by multiplying the elasticity of both shapes.
--   Defaults to zero.
type Elasticity = CpFloat
getElasticity :: Shape -> IO Elasticity
getElasticity (S shape _) =
  withForeignPtr shape #{peek cpShape, e}
setElasticity :: Shape -> Elasticity -> IO ()
setElasticity (S shape _) e =
  withForeignPtr shape $ \shape_ptr -> do
    #{poke cpShape, e} shape_ptr e

-- | The friction coefficient of the shape according
--   to Coulumb friction model (i.e. @0.0@ is frictionless,
--   and iron on iron is around @1.0@).
--
--   The amount of friction applied during a collision is
--   determined by multiplying the friction coefficient
--   of both shapes. Defaults to zero.
type Friction = CpFloat
getFriction :: Shape -> IO Friction
getFriction (S shape _) =
  withForeignPtr shape #{peek cpShape, u}
setFriction :: Shape -> Friction -> IO ()
setFriction (S shape _) u =
  withForeignPtr shape $ \shape_ptr -> do
    #{poke cpShape, u} shape_ptr u

-- | The surface velocity of the shape. Useful to create
--   conveyor belts and players that move around. This
--   value is only used when calculating friction, not
--   collision. Defaults to zero (i.e. @Vector 0 0@).
type SurfaceVel = Vector
getSurfaceVel :: Shape -> IO SurfaceVel
getSurfaceVel (S shape _) =
  withForeignPtr shape #{peek cpShape, surface_v}
setSurfaceVel :: Shape -> SurfaceVel -> IO ()
setSurfaceVel (S shape _) sv =
  withForeignPtr shape $ \shape_ptr -> do
    #{poke cpShape, surface_v} shape_ptr sv




-- | @momentForCircle m (ri,ro) off@ is the moment of inertia
--   of a circle of @m@ mass, inner radius of @ri@, outer radius
--   of @ro@ and at an offset @off@ from the center of the body.
momentForCircle :: CpFloat -> (CpFloat, CpFloat) -> Position -> CpFloat
momentForCircle m (ri,ro) off = (m/2)*(ri*ri + ro*ro) + m*(off `dot` off)
-- We recoded the C function to avoid FFI and unsafePerformIO
-- on this simple function.


-- | @momentForPoly m verts off@ is the moment of inertia of a
--   polygon of @m@ mass, at offset @off@ from the center of
--   the body and comprised of @verts@ vertices. This is similar
--   to 'shapePoly' (and the same restrictions for the vertices
--   apply as well).
momentForPoly :: CpFloat -> [Position] -> Position -> CpFloat
momentForPoly m verts off = (m*sum1)/(6*sum2)
  where
    verts' = if off /= 0 then map (+off) verts else verts
    (sum1,sum2) = calc (pairs (,) verts') 0 0

    calc a b c | a `seq` b `seq` c `seq` False = undefined
    calc []           acc1 acc2 = (acc1, acc2)
    calc ((v1,v2):vs) acc1 acc2 =
      let a = v2 `cross` v1
          b = v1 `dot` v1 + v1 `dot` v2 + v2 `dot` v2
      in calc vs (acc1 + a*b) (acc2 + a)
-- We recoded the C function to avoid FFI, unsafePerformIO
-- and a bunch of malloc + poke. Is it worth?

-- | Internal. For @l = [x1,x2,...,xn]@, @pairs f l@ is
--   @[f x1 x2, f x2 x3, ...,f xn x1]@.
pairs :: (a -> a -> b) -> [a] -> [b]
pairs f l = zipWith f l (tail $ cycle l)

-- | @shapeQuery shape p@ returns @True@ iff the point in
--   position @p@ (in world's coordinates) lies within
--   the shape @shape@.
shapeQuery :: Shape -> Position -> IO Bool
shapeQuery (S shape _) p =
  withForeignPtr shape $ \shape_ptr ->
  with p $ \p_ptr -> do
    i <- wrShapePointQuery shape_ptr p_ptr
    return (i /= 0)

foreign import ccall unsafe "wrapper.h"
    wrShapePointQuery :: ShapePtr -> VectorPtr -> IO CInt



-- $polygon_util
--   This section is inspired by @pymunk.util@,
--   a Python module made from <http://code.google.com/p/pymunk/>,
--   although implementations are quite different.
--
--   Also, unless noted otherwise all polygons are
--   assumed to be simple (i.e. no overlapping edges).

-- | /O(n)/. @isClockwise verts@ is @True@ iff @verts@ form
--   a clockwise polygon.
isClockwise :: [Position] -> Bool
isClockwise = (<= 0) . foldl' (+) 0 . pairs cross

-- | @isLeft (p1,p2) vert@ is
--
--    * @LT@ if @vert@ is at the left of the line defined by @(p1,p2)@.
--
--    * @EQ@ if @vert@ is at the line @(p1,p2)@.
--
--    * @GT@ otherwise.
isLeft :: (Position, Position) -> Position -> Ordering
isLeft (p1,p2) vert = compare 0 $ (p1 - vert) `cross` (p2 - vert)

-- | /O(n)/. @isConvex verts@ is @True@ iff @vers@ form a convex
--   polygon.
isConvex :: [Position] -> Bool
isConvex = foldl1 (==) . map (0 <) . filter (0 /=) . pairs cross . pairs (-)
-- From http://apocalisp.wordpress.com/category/programming/haskell/page/2/

-- | /O(n)/. @polyReduce delta verts@ removes from @verts@ all
--   points that have less than @delta@ distance
--   in relation to the one preceding it.
--
--   Note that a very small polygon may be completely \"eaten\"
--   if all its vertices are within a @delta@ radius from the
--   first.
polyReduce :: CpFloat -> [Position] -> [Position]
polyReduce delta = go
    where
      go (p1:p2:ps) | len (p2-p1) < delta = go (p1:ps)
                    | otherwise           = p1 : go (p2:ps)
      go other = other

-- | /O(n)/. @polyCenter verts@ is the position in the center
--   of the polygon formed by @verts@.
polyCenter :: [Position] -> Position
polyCenter verts = foldl' (+) 0 verts `scale` s
    where s = recip $ toEnum $ length verts


-- | /O(n log n)/. @convexHull verts@ is the convex hull of the
--   polygon defined by @verts@. The vertices of the convex
--   hulls are given in clockwise winding. The polygon
--   doesn't have to be simple.
--
--   Implemented using Graham scan, see
--   <http://cgm.cs.mcgill.ca/~beezer/cs507/3coins.html>.
convexHull :: [Position] -> [Position]
convexHull verts =
  let (p0,ps) = takeMinimum verts
      (_:p1:points) = p0 : sortBy (isLeft . (,) p0) ps

      -- points is going counterclockwise now.
      -- In go we use 'hull' with the last added
      -- vertex as the head, so our result is clockwise.

      -- Remove right turns
      go hull@(h1:h2:hs) (q1:qs) =
          case (isLeft (h2,h1) q1, hs) of
            (LT,_) -> go (q1:hull) qs    -- Left turn
            (_,[]) -> go (q1:hull) qs    -- Maintain at least 2 points
            _      -> go (h2:hs) (q1:qs) -- Right turn or straight
      go hull [] = hull
      go _ _     = error "Physics.Hipmunk.Shape.convexHull: never get here"

  in go [p1,p0] points


-- | Internal. Works like minimum but also returns the
--   list without it. The order of the list may be changed.
--   We have @fst (takeMinimum xs) == minimum xs@ and
--   @sort (uncurry (:) $ takeMinimum xs) == sort xs@
takeMinimum :: Ord a => [a] -> (a, [a])
takeMinimum [] = error "Physics.Hipmunk.Shape.takeMinimum: empty list"
takeMinimum (x:xs) = go x [] xs
    where
      go min_ acc (y:ys) | y < min_  = go y (min_:acc) ys
                         | otherwise = go min_ (y:acc) ys
      go min_ acc [] = (min_, acc)


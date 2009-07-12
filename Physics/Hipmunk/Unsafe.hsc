-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Unsafe.hsc
-- Copyright   :  (c) Felipe A. Lessa 2008
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  provisional
-- Portability :  portable (needs FFI)
--
-- All functions on this module are /UNSAFE/ in the sense that
-- they may reduce the physical accuracy or numerical stability
-- of the simulation if you use them correctly, or may crash your
-- system if you are not careful enough.  Read their documentation
-- carefully and use them only if you really need them and know
-- what you are doing.
--
-----------------------------------------------------------------------------

module Physics.Hipmunk.Unsafe
    (-- * Shapes
     unsafeRedefine
    )
    where

import Foreign hiding (rotate, new)
import Foreign.C
#include "wrapper.h"

import Physics.Hipmunk.Common
import Physics.Hipmunk.Internal
import Physics.Hipmunk.Shape

-- | @unsafeRedefine shape type off@ redefines @shape@ to have
--   new parameters described on @type@ and to be at offset
--   @off@.  Be careful, /you should not change the shape type/.
--   For example, it is unsafe to change a circle shape's radius,
--   but it is an error to try to change a circle into a segment
--   or a polygon.  Note also that these errors /are not
--   checked/, meaning /they will probably crash Chipmunk/.
unsafeRedefine :: Shape -> ShapeType -> Position -> IO ()
unsafeRedefine (S shape _) (Circle r) off =
  withForeignPtr shape $ \shape_ptr ->
  with off $ \off_ptr -> do
    cpCircleShapeSetRadius shape_ptr r
    wrCircleShapeSetOffset shape_ptr off_ptr

unsafeRedefine (S shape _) (LineSegment p1 p2 r) off =
  withForeignPtr shape $ \shape_ptr ->
  with (p1+off) $ \p1off_ptr ->
  with (p2+off) $ \p2off_ptr -> do
    wrSegmentShapeSetEndpoints shape_ptr p1off_ptr p2off_ptr
    cpSegmentShapeSetRadius shape_ptr r

unsafeRedefine (S shape _) (Polygon verts) off =
  withForeignPtr shape $ \shape_ptr ->
  with off $ \off_ptr ->
  withArrayLen verts $ \verts_len verts_ptr -> do
    let verts_len' = fromIntegral verts_len
    wrPolyShapeSetVerts shape_ptr verts_len' verts_ptr off_ptr

foreign import ccall unsafe "wrapper.h"
    cpCircleShapeSetRadius :: ShapePtr -> CpFloat -> IO ()
foreign import ccall unsafe "wrapper.h"
    wrCircleShapeSetOffset :: ShapePtr -> VectorPtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    wrSegmentShapeSetEndpoints :: ShapePtr -> VectorPtr -> VectorPtr -> IO ()
foreign import ccall unsafe "wrapper.h"
    cpSegmentShapeSetRadius :: ShapePtr -> CpFloat -> IO ()
foreign import ccall unsafe "wrapper.h"
    wrPolyShapeSetVerts :: ShapePtr -> CInt -> VectorPtr -> VectorPtr -> IO ()

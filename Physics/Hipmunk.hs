-----------------------------------------------------------------------------
-- |
-- Module      :  Physics/Hipmunk/Hipmunk.hs
-- Copyright   :  (c) Felipe A. Lessa 2008
-- License     :  MIT (see LICENSE)
--
-- Maintainer  :  felipe.lessa@gmail.com
-- Stability   :  provisional
-- Portability :  portable (needs FFI)
--
-- This module re-exports all other Hipmunk modules. It is
-- meant to be imported qualified such as
--
-- @
-- import qualified Physics.Hipmunk as H
-- @
--
-- however it doesn't clash with the 'Prelude'.
--
-----------------------------------------------------------------------------

module Physics.Hipmunk
    (module Physics.Hipmunk.Common,
     module Physics.Hipmunk.Body,
     module Physics.Hipmunk.Shape,
     module Physics.Hipmunk.Joint,
     module Physics.Hipmunk.Space
    )
    where

import Physics.Hipmunk.Common
import Physics.Hipmunk.Body
import Physics.Hipmunk.Shape
import Physics.Hipmunk.Joint
import Physics.Hipmunk.Space

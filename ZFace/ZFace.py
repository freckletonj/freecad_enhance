'''

ZFace dressup

  Wrap a 2D CAM Path around a 3D set of faces by adjusting their Z-height. Works with square endmills only (but easily can be upgraded to arbitrary endmills).



DEV NOTES:

##################################################

DOCS:
  FeaturePython: https://wiki.freecad.org/Create_a_FeaturePython_object_part_I

  FeaturePython Custom Properties: https://wiki.freecad.org/index.php?title=FeaturePython_Custom_Properties

  obj.addProperty: addProperty(...) method of builtins.FeaturePython instance
    addProperty(type: string, name: string, group="", doc="", attr=0, read_only=False, hidden=False, locked = False, enum_vals=[])

##################################################

ExpressionBindings:
  https://forum.freecad.org/viewtopic.php?t=56703
  https://forum.freecad.org/viewtopic.php?t=24732
  https://forum.freecad.org/viewtopic.php?style=3&t=67732
  https://forum.freecad.org/viewtopic.php?t=78122&start=10
  https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Fem/femtaskpanels/task_mesh_gmsh.py
    https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Fem/Gui/Resources/ui/MeshGmsh.ui
  Notes on expressions: https://forum.freecad.org/viewtopic.php?t=83660

  Depth Base: https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/CAM/Path/Op/Gui/Base.py#L955

  Mod.CAM.Op.Gui.Base.updateData: https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/CAM/Path/Op/Gui/Base.py#L353
    ui: https://github.com/FreeCAD/FreeCAD/blob/4911c23d1fe69995e507b5214ba749bc905187b1/src/Mod/CAM/Gui/Resources/panels/PageDepthsEdit.ui

  Path.Base.Gui.Util.QuantitySpinBox: https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/CAM/Path/Base/Gui/Util.py#L112


  updateWidget/updateProperty: https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/CAM/Path/Base/Gui/Util.py#L185

  TaskPanel opFeatures: https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/CAM/Path/Op/Base.py#L52

  updateDepths sets OpStockZMax/ZMin: https://github.com/FreeCAD/FreeCAD/blob/f51893fbc698f04bb778e52658eb1319c6164216/src/Mod/CAM/Path/Op/Base.py#L633


##################################################

Path.Geom

https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/CAM/Path/Geom.py


##################################################

TechDraw.findShapeOutline: project to plane (can fail to connect points)

def findShapeOutline(shape, scale, direction):
    """
    Extract the outer boundary (outline) of a 3D shape when projected in a specified direction.

    This function is commonly used in technical drawing applications to generate 2D
    representations of 3D objects by projecting the shape and identifying its outer boundary.

    Args:
        shape (TopoShape): The 3D shape object to find the outline of. Must be a valid
            TopoShape object containing geometric data.
        scale (float): Scale factor applied during projection. Typically 1.0 for
            unscaled projection, but can be adjusted to resize the output.
        direction (Base.Vector): 3D vector specifying the projection direction. This
            determines the viewing angle for the projection (e.g., Vector(0,0,1)
            projects along the Z-axis).

    Returns:
        TopoShapeWire or None: Returns a TopoShapeWire object representing the outer
            boundary of the projected shape if successful. Returns None if no outline
            is found, if the input shape is invalid, or if processing fails.

    Raises:
        TypeError: If arguments are not of the expected types (shape must be TopoShape,
            direction must be Base.Vector).
        Part.PartExceptionOCCError: If OpenCASCADE geometric processing encounters
            an error during projection or edge extraction.
        Base.Exception: If edge walking or wire formation fails during processing.
    """

##################################################

TODO:
- [ ] clean up `execute`
- [ ] correct g-code
  - [ ] strip feed rates, re generate
  - [ ] strip safety/clearance moves, re generate

- [ ] non-move commands with empty pointlist slipping through with XYZ still set (but missing the offset step)

- [ ] Apply instead of Recompute

- [ ] set ObjectZFace init name correctly
  - if I say "Job": SetupSheet.INFO: SetupSheet has no support for Job


TODO v2:
  - [ ] decouple from being a dressup, generate good paths right here

  - [ ] surface angle dependent stepover (shallow regions get higher density)

  - [ ] XY changes on step down so we don't need to rapid through on-contour recuts

  - [ ] reify many small lines into big lines/arcs to shrink g-code

  - [ ] save 3d geom cache to worktree so other ops can reference it (could have different endmill types in same file)

  - [ ] `part.section` is capable of parallel intersections. I'm not sure what the
        speed up would be. A single section gets returned as multiple edges, so
        we'd need a cleanup step to id which edges belong to which intersection

  - [ ] could make a separate server to run parallel generation, gpu, etc

  - [ ] preserve cache on a per-face basis (so adding faces doesn't need huge
        recompute). BUT maybe we have a cache on what was a cliff of a face, and
        you add an adjacent face, so the wrong points will be cached, so, maybe
        not possible.

  - [ ] add validation checks at end of path generation (feed rates, collisions)

  - [ ] add metrics to task panel (stock-to-lowest cut height)

  - [ ] add SegInterpolate/ArcInterpolate to TaskPanel

  - [ ] color the g-code display according to feed rate

  - [ ] cycle times don't update in TaskPanel (prob GUI doesn't hear it)

  - [ ] clean up regions that are just recutting only

  - [ ] add ZFace to "Supplemental Commands" (per tarman3's suggestion: https://github.com/tarman3/FreeCAD/commit/6be9f07b8e16dc9f1cc3261b444d197713d01a56)
'''



import math
import hashlib
import traceback
import random
from enum import Enum, auto
from dataclasses import dataclass

from QuadTree import QuadTree
from ZFaceSection import intersect_z

from PySide import QtGui, QtCore
from PySide.QtCore import QT_TRANSLATE_NOOP

import FreeCAD
import FreeCADGui

import Part

import Path
import PathScripts.PathUtils as PathUtils
import Path.Geom as PathGeom
import Path.Base.Gui.Util as PathGuiUtil
import Path.Op.Gui.Base as PathGuiBase
import Path.Op.Base as PathOpBase
from Path.Geom import isRoughly

import time

LOG_MODULE = Path.Log.thisModule()
Path.Log.setLevel(Path.Log.Level.DEBUG, LOG_MODULE)

translate = FreeCAD.Qt.translate


##################################################
# ZFace

class SegmentKind(Enum):
    IS_ON_PLANE = auto() # Command is on XY plane, z=0
    IS_HIGH = auto()     # z > 0
    OTHER = auto()       # Tool changes, M-codes, etc.

@dataclass
class TaggedSegment:
    kind: SegmentKind
    command: Path.Command           # the original FreeCAD command
    pointlist: list[FreeCAD.Vector] # discretized points (may be empty)


class ObjectZFace(PathOpBase.ObjectOp):
    def __init__(self, obj):
        '''__init__ is only called the first time this dressup is invoked. It
        is not called when a document is reloaded and a ZFace dressup is within
        it. Computation that needs to happen at that time can go in
        `onDocumentRestored`

        obj: the "Path::FeaturePython" that gets added to the ActiveDocument

        Many properties (StartDepth, OpStockMax, Base (geom), etc) will be added by opFeatures
        '''
        super().__init__(obj, "Job")  # TODO: what name do I set?!

        # Base (geom): [ (object, [subname, subname, ...]) ]
        #   https://freecad.github.io/SourceDoc/d9/d92/classApp_1_1PropertyLinkSubList.html
        # obj.addProperty("App::PropertyLinkSubList", "Base", "Path",
        #                 QT_TRANSLATE_NOOP("App::Property", "Faces or other subelements to dress up"))


        # Cycle Time
        obj.CycleTime = "not calculated"

        # Base Toolpath
        obj.addProperty("App::PropertyLink", "BasePath", "Path",
                        QT_TRANSLATE_NOOP("App::Property", "The base toolpath to modify"))

        # # Offset mode
        # obj.addProperty("App::PropertyBool", "Relative", "Parameters",
        #                 "If true, preserve original Z-offsets and shift the entire path. If false (absolute), map all points onto the surface.")
        # obj.Relative = False


        ##########
        # Depths
        #   will be controlled by TaskPanelDepthsPage:
        #   https://github.com/FreeCAD/FreeCAD/blob/4911c23d1fe69995e507b5214ba749bc905187b1/src/Mod/CAM/Path/Op/Gui/Base.py#L955

        # obj.setExpression("StartDepth", "OpStartDepth")
        # obj.setExpression("FinalDepth", "OpFinalDepth")
        obj.setExpression("StepDown", "OpToolDiameter * 100")

        ##########

        # Block Cache Invalidation
        obj.addProperty("App::PropertyBool", "BlockCacheInvalidation", "Parameters",
                        "Don't allow recomputes to ever invalidate the cache.")
        obj.BlockCacheInvalidation = False


        # interpolation/sample params
        obj.addProperty("App::PropertyDistance", "ArcInterpolate", "Interpolate")
        obj.ArcInterpolate = 0.1

        obj.addProperty("App::PropertyDistance", "SegInterpolate", "Interpolate")
        obj.SegInterpolate = 1.0

        obj.addProperty("App::PropertyDistance", "CacheGrid", "Interpolation",
                        "The grid size for caching Z-height lookups. Larger values improve performance but reduce accuracy for rapidly changing surfaces.")
        obj.CacheGrid = 12  # for faster development cycles, huge number

        obj.addProperty("App::PropertyBool", "LogProgress", "Interpolation")
        obj.LogProgress = True

        # persistent cache stored on the object (Python object property)
        obj.addProperty("App::PropertyPythonObject", "PointCache", group="PointCache", read_only=True, hidden=False,
                        doc="A cache of computed Z-heights to speed up recomputes.")
        self._clear_cache(obj)  # create empty PointCache and _runtime_quadtree

        # state hash for cache invalidation/determining necessity of recompute
        obj.addProperty("App::PropertyString", "_CacheStateHash",
                        group="Internal",
                        doc="Internal state hash for cache validation",
                        read_only=True,
                        hidden=False
                        )

        obj.Proxy = self  # give the FeaturePython access to this class

        # set later
        self.faces = []
        self.faces_compound = None


        # Update depths on ObjectOp
        # NOTE: this must be here to make sure Safe/ClearanceHeights get calculated correctly
        self.updateDepths(obj)


    def opFeatures(self, obj):
        ''' Pages/features from PathOpBase.ObjectOp '''
        return (
            PathOpBase.FeatureTool +
            PathOpBase.FeatureDepths +
            PathOpBase.FeatureHeights +
            PathOpBase.FeatureStepDown +
            PathOpBase.FeatureBaseFaces
        )

    def __getstate__(self):
        # extras, called on saving file.
        return None

    def __setstate__(self, state):
        # extras, called on load.
        return None


    def onBeforeChange(self, obj, prop):
        '''Called before a property value is changed prop is the name of the
        property to be changed, not the property object itself. Property
        changes cannot be cancelled. Previous / next property values are not
        simultaneously available for comparison.
        '''
        pass


    def onDocumentRestored(self, obj):
        '''Called after a document is restored or a FeaturePython object is
        copied.  Occasionally, references to the FeaturePython object from the
        class, or the class from the FeaturePython object may be broken, as the
        class __init__() method is not called when the object is
        reconstructed. Adding self.Object = obj or obj.Proxy = self often
        solves these issues.
        '''
        self._runtime_quadtree = None  # will be restored from PointCache (cheap)
        pass

    def sanitizeBase(self, obj):
        pass


    def _get_hash_string(self, obj):
        """Generates a deterministic string representing the state of the source geometry.
        This string is used to create a hash for cache invalidation.

        """
        parts = []

        # NOTE: Don't put CacheGrid here. Previous cache results are still
        # relevant if geom hasn't changed, so CacheGrid changing should not
        # trigger cache invalidation
        #
        # parts.append(f'CacheGrid={str(obj.CacheGrid)}')

        # Source names
        sorted_sources = sorted(obj.Base, key=lambda item: item[0].Name if item[0] else "")
        parts.append(f"Base={str(sorted_sources)}")

        # Geometric Signature
        all_faces = self._collect_faces_from_links(obj)

        if not all_faces:
            parts.append("Geometry=None")
            return "\n".join(parts)

        if self.faces_compound.isNull():
            parts.append("Geometry=Invalid")
            return "\n".join(parts)

        # NOTE: the floats are printed with fairly low accuracy bc, for some
        # reason, FreeCAD/OCC is not deterministic in calculations like Area.

        parts.append(f"NumFaces={len(self.faces_compound.Faces)}")
        parts.append(f"NumEdges={len(self.faces_compound.Edges)}")
        parts.append(f"NumVertexes={len(self.faces_compound.Vertexes)}")

        bb = self.faces_compound.BoundBox
        bbox_str = (f"{bb.XMin:.2f},{bb.YMin:.2f},{bb.ZMin:.2f},"
                    f"{bb.XMax:.2f},{bb.YMax:.2f},{bb.ZMax:.2f}")
        parts.append(f"BoundBox={bbox_str}")
        parts.append(f"Area={self.faces_compound.Area:.1f}")

        com = self.faces_compound.CenterOfGravity
        com_str = f"{com.x:.2f},{com.y:.2f},{com.z:.2f}"
        parts.append(f"CenterOfGravity={com_str}")

        return "\n".join(parts)

    def _compute_state_hash(self, obj):
        """
        Computes a deterministic SHA256 hash of the inputs that affect the PointCache, to protect against recompute.

        * Using `.isTouched` on the faces gives false negatives - they were reported as not touched, even if they were
        * Using `hashCode` on faces gives false positives - it's a non-deterministic code, so changes on file reload
        """
        state_string = self._get_hash_string(obj)
        return hashlib.sha256(state_string.encode('utf-8')).hexdigest()


    def onChanged(self, obj, prop):
        '''Callback for when a property changes. Note: this is called when the
        file is first loaded (so be careful not to invalidate the PointCache).

        obj: FeaturePython
        prop: name of property that changed

        '''

        if not FreeCAD.ActiveDocument.Restoring:
            # If selected faces change, invalidate PointCache
            #   NOTE: we don't care if CacheGrid changes, because previous cache results are still relevant.
            # if prop in ("Base", ):
            #     Path.Log.info('Base (faces) prop changed, clearing cache')
            #     self._clear_cache(obj)

            # WAIT, this circumvents the hash check stuff
            pass

        return


    def _collect_faces_from_links(self, obj):
        faces = []
        for (parent, subnames) in obj.Base:
            if not parent or not hasattr(parent, "Shape"):
                continue
            shape = parent.Shape
            for sub in subnames:
                try:
                    subshape = shape.getElement(sub)
                    if subshape.ShapeType == "Face":
                        faces.append(subshape)
                except Exception:
                    Path.Log.warning(f"ZFace: could not resolve subelement {sub}")
        return faces


    def _rebuild_quadtree(self, obj, bounds=None):
        """
        Build a runtime quadtree from obj.PointCache['points'].
        Stores it on self._runtime_quadtree.
        If bounds (xmin,xmax,ymin,ymax) provided, use it to initialize quadtree extent.
        """
        pts = []
        try:
            pts = obj.PointCache.get("points", []) if isinstance(obj.PointCache, dict) else []
        except Exception:
            pts = []

        if pts:
            # If we have points, build quadtree around them
            qt = QuadTree.from_point_list(pts, bounds_padding=0.0, capacity=8)
            self._runtime_quadtree = qt
            return qt

        # no points; create an empty quadtree using provided bounds or fallback bounds
        if bounds is not None:
            xmin, xmax, ymin, ymax = bounds
        else:
            # default small bounds - get expanded in first insert if needed
            xmin, xmax, ymin, ymax = -1000.0, 1000.0, -1000.0, 1000.0

        self._runtime_quadtree = QuadTree(xmin, xmax, ymin, ymax, capacity=8)
        return self._runtime_quadtree


    def _clear_cache(self, obj):
        if not obj.BlockCacheInvalidation:
            obj.PointCache = {}
            self._runtime_quadtree = None
        else:
            Path.Log.info('ZFace: blocking cache invalidation')


    ####################
    # 3D Surface intersections

    def _insert_cache_point(self, obj, x, y, z):
        """
        Insert a point into both the runtime quadtree and the persistent obj.PointCache.
        """

        pts = obj.PointCache.get("points")
        if pts is None:
            obj.PointCache["points"] = []
            pts = obj.PointCache["points"]

        # append to persistent store
        pts.append((float(x), float(y), float(z)))

        qt = self._runtime_quadtree
        inserted = qt.insert(x, y, z)
        if not inserted:
            raise ValueError(f'couldnt add point ({x=}, {y=}, {z=}) to quad tree')


    def get_surface_z(self, obj, x, y):
        """
        Return the Z height of the tool center so that the tool body
        just touches the selected surfaces at (x,y).

        Uses a quadtree-based nearest-neighbor cache persisted in obj.PointCache.
        If a cached sample within tolerance (obj.CacheGrid.Value) exists, returns it.
        Otherwise computes intersect_z, inserts it into the cache, and returns result.
        """

        # 1) check cache first
        tol = obj.CacheGrid.Value  # tolerance is grid-like radius for cache acceptance
        qt = self._runtime_quadtree

        nearest = qt.nearest(x, y)
        if nearest is not None:
            px, py, pz, dist = nearest
            if dist <= tol:
                return pz  # cache hit within tolerance

        # 2) Expensive intersection
        z_val = intersect_z(
            self.faces_compound,
            self.faces,
            obj.ToolController.Tool.Diameter.Value / 2,
            x,
            y)

        # 3) Add to Cache
        if z_val is not None:
            try:
                self._insert_cache_point(obj, x, y, z_val)
            except Exception:
                # ensure we at least persist the point even if runtime insert fails
                try:
                    if obj.PointCache is None or not isinstance(obj.PointCache, dict):
                        obj.PointCache = {"points": []}
                    obj.PointCache.setdefault("points", []).append((float(x), float(y), float(z_val)))
                except Exception:
                    pass

        return z_val


    ####################
    # Execute

    def opExecute(self, obj):
        """Called during document recomputes. Builds a new Path.Path with corrected Zs."""

        if obj.ToolController.Tool.ShapeID != 'endmill':
            Path.Log.error('only square endmills are supported right now (but other shapes should be easy to add)')
            return

        ##########
        # Setup

        self.updateDepths(obj)  # Update depths on ObjectOp

        h_feed = obj.ToolController.HorizFeed.Value
        v_feed = obj.ToolController.VertFeed.Value
        h_rapid = obj.ToolController.HorizRapid.Value
        v_rapid = obj.ToolController.VertRapid.Value
        safe_height = obj.SafeHeight.Value
        clearance_height = obj.ClearanceHeight.Value


        ##########
        # Determine if we need to recompute z-offsets or not (by invalidating the cache)

        faces = self._collect_faces_from_links(obj)
        self.faces = faces
        self.faces_compound = Part.Compound(self.faces)
        self.faces_compound.tessellate(0.01)  # TODO: necessary?

        stored_hash = obj._CacheStateHash
        current_hash = self._compute_state_hash(obj)
        if current_hash != stored_hash:
            Path.Log.info('ZFace: State hash has changed, invalidating point cache.')
            # Path.Log.info(f'{self._get_hash_string(obj)}\n')
            self._clear_cache(obj)
        obj._CacheStateHash = current_hash


        ##########
        # Sanity checks

        if not faces:
            Path.Log.info(f"{LOG_MODULE}: No faces collected. Reverting to original path.")
            obj.Path = PathUtils.getPathWithPlacement(obj.BasePath)
            return

        if (
                not obj.BasePath or not
                obj.BasePath.isDerivedFrom("Path::Feature") or not
                getattr(obj.BasePath, "Path", None) or not
                obj.BasePath.Path.Commands
        ):

            Path.Log.warning(f'ZFace: Base feature is not a CAM Path: {obj.BasePath=}')
            obj.Path = Path.Path()
            return


        ##########
        # Quad tree for cache
        #
        # Rebuild quadtree runtime store from persistent cache, using geometry bounds so
        # the quadtree has sensible extents for inserts & nearest queries.

        try:
            comp_for_bounds = self.faces_compound
            bb = comp_for_bounds.BoundBox
            bounds = (bb.XMin, bb.XMax, bb.YMin, bb.YMax)
        except Exception:
            bounds = None

        # Create runtime quadtree (if persistent points exist, they will be loaded there)
        self._rebuild_quadtree(obj, bounds=bounds)


        ##########
        # Discretize BasePath Commands
        #
        #   build discretized segments of the original path (easier to z-offset
        #   linear segments), and classify
        # Classify BasePath commands

        start_location = FreeCAD.Vector(0, 0, obj.SafeHeight)  # TODO: where should start point be?

        segments = discretize_commands(
            obj.BasePath.Path.Commands,  # original path
            start_location,
            safe_height,
            clearance_height,
            sampleD=obj.SegInterpolate.Value,  # for discretizing lines
            curveD=obj.ArcInterpolate.Value,  # for discretizing arcs
        )  # List[TaggedSegment]


        ##########

        start_time = time.time()
        last_update = start_time  # for reporting ever N seconds
        current_step = 0
        total_steps = sum(len(seg.pointlist) if seg.pointlist else 1 for seg in segments)

        # ProgressIndicator is buggy, doesn't show
        progress = FreeCAD.Base.ProgressIndicator()
        progress.start("Recomputing ZFace with surface intersection...", total_steps)

        # Keep track of progress to occassionally allow gui updates
        progress_counter = 0

        # Find min z-height for relative mode
        z_original_min = 0.0

        all_points = [p
                      for segment in segments
                      for p in segment.pointlist]
        if all_points:
            # Find the minimum Z-height from the original, discretized path
            z_original_min = min(p.z for p in all_points)
        else:
            Path.Log.warning("Path has no movable points to establish a base Z-height.")


        ##########
        # Generate Z-raised Path Commands

        newcommandlist = []
        currLocation = {"X": 0.0, "Y": 0.0, "Z": 0.0}

        n_warnings = 0

        for seg in segments:
            c = seg.command
            pointlist = seg.pointlist

            # if n_warnings >= 20:
            #     Path.Log.error('too many warnings')
            #     return

            # if seg.kind is SegmentKind.CLEARANCE:
            #     # Explicit upward rapid to clearance/safe height.
            #     # We don't remap Z here; just regenerate a fresh clearance move.
            #     newcommandlist.append(Path.Command("G0", {"Z": clearance_height}))
            #     currLocation["Z"] = clearance_height
            #     progress.next()
            #     continue

            # elif seg.kind is SegmentKind.SURFACE_RAPID:
            #     # Rapid move (G0) near the surface. Needs Z remap like cutting moves.
            #     # Fall through to CUTTING logic, since pointlist will be discretized.
            #     pass

            # elif seg.kind is SegmentKind.FEED_ONLY:
            #     # Only feedrate changed; preserve as-is.

            #     # newcommandlist.append(c)
            #     # currLocation.update(c.Parameters)

            #     progress.next()
            #     continue

            # elif seg.kind is SegmentKind.ZERO_LENGTH:
            #     # Degenerate move (start=end). Preserve stripped feed if present.

            #     # newcommandlist.append(c)
            #     # currLocation.update(c.Parameters)

            #     progress.next()
            #     continue

            # elif seg.kind is SegmentKind.OTHER:
            #     # Non-motion command (M-codes, tool changes, etc.). Pass through unchanged.
            #     newcommandlist.append(c)
            #     currLocation.update(c.Parameters)
            #     progress.next()
            #     continue


            if seg.kind is SegmentKind.OTHER:
                # Non-motion command (M-codes, tool changes, etc.). Pass through unchanged.
                newcommandlist.append(c)
                # currLocation.update(c.Parameters)
                progress.next()
                current_step += 1
                continue


            if not pointlist:
                # BE CAREFUL: If these have G0/G1/G2/G3, the corresponding Z value
                # won't get updated, resulting in collisions.
                has_xyz = any(axis in c.Parameters for axis in ("X", "Y", "Z"))
                if has_xyz:
                    Path.Log.warning(f"Empty pointlist but XYZ present: {c}, params={c.Parameters}")

                newcommandlist.append(c)
                currLocation.update(c.Parameters)
                progress.next()
                current_step += 1
                continue

            for point in pointlist:
                if seg.kind is SegmentKind.IS_ON_PLANE:
                    z_surface = self.get_surface_z(obj, point.x, point.y)
                else:
                    z_surface = point.z + obj.OpStockZMax.Value  # remap to top of stock

                if z_surface is None:
                    # we failed to map this XY to a Z height, move to safe height
                    #
                    # this may happen near start points that are away from part
                    z_surface = obj.SafeHeight.Value

                # if obj.Relative:
                #     # Relative mode: surface_z + (original_point_z - original_path_min_z)
                #     internal_offset = point.z - z_original_min
                #     final_z = (z_surface + internal_offset) if z_surface is not None else point.z
                # else:
                #     # Absolute mode: just the surface Z (or original if None)
                #     final_z = z_surface if z_surface is not None else point.z

                internal_offset = point.z - z_original_min
                final_z = (z_surface + internal_offset) if z_surface is not None else point.z

                if final_z > safe_height:
                    final_z = safe_height

                commandparams = {"X": point.x, "Y": point.y, "Z": final_z}

                # NOTE: we'll set F in `process_stepdown`
                # if "F" in c.Parameters:
                #     commandparams["F"] = c.Parameters["F"]

                # All motions become G1 (G0 can "dogleg" as axes travel to
                # individual locations at max speed)
                newcommand = Path.Command("G1", commandparams)
                newcommandlist.append(newcommand)
                currLocation.update(newcommand.Parameters)
                progress.next()
                current_step += 1

                # Occasionally allow GUI updates
                progress_counter += 1
                if progress_counter % 100 == 0 and FreeCAD.GuiUp:
                    # QtGui.QApplication.processEvents()  # doesn't update gui throughout computation
                    FreeCADGui.updateGui()

                    now = time.time()
                    if now - last_update > 5.0:
                        last_update = now
                        elapsed = now - start_time
                        percent_done = current_step / total_steps
                        remaining_seconds = (elapsed / percent_done) - elapsed if percent_done > 0 else 0

                        # Format as MM:SS
                        remaining_minutes = int(remaining_seconds // 60)
                        remaining_secs = int(remaining_seconds % 60)
                        remaining_formatted = f"{remaining_minutes}:{remaining_secs:02d}"

                        print(f'working: {percent_done * 100:.1f}%, remaining: {remaining_formatted}')

        progress.stop()

        stepped_down = process_stepdown(
            Path.Path(newcommandlist),
            obj.StepDown.Value,
            obj.StartDepth.Value,
            clearance_height,
            safe_height,
            h_feed, v_feed,
            h_rapid, v_rapid,
        )

        self.commandlist = stepped_down

        # return self.commandlist  # i don't think this needs to return?


##################################################
# Path/Command stuff


def is_on_plane(cmd, prev_location):
    """
    True if z==0
    """
    if 'Z' in cmd.Parameters and cmd.Z:
        return isRoughly(cmd.Z, 0.0)
    else:
        return isRoughly(prev_location.z, 0.0)

def is_motion(cmd):
    """
    True if a G0/G1/G2/G3
    """
    return cmd.Name in (
        PathGeom.CmdMoveStraight +
        PathGeom.CmdMoveArc +
        PathGeom.CmdMoveRapid
    )

def discretize_commands(
        commands,
        start_location,
        safe_height,
        clearance_height,
        sampleD,
        curveD
):
    '''
    * Discretize segments + arcs
    * classify paths

    Args:
      commands: List[Path.Command]
      start_location: FreeCAD.Vector

    Returns:
      List[TaggedSegment]
    '''
    temp_location = FreeCAD.Vector(
        start_location.x,
        start_location.y,
        start_location.z
    )
    segments = []  # [TaggedSegment]
    for c in commands:
        # Classify
        if not is_motion(c):
            kind = SegmentKind.OTHER
        elif is_on_plane(c, temp_location):
            kind = SegmentKind.IS_ON_PLANE
        else:
            kind = SegmentKind.IS_HIGH

        # Discretize motions
        if is_motion(c):
            # endPoint = PathGeom.commandEndPoint(c, temp_location)
            edge = PathGeom.edgeForCmd(c, temp_location)  # can return None
            if edge:
                # get discretized segments
                if c.Name in PathGeom.CmdMoveArc:
                    pointlist = edge.discretize(Deflection=curveD)  # maximum deviation from curve
                else:
                    pointlist = edge.discretize(Distance=sampleD)  # distance between points

                # this ignores 0-motion commands, eg maybe a command only set F, drop it
                segments.append(TaggedSegment(kind, c, pointlist))
        else:
            segments.append(TaggedSegment(kind, c, pointlist=[]))

        # Update temp_location
        temp_location.x = c.Parameters.get("X", temp_location.x)
        temp_location.y = c.Parameters.get("Y", temp_location.y)
        temp_location.z = c.Parameters.get("Z", temp_location.z)
    return segments


##################################################
# Stepdown
#
# inspired by: tarman3:
#   https://github.com/FreeCAD/FreeCAD/blob/e5e0b2c9485112377aee74d243dffb4c93e80be5/src/Mod/CAM/Path/Op/Gui/PathShapeTC.py#L662C1-L708C24

def getPoint(commands):
    '''
    Get coordinates of each axis from path commands
    '''
    x = y = z = None
    for cmd in commands:
        x = cmd.x if x is None and cmd.x is not None else x
        y = cmd.y if y is None and cmd.y is not None else y
        z = cmd.z if z is None and cmd.z is not None else z
        if x is not None and y is not None and z is not None:
            return FreeCAD.Vector(x, y, z)  # "vector" means 3D point

    if x is None:
        Path.Log.warning(translate("PathShape", "Can not determine X"))
    if y is None:
        Path.Log.warning(translate("PathShape", "Can not determine Y"))
    if z is None:
        Path.Log.warning(translate("PathShape", "Can not determine Z"))

    return None


def isClosedPath(point1, point2):
    if point1 is None or point2 is None:
        return False
    if not isRoughly(point1.x, point2.x):
        return False
    if not isRoughly(point1.y, point2.y):
        return False

    return True


def isLower(z1, z2):
    if z1 is None or z2 is None:
        return False
    if isRoughly(z1, z2):
        return False
    if z1 < z2:
        return True
    return False


def interpolate_feedrate(p0, p1, horiz_feed, vert_feed, tol=1e-9, test_feeds=False):
    '''
    Compute the linear feedrate for a move from p0 to p1 such that:
    - The downward vertical component (Z-) never exceeds `vert_feed` (hard cap).
    - Subject to that constraint, the horizontal component matches `horiz_feed` as closely as possible.
    - Upward (Z+) moves are not limited by `vert_feed` (go full speed).
    - Pure horizontal and pure vertical moves are handled explicitly.

    Returns:
        linear_feed
    '''
    dx = p1.x - p0.x
    dy = p1.y - p0.y
    dz = p1.z - p0.z

    dist_xy = math.hypot(dx, dy)
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)


    if not isRoughly(dist, 0.0):  # has displacement
        if isRoughly(dist_xy, 0.0):
            # Pure vertical
            if dz > 0:
                F = horiz_feed   # upward: fast
            else:
                F = vert_feed    # downward: capped
        elif isRoughly(dz, 0.0):
            # Pure horizontal
            F = horiz_feed
        else:
            # General diagonal
            Fh = horiz_feed * dist / dist_xy
            if dz > 0:
                F = Fh  # upward move, no vertical cap
            else:
                Fv = vert_feed * dist / abs(dz)
                F = min(Fh, Fv)
    else:
        F = vert_feed

    if not test_feeds:
        return F
    else:
        h_comp = F * dist_xy / dist if dist > 0 else 0.0
        v_comp = F * abs(dz) / dist if dist > 0 else 0.0
        return F, h_comp, v_comp


# @@@@@@@@@@
# interpolate_feed tests
if True:
    tests = [
        # (p0, p1, hfeed, vfeed, expected feed)
        ((0,0,0), (0,0,0), 100, 50, 50.0),  # 0. No motion
        ((0,0,0), (10,0,0), 100, 50, 100.0),  # 1. Pure horizontal X
        ((0,0,0), (0,10,0), 120, 60, 120.0),  # 2. Pure horizontal Y
        ((0,0,0), (0,0,10), 100, 50, 100.0),  # 3. Pure vertical up (full speed)
        ((0,0,0), (0,0,-10), 100, 50, 50.0),  # 4. Pure vertical down (vmax limited speed)
        ((0,0,0), (10,0,10), 100, 50, 141.42),  # 5. Upward 45° in XZ
        ((0,0,0), (10,0,-10), 100, 50, 70.71),  # 6. Downward 45° in XZ
        ((0,0,0), (1,0,100), 100, 50, 10000.50),  # 7. Very steep upward (linear feed can approach inf with steepness)
        ((0,0,0), (1,0,-100), 100, 50, 50.0025),  # 8. Very steep downward
        ((0,0,0), (100,0,-1), 100, 50, 100.0050),  # 9. Shallow downward slope
    ]
    for i, (p0, p1, hf, vf, expected) in enumerate(tests):
        v0 = FreeCAD.Vector(*p0)
        v1 = FreeCAD.Vector(*p1)
        is_uphill = p1[2] > p0[2]
        result, h_comp, v_comp = interpolate_feedrate(v0, v1, hf, vf, test_feeds=True)
        assert isRoughly(result, expected, error=1e-2), f"Test {i} failed: got {result}, expected {expected}"
        assert h_comp <= hf + 1e-6, f'Test {i} failed: {h_comp=} was not <= expected h feed {hf=}'
        if is_uphill:
            # v_comp can be super high, higher than hf
            # DONT: assert v_comp <= hf + 1e-6, f'Test {i} failed: uphill, {v_comp=} was not <= expected h feed {hf=}'
            pass
        else:
            assert v_comp <= vf + 1e-6, f'Test {i} failed: downhill, {v_comp=} was not <= expected v feed {vf=}'
    print("interpolate_feedrate: tests passed!")


def process_stepdown(
        path,
        step_depth,
        start_depth,
        clearance_height,
        safe_height,
        h_feed, v_feed,
        h_rapid, v_rapid,
        rapid_through_cleared=True
):
    '''
    1. Add several passes with step down
    2. Moves through cleared space are at rapid speeds, and 0.5mm above the actual contours
    3.
    '''
    commands = []
    startPoint = getPoint(path.Commands)
    endPoint = getPoint(reversed(path.Commands))

    limit_depth = start_depth  # a sinking floor of how deep to plunge
    repeat = True
    firstStep = True

    # will recut 0.5mm of actual surface, then raise to 0.5mm above it to rapid
    # until back below 0.5mm line
    rapid_margin = 0.5

    while repeat:
        repeat = False
        limit_depth -= step_depth

        skipZ = not firstStep and isClosedPath(startPoint, endPoint)
        if not skipZ:
            # add safety moves to start point before next step down
            commands.append(Path.Command("G0", {"Z": startPoint.z}))
            commands.append(Path.Command("G0", {"X": startPoint.x, "Y": startPoint.y}))

        # track position state
        current_x, current_y, current_z = startPoint.x, startPoint.y, startPoint.z
        prevPoint = FreeCAD.Vector(current_x, current_y, current_z)

        for cmd in path.Commands:
            if (
                skipZ
                and cmd.x is None
                and cmd.y is None
                and (cmd.z == clearance_height or
                     cmd.z == safe_height)
            ):
                # skip start move for closed profile
                continue

            # Update stateful position
            if cmd.x is not None:
                current_x = cmd.x
            if cmd.y is not None:
                current_y = cmd.y
            if cmd.z is not None:
                # threshold depth
                if isLower(cmd.z, limit_depth):
                    repeat = True
                    current_z = limit_depth
                else:
                    current_z = cmd.z

            # Threshold depth
            if isLower(cmd.z, limit_depth):
                repeat = True  # did not get final depth, so repeat step down
                current_z = limit_depth

            if (
                    rapid_through_cleared and
                    not isLower(current_z, limit_depth + step_depth + rapid_margin)
            ):
                # Convert moves in cleared space to rapids
                #
                # Use G1 moves instead of G0 rapids since G0 may suffer from dogleg
                # moves (G0 doesn't guarantee the path taken)
                current_z += rapid_margin
                thisFeed = h_rapid
            else:
                # Is cutting path. Interpolate feed rate
                thisPoint = FreeCAD.Vector(current_x, current_y, current_z)
                thisFeed = interpolate_feedrate(prevPoint, thisPoint, h_feed, v_feed)

            cmd = Path.Command('G1', {'X': current_x, 'Y': current_y, 'Z': current_z, 'F': thisFeed})
            commands.append(cmd)

        firstStep = False

    return commands


##################################################
# G-Code

# def is_feedrate_only_move(cmd, prev_location):
#     """
#     True if the command only changes feed rate (F), with no real XYZ motion.
#     Sometimes XYZ values are present but identical to prev_location.
#     """
#     params = cmd.Parameters
#     has_f = "F" in params

#     # Extract XYZ with fallback to previous location
#     x = params.get("X", prev_location.x)
#     y = params.get("Y", prev_location.y)
#     z = params.get("Z", prev_location.z)

#     no_motion = (
#         isRoughly(x, prev_location.x) and
#         isRoughly(y, prev_location.y) and
#         isRoughly(z, prev_location.z)
#     )

#     return has_f and no_motion


# def is_clearance_rapid(cmd, prev_location, safe_height, clearance_height):
#     """
#     True if this is a rapid (G0) to a safe/clearance height.
#     Typically upward-only vertical moves to Z >= safe/clearance.
#     These will be regenerated later.
#     """
#     if cmd.Name not in PathGeom.CmdMoveRapid:
#         return False

#     params = cmd.Parameters
#     if "Z" not in params:
#         return False

#     new_z = params.get("Z", prev_location.z)

#     upward = new_z > prev_location.z
#     high_enough = new_z >= min(safe_height, clearance_height)

#     return upward and high_enough


# def is_surface_rapid(cmd, prev_location, safe_height):
#     """
#     True if this is a rapid (G0) move near the cutting surface,
#     i.e. Z below safe height and some X/Y motion.
#     These should be kept and Z-remapped.
#     """
#     if cmd.Name not in PathGeom.CmdMoveRapid:
#         return False

#     params = cmd.Parameters
#     new_x = params.get("X", prev_location.x)
#     new_y = params.get("Y", prev_location.y)
#     new_z = params.get("Z", prev_location.z)

#     xy_motion = (
#         not isRoughly(new_x, prev_location.x) or
#         not isRoughly(new_y, prev_location.y)
#     )

#     below_safe = new_z < safe_height

#     return xy_motion and below_safe


# def is_zero_length_motion(cmd, prev_location):
#     """
#     True if this is a G1/G2/G3 with coincident start/end points.
#     Usually means edgeForCmd() bailed.
#     These may still carry feed (F).
#     """
#     if cmd.Name not in (
#             PathGeom.CmdMoveStraight +  # G1 (*not* rapids/G0)
#             PathGeom.CmdMoveArc  # G2/G3
#     ):
#         return False

#     params = cmd.Parameters
#     new_x = params.get("X", prev_location.x)
#     new_y = params.get("Y", prev_location.y)
#     new_z = params.get("Z", prev_location.z)

#     coincident = (
#         isRoughly(new_x, prev_location.x) and
#         isRoughly(new_y, prev_location.y) and
#         isRoughly(new_z, prev_location.z)
#     )

#     return coincident




##################################################
# UI

class ZFacePage(PathGuiBase.TaskPanelPage):
    def getTitle(self, obj):
        return 'ZFace'

    def __init__(self, obj):
        self.obj = obj

        self.icon = None
        self.isdirty = False

        self.form = QtGui.QWidget()
        layout = QtGui.QVBoxLayout()

        # --- Parameters Group ---
        params_group = QtGui.QGroupBox(translate("CAM_ZFace", "Parameters"))
        params_layout = QtGui.QVBoxLayout()

        # Cycle Time
        cycle_time_layout = QtGui.QHBoxLayout()
        cycle_time_label = QtGui.QLabel(translate("CAM_ZFace", "Cycle Time:"))
        self.cycle_time = QtGui.QLabel(translate("CAM_ZFace", "Cycle Time"))
        cycle_time_layout.addWidget(cycle_time_label)
        cycle_time_layout.addWidget(self.cycle_time)
        params_layout.addLayout(cycle_time_layout)

        # # Relative Offset
        # self.relative_check = QtGui.QCheckBox(translate("CAM_ZFace", "Use Relative Offset"))
        # self.relative_check.setToolTip(translate("CAM_ZFace", "Preserves the original Z-variations of the path, shifting it globally."))
        # params_layout.addWidget(self.relative_check)

        # Block Cache Invalidation
        self.block_invalidation = QtGui.QCheckBox(translate("CAM_ZFace", "Block Cache Invalidation"))
        self.block_invalidation.setToolTip(translate("CAM_ZFace", "Don't allow the cache to ever invalidate (ie when FreeCAD does senseless recomputes)."))
        params_layout.addWidget(self.block_invalidation)

        # Cache Params
        cache_layout = QtGui.QHBoxLayout()
        cache_label = QtGui.QLabel(translate("CAM_ZFace", "Cache Grid (mm):"))
        self.cache_grid_edit = QtGui.QDoubleSpinBox()
        self.cache_grid_edit.setMinimum(0.001)
        self.cache_grid_edit.setMaximum(1000.0)
        self.cache_grid_edit.setSingleStep(0.1)
        cache_layout.addWidget(cache_label)
        cache_layout.addWidget(self.cache_grid_edit)
        params_layout.addLayout(cache_layout)

        self.log_progress_check = QtGui.QCheckBox(translate("CAM_ZFace", "Log Progress"))
        params_layout.addWidget(self.log_progress_check)

        params_group.setLayout(params_layout)

        btn_layout = QtGui.QHBoxLayout()
        self.btn_clear_cache = QtGui.QPushButton(translate("CAM_ZFace", "Clear Point Cache"))
        self.btn_recompute = QtGui.QPushButton(translate("CAM_ZFace", "Recompute"))
        btn_layout.addWidget(self.btn_clear_cache)
        btn_layout.addWidget(self.btn_recompute)

        # Usage
        usage_group = QtGui.QGroupBox(translate("CAM_ZFace", "Usage"))
        usage_layout = QtGui.QVBoxLayout()

        self.usage_text = QtGui.QTextEdit()
        self.usage_text.setReadOnly(True)
        self.usage_text.setFrameStyle(QtGui.QFrame.NoFrame)

        usage_content = """
        <div style="font-family: sans-serif; font-size: 11px; line-height: 1.4; color: #34495e;">
        <b style="color: #2980b9;">ZFace Operation:</b><br/>
        Lift a 2D path onto some faces. Tool-shape aware.<br/><br/>

        <b style="color: #27ae60;">Workflow:</b><br/>
        • Use ZFace as dressup over a planar path on XY at Z=0 (eg pocket)
        • Ensure the base path's clearance/safety is above stock, we reuse these heights
        • Select target faces from your model geometry<br/>
        • Use Recompute to preview changes before applying<br/><br/>

        <b style="color: #e74c3c;">Performance Tips:</b><br/>
        • Smaller cache grid = higher detail but slower processing<br/>
        • Block cache invalidation for stable geometry
        </div>
        """

        self.usage_text.setHtml(usage_content)

        # Style the text area
        self.usage_text.setStyleSheet("""
            QTextEdit {
                background-color: #ecf0f1;
                border: 1px solid #bdc3c7;
                border-radius: 6px;
                padding: 8px;
            }
        """)

        # Calculate and set the proper height to show all content without scrollbars
        self.usage_text.document().setTextWidth(self.usage_text.viewport().width())
        doc_height = self.usage_text.document().size().height()
        self.usage_text.setFixedHeight(int(doc_height) + 20)  # Add padding for margins

        # Disable scrollbars
        self.usage_text.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.usage_text.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        usage_layout.addWidget(self.usage_text)
        usage_group.setLayout(usage_layout)


        # add to main layout
        layout.addWidget(params_group)
        layout.addSpacing(15)
        layout.addLayout(btn_layout)
        layout.addWidget(usage_group)
        self.form.setLayout(layout)


        # Connect Signals
        self.btn_clear_cache.clicked.connect(self.clear_point_cache)
        self.btn_recompute.clicked.connect(self.recompute_now)

        # hardcode the face selection for development
        if 'cam_z_depth_adaptive_path' in FreeCAD.ActiveDocument.Name:
            Path.Log.info('Applying hardcoded face selection for development')
            try:
                # Get the object named "Clone" from the active document.
                clone_obj = FreeCAD.ActiveDocument.getObject("Clone")
                if clone_obj:
                    # The required type for obj.Sources is a list of tuples:
                    # [(<DocumentObject>, ['FaceName1', 'FaceName2', ...])]
                    self.obj.Base = [(clone_obj, ['Face1', 'Face2', 'Face3'])]
                else:
                    FreeCAD.Console.PrintError("ZFace: Could not find object named 'Clone'.\n")
                    self.obj.Base = []
            except Exception as e:
                FreeCAD.Console.PrintError(f"ZFace: Error setting hardcoded faces: {e}\n")
                self.obj.Sources = []

    def clear_point_cache(self):
        self.obj.Proxy._clear_cache(self.obj)
        FreeCAD.Console.PrintMessage("ZFace PointCache cleared.\n")

    def reject(self):
        """Called on Cancel or closing the panel."""
        # Abort the transaction
        FreeCAD.ActiveDocument.abortTransaction()
        self.cleanup()
        FreeCADGui.Control.closeDialog()
        FreeCAD.ActiveDocument.recompute()

    def accept(self):
        """Called on OK."""

        # Apply the final values from the panel to the object
        self.getFields(self.obj)

        # Commit the transaction, making all changes permanent and
        # creating a single entry in the undo stack.
        FreeCAD.ActiveDocument.commitTransaction()
        self.cleanup()
        FreeCADGui.Control.closeDialog()
        FreeCAD.ActiveDocument.recompute()

    def getFields(self, obj):
        ''' transfer values from UI to obj's properties '''
        # self.obj.Relative = self.relative_check.isChecked()
        self.obj.BlockCacheInvalidation = self.block_invalidation.isChecked()
        self.obj.CacheGrid = self.cache_grid_edit.value()
        self.obj.LogProgress = self.log_progress_check.isChecked()

    def setFields(self, obj):
        ''' transfer obj's property values to UI '''
        self.cycle_time.setText(self.obj.CycleTime)
        # self.relative_check.setChecked(getattr(self.obj, "Relative", False))
        self.block_invalidation.setChecked(getattr(self.obj, "BlockCacheInvalidation", False))
        cache_grid_val = getattr(self.obj.CacheGrid, "Value", getattr(self.obj, "CacheGrid", 0.5))
        self.cache_grid_edit.setValue(cache_grid_val)
        self.log_progress_check.setChecked(getattr(self.obj, "LogProgress", True))

    def recompute_now(self):
        self.getFields(self.obj)
        try:
            # This recompute happens inside the transaction, allowing
            # the user to preview changes before committing.
            FreeCAD.ActiveDocument.recompute()

            # set cycletime
            self.setFields(self.obj)
        except Exception as e:
            traceback.print_exc()
            FreeCAD.Console.PrintError(f"Error during recompute: {e}\n")

    def cleanup(self):
        """Explicitly disconnect signals and drop references so Qt can delete widgets safely."""
        try:
            self.btn_clear_cache.clicked.disconnect()
        except Exception:
            pass
        try:
            self.btn_recompute.clicked.disconnect()
        except Exception:
            pass

        # Drop references to widgets
        self.form = None
        self.cycle_time = None
        # self.relative_check = None
        self.block_invalidation = None
        self.cache_grid_edit = None
        self.log_progress_check = None
        self.btn_clear_cache = None
        self.btn_recompute = None


class ViewProviderZFace:
    def __init__(self, vobj):
        # reference to task_panel. Really weird, this has to be defined before
        # `vobj.Proxy = self`, or we'll get tons of `updateData` calls before it
        # exists (or should I define in `attach`?)
        self.task_panel = None

        vobj.Proxy = self


    def attach(self, vobj):
        self.obj = vobj.Object
        if self.obj and self.obj.BasePath:
            # Remove the base object from the worktree, so it's only nested
            # under the new dressup

            self.obj.BasePath.Visibility = False

            # InList is DAG arrows - a list of all features that depend on this object:
            #   https://wiki.freecad.org/index.php?title=PropertyLink:_InList_and_OutList
            for i in self.obj.BasePath.InList:
                if hasattr(i, "Group"):
                    group = i.Group
                    for g in group:
                        if g.Name == self.obj.BasePath.Name:
                            group.remove(g)
                    i.Group = group
        return

    def claimChildren(self):
        if hasattr(self.obj, "BasePath") and self.obj.BasePath:
            return [self.obj.BasePath]
        return []

    def setEdit(self, vobj, mode=0):
        ''' Used when editing a ZFace '''
        FreeCADGui.Control.closeDialog()

        FreeCAD.ActiveDocument.openTransaction("Edit ZFace Dressup")

        zface_page = ZFacePage(vobj.Object)
        self.task_panel = PathGuiBase.TaskPanel(
                        obj=vobj.Object,
                        deleteOnReject=False,
                        opPage=zface_page,
                        selectionFactory=lambda: None
                    )


        FreeCADGui.Control.showDialog(self.task_panel)
        self.task_panel.setupUi()
        return True

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        self.task_panel = None  # cleanup
        pass

    def onDelete(self, vobj, *args):
        obj = vobj.Object
        if obj and hasattr(obj, 'BasePath') and obj.BasePath: # hasattr bc of quirk when 2 paths share underlying BasePath
            if obj.BasePath.ViewObject:
                obj.BasePath.ViewObject.Visibility = True
            job = PathUtils.findParentJob(obj)
            if job and job.Proxy:
                job.Proxy.addOperation(obj.BasePath)
            obj.BasePath = None
        return True

    def updateData(self, obj, prop):
        ''' Called when connected FeaturePython changes.

        obj: FeaturePython
        prop: name of property that changed'''

        if hasattr(self, "task_panel"):
            if self.task_panel is not None:
                self.task_panel.updateData(obj, prop)
        else:
            Path.Log.warning('for some reason ViewProviderZFace doesnt have a Taskpanel')



    def onChanged(self, vobj, prop):
        '''Called when ViewProperty changes.

        vobj: ViewObject
        prop: name of property that changed
        '''
        pass

    def clearTaskPanel(self):
        if self.task_panel:
            self.task_panel = None  # so pyqt doesn't call a deleted reference


class CommandPathZFace:
    def GetResources(self):
        return {
            "Pixmap": "Sketcher_Conics",
            "MenuText": QT_TRANSLATE_NOOP("CAM_ZFace", "Z Face Wrap"),
            "Accel": "",
            "ToolTip": QT_TRANSLATE_NOOP("CAM_ZFace", "Wraps a path's Z depth onto arbitrary faces"),
        }

    def IsActive(self):
        if FreeCAD.ActiveDocument is None:
            return False
        selection = FreeCADGui.Selection.getSelection()
        if len(selection) != 1:
            return False
        selected_obj = selection[0]
        return selected_obj.isDerivedFrom("Path::Feature") and not selected_obj.isDerivedFrom("Path::FeatureCompoundPython")

    def Activated(self):
        ''' Called when ZFace is first created (?) '''
        selection = FreeCADGui.Selection.getSelection()
        if len(selection) != 1:
            FreeCAD.Console.PrintError(translate("CAM_ZFace", "Please select exactly one Path toolpath object.\n"))
            return

        selected_obj = selection[0]
        if not selected_obj.isDerivedFrom("Path::Feature") or selected_obj.isDerivedFrom("Path::FeatureCompoundPython"):
            FreeCAD.Console.PrintError(translate("CAM_ZFace", "The selected object is not a valid base toolpath.\n"))
            return


        try:
            # Use Macro commands so this gets saved into the transaction (to make it reversible)
            #   FreeCADGui.addModule: imports once
            #   FreeCADGui.doCommand: Prints the given string in the python console and runs it.

            # Start the transaction. The TaskPanel will be responsible for its fate.
            FreeCAD.ActiveDocument.openTransaction("Create Z Face Dress-up")
            FreeCADGui.addModule('ZFace')
            FreeCADGui.addModule('PathScripts.PathUtils')
            FreeCADGui.doCommand(
                '''obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "ZFace")'''
            )
            FreeCADGui.doCommand('zobj = ZFace.ObjectZFace(obj)')
            FreeCADGui.doCommand(f'obj.BasePath = FreeCAD.ActiveDocument.{selected_obj.Name}')
            FreeCADGui.doCommand('ZFace.ViewProviderZFace(obj.ViewObject)')

            FreeCADGui.doCommand('PathScripts.PathUtils.addToJob(obj)')

            FreeCADGui.doCommand("obj.ViewObject.Document.setEdit(obj.ViewObject, 0)")

            FreeCAD.ActiveDocument.recompute()
        except Exception as e:
            FreeCAD.Console.PrintError(f"Error creating ZFace dressup: {e}\n")
            traceback.print_exc()
            FreeCAD.ActiveDocument.abortTransaction()
            FreeCAD.ActiveDocument.recompute()

# register the command
FreeCADGui.addCommand("Path_ZFace", CommandPathZFace())

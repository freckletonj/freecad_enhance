"""
Carve (Sketcher) - v0.1

Goal:
    Create a sketch representing the geometry of a piece of stock after it has
    been cut by an N mm bit (N = diameter).

Method:
    1) Identify islands in a sketch
    2) Offset outer wires outward
    3) Offset inner wires inward (proper Minkowski sum with circular bit)
    4) Union these independently offset shapes
    5) Cut the inner holes from the outer shapes
    6) Offset the result back inward
    7) Now you have a constrained sketch representing the path a bit will leave against your sketch

----------
Downgrade: https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Draft/draftfunctions/downgrade.py

if a block, explode it
elif array, explode it
elif parameterized sketch, deparameterize
elif multi-solids compound object, splitCompounds
elif 2 objects, cut2 (cut second from first)
elif one face, getWire (extract wires)
elif no faces, splitWires into single edges

----------
Upgrade: https://github.com/FreeCAD/FreeCAD/blob/main/src/Mod/Draft/draftfunctions/upgrade.py

if there are any groups, close each wire
elif there are any meshes, turn them to shapes
elif only faces,
  if 1 3d shell make solid
  elif 2 objects, makeFusion
  elif many separate faces not in body, makeShell (make compound?)
  elif many coplanar faces, joinFaces
  elif 1 non-parametric face, draftify
elif not faces:
  if all closed wires:
    if 1 sketch, extract face
    else: makeFaces
  elif loose wires, join them (makeWires)
  else 1 wire, closeWire
  else 1 non-parametric, draftify
else, makeCompound


"""

import FreeCAD as App
import FreeCADGui as Gui
import Draft
import Part
import Sketcher
import math
from typing import List, Optional, Dict
from PySide2 import QtWidgets
import traceback


# --- Configuration ---

def log_info(msg: str) -> None:
    App.Console.PrintMessage(f"{msg}\n")


def log_warn(msg: str) -> None:
    App.Console.PrintWarning(f"{msg}\n")


def log_error(msg: str) -> None:
    App.Console.PrintError(f"{msg}\n")

def get_active_sketch() -> Optional["Sketcher.SketchObject"]:
    if not App.ActiveDocument:
        log_error("No active document.")
        return None
    sel = Gui.Selection.getSelection()
    if len(sel) != 1:
        log_error(f"Please select exactly one Sketch (found {len(sel)}).")
        return None
    obj = sel[0]
    if not hasattr(obj, "TypeId") or not obj.TypeId.startswith("Sketcher::Sketch"):
        log_error("Selected object is not a Sketch.")
        return None
    return obj


def clone(obj, name):
    clone = Draft.clone(obj)
    clone.Label = name


def prompt_bit_diameter(default) -> Optional[float]:
    """
    Prompt the user for a bit diameter (mm). Returns None if canceled or not available.
    """
    if QtWidgets is None:
        log_warn("GUI toolkit not available; using default bit diameter.")
        return default
    parent = None
    try:
        parent = Gui.getMainWindow()
    except Exception:
        parent = None
    try:
        val, ok = QtWidgets.QInputDialog.getDouble(
            parent,
            "Carve",
            "Bit diameter (mm):",
            default,
            -1e6,
            1e6,
            3,
        )
        if not ok:
            return None
        return float(val)
    except Exception as ex:
        log_warn(f"Could not open input dialog: {ex}")
        return default


##################################################
# offset_one_by_one

"""
================================================================================
Offset One-By-One (Sketcher)
================================================================================

sketch offset has some quirks where it behaves unexpectedly with sketches of
different windings, so we circumvent this by generating offsets for each wire
independently, which it works ok with, then unioning and cutting them.

Goal:
    Work directly with a selected Sketch: split its wires into "outer" islands
    and "inner" holes, then create two new Sketches:
      - one containing offsets of all outer wires (offset outward),
      - one containing offsets of all inner wires (offset inward).
    Offsets are computed per-wire independently using FreeCAD's default 2D
    offset for wires (Part.Wire.makeOffset2D). Results are not unioned, which
    can happen in a later step.

Notes:
    - Small holes may vanish on inward offset; that's expected and accurate.
    - Resulting edges are added as line segments and circular arcs where
      possible; other curve types should not be supported.

"""


def split_outer_inner_wires(sketch: 'Sketcher.SketchObject'):
    wires = [w for w in sketch.Shape.Wires if not w.isNull() and w.isClosed()]

    App.Console.PrintMessage(f"len wires = {len(sketch.Shape.Wires)}\n")
    App.Console.PrintMessage(f"len closed wires = {len(wires)}\n")

    if not wires:
        return {'outers': [], 'inners': []}

    # Make faces individually to avoid winding order problems
    faces = [Part.makeFace(w) for w in wires]

    outers = []
    inners = []

    def is_contained(inner_face, outer_face):
        """Check if inner_face is completely inside outer_face using vertices."""
        for v in inner_face.Vertexes:
            if not outer_face.isInside(v.Point, 1e-7, True):
                return False
        return True

    contained_in = {i: [] for i in range(len(wires))}
    for i, outer_face in enumerate(faces):
        for j, inner_face in enumerate(faces):
            if i == j:
                continue
            if is_contained(inner_face, outer_face):
                contained_in[j].append(i)

    # Classify: outer if not contained in any other
    for idx, w in enumerate(wires):
        if contained_in[idx]:
            inners.append(w)
        else:
            outers.append(w)

    App.Console.PrintMessage(f"Found {len(outers)} Outer wires\n")
    App.Console.PrintMessage(f"Found {len(inners)} Inner wires\n")
    return {'outers': outers, 'inners': inners}


def add_edge_to_sketch(sk: 'Sketcher.SketchObject', edge: Part.Edge) -> None:
    """Add a single edge to sketch as native geometry where possible, otherwise discretize."""
    try:
        curve = edge.Curve
        if isinstance(curve, (Part.Line, Part.LineSegment)):
            p1 = edge.Vertexes[0].Point
            p2 = edge.Vertexes[-1].Point
            sk.addGeometry(Part.LineSegment(p1, p2), False)
            return
        if isinstance(curve, (Part.Circle, Part.ArcOfCircle)):
            # Rebuild arc with its parameter range
            arc = Part.ArcOfCircle(curve, edge.FirstParameter, edge.LastParameter)
            sk.addGeometry(arc, False)
            return
        # Optional: simple ellipse as poly approximation
        # Fallback: discretize into short line segments
        pts = edge.discretize(25)
        for i in range(len(pts) - 1):
            sk.addGeometry(Part.LineSegment(pts[i], pts[i + 1]), False)
    except Exception as ex:
        App.Console.PrintWarning(f"Could not add edge to sketch: {ex}\n")


def add_shape_to_sketch(sk: 'Sketcher.SketchObject', shp: Part.Shape) -> int:
    """Add all edges of shape (or its wires) to the sketch. Returns number of geometry elements added."""
    before = len(sk.Geometry)
    try:
        wires = list(shp.Wires) if hasattr(shp, 'Wires') else []
        if not wires and isinstance(shp, Part.Wire):
            wires = [shp]
        if not wires and hasattr(shp, 'Edges'):
            # Add edges directly
            for e in shp.Edges:
                add_edge_to_sketch(sk, e)
        else:
            for w in wires:
                for e in w.Edges:
                    add_edge_to_sketch(sk, e)
    except Exception as ex:
        App.Console.PrintWarning(f"Could not add shape to sketch: {ex}\n")
    return len(sk.Geometry) - before


def offset_wire(wire: Part.Wire, dist: float) -> Optional[Part.Shape]:
    """Compute default 2D offset on a wire, returning the resulting shape (may contain multiple wires)."""
    try:
        # Use the native 2D offset of Wire. It may return a Wire or a Shape containing wires.
        result = wire.makeOffset2D(dist)
        if result is None:
            return None
        if isinstance(result, Part.Wire):
            return Part.Shape([result])  # Wrap into Shape for uniform handling
        return result
    except Exception as ex:
        # I think this is mainly the case when a small inner hole shrinks to 0
        return None

def prefix(objs, name):
    if isinstance(objs, list):
        for o in objs:
            prefix(o, name)
    elif isinstance(objs, dict):
        for o in objs.values():
            prefix(o, name)
    else:
        if hasattr(objs, 'Label'):
            objs.Label = f'{name}_{objs.Label}'


def upgrade_sketch_to_union(sk, do_self_intersection_check=True, CLEANUP=True, name='unnamedUnion'):
    ''' when you have a bunch of wires, some overlapping, and want to union them

    NOTE: I went nuts adding objs for deletion to temp_objs. It seems `delete=True` may have a bug.
    '''
    doc = App.ActiveDocument

    # CLEANUP = not ('outpass' in name)  # for debugging

    temp_objs = []

    obj = [sk]
    try:
        (obj, _) = Draft.downgrade(obj, delete=CLEANUP, force="getWire")
        if CLEANUP or True: temp_objs.extend(obj)
    except:
        pass

    try:
        (obj, _) = Draft.downgrade(obj, delete=CLEANUP, force="splitWires")
        if CLEANUP or True: temp_objs.extend(obj)
    except:
        pass

    try:
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="makeWires")
        prefix(obj, name)
        if CLEANUP: temp_objs.extend(obj)
    except:
        pass

    try:
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="makeFaces")
        prefix(obj, name)
        if CLEANUP: temp_objs.extend(obj)
    except:
        pass


    if len(obj) > 0 and do_self_intersection_check:
        # After this point we might lose newly created holes when a wire
        # intersects itself (like a "C" that ges offset and touches itself), as
        # the hole faces get unioned away. This branch finds the new holes and
        # cuts them out.
        ski = Draft.make_sketch(obj, autoconstraints=True)

        doc.recompute()

        x = split_outer_inner_wires(ski)
        prefix(x, name)
        doc.recompute()

        # Outer wires
        sk_out = doc.addObject('Sketcher::SketchObject', 'self_outs')
        sk_out.Placement = ski.Placement
        for w in x['outers']:
            add_shape_to_sketch(sk_out, w)

        # Inner Wires
        sk_in = doc.addObject('Sketcher::SketchObject', 'self_ins')
        sk_in.Placement = ski.Placement
        for w in x['inners']:
            add_shape_to_sketch(sk_in, w)

        if CLEANUP: temp_objs.extend([ski, sk_in, sk_out] + x['outers'] + x['inners'])
        doc.recompute()

        # Outer Unions
        outs = upgrade_sketch_to_union(sk_out, False, CLEANUP, name=f'{name}_selfOut')
        prefix(outs, name)
        if CLEANUP: temp_objs.append(outs)
        doc.recompute()

        # Inner Unions
        ins = upgrade_sketch_to_union(sk_in, False, CLEANUP, name=f'{name}_selfIn')
        prefix(ins, name)
        if CLEANUP: temp_objs.append(ins)
        doc.recompute()

        temp_objs.extend([outs, ins])

        # Cut
        if ins is not None:
            (obj, _) = Draft.downgrade([outs, ins], delete=False, force="cut2")
            prefix(obj, name)
            if CLEANUP: temp_objs.extend(obj)
        else:
            obj = outs

        doc.recompute()

    try:
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="joinFaces")
        prefix(obj, name)
        doc.recompute()
    except:
        pass

    # In testing, it appears I sometimes need to "joinFaces" twice, seems like a bug
    try:
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="joinFaces")
        prefix(obj, name)
        doc.recompute()
    except:
        pass

    for o in temp_objs:
        try:
            name = o.Name
            doc.removeObject(name)
        except Exception as e:
            pass
    doc.recompute()

    if len(obj) == 1:
        return obj[0]
    elif len(obj) > 1:
        log_error('Too many objects after making union')
    else:
        return None



def outer_inner_offsets(sketch: 'Sketcher.SketchObject', distance: float, name):
    """Split into outer/inner wires, and offset separately."""
    doc = App.ActiveDocument
    if not doc:
        App.Console.PrintError("No active document.\n")
        return {'outers': None, 'inners': None}

    split = split_outer_inner_wires(sketch)
    outers = split['outers']
    inners = split['inners']

    if not outers and not inners:
        App.Console.PrintError("Sketch has no valid wires to offset.\n")
        return {'outers': None, 'inners': None}

    base_label = getattr(sketch, 'Label', sketch.Name)
    name_out = f"{base_label}_Offset_Outsides"
    name_in = f"{base_label}_Offset_Insides"

    sk_out = doc.addObject('Sketcher::SketchObject', name_out)
    sk_in = doc.addObject('Sketcher::SketchObject', name_in)
    # Keep same placement so world coordinates match
    try:
        sk_out.Placement = sketch.Placement
        sk_in.Placement = sketch.Placement
    except Exception:
        pass

    created_out = 0
    created_in = 0

    # Process outers (outward)
    for w in outers:
        signed = +distance
        shp = offset_wire(w, signed)
        if shp:
            created_out += add_shape_to_sketch(sk_out, shp)

    # Process inners (inward for the solid, which follows the same sign-by-orientation rule)
    for w in inners:
        signed = -distance  # (-distance) if ccw else (+distance)
        shp = offset_wire(w, signed)
        if shp:  # small holes may shrink to 0/None
            created_in += add_shape_to_sketch(sk_in, shp)

    doc.recompute()

    App.Console.PrintMessage(
        f"Created '{sk_out.Name}' (added {created_out} element(s)) and "
        f"'{sk_in.Name}' (added {created_in} element(s)).\n"
    )

    # upgrade to Union
    intersect_out = distance > 0
    union_out = upgrade_sketch_to_union(sk_out, do_self_intersection_check=intersect_out, name=f'{name}_unionOut')
    union_in = upgrade_sketch_to_union(sk_in, do_self_intersection_check=False, name=f'{name}_unionIn')
    doc.recompute()
    return {'outers': union_out, 'inners': union_in}


def run_carve() -> Optional["Sketcher.SketchObject"]:
    """
    Execute the carve macro:
      - Requires exactly one Sketch selected.
      - Prompts for bit diameter.
      - Produces a final carved Sketch
    Returns the final carved Sketch or None on failure.
    """

    log_warn('May not be accurate. In the first offset a single path like a "C" may create a new hole, that I think gets lost during an Upgrade')

    sk = get_active_sketch()
    if not sk:
        return None

    bit_diam = prompt_bit_diameter(3.0)
    if bit_diam is None:
        log_info("Operation canceled.")
        return None

    R = float(bit_diam) / 2.0
    doc = App.ActiveDocument

    temp_objs = []

    doc.openTransaction("Carve")
    try:
        # Step 1: Offset by +R (reliable offset macro) -> two sketches
        log_info(f"Offsetting '{sk.Name}' by +{R:.3f} mm (tool radius).")
        off1 = outer_inner_offsets(sk, R, 'outpass')
        out1 = off1.get("outers")
        in1 = off1.get("inners")
        temp_objs.extend([out1, in1])

        log_info(f'{out1=}')
        log_info(f'{in1=}')

        if in1 is not None:
            (cut, _) = Draft.downgrade([out1, in1], delete=True, force='cut2')
            temp_objs.extend(cut if isinstance(cut, list) else [cut])
            doc.recompute()
        else:
            cut = out1
            temp_objs.append(cut)

        # clone(cut, 'CLONE_CUT')

        # Step 2: Cut holes
        combined = Draft.make_sketch(cut, autoconstraints=True)
        temp_objs.append(combined)
        doc.recompute()

        # Step 3: Offset back by -R using our reliable offset macro
        log_info(f"Offsetting combined sketch '{combined.Name}' by -{R:.3f} mm.")
        off2 = outer_inner_offsets(combined, -R, 'inpass')
        log_info('done with negative offset')

        out2 = off2.get("outers")
        in2 = off2.get("inners")
        temp_objs.extend([out2, in2])

        # Step 4: Apply cut between outer/inner
        if in2 is not None:
            (cut2, _) = Draft.downgrade([out2, in2], delete=True, force='cut2')
            temp_objs.extend(cut2 if isinstance(cut2, list) else [cut2])
            doc.recompute()
            if len(cut2) == 1:
                cut2 = cut2[0]
            else:
                log_error('cut2 had too many components')
        else:
            cut2 = out2
            temp_objs.append(cut2)

        final_sk = Draft.make_sketch(cut2, autoconstraints=True)
        doc.recompute()

        # remove temp objs
        for o in temp_objs:
            try:
                if o and doc.getObject(o.Name):
                    doc.removeObject(o.Name)
            except:
                # it's hard knowing when an Up/Downgrade will delete an object, and this errors
                pass

        doc.recompute()
        log_info(f"Carve completed: created '{final_sk.Name}'.")
        return final_sk

    except Exception as ex:
        log_error(f"Carve failed: {ex}\n{traceback.format_exc()}")
        return None
    finally:
        try:
            doc.commitTransaction()
        except Exception:
            pass

# Execute when run as a macro
if __name__ == "__main__":
    run_carve()

"""
Sketch Offset One-by-One (Standalone Macro) - v1.0

Goal:
    Create a new sketch representing a proper 2D offset of a selected sketch.

Method:
    This macro is an extraction of the core offsetting logic from the more complex
    "Carve (Sketcher)" macro. It preserves the original, robust method for
    creating offsets, which circumvents some of FreeCAD's native limitations.

    1) Prompts the user for an offset distance (positive for outward, negative for inward).
    2) Identifies all separate "islands" (outer wires) and "holes" (inner wires)
       in the selected sketch.
    3) Offsets all outer wires and inner wires independently. This avoids issues
       where the native offset tool fails on complex sketches with multiple windings.
    4) The separately offset shapes for the outer wires are unioned together. This
       step includes logic to detect and handle new holes created by self-intersections
       (e.g., when offsetting a 'C' shape causes the ends to touch and merge).
    5) The separately offset shapes for the inner holes are also unioned.
    6) The final unioned holes are cut from the final unioned outer shape.
    7) The result is converted back into a new, clean Sketch object.

"""

import FreeCAD as App
import FreeCADGui as Gui
import Draft
import Part
import Sketcher
import traceback
from PySide2 import QtWidgets

# --- Configuration ---

def log_info(msg: str) -> None:
    App.Console.PrintMessage(f"{msg}\n")

def log_warn(msg: str) -> None:
    App.Console.PrintWarning(f"{msg}\n")

def log_error(msg: str) -> None:
    App.Console.PrintError(f"{msg}\n")

def get_active_sketch():
    """Gets the currently selected Sketch object."""
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

def prompt_offset_distance(default: float):
    """Prompts the user for an offset distance in mm."""
    if QtWidgets is None:
        log_warn("GUI toolkit not available; using default.")
        return default
    parent = None
    try:
        parent = Gui.getMainWindow()
    except Exception:
        parent = None
    try:
        val, ok = QtWidgets.QInputDialog.getDouble(
            parent,
            "Sketch Offset",
            "Offset distance (mm, positive is outward):",
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


# --- Core Logic (Copied directly from Carve macro) ---

def split_outer_inner_wires(sketch: 'Sketcher.SketchObject'):
    """Classifies wires into outer loops and inner holes."""
    wires = [w for w in sketch.Shape.Wires if not w.isNull() and w.isClosed()]
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

    # Classify: an outer wire is not contained in any other wire.
    for idx, w in enumerate(wires):
        if contained_in[idx]:
            inners.append(w)
        else:
            outers.append(w)

    log_info(f"Found {len(outers)} outer wires and {len(inners)} inner wires.")
    return {'outers': outers, 'inners': inners}

def add_edge_to_sketch(sk: 'Sketcher.SketchObject', edge: Part.Edge) -> None:
    """Add a single edge to sketch as native geometry where possible."""
    try:
        curve = edge.Curve
        if isinstance(curve, (Part.Line, Part.LineSegment)):
            p1 = edge.Vertexes[0].Point
            p2 = edge.Vertexes[-1].Point
            sk.addGeometry(Part.LineSegment(p1, p2), False)
        elif isinstance(curve, (Part.Circle, Part.ArcOfCircle)):
            arc = Part.ArcOfCircle(curve, edge.FirstParameter, edge.LastParameter)
            sk.addGeometry(arc, False)
        else:
            # Fallback for other curve types: discretize into line segments
            pts = edge.discretize(25)
            for i in range(len(pts) - 1):
                sk.addGeometry(Part.LineSegment(pts[i], pts[i + 1]), False)
    except Exception as ex:
        log_warn(f"Could not add edge to sketch: {ex}")

def add_shape_to_sketch(sk: 'Sketcher.SketchObject', shp: Part.Shape) -> int:
    """Add all edges of a shape to the sketch. Returns number of elements added."""
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
        log_warn(f"Could not add shape to sketch: {ex}")
    return len(sk.Geometry) - before

def offset_wire(wire: Part.Wire, dist: float):
    """Compute 2D offset on a wire, returning the resulting shape."""
    try:
        result = wire.makeOffset2D(dist)
        if result is None: return None
        if isinstance(result, Part.Wire):
            return Part.Shape([result])
        return result
    except Exception:
        # This often happens when a small hole shrinks to nothing, which is expected.
        return None

def prefix(objs, name):
    """Adds a prefix to the Label of FreeCAD objects for easier debugging."""
    if isinstance(objs, list):
        for o in objs: prefix(o, name)
    elif isinstance(objs, dict):
        for o in objs.values(): prefix(o, name)
    else:
        if hasattr(objs, 'Label'):
            objs.Label = f'{name}_{objs.Label}'

def upgrade_sketch_to_union(sk, do_self_intersection_check=True, CLEANUP=True, name='unnamedUnion'):
    """
    Unions a sketch containing multiple, possibly overlapping, wires into a single face.
    Crucially includes logic to handle new holes created by self-intersection.
    """
    doc = App.ActiveDocument
    temp_objs = []
    obj = [sk]

    try:
        (obj, _) = Draft.downgrade(obj, delete=CLEANUP, force="getWire")
        if CLEANUP: temp_objs.extend(obj)
    except: pass

    try:
        (obj, _) = Draft.downgrade(obj, delete=CLEANUP, force="splitWires")
        if CLEANUP: temp_objs.extend(obj)
    except: pass

    try:
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="makeWires")
        prefix(obj, name)
        if CLEANUP: temp_objs.extend(obj)
    except: pass

    try:
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="makeFaces")
        prefix(obj, name)
        if CLEANUP: temp_objs.extend(obj)
    except: pass

    if len(obj) > 0 and do_self_intersection_check:
        # This block handles self-intersections. An offset can cause a wire to
        # overlap itself (e.g., 'C' -> 'O'), creating a new hole. This logic
        # finds that new hole and ensures it's cut out properly.
        ski = Draft.make_sketch(obj, autoconstraints=True)
        doc.recompute()
        x = split_outer_inner_wires(ski)
        prefix(x, name)
        doc.recompute()

        sk_out = doc.addObject('Sketcher::SketchObject', 'self_outs')
        sk_out.Placement = ski.Placement
        for w in x['outers']: add_shape_to_sketch(sk_out, w)

        sk_in = doc.addObject('Sketcher::SketchObject', 'self_ins')
        sk_in.Placement = ski.Placement
        for w in x['inners']: add_shape_to_sketch(sk_in, w)

        if CLEANUP: temp_objs.extend([ski, sk_in, sk_out] + x['outers'] + x['inners'])
        doc.recompute()

        # Recursively union the new outer and inner parts
        outs = upgrade_sketch_to_union(sk_out, False, CLEANUP, name=f'{name}_selfOut')
        if CLEANUP: temp_objs.append(outs)
        ins = upgrade_sketch_to_union(sk_in, False, CLEANUP, name=f'{name}_selfIn')
        if CLEANUP: temp_objs.append(ins)
        doc.recompute()

        # Cut the new holes from the new outer shape
        if ins is not None:
            (obj, _) = Draft.downgrade([outs, ins], delete=CLEANUP, force="cut2")
            prefix(obj, name)
            if CLEANUP: temp_objs.extend(obj)
        else:
            obj = [outs] if outs else []
        doc.recompute()

    try:
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="joinFaces")
        prefix(obj, name)
        doc.recompute()
    except: pass

    try: # In testing, sometimes a second join is needed.
        (obj, _) = Draft.upgrade(obj, delete=CLEANUP, force="joinFaces")
        prefix(obj, name)
        doc.recompute()
    except: pass

    for o in temp_objs:
        try:
            if o and doc.getObject(o.Name):
                doc.removeObject(o.Name)
        except Exception: pass
    doc.recompute()

    if len(obj) == 1:
        return obj[0]
    elif len(obj) > 1:
        log_error('Union result contained multiple unjoined objects.')
    return None

def outer_inner_offsets(sketch: 'Sketcher.SketchObject', distance: float, name: str):
    """Orchestrates the main offset, union, and cleanup process."""
    doc = App.ActiveDocument
    if not doc:
        log_error("No active document.")
        return {'outers': None, 'inners': None}

    split = split_outer_inner_wires(sketch)
    outers, inners = split['outers'], split['inners']

    if not outers and not inners:
        log_error("Sketch has no valid wires to offset.")
        return {'outers': None, 'inners': None}

    base_label = getattr(sketch, 'Label', sketch.Name)
    sk_out = doc.addObject('Sketcher::SketchObject', f"{base_label}_TmpOffset_Out")
    sk_in = doc.addObject('Sketcher::SketchObject', f"{base_label}_TmpOffset_In")
    sk_out.Placement = sketch.Placement
    sk_in.Placement = sketch.Placement

    # Process outers (offset outward by `distance`)
    for w in outers:
        shp = offset_wire(w, distance)
        if shp: add_shape_to_sketch(sk_out, shp)

    # Process inners (offset inward by `distance`)
    for w in inners:
        shp = offset_wire(w, -distance)
        if shp: add_shape_to_sketch(sk_in, shp)

    doc.recompute()

    # Union the results. Crucially, check for self-intersections on the outer part.
    # A positive distance means expansion, which can cause self-intersection.
    # A negative distance means shrinking, which cannot.
    check_self_intersect = distance > 0
    union_out = upgrade_sketch_to_union(sk_out, do_self_intersection_check=check_self_intersect, name=f'{name}_unionOut')
    union_in = upgrade_sketch_to_union(sk_in, do_self_intersection_check=False, name=f'{name}_unionIn')
    doc.recompute()

    return {'outers': union_out, 'inners': union_in}


# --- Main Execution ---

def run_offset_macro():
    """Main function to run the offset operation."""
    sk = get_active_sketch()
    if not sk:
        return

    dist = prompt_offset_distance(1.0)
    if dist is None:
        log_info("Operation canceled.")
        return

    doc = App.ActiveDocument
    temp_objs = []
    doc.openTransaction("Robust Sketch Offset")
    try:
        log_info(f"Offsetting '{sk.Label}' by {dist:.3f} mm...")

        # 1. Perform the offset on outer and inner wires separately
        offsets = outer_inner_offsets(sk, dist, 'offset')
        out_face = offsets.get("outers")
        in_face = offsets.get("inners")
        if out_face: temp_objs.append(out_face)
        if in_face: temp_objs.append(in_face)

        final_cut_obj = None
        # 2. Cut the inner shapes from the outer shapes
        if out_face and in_face:
            log_info("Cutting inner offsets from outer offset.")
            # Note: Downgrade can return a list or single object
            (cut_result, _) = Draft.downgrade([out_face, in_face], delete=True, force='cut2')
            temp_objs.extend(cut_result if isinstance(cut_result, list) else [cut_result])
            final_cut_obj = cut_result[0] if isinstance(cut_result, list) else cut_result
        elif out_face:
            log_info("No inner holes to cut.")
            final_cut_obj = out_face
        else: # Only inners existed and they were offset to nothing
             log_info("Offset resulted in an empty shape.")
             final_cut_obj = None

        if not final_cut_obj:
            log_warn("Final offset operation resulted in an empty shape.")
            doc.commitTransaction()
            return

        doc.recompute()

        # 3. Convert the final result back to a sketch
        log_info("Creating final sketch...")
        final_sk = Draft.make_sketch(final_cut_obj, autoconstraints=True)
        final_sk.Label = f"{sk.Label}_Offset"
        doc.recompute()

        # 4. Clean up remaining intermediate objects
        for o in temp_objs:
            try:
                if o and doc.getObject(o.Name):
                    doc.removeObject(o.Name)
            except Exception:
                pass # Object might have already been deleted by an operation

        doc.recompute()
        log_info(f"Offset completed. Created '{final_sk.Label}'.")

    except Exception as e:
        log_error(f"Offset macro failed: {e}\n{traceback.format_exc()}")
        doc.abortTransaction()
    finally:
        try:
            doc.commitTransaction()
        except Exception:
            pass

if __name__ == "__main__":
    run_offset_macro()

'''

ZFaceSection : find sections between cutting tools and faces


##########

OpenCascade `section` docs:
  https://dev.opencascade.org/doc/overview/html/specification__boolean_operations.html

Notes on `section`:
  - `approximate` cannot be applied as a kwarg, and seems to help
  - rotating a cutting head around Z helps it succeed
  - polygonal approximations seem to help `section` succeed
  - either do
    - part.section(tool, True)  # for approximation=True
    - part.section(tool, 0.5)   # for tolerance=0.5
  - section is parallel-capable, but returns a list of edges, maybe multiple per tool intersection, so I don't know if its worth linking back up


section(...) method of Part.Compound instance
    Section of this with a given (list of) topo shape.
    section(tool,[approximation=False]) -> Shape
      or
    section((tool1,tool2,...),[tolerance=0.0, approximation=False]) -> Shape
    --
    If approximation is True, section edges are approximated to a C1-continuous BSpline curve.

    Supports:
    - Fuzzy Boolean operations (global tolerance for a Boolean operation)
    - Support of multiple arguments for a single Boolean operation (s1 AND (s2 OR s3))
    - Parallelization of Boolean Operations algorithm

    OCC 6.9.0 or later is required.


'''

import math
import FreeCAD
import Part
import Path


##################################################
# Intersection helpers

def mk_poly_cylinder(radius, height, segments, center):
    ''' a polygonal approximation to a cylinder '''
    pts = []
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = center.x + radius * math.cos(angle)
        y = center.y + radius * math.sin(angle)
        pts.append(FreeCAD.Vector(x, y, center.z))
    pts.append(pts[0])  # close the loop

    wire = Part.makePolygon(pts)
    face = Part.Face(wire)
    solid = face.extrude(FreeCAD.Vector(0, 0, height))
    return solid


def is_valid_section(section, radius):
    ''' Verify that intersect_z sections were usable.

    fail_if_no_collision: if testing a compound, set False. If testing faces individually, set True.
    '''
    # basic checks
    if not section.isValid():
        Path.Log.warning('section not valid')
        return False

    if section.isNull():
        Path.Log.warning('section is null')
        return False

    # FC Bug: https://github.com/FreeCAD/FreeCAD/issues/24111
    for axis in ['XMin', 'XMax', 'YMin', 'YMax', 'ZMin', 'ZMax']:
        if abs(getattr(section.BoundBox, axis)) > 1e6:
            # Path.Log.warning(f'section has extreme bounds: {section.BoundBox=}')
            return False
    return True


def is_full_length(section, radius):
    ''' '''
    # check for partial section results. Could be:
    #   1. section failed
    #   2. bit was hanging over an edge (but a correct result)
    min_len = 2 * 3.14159 * radius * 0.98
    if section.Length < min_len:
        # Path.Log.warning(f'path too short: {section.Length=}')
        # Part.show(section)
        return False
    return True


def is_empty_section(section):
    '''  '''
    # if testing per-face, we don't want to fail for non-collision (won't
    # collide with most faces)
    if len(section.Edges) == 0:
        # Path.Log.warning('section is empty')
        return False
    return True


def try_intersect_z(
        faces_compound,
        faces,
        x, y,  # find z height at this XY
        radius,  # cylinder radius
        tool_height,  # height of fake cylinder we'll intersect with part
        rotate_x=None,
        rotate_y=None,
        rotate_z=None,
        use_approximation=False,
        use_compound=True,
        use_poly_cylinder=False,
        rule_out_short_length=True
):
    """
    Computes the highest Z coordinate of the intersection between a workpiece and a tool (collision bound's z offset).

    NOTE: `section` has bugs:
      - https://github.com/FreeCAD/FreeCAD/issues/24115
      - https://github.com/FreeCAD/FreeCAD/issues/24111
      - https://discord.com/channels/870877411049357352/870877411623985154/1419398872790466680

    returns:
      success:bool, best_guess_zmax

    """

    N_POLY_SEGMENTS = 32

    # make cylinder (square endmill)
    center = FreeCAD.Vector(x, y, -tool_height/2)
    if not use_poly_cylinder:
        cyl = Part.makeCylinder(
            radius, # radius
            tool_height, # height
            center, # pnt
            FreeCAD.Vector(0, 0, 1) # dir
        )
    else:
        # polygonal approximation to cylinder can be more robust with `section`
        cyl = mk_poly_cylinder(
            radius,
            tool_height,
            N_POLY_SEGMENTS,
            center
        )

    # optional rotations, for use when `section` is flaky
    rot_center = FreeCAD.Vector(x, y, 0)
    if rotate_z:  # for rotationally symmetric endmill, should give an equivalent result
        cyl.rotate(rot_center, FreeCAD.Vector(0, 0, 1), rotate_z)
    if rotate_x:  # will break results
        cyl.rotate(rot_center, FreeCAD.Vector(1, 0, 0), rotate_x)
    if rotate_y:  # will break results
        cyl.rotate(rot_center, FreeCAD.Vector(0, 1, 0), rotate_y)

    # 2. Find the collision region
    if use_compound:
        # Collide with compound faces
        section = cyl.section(faces_compound, use_approximation)
        section.tessellate(0.01)  # populate BBox. TODO: not sure what value to use
        zmax = section.BoundBox.ZMax

        # m = Part.show(section)
        # m.ViewObject.LineColor = (0.0, 0.0, 1.0)

        if not is_valid_section(section, radius):
            return False, None  # fail, no best guess

        if rule_out_short_length and not is_full_length(section, radius):
            return False, zmax  # fail, best guess

        if zmax is None:
            return False, None

        return True, zmax  # success, true zmax

    else:
        # Check collisions with each face separately
        zmax = None
        sections = []
        for face in faces:
            section = cyl.section(face, use_approximation)
            section.tessellate(0.01)  # populate BBox. TODO: not sure what value to use

            if not is_valid_section(section, radius):
                continue  # cant guess from invalid sections
            if not is_empty_section(section):
                sections.append(section)

        if len(sections) == 0:
            return False, None

        min_len = 2 * 3.14159 * radius * 0.98
        length = sum([section.Length for section in sections])
        zmax = max([section.BoundBox.ZMax for section in sections
                    if section.BoundBox.ZMax is not None])
        if rule_out_short_length and length < min_len:
            return False, zmax
        return True, zmax


def intersect_z(
        faces_compound,
        faces,
        tool_radius,
        x, y  # check collision height at this XY
):
    '''NOTE: hacky af, but `section` often fails to find the right section,
    so I compare the Length of the found segment against the diam of the
    cylinder. If it's off, I rotate the cylinder around Z axis, then try
    again.

    '''
    # hack, could ray cast and position endmill on surface, then continue checking collision section
    tool_height = 1000

    # HACKY: `section` is buggy (OCC bug), and returns partial
    # intersections, so try a bunch of times
    runs = [
        # try safe things
        {'radius': tool_radius, 'tool_height': tool_height, 'use_approximation': False, 'rule_out_short_length': True},
        {'radius': tool_radius, 'tool_height': tool_height, 'rotate_z': 180, 'use_approximation': False, 'rule_out_short_length': True},
        {'radius': tool_radius, 'tool_height': tool_height, 'rotate_z': 45, 'use_approximation': False, 'rule_out_short_length': True},

        # use approximation
        {'radius': tool_radius, 'tool_height': tool_height, 'use_approximation': True, 'rule_out_short_length': False},
        {'radius': tool_radius, 'tool_height': tool_height, 'rotate_z': 180, 'use_approximation': True, 'rule_out_short_length': False},

        # use hacks
        {'radius': tool_radius, 'tool_height': tool_height, 'use_compound': False, 'rule_out_short_length': False},
        {'radius': tool_radius, 'tool_height': tool_height, 'use_poly_cylinder': True, 'rule_out_short_length': False},
        {'radius': tool_radius, 'tool_height': tool_height, 'use_poly_cylinder': True, 'use_compound': False, 'rule_out_short_length': False},
    ]

    zmaxs = []  # for use when try_intersect_z may fail, but we might still want to guess from its partial successes
    for i, run_params in enumerate(runs):
        success, z = try_intersect_z(
            faces_compound,
            faces,
            x,
            y,
            **run_params
        )
        if z is not None:
            # maybe success was False, but if z is not None, we'll count it as a partial success
            zmaxs.append(z)

        if success:
            if i > 0:
                # Path.Log.warning(f'intersection failed until run {i}')
                pass
            return max(zmaxs)

    # Mark problem regions
    # Path.Log.error(f'failed to intersect at point {x=} {y=}')
    if False:
        marker = Part.makeCylinder(
            tool_radius, # radius
            3, # height
            FreeCAD.Vector(x, y, 0.0), # pnt
            FreeCAD.Vector(0, 0, 1) # dir
        )
        m = Part.show(marker)
        m.ViewObject.LineColor = (1.0, 0.0, 0.0)

    return None

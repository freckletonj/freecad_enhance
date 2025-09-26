# -*- coding: utf-8 -*-
"""
Macro: Color Faces by Z Level (Max Contrast)
Author: Adapted from Mario52's macro by ChatGPT
Date: 2025-08-10
Description:
    Colors only faces parallel to the X–Y plane.
    Faces at the same Z offset share the same color.
    Colors are chosen to maximize perceptual contrast,
    so adjacent Z levels never look too similar.
"""

import FreeCAD
import FreeCADGui
import colorsys
import math
import itertools


def is_face_parallel_to_xy(face, tolerance=1e-6):
    """Check if the face is parallel to the X–Y plane based on its normal vector."""
    normal = face.normalAt(0, 0)  # center normal
    return abs(normal.x) < tolerance and abs(normal.y) < tolerance and abs(abs(normal.z) - 1.0) < tolerance


def get_face_z_offset(face):
    """Return the average Z coordinate of the face's vertices."""
    verts = face.Vertexes
    return round(sum(v.Point.z for v in verts) / len(verts), 3)  # mm precision


def rgb_distance(c1, c2):
    """Euclidean distance in RGB space."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(c1[:3], c2[:3])))


def generate_max_contrast_colors(n):
    """
    Generate n maximally contrasting colors.
    Uses HSV for a broad range, then reorders to maximize contrast between neighbors.
    """
    # Start with evenly spaced hues, but alternate saturation/value for variation
    base_colors = []
    for i in range(n):
        hue = i / float(n)
        sat = 0.9 if i % 2 == 0 else 0.6
        val = 0.9 if i % 3 != 0 else 0.6
        rgb = colorsys.hsv_to_rgb(hue, sat, val)
        base_colors.append((rgb[0], rgb[1], rgb[2], 1.0))

    # Greedy reordering: start with one color, always pick farthest remaining
    if n > 1:
        reordered = [base_colors.pop(0)]
        while base_colors:
            last = reordered[-1]
            farthest_color = max(base_colors, key=lambda c: rgb_distance(last, c))
            base_colors.remove(farthest_color)
            reordered.append(farthest_color)
        return reordered
    else:
        return base_colors


def generate_max_contrast_colors(n):
    """
    Generate n maximally contrasting colors from 6 hardcoded aesthetic colors.
    Colors are chosen for maximum visual distinction and aesthetic appeal.
    """
    if n <= 0:
        return []
    
    # 6 aesthetic colors chosen for maximum contrast and visual appeal
    colors = [
        (0.9, 0.3, 0.3, 1.0),   # Coral Red
        (0.3, 0.7, 0.9, 1.0),   # Sky Blue  
        (0.9, 0.7, 0.2, 1.0),   # Golden Yellow
        (0.4, 0.8, 0.5, 1.0),   # Fresh Green
        (0.8, 0.4, 0.9, 1.0),   # Lavender Purple
        (0.9, 0.6, 0.3, 1.0),   # Warm Orange
    ]
    
    return [colors[i % 6] for i in range(n)]

def color_faces_by_z_levels(obj):
    """Main logic to color faces of an object based on Z levels."""
    shape = obj.Shape

    # Prepare diffuse colors list
    colors = list(obj.ViewObject.DiffuseColor)
    if len(colors) != len(shape.Faces):
        base_color = colors[0] if colors else (0.8, 0.8, 0.8, 1.0)
        colors = [base_color] * len(shape.Faces)

    # Collect Z offsets for parallel faces
    z_offsets = sorted({get_face_z_offset(face) for face in shape.Faces if is_face_parallel_to_xy(face)})

    # Generate high-contrast colors
    z_color_map = {z: col for z, col in zip(z_offsets, generate_max_contrast_colors(len(z_offsets)))}

    # Apply colors to matching faces
    for i, face in enumerate(shape.Faces):
        if is_face_parallel_to_xy(face):
            z_offset = get_face_z_offset(face)
            colors[i] = z_color_map[z_offset]

    obj.ViewObject.DiffuseColor = colors
    FreeCAD.Console.PrintMessage(f"Applied {len(z_offsets)} high-contrast colors for Z-levels: {z_offsets}\n")


if __name__ == "__main__":
    try:
        selection_objects = FreeCADGui.Selection.getSelection()
        if not selection_objects:
            raise Exception("No object selected")

        color_faces_by_z_levels(selection_objects[0])

    except Exception as e:
        FreeCAD.Console.PrintError(f"Error: {e}\n")

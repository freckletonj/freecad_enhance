# FreeCAD Enhance

## ZFace

Forum: https://forum.freecad.org/viewtopic.php?t=99834

Lift a 2D operation (eg pocket) up onto some faces to make it 3D. It's still very beta. Use at your own risk.

1. Add `ZFace` at `~/.local/share/FreeCAD/Mod/ZFace`
2. Add a button for this tool through `Tools > Customize > Search "ZFace"`
3. Make a `CAM Pocket Op`
4. Invoke this tool
5. Select some faces to lift the pocket up onto
6. It will respect square endmill geom (other geoms should be trivial to add in the future)


## Macros

### `smart_offset.py`

A more intelligent sketch offset, that is robust in the face of multiple islands and holes in your geom, and is not sensitive to "winding order" (the order you drew the points/lines in).

### `carve.py`

Offset a sketch out, and then back in. The resulting geometry is exactly the outlines that would be left after a square endmill has gone through and cut. Eg convex curves/lines stay don't shift, but narrow angles will get a fillet with the diameter of your bit you use, and separate islands that are closer than `diam` will get a bridge where the bit couldn't fit.

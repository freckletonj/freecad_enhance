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

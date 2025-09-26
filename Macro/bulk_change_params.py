# -*- coding: utf-8 -*-

# Macro Begin: /home/person/.local/share/FreeCAD/Macro/bulk_change_params.FCMacro +++++++++++++++++++++++++++++++++++++++++++++++++
# import FreeCAD

# Gui.runCommand('Std_DlgMacroRecord',0)
# App.getDocument('t02_cnc_cam_clamp').Profile001.setExpression('StepDown', u'30 mm')
# Gui.Selection.clearSelection()
# Gui.Selection.addSelection('t02_cnc_cam_clamp','Profile009')
# Gui.Selection.clearSelection()
# Gui.Selection.addSelection('t02_cnc_cam_clamp','Profile008')
# Gui.Selection.clearSelection()
# Gui.Selection.addSelection('t02_cnc_cam_clamp','Profile009')
# Gui.Selection.clearSelection()
# Gui.Selection.addSelection('t02_cnc_cam_clamp','Profile010')
# Gui.Selection.clearSelection()
# Gui.Selection.addSelection('t02_cnc_cam_clamp','Profile009')
# Macro End: /home/person/.local/share/FreeCAD/Macro/bulk_change_params.FCMacro +++++++++++++++++++++++++++++++++++++++++++++++++


import FreeCAD as App
import FreeCADGui as Gui

try:
    from PySide6 import QtWidgets  # FreeCAD == 1.1
except ImportError:
    from PySide2 import QtWidgets  # FreeCAD < 1.1


def get_string_input(title: str, message: str, default: str = "") -> str | None:
    """Prompts the user for a string input using QInputDialog."""
    if QtWidgets is None:
        App.Console.PrintError("ERROR: GUI toolkit (QtWidgets) not available.\n")
        return None

    try:
        parent = Gui.getMainWindow()
    except Exception:
        parent = None

    try:
        text, ok = QtWidgets.QInputDialog.getText(
            parent,
            title,
            message,
            QtWidgets.QLineEdit.Normal,
            default
        )
        if not ok:
            return None
        return text.strip()
    except Exception as ex:
        App.Console.PrintError(f"ERROR: Could not open input dialog: {ex}\n")
        return None


# --- MAIN SCRIPT ---

# Ask user for parameter path (property name)
param_path = get_string_input(
    "Parameter Path",
    "Enter parameter name (e.g. StepDown, FinalDepth, SafeHeight):",
    ""
)

if param_path is None or param_path == "":
    App.Console.PrintMessage("Operation cancelled by user.\n")
else:
    # Ask user for parameter value (expression or plain value)
    param_value = get_string_input(
        "Parameter Value",
        "Enter value (with units if needed, e.g. 30 mm, 0.5):",
        ""
    )

    if param_value is None or param_value == "":
        App.Console.PrintMessage("Operation cancelled by user.\n")
    else:
        App.Console.PrintMessage(f"Setting parameter: {param_path}\n")
        App.Console.PrintMessage(f"To value: {param_value}\n")
        App.Console.PrintMessage("=" * 50 + "\n")

        selection = Gui.Selection.getSelection()

        if not selection:
            App.Console.PrintError("No objects selected. Please select one or more CAM operations.\n")
        else:
            # --- VALIDATION ---
            App.Console.PrintMessage("VALIDATION PHASE:\n")
            valid_objects = []
            validation_passed = True

            for obj in selection:
                obj_valid = True

                # Check if object looks like a Path/CAM feature
                if not obj.isDerivedFrom("Path::Feature"):
                    App.Console.PrintError(f"ERROR: {obj.Name} - Not a CAM operation\n")
                    validation_passed = False
                    obj_valid = False

                # Check if property exists
                elif param_path not in obj.PropertiesList:
                    App.Console.PrintError(f"ERROR: {obj.Name} - Missing parameter '{param_path}'\n")
                    validation_passed = False
                    obj_valid = False
                else:
                    # App.Console.PrintMessage(f"✓ {obj.Name} has parameter '{param_path}'\n")
                    pass

                if obj_valid:
                    valid_objects.append(obj)
                    # App.Console.PrintMessage(f"✓ {obj.Name} - Ready for processing\n")

            App.Console.PrintMessage(
                f"\nValidation complete. {len(valid_objects)} of {len(selection)} objects passed validation.\n"
            )

            if not validation_passed:
                App.Console.PrintError("VALIDATION FAILED - No changes will be made.\n")
            elif len(valid_objects) == 0:
                App.Console.PrintError("No valid objects to process.\n")
            else:
                # --- EXECUTION ---
                App.Console.PrintMessage("\nEXECUTION PHASE:\n")
                processed_count = 0

                for obj in valid_objects:
                    try:
                        # Prefer setExpression for values with units or formulas
                        obj.setExpression(param_path, param_value)
                        App.Console.PrintMessage(f"✓ Set {obj.Name}.{param_path} = {param_value}\n")
                        processed_count += 1

                    except Exception as e:
                        App.Console.PrintError(f"ERROR: Failed to set parameter for {obj.Name}: {str(e)}\n")

                App.Console.PrintMessage(f"\nSuccessfully processed {processed_count} object(s)\n")

                if processed_count > 0:
                    App.Console.PrintMessage("Recomputing document...\n")
                    App.ActiveDocument.recompute()
                    App.Console.PrintMessage("Done!\n")
                else:
                    App.Console.PrintError("No objects were successfully processed.\n")

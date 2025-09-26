import FreeCADGui
import ZFace

FreeCADGui.addCommand("CAM_ZFace", ZFace.CommandPathZFace())


##################################################
# Run some dev helpers on startup


##########
# Reload ZFace

dev = [
    'import ZFaceSection',
    'import QuadTree',
    'import ZFace',
    'import importlib',
    'from PySide import QtGui, QtCore',
    '''
def r():
    print('reloading ZFace')
    importlib.reload(ZFaceSection)
    importlib.reload(QuadTree)
    importlib.reload(ZFace)
'''
'''
def setupReloadKey():
    try:
        # Create action
        action = QtGui.QAction("Reload ZFace", FreeCADGui.getMainWindow())
        action.setShortcut(QtGui.QKeySequence("Ctrl+Shift+R"))
        action.triggered.connect(r)
        FreeCADGui.getMainWindow().addAction(action)
    except Exception as e:
        print(f"Failed to register console hotkey: {e}")
setupReloadKey()
'''
]

for d in dev:
    FreeCADGui.doCommand(d)


##########
#

dev = [
    'import FreeCAD',
    'import FreeCADGui',
    'from PySide import QtGui, QtCore',
    '''

def focusPythonConsole():
    """Focus the FreeCAD Python console widget"""
    print("trying to focus console")
    mw = FreeCADGui.getMainWindow()

    consoleDock = None
    for dock in mw.findChildren(QtGui.QDockWidget):
        if dock.windowTitle().lower() == "python console":
            consoleDock = dock
            break

    if consoleDock:
        print("found console dock")
        consoleDock.raise_()
        consoleDock.activateWindow()

        # Target the custom console widget inside
        pyConsole = consoleDock.findChild(QtGui.QWidget, "Python console")
        if pyConsole:
            print(f"focusing {pyConsole.metaObject().className()}")
            pyConsole.setFocus()
        else:
            print("⚠ found dock but no Gui::PythonConsole widget")
    else:
        print("could not find console dock")
''',

    '''
def setupConsoleHotkey():
    """Set up hotkey for focusing Python console"""
    try:
        # Create action
        action = QtGui.QAction("Focus Python Console", FreeCADGui.getMainWindow())
        action.setShortcut(QtGui.QKeySequence("Ctrl+Shift+P"))
        action.triggered.connect(focusPythonConsole)

        # Add to main window
        FreeCADGui.getMainWindow().addAction(action)
        print("Python console hotkey (Ctrl+Shift+P) registered successfully!")

    except Exception as e:
        print(f"Failed to register console hotkey: {e}")
''',
    'setupConsoleHotkey()'
]

for d in dev:
    FreeCADGui.doCommand(d)

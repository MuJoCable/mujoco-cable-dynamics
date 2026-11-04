# Biomimetic finger CAD assets

The four STL files are copied without geometric modification from the user-provided
SolidWorks assembly export. The source geometry is in millimeters and the MJCF applies
`scale="0.001 0.001 0.001"`.

STEP analytic cylinders and STL vertices were matched with one common translation and
no rotation:

```text
p_stl_mm = p_step_mm + (-178.930059, 254.595927, 814.585419)
```

The recovered hinge axes in STL assembly coordinates are:

```text
MCP: point (14.069941, 120.095927, 897.585419) mm, direction +Y
PIP: point (14.069941, 120.095927, 939.585419) mm, direction +Y
DIP: point (14.069941, 120.095927, 959.585419) mm, direction +Y
```

The debug model shifts the world origin to MCP but retains the STL axis order. The full
STL meshes remain visual geometry, while the six `*_contact.obj` files are deterministic
low-resolution joint-surface patches extracted from those same STLs. They preserve the
non-convex triangle contact shape without generating thousands of coplanar whole-shell
contacts. Regenerate them with:

```bash
python scripts/prepare_biomimetic_finger_contact_patches.py
```

Adjacent-body kinematics are provided by the recovered hinge axes. Cable wrapping in
this tuning model initially uses analytic cylinders so route-side changes remain easy to
diagnose independently from STL contact.

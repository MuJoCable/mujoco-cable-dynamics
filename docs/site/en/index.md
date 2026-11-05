# MuJoCable Documentation

MuJoCable is a massless, pull-only cable-transmission plugin for MuJoCo. It computes routes over moving cylinders and closed meshes, then evaluates extension, tension, friction propagation and rigid-body forces.

<div class="manual-links"><a href="https://mujocable.github.io/">Project</a><a href="https://github.com/MuJoCable/mujoco-cable-dynamics">GitHub</a><a href="https://arxiv.org/abs/2609.09612">Paper</a><a href="../index.html" lang="zh-CN">中文文档</a></div>

<p class="manual-meta">Updated 2026-11-05 · Reference environment: MuJoCo 3.4.0 / Python 3.12 · Resources are pinned to a public commit. Historical recordings and fresh numerical reproductions are labeled separately.</p>

## Where to Start

| Your goal | Reading order |
|---|---|
| Run your first cable model | [Installation](installation.md) → [Quick start](quickstart.md) → [Actuation](actuation.md) |
| Thread a new robot | [MJCF structure](modeling.md) → [Routing](routing.md) → [Friction and eyelets](friction.md) |
| Record experiments | [Sensors and Python](sensors.md) → [Examples](gallery.md) → [Validation](validation.md) |
| Understand the implementation | [Physics](computation.md) → [MuJoCo interfaces](architecture.md) → [XML reference](reference.md) |
| Diagnose a model | [Troubleshooting](debugging.md) → [Limitations](limitations.md) |

## Division of Responsibility

MuJoCo handles rigid-body mass and inertia, joints, rigid-body contacts, constraints and time integration. MuJoCable handles the configured cable route, tensile constitutive law, segment friction and cable-force map.

**A cable does not automatically collide with a geom merely because its displayed line crosses it.** Configure the relevant `wrap_geoms` or physical guides. Initialization hints, the native seed polyline and the solved runtime route have different roles.

<img class="example-poster" src="../_static/media/overview.jpg" alt="MuJoCo simulation of the Faive surface-routed rolling joint">

## Scope

Use the plugin for short cable transmissions, pulleys, winches, passive ligaments, tensegrity and underactuated robots. It represents routing, pretension, slack reserve, axial compliance and sliding friction.

It is not a massive-rope, sagging-rope, knotting or general free-cable collision solver. The initialized route branch is retained. A point guide is not finite-diameter cable contact with a hole wall. Read [limitations](limitations.md) before choosing a modeling approach.

```{toctree}
:maxdepth: 2
:caption: User Guide

installation
quickstart
modeling
actuation
routing
friction
sensors
debugging
```

```{toctree}
:maxdepth: 2
:caption: Physics and Interfaces

computation
architecture
reference
limitations
```

```{toctree}
:maxdepth: 2
:caption: Examples and Resources

gallery
_generated/case_index
validation
downloads
english
changelog
```

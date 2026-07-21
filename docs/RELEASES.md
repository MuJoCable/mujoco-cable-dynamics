# Binary Release Policy

MuJoCo engine plugins are native shared libraries. A binary is compatible only
with the operating system, CPU architecture, and MuJoCo ABI against which it
was built. Release bundles therefore use names such as:

```text
mujoco-cable-dynamics-v0.1.0-darwin-arm64.tar.gz
mujoco-cable-dynamics-v0.1.0-linux-x86_64.tar.gz
```

Each bundle contains:

```text
lib/                         compiled plugin
cable_plugin_demos/          selected MJCF models and assets
scripts/run_demo.sh          relative-path launcher
scripts/view_cpp_plugin_demo.py
README.md and README_zh.md
LICENSE and third-party notices
```

The release workflow installs `mujoco==3.4.0`, builds and smoke-tests the
plugin, packages the bundle, and publishes a SHA-256 file. Users should install
a MuJoCo `3.4.x` Python runtime before running a bundle.

The source repository remains the portable fallback for unsupported platforms.
Building from source against the user's installed MuJoCo is preferred when an
exact ABI match is required.

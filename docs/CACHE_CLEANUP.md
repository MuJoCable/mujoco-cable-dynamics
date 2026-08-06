# Cache Cleanup

This repository separates disposable local caches from source code, runtime
assets, release packages, and experiment results. The cleanup command is a dry
run by default.

## Keep

The following content is part of the project or its reproducible evidence and
must not be treated as cache:

- `plugin/`, `python/`, `scripts/`, and `tests/`;
- `cable_plugin_demos/*.xml` and all STL/OBJ route and visual assets;
- asset manifests and generated route meshes required at runtime;
- screenshots, GIFs, videos, CSV/JSON measurements, and `docs/figures/`;
- Markdown/PDF reports and technical notes;
- `dist/` release archives and checksum files;
- `.git/` and GitHub workflow configuration.

## Safe cleanup

Preview interpreter caches, MuJoCo logs, and macOS metadata:

```bash
./scripts/clean_cache.sh
```

Delete only those previewed targets:

```bash
./scripts/clean_cache.sh --apply
```

Include local CMake build trees when a clean rebuild is acceptable:

```bash
./scripts/clean_cache.sh --include-build
./scripts/clean_cache.sh --include-build --apply
```

The script never selects `dist/`, result directories, figures, videos, meshes,
or source files. Generated experiment outputs should be archived and checked
before any manual cleanup of `output/` or `outputs/`.

## External temporary files

Mesh repair and packaging may leave disposable staging directories under the
system temporary root. Inspect them first:

```bash
TMP_ROOT="${TMPDIR:-/tmp}"
du -sh "$TMP_ROOT"/mujocable_* 2>/dev/null
```

Known disposable names from the local asset-generation workflow include:

```text
$TMP_ROOT/mujocable_mesh_deps
$TMP_ROOT/mujocable_mesh_route_stage
$TMP_ROOT/mujocable_index_stage
```

Remove only entries that are no longer used by a running build or viewer.

## Recommended policy

1. Run the dry run after tests or asset generation.
2. Use the default cleanup routinely.
3. Use `--include-build` only when rebuilding the plugin is acceptable.
4. Keep release archives until their GitHub Release and checksum are verified.
5. Keep raw and processed experiment results; compress or archive them instead
   of classifying them as cache.

# 缓存清理说明

本仓库将可丢弃缓存与核心代码、运行资产、Release 包和实验结果分开管理。
清理脚本默认只预览，不会直接删除。

## 必须保留

- `plugin/`、`python/`、`scripts/` 和 `tests/`；
- `cable_plugin_demos/*.xml` 以及 STL/OBJ 显示和绳路资产；
- 运行时需要的 manifest 与修复后 route mesh；
- 截图、GIF、视频、CSV/JSON 数据和 `docs/figures/`；
- Markdown/PDF 报告和技术说明；
- `dist/` 中的 Release 压缩包与校验文件；
- `.git/` 和 GitHub CI 配置。

## 安全清理命令

仅预览 Python 缓存、MuJoCo 日志和 macOS 元数据：

```bash
./scripts/clean_cache.sh
```

确认列表后执行删除：

```bash
./scripts/clean_cache.sh --apply
```

需要彻底重新编译时，再显式加入本地 CMake 构建目录：

```bash
./scripts/clean_cache.sh --include-build
./scripts/clean_cache.sh --include-build --apply
```

脚本不会选择 `dist/`、实验结果、图表、视频、mesh 或源代码。手工删除
`output/`、`outputs/` 前，应先检查并归档其中的数据。

## 仓库外临时目录

mesh 修复和打包过程可能在系统临时目录留下 `mujocable_*` 临时目录。
先使用以下命令查看占用：

```bash
TMP_ROOT="${TMPDIR:-/tmp}"
du -sh "$TMP_ROOT"/mujocable_* 2>/dev/null
```

确认没有构建或 viewer 正在使用后，再删除不需要的目录。建议日常只运行默认
清理；只有确认可以重新编译时才使用 `--include-build`，Release 上传并验证前
保留 `dist/`，实验数据优先归档而不是删除。

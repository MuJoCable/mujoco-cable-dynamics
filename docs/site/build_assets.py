"""Build local media, dependency-closed model bundles and generated reference."""
import argparse
import hashlib
import html
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET
import zipfile

from catalog import CASES, CONFIG

SITE = Path(__file__).resolve().parent
REF = "fccad906154cddb8e05227924a6c72efc80e3387"
REPO_URL = "https://github.com/MuJoCable/mujoco-cable-dynamics"


def dependencies(model, repo, seen=None):
    seen = set() if seen is None else seen
    model = model.resolve()
    if model in seen:
        return seen
    if not model.is_relative_to(repo) or not model.is_file():
        raise ValueError(f"Missing or external model: {model.name}")
    seen.add(model)
    root = ET.parse(model).getroot()
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    texdir = compiler.get("texturedir", "") if compiler is not None else ""
    for node in root.iter():
        name = node.get("file")
        if not name:
            continue
        base = meshdir if node.tag in {"mesh", "flexcomp"} else texdir if node.tag == "texture" else ""
        target = (model.parent / base / name).resolve()
        if not target.is_relative_to(repo) or not target.is_file():
            raise ValueError(f"Missing dependency: {name} in {model.name}")
        if node.tag == "include":
            dependencies(target, repo, seen)
        else:
            seen.add(target)
            for notice in target.parent.glob("LICENSE*"):
                seen.add(notice)
    return seen


def bundle(target, files, repo):
    with zipfile.ZipFile(target, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as package:
        for path in sorted(set(files) | {repo / "LICENSE"}):
            package.write(path, path.relative_to(repo).as_posix())
        sources = repo / "examples/paper/ASSET_SOURCES.md"
        package.write(sources, "ASSET_SOURCES.md")
        package.writestr("BUNDLE_README.txt", "MuJoCable documentation resource bundle\nSource: " + REPO_URL + "/tree/" + REF + "\nExtract into a separate folder; preserve relative model/asset paths.\nUse a matching, compiled MuJoCable plugin and the repository viewer.\nThe model bundle is not a Python/MuJoCo runtime.\n")


def viewer_command(case):
    key = case["id"]
    view_key = dict(rolling="rolling_mesh", calibrated="spirobs_calibrated",
                    moving="moving_pulley", fixed="fixed_friction").get(key, key)
    if case["model"].startswith("examples/showcase") or key == "friction":
        parts = ["mjpython scripts/view_cpp_plugin_demo.py",
                 "--plugin build/plugin/libcable_unilateral.dylib",
                 f"--model {case['model']} --no-auto-control --show-cable-state"]
        return (" \\" + "\n  ").join(parts)
    return f"python examples/paper/run.py view {view_key} --show-cable-state"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=SITE.parents[1])
    args = parser.parse_args()
    repo = args.repo.resolve()
    generated = SITE / "_generated"
    media = SITE / "_static/media"
    downloads = SITE / "downloads"
    for path in (generated, media, downloads, SITE / "_includes"):
        path.mkdir(parents=True, exist_ok=True)
    source = (repo / "plugin/unilateral_cable.cc").read_text()
    constants = dict(re.findall(r'constexpr char (kAttr\w+)\[\]\s*=\s*"([^"]+)";', source))
    registry = source.split("static const char *attributes[] =", 1)[1].split("};", 1)[0]
    actual = {constants[name] for name in re.findall(r"kAttr\w+", registry)}
    if actual != {item[0] for item in CONFIG}:
        raise ValueError(f"Reference mismatch: {actual.symmetric_difference({item[0] for item in CONFIG})}")
    rows = ["| 配置键 | 默认 | 单位/类型 | 含义 |", "|---|---|---|---|"]
    rows += [f"| `{key}` | `{default}` | {unit} | {description} |" for key, default, unit, description in CONFIG]
    (SITE / "_includes/configuration.md").write_text("\n".join(rows) + "\n")
    gallery = ['# 案例库', '', '按机构和问题浏览。每个案例页都有截图、可播放视频、完整模型资产 ZIP 和运行命令。视频为历史录制；2026-11-04 的新版数值复跑另见验证章节。', '', '<input class="example-filter" data-example-filter aria-label="筛选案例" placeholder="筛选：滑轮、摩擦、手指、mesh…">', '<p data-example-count>14 / 14</p>', '<div class="example-grid">']
    inventory = []
    for case in CASES:
        key = case["id"]
        model = repo / case["model"]
        video = repo / case["video"]
        local_video = media / video.name
        shutil.copy2(video, local_video)
        raw = video.with_name(video.name.replace("_annotated", "_raw"))
        shutil.copy2(raw, media / raw.name)
        poster = media / f"{key}.jpg"
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", "1", "-i", str(raw), "-frames:v", "1", "-vf", "scale=960:-2", str(poster)], check=True)
        models = list(model.parent.glob("*.xml")) if case["model"].startswith("examples/paper") else [model]
        inputs = set()
        for item in models:
            inputs |= dependencies(item, repo)
        bundle(downloads / f"{key}-model.zip", inputs, repo)
        folder = repo / case.get("data", str(model.parent.parent.relative_to(repo)))
        evidence = [path for path in folder.rglob("*") if path.is_file() and path.suffix in {".csv", ".json", ".pdf", ".svg"} and "assets" not in path.parts and "models" not in path.parts]
        if not evidence:
            evidence = [repo / "examples/showcase/latest_validation.json"]
        bundle(downloads / f"{key}-results.zip", evidence, repo)
        shutil.copy2(model, downloads / f"{key}.xml")
        url = f"{REPO_URL}/blob/{REF}/{case['model']}"
        actuator = ET.parse(model).getroot().find("actuator")
        names = [node.get("name", "") for node in actuator] if actuator is not None else []
        commands = viewer_command(case)
        text = [f"# {case['title']}", "", case["description"], "", f"**机构分组：** {case['group']}。本页模型资源固定到公开提交 `{REF[:8]}`。", "", f"![{case['title']}](../_static/media/{key}.jpg)", "", f'<video class="example-video" controls preload="none" playsinline poster="../_static/media/{key}.jpg"><source src="../_static/media/{video.name}" type="video/mp4"><a href="../_static/media/{video.name}">下载视频</a></video>', "", f"[带数字视频](../_static/media/{video.name}) · [Raw 视频](../_static/media/{raw.name})", "", "## 下载与源文件", "", f"[模型与完整依赖 ZIP](../downloads/{key}-model.zip) · [数据/图表 ZIP](../downloads/{key}-results.zip) · [单个 XML](../downloads/{key}.xml) · [GitHub 源文件]({url})", "", "ZIP 保留相对路径；解压到独立目录，不要把它误当成包含 MuJoCo 的独立应用。单个 XML 仍可能需要 mesh。", "", "## 如何驱动", "", "```bash", commands, "```", "", "macOS 使用 `mjpython`；Linux 使用 `python`，库后缀改为 `.so`。论文入口默认关闭自动控制，用 Control 滑块手动输入。", "", f"**实际 actuator 名称：** {', '.join('`'+name+'`' for name in names) or '无：这是被动模型'}。单位与通道类型请结合 XML 的 actuator 类型和 `ctrl_mode`，不要一律按 N 或 deg 输入。", "", "自动实验：仓库根目录运行 `python examples/paper/run.py result1 --skip-videos` 或 `result2 --skip-videos`。校准双向周期使用 `cycle`；曲面展示的自动控制命令见 examples/showcase/README.md。", "", "## 解释与限制", "", case["note"], "", "历史视频不冒称最新版重新录制。下载结果包含原始统计，保留偏差和失败诊断；不要只截取最好的一帧作为完整验证。"]
        (generated / f"{key}.md").write_text("\n".join(text) + "\n")
        gallery.append(f'<section class="example-item"><a href="_generated/{key}.html"><img src="_static/media/{key}.jpg" alt="{html.escape(case["title"])}" loading="lazy"><h3>{case["title"]}</h3></a><p>{case["group"]} · {case["description"]}</p></section>')
        inventory.append({"id": key, "model": case["model"], "assets": len(inputs), "model_zip_sha256": hashlib.sha256((downloads / f"{key}-model.zip").read_bytes()).hexdigest()})
    gallery += ["</div>", "", "论文完整案例与 46 段视频目录见 [GitHub examples](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/examples)。"]
    (SITE / "gallery.md").write_text("\n".join(gallery) + "\n")
    (generated / "case_index.md").write_text("# 案例详情\n\n```{toctree}\n:maxdepth: 1\n\n" + "\n".join(item["id"] for item in CASES) + "\n```\n")
    shutil.copy2(media / "rolling.jpg", media / "overview.jpg")
    for model in (SITE / "_models").glob("*.xml"):
        shutil.copy2(model, downloads / model.name)
    passive = ET.parse(SITE / "_models/minimal_surface.xml")
    passive.getroot().set("model", "mujocable_minimal_passive")
    passive.getroot().remove(passive.getroot().find("actuator"))
    passive.write(downloads / "minimal_passive.xml", encoding="unicode")
    bundle(downloads / "latest-validation.zip", [path for path in (repo / "examples/paper/latest_plugin").rglob("*") if path.is_file()], repo)
    for name in ("RESULT1_CASE_THEORY_DERIVATIONS_ZH.md", "RESULT1_THEORY_AND_RESULTS_ZH.md", "RESULT2_MODEL_AND_METHOD_COMPARISON_ZH.md", "RESULT2_THEORY_AND_RESULTS_ZH.md", "VIDEO_INVENTORY.csv"):
        shutil.copy2(repo / "examples/paper" / name, downloads / name)
    for language in ("zh", "en"):
        name = f"plugin_method_strategy_report_{language}.pdf"
        shutil.copy2(repo / "docs" / name, downloads / name)
    shutil.copy2(repo / "examples/paper/latest_plugin/result2/result_2_3/figures/result_2_3_friction_summary.png", media / "friction_summary.png")
    (downloads / "resource_manifest.json").write_text(json.dumps(inventory, indent=2) + "\n")
    print(f"Prepared {len(CASES)} cases and verified {len(CONFIG)} registered config keys")


if __name__ == "__main__":
    main()

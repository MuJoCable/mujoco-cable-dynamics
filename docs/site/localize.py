"""Prepare English sources, keeping example resources shared with Chinese."""
import html
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from build_assets import REF, REPO_URL, viewer_command
from catalog import CASES, CONFIG
from catalog_en import CASES_EN, CONFIG_EN

SITE = Path(__file__).resolve().parent


def prepare_english(destination, repo):
    expected = {p.name for p in SITE.glob("*.md")} - {"README.md", "gallery.md"}
    actual = {p.name for p in (SITE / "en").glob("*.md")}
    if expected != actual:
        raise ValueError(f"Untranslated chapters: {expected.symmetric_difference(actual)}")
    if {case["id"] for case in CASES} != set(CASES_EN):
        raise ValueError("Translated case IDs differ")
    if [row[0] for row in CONFIG] != [row[0] for row in CONFIG_EN]:
        raise ValueError("Translated config keys differ")
    destination.mkdir(parents=True, exist_ok=True)
    for path in (SITE / "en").glob("*.md"):
        shutil.copy2(path, destination / path.name)
    shutil.copy2(SITE / "conf.py", destination / "conf.py")
    shutil.copytree(SITE / "_static", destination / "_static", dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("media"))
    shutil.copytree(SITE / "_templates", destination / "_templates", dirs_exist_ok=True)
    (destination / "downloads").mkdir(exist_ok=True)
    for path in (SITE / "downloads").glob("minimal_*.xml"):
        shutil.copy2(path, destination / "downloads" / path.name)
    includes = destination / "_includes"
    includes.mkdir(exist_ok=True)
    rows = ["| Key | Default | Unit/type | Meaning |", "|---|---|---|---|"]
    rows += [f"| <code>{key}</code> | <code>{default}</code> | {unit} | {desc} |"
             for key, default, unit, desc in CONFIG_EN]
    (includes / "configuration.md").write_text("\n".join(rows) + "\n")
    generated = destination / "_generated"
    generated.mkdir(exist_ok=True)
    gallery = ["# Example Gallery", "",
        "Every case includes screenshots, playable video, a complete model/assets ZIP and commands. Videos are historical recordings; fresh numerical reproductions are documented under Validation.", "",
        '<input class="example-filter" data-example-filter aria-label="Filter examples" placeholder="Filter: pulley, friction, finger, mesh...">',
        f'<p data-example-count>{len(CASES)} / {len(CASES)}</p>', '<div class="example-grid">']
    fence = chr(96) * 3
    for case in CASES:
        key = case["id"]
        title, group, description, note = CASES_EN[key]
        video = Path(case["video"]).name
        raw = video.replace("_annotated", "_raw")
        media, download = "../../_static/media/", "../../downloads/"
        actuator = ET.parse(repo / case["model"]).getroot().find("actuator")
        names = [node.get("name", "") for node in actuator] if actuator is not None else []
        links = " · ".join(f'<a href="{download}{key}{suffix}">{label}</a>' for suffix, label in
            (("-model.zip", "Model and complete assets ZIP"), ("-results.zip", "Data/figures ZIP"), (".xml", "Single XML")))
        text = [f"# {title}", "", description, "",
            f"**Category:** {group}. Public source revision: {REF[:8]}.", "",
            f'<img class="example-poster" src="{media}{key}.jpg" alt="{html.escape(title)}">', "",
            f'<video class="example-video" controls preload="none" playsinline poster="{media}{key}.jpg"><source src="{media}{video}" type="video/mp4"><a href="{media}{video}">Download video</a></video>', "",
            f'<a href="{media}{video}">Annotated video</a> · <a href="{media}{raw}">Raw video</a>', "",
            "## Downloads and Source", "",
            links + f' · <a href="{REPO_URL}/blob/{REF}/{case["model"]}">GitHub source</a>', "",
            "Preserve relative paths when extracting. This is not an application or a MuJoCo runtime. A single XML may still require meshes.", "",
            "## How to Drive It", "", fence + "bash", viewer_command(case), fence, "",
            "Use mjpython on macOS; use python and a .so library on Linux. The paper wrapper disables automatic control. Use the standard Control sliders.", "",
            "**Actual actuator names:** " + (", ".join(f"<code>{name}</code>" for name in names) or "none; this model is passive") + ". Interpret units using actuator type and ctrl_mode, not universally as N or degrees.", "",
            "Automatic experiments run from the repository root. Use cycle for the calibrated bidirectional trajectory; see examples/showcase/README.md for showcase controls.", "",
            fence + "bash", "python examples/paper/run.py result1 --skip-videos", "python examples/paper/run.py result2 --skip-videos", fence, "",
            "## Interpretation and Limits", "", note, "",
            "Historical videos were not re-rendered with the latest plugin. Statistics retain discrepancies and failure diagnostics; one attractive frame is not complete validation."]
        (generated / f"{key}.md").write_text("\n".join(text) + "\n")
        gallery.append(f'<section class="example-item"><a href="_generated/{key}.html"><img src="../_static/media/{key}.jpg" alt="{html.escape(title)}" loading="lazy"><h3>{title}</h3></a><p>{group} · {description}</p></section>')
    gallery += ["</div>", "", "The complete public archive and its 46 videos are listed in [GitHub examples](https://github.com/MuJoCable/mujoco-cable-dynamics/tree/main/examples)."]
    (destination / "gallery.md").write_text("\n".join(gallery) + "\n")
    (generated / "case_index.md").write_text("# Example Details\n\n" + fence + "{toctree}\n:maxdepth: 1\n\n" + "\n".join(case["id"] for case in CASES) + "\n" + fence + "\n")
    print(f"English: {len(actual)} chapters, {len(CASES_EN)} examples, {len(CONFIG_EN)} config entries")

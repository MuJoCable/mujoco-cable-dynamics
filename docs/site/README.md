# MuJoCable Technical Manual

Chinese task-oriented manual with an English quick start. Sphinx/Furo provides
search, navigation, light/dark themes and static GitHub Pages output. KaTeX
assets are local. Models, video and result resources come from public examples;
no private working directories are required.

## Build

From the repository root:

```bash
python -m venv .venv-docs
source .venv-docs/bin/activate
python -m pip install -r docs/site/requirements.txt
python docs/site/build.py
python -m http.server 8000 --directory docs/site/_build/html
```

FFmpeg must be available to extract example posters. The plugin runtime and
OpenGL are not required for a documentation build. To build this folder before
it is installed at `docs/site`, pass `--repo PATH_TO_PUBLIC_CHECKOUT`.

Generated output is intentionally ignored: case pages, resource bundles, media
and HTML are rebuilt from `catalog.py` and the repository examples. Source
diagrams, minimal XMLs and the local KaTeX assets are versioned.

## Publish

Copy `_build/html/` into the `docs/` directory of `MuJoCable/mujocable.github.io`,
preserve its root `.nojekyll`, and commit to the Pages source branch. The site
is then served at `https://mujocable.github.io/docs/`. Do not replace the
academic homepage or its `/assets` directory. The HTML is independent of a
running Python server.

## Maintain

- Update tutorial claims against `plugin/unilateral_cable.cc` and route sources.
- `build_assets.py` checks that every registered configuration key is documented.
- `check_site.py` checks local links, ZIP integrity and private paths in HTML.
- Keep archived videos separate from newly measured numerical evidence.
- Re-run the simulation tests when editing XML examples or executable snippets.
- Run `_tests/verify_minimal.py --plugin LIBRARY` and, with this directory on
  `PYTHONPATH`, `_tests/verify_bundles.py --plugin LIBRARY` to load the actual
  downloadable models in an independent extraction folder.
- Third-party media/model permissions are recorded in examples' asset notices.

The CI workflow uploads a checked documentation artifact. Publishing to the
separate Pages repository remains a deliberate maintainer step.

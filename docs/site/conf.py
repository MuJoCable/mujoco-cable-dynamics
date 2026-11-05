project = "MuJoCable"
author = "MuJoCable contributors"
copyright = "2026, MuJoCable contributors"
release = "2026.11"
language = "zh_CN"
extensions = ["myst_parser", "sphinx_copybutton", "sphinxcontrib.katex"]
myst_enable_extensions = ["colon_fence", "dollarmath", "deflist"]
myst_heading_anchors = 3
exclude_patterns = ["_build", "_includes/**", "downloads/**", "README.md", "requirements.txt", "build_assets.py", "check_site.py"]
html_theme = "furo"
html_title = "MuJoCable Documentation"
html_static_path = ["_static"]
html_css_files = ["manual.css"]
html_js_files = ["manual.js"]
html_favicon = "_static/favicon.jpg"
html_theme_options = {
    "light_css_variables": {"color-brand-primary": "#177348", "color-brand-content": "#177348", "font-stack": "-apple-system,BlinkMacSystemFont,'Segoe UI','Noto Sans CJK SC','Microsoft YaHei',sans-serif"},
    "dark_css_variables": {"color-brand-primary": "#80cba1", "color-brand-content": "#80cba1"},
    "sidebar_hide_name": False,
}
html_show_sourcelink = False
html_copy_source = False
html_baseurl = "https://mujocable.github.io/docs/"
html_last_updated_fmt = None
copybutton_prompt_text = r"\$ |>>> |\.\.\. "
copybutton_prompt_is_regexp = True
copybutton_copy_empty_lines = False
katex_css_path = "katex/katex.min.css"
nitpicky = True

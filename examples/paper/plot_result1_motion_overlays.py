#!/usr/bin/env python3
"""Assemble the Result 1 motion overlays as an IEEE single-column figure."""

from __future__ import annotations

import argparse
import base64
import io
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas as pdfcanvas


WIDTH_PX = 2100
FIGURE_WIDTH_IN = 3.5
DPI = 600
MARGIN_PX = 36
GAP_PX = 24
TOP_HEIGHT_PX = 930
BOTTOM_HEIGHT_PX = 744
HEIGHT_PX = MARGIN_PX + TOP_HEIGHT_PX + GAP_PX + BOTTOM_HEIGHT_PX + MARGIN_PX
FONT_SIZE_PX = 84  # 10.1 pt at a 3.5-inch final width.
fontsize = 10.1
SVG_FONT_FAMILY = "Times New Roman, Times, Helvetica, serif"
# Direct SVG text and ReportLab PDF text remain editable; this is equivalent to
# the matplotlib policy svg.fonttype='none' and pdf.fonttype=42.
EDITABLE_VECTOR_POLICY = "svg.fonttype='none'; pdf.fonttype=42"
FONT_REGULAR = Path("/System/Library/Fonts/Supplemental/Times New Roman.ttf")
FONT_BOLD = Path("/System/Library/Fonts/Supplemental/Times New Roman Bold.ttf")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    return parser.parse_args()


def content_crop(image: Image.Image, threshold: int = 8, padding: int = 12) -> Image.Image:
    rgb = np.asarray(image.convert("RGB"))
    mask = np.max(rgb, axis=2) > threshold
    if not np.any(mask):
        return image.convert("RGB")
    ys, xs = np.where(mask)
    left = max(0, int(xs.min()) - padding)
    top = max(0, int(ys.min()) - padding)
    right = min(image.width, int(xs.max()) + padding + 1)
    bottom = min(image.height, int(ys.max()) + padding + 1)
    return image.convert("RGB").crop((left, top, right, bottom))


def fit_box(image: Image.Image, box: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    x, y, width, height = box
    scale = min(width / image.width, height / image.height)
    draw_width = int(round(image.width * scale))
    draw_height = int(round(image.height * scale))
    draw_x = x + (width - draw_width) // 2
    draw_y = y + (height - draw_height) // 2
    return draw_x, draw_y, draw_width, draw_height


def panel_layout() -> list[dict[str, object]]:
    top_width = (WIDTH_PX - 2 * MARGIN_PX - GAP_PX) // 2
    bottom_width = (WIDTH_PX - 2 * MARGIN_PX - 2 * GAP_PX) // 3
    top_y = MARGIN_PX
    bottom_y = MARGIN_PX + TOP_HEIGHT_PX + GAP_PX
    return [
        {"file": "FS00.png", "label": "(a) FS0, μ = 0", "rect": (MARGIN_PX, top_y, top_width, TOP_HEIGHT_PX)},
        {"file": "FS15.png", "label": "(b) FS15, μ = 0.15", "rect": (MARGIN_PX + top_width + GAP_PX, top_y, top_width, TOP_HEIGHT_PX)},
        {"file": "WS.png", "label": "(c) WS", "rect": (MARGIN_PX, bottom_y, bottom_width, BOTTOM_HEIGHT_PX)},
        {"file": "MP.png", "label": "(d) MP", "rect": (MARGIN_PX + bottom_width + GAP_PX, bottom_y, bottom_width, BOTTOM_HEIGHT_PX)},
        {"file": "AT.png", "label": "(e) AT", "rect": (MARGIN_PX + 2 * (bottom_width + GAP_PX), bottom_y, bottom_width, BOTTOM_HEIGHT_PX)},
    ]


def load_panels(input_dir: Path) -> list[dict[str, object]]:
    panels = panel_layout()
    for panel in panels:
        source = input_dir / str(panel["file"])
        if not source.exists():
            raise FileNotFoundError(source)
        image = content_crop(Image.open(source))
        x, y, width, height = panel["rect"]  # type: ignore[misc]
        image_box = (x + 20, y + FONT_SIZE_PX + 40, width - 40, height - FONT_SIZE_PX - 60)
        panel["image"] = image
        panel["image_rect"] = fit_box(image, image_box)
    return panels


def render_png(panels: list[dict[str, object]], output: Path) -> None:
    canvas = Image.new("RGB", (WIDTH_PX, HEIGHT_PX), "white")
    draw = ImageDraw.Draw(canvas)
    font = ImageFont.truetype(str(FONT_BOLD), FONT_SIZE_PX)
    for panel in panels:
        x, y, width, height = panel["rect"]  # type: ignore[misc]
        draw.rectangle((x, y, x + width, y + height), fill="black", outline=(120, 120, 120), width=2)
        image = panel["image"]  # type: ignore[assignment]
        image_x, image_y, image_width, image_height = panel["image_rect"]  # type: ignore[misc]
        resized = image.resize((image_width, image_height), Image.Resampling.LANCZOS)
        canvas.paste(resized, (image_x, image_y))
        draw.text((x + 28, y + 22), str(panel["label"]), font=font, fill="white")
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, dpi=(DPI, DPI), optimize=True)


def image_data_uri(image: Image.Image) -> str:
    buffer = io.BytesIO()
    image.save(buffer, format="PNG", optimize=True)
    return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def render_svg(panels: list[dict[str, object]], output: Path) -> None:
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" viewBox="0 0 {WIDTH_PX} {HEIGHT_PX}" width="{FIGURE_WIDTH_IN}in" height="{FIGURE_WIDTH_IN * HEIGHT_PX / WIDTH_PX:.4f}in">',
        f'<rect width="{WIDTH_PX}" height="{HEIGHT_PX}" fill="white"/>',
    ]
    for panel in panels:
        x, y, width, height = panel["rect"]  # type: ignore[misc]
        image_x, image_y, image_width, image_height = panel["image_rect"]  # type: ignore[misc]
        uri = image_data_uri(panel["image"])  # type: ignore[arg-type]
        label = escape(str(panel["label"]))
        parts.extend(
            [
                f'<rect x="{x}" y="{y}" width="{width}" height="{height}" fill="black" stroke="#787878" stroke-width="2"/>',
                f'<image x="{image_x}" y="{image_y}" width="{image_width}" height="{image_height}" preserveAspectRatio="none" xlink:href="{uri}"/>',
                f'<text x="{x + 28}" y="{y + 22 + int(0.82 * FONT_SIZE_PX)}" fill="white" font-family="{SVG_FONT_FAMILY}" font-size="{FONT_SIZE_PX}" font-weight="bold">{label}</text>',
            ]
        )
    parts.append("</svg>")
    output.write_text("\n".join(parts), encoding="utf-8")


def render_pdf(panels: list[dict[str, object]], output: Path) -> None:
    width_pt = FIGURE_WIDTH_IN * 72
    scale = width_pt / WIDTH_PX
    height_pt = HEIGHT_PX * scale
    pdfmetrics.registerFont(TTFont("TimesNewRomanBold", str(FONT_BOLD)))
    pdf = pdfcanvas.Canvas(str(output), pagesize=(width_pt, height_pt), pageCompression=1)
    for panel in panels:
        x, y, width, height = panel["rect"]  # type: ignore[misc]
        pdf.setFillColorRGB(0, 0, 0)
        pdf.setStrokeColorRGB(0.47, 0.47, 0.47)
        pdf.setLineWidth(2 * scale)
        pdf.rect(x * scale, (HEIGHT_PX - y - height) * scale, width * scale, height * scale, fill=1, stroke=1)
        image_x, image_y, image_width, image_height = panel["image_rect"]  # type: ignore[misc]
        pdf.drawImage(
            ImageReader(panel["image"]),  # type: ignore[arg-type]
            image_x * scale,
            (HEIGHT_PX - image_y - image_height) * scale,
            image_width * scale,
            image_height * scale,
            preserveAspectRatio=True,
            mask="auto",
        )
        pdf.setFillColorRGB(1, 1, 1)
        pdf.setFont("TimesNewRomanBold", FONT_SIZE_PX * scale)
        baseline_from_top = y + 22 + int(0.82 * FONT_SIZE_PX)
        pdf.drawString((x + 28) * scale, (HEIGHT_PX - baseline_from_top) * scale, str(panel["label"]))
    pdf.showPage()
    pdf.save()


def main() -> None:
    args = parse_args()
    if not FONT_BOLD.exists() or not FONT_REGULAR.exists():
        raise FileNotFoundError("Times New Roman fonts are unavailable")
    panels = load_panels(args.input_dir)
    prefix = args.output_prefix
    render_png(panels, prefix.with_suffix(".png"))
    render_svg(panels, prefix.with_suffix(".svg"))
    render_pdf(panels, prefix.with_suffix(".pdf"))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Regenerate sfm-gui logo/icon sizes from packaging/appimage/app.original.png (or app.png).

The design is a camera badge shooting a mountain peak. The mountain is stored as
near-black RGB with alpha in the source export; this script materializes it as
visible charcoal and emits UI + desktop icon sizes.
"""
from __future__ import annotations

import sys
from pathlib import Path

try:
    from PIL import Image, ImageDraw
    import numpy as np
except ImportError:
    print('Requires: pip install pillow numpy', file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parents[2]
SRC_CANDIDATES = [
    ROOT / 'packaging' / 'appimage' / 'app.original.png',
    ROOT / 'packaging' / 'appimage' / 'app.png',
]
ASSETS = ROOT / 'sfm-gui' / 'assets'
ICONS = ASSETS / 'icons'
SIZES = [16, 24, 32, 48, 64, 128, 256, 512]


def load_source() -> Image.Image:
    for path in SRC_CANDIDATES:
        if path.is_file():
            print(f'source: {path}')
            return Image.open(path).convert('RGBA')
    raise SystemExit('No packaging/appimage/app.png (or app.original.png) found')


def rebuild_logo(src: Image.Image) -> Image.Image:
    arr = np.array(src).astype(np.int16)
    rgb = arr[:, :, :3]
    a = arr[:, :, 3]
    lum = rgb.sum(axis=2)

    out = np.zeros_like(arr, dtype=np.uint8)
    is_ink = (a > 8) & (lum <= 30)  # mountain / dark geometry
    is_color = (a > 8) & (lum > 30)  # red/white camera badge

    mountain = np.array([55, 55, 60], dtype=np.uint8)
    out[is_ink, 0:3] = mountain
    out[is_ink, 3] = np.clip(a[is_ink].astype(np.int16) + 80, 180, 255).astype(np.uint8)
    out[is_color] = arr[is_color].astype(np.uint8)

    master = Image.fromarray(out, 'RGBA')
    mask = out[:, :, 3] > 16
    ys, xs = np.where(mask)
    minx, miny, maxx, maxy = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    half = max(maxx - minx, maxy - miny) / 2 + 4
    box = (
        int(max(0, np.floor(cx - half))),
        int(max(0, np.floor(cy - half))),
        int(min(master.width, np.ceil(cx + half))),
        int(min(master.height, np.ceil(cy + half))),
    )
    cropped = master.crop(box)
    side = max(cropped.width, cropped.height)
    square = Image.new('RGBA', (side, side), (0, 0, 0, 0))
    square.paste(cropped, ((side - cropped.width) // 2, (side - cropped.height) // 2), cropped)
    return square


def make_badge(logo: Image.Image, size: int) -> Image.Image:
    badge = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(badge)
    radius = max(2, int(size * 0.18))
    draw.rounded_rectangle((0, 0, size - 1, size - 1), radius=radius, fill=(245, 247, 251, 255))
    pad = max(1, size // 12)
    inner = size - pad * 2
    scaled = logo.resize((inner, inner), Image.Resampling.LANCZOS)
    badge.paste(scaled, (pad, pad), scaled)
    return badge


def main() -> None:
    logo = rebuild_logo(load_source())
    ASSETS.mkdir(parents=True, exist_ok=True)
    ICONS.mkdir(parents=True, exist_ok=True)
    for old in ICONS.glob('*.png'):
        old.unlink()

    logo.save(ASSETS / 'logo-master.png')
    logo.resize((256, 256), Image.Resampling.LANCZOS).save(ASSETS / 'logo.png')
    logo.resize((128, 128), Image.Resampling.LANCZOS).save(ASSETS / 'logo-128.png')

    for size in SIZES:
        make_badge(logo, size).save(ICONS / f'{size}x{size}.png')
    make_badge(logo, 512).save(ASSETS / 'icon.png')
    print(f'wrote UI logos + {len(SIZES)} desktop icons under {ASSETS}')


if __name__ == '__main__':
    main()

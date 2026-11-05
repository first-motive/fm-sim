#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["opencv-python-headless==4.14.0.94", "pyyaml>=6"]
# ///
"""Draw the ChArUco board textures the catalog's scenarios name, and write them as PNG.

    uv run tools/make_charuco_textures.py

Each scenario object with a ``charuco`` block gets the texture
``fm_sim_scenarios.mujoco_world`` maps onto its top face: the board drawn edge to
edge, the image's top row along the object's +y side, as OpenCV's
``CharucoBoard.generateImage`` draws it. The OpenCV pin is the one fm-twin's
``board.py`` detects with, so the texture is that board, marker ids offset by
``first_marker_id``. Only the board's layout is in the image; its size in metres is
the object's. The PNGs are committed, so neither a build nor a run needs OpenCV;
run this again after changing a board, and the scenario tests check the result.
"""

import argparse
import pathlib
import sys

import cv2
import numpy as np
import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent
CATALOG_SCENARIOS = REPO / 'fm_sim_bringup' / 'catalog' / 'scenarios'
TEXTURES = REPO / 'fm_sim_scenarios' / 'fm_sim_scenarios' / 'textures'
# Pixels per square: a 5x5 marker in a 0.75 square is then about 13 px per bit, sharper
# than any sim camera resolves it.
SQUARE_PX = 120


def texture_name(board):
    """The texture file of a board; the generator names it the same way."""
    return f'charuco_ids{board["first_marker_id"]}.png'


def boards(catalog_dir):
    """Every distinct ``charuco`` block the scenarios in ``catalog_dir`` declare."""
    found = {}
    for path in sorted(catalog_dir.glob('*.yaml')):
        for obj in yaml.safe_load(path.read_text()).get('objects', []):
            if 'charuco' in obj:
                board = obj['charuco']
                found.setdefault(texture_name(board), board)
    return found


def draw(board):
    """The board as a greyscale image, one ``SQUARE_PX`` per square, with no margin."""
    columns, rows = board['squares']
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board['dictionary']))
    first = board['first_marker_id']
    markers = columns * rows // 2
    charuco = cv2.aruco.CharucoBoard(
        (columns, rows), 1.0, board['marker_fraction'], dictionary,
        np.arange(first, first + markers),
    )
    size = (columns * SQUARE_PX, rows * SQUARE_PX)
    return charuco.generateImage(size, marginSize=0, borderBits=1)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--out', type=pathlib.Path, default=TEXTURES)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    for name, board in boards(CATALOG_SCENARIOS).items():
        ok, png = cv2.imencode('.png', draw(board))
        if not ok:
            sys.exit(f'could not encode {name}')
        (args.out / name).write_bytes(png.tobytes())
        print(args.out / name)


if __name__ == '__main__':
    main()

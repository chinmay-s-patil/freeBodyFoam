#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Chinmay S. Patil
"""
setup_grids.py

Automates the generation of Coarse (Grid 3), Medium (Grid 2), and Fine (Grid 1)
grids for ASME V&V 20 Grid Convergence Index (GCI) analysis.
"""

import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_MESH_DIR = os.path.join(BASE_DIR, "..", "thinDisk2mm", "meshing")
CASE_DIR = os.path.join(BASE_DIR, "..", "thinDisk2mm", "case")


def main():
    print(f"Base dir: {BASE_DIR}")
    print(f"Source meshing templates: {SRC_MESH_DIR}")


if __name__ == "__main__":
    main()

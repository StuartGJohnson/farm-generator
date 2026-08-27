"""
generation/material.py

Material/traction-patch generation. Explicitly OUT OF SCOPE this phase
(see CLAUDE.md "Explicitly out of scope this phase" and "Generation
order": "[later] material patches"). The schema already supports it
(`MaterialPatch`, `FarmScene.material_patches`) -- this module is a
placeholder giving that eventual stage a home. `generation/orchestrator.
generate_farm()` does NOT call this yet.
"""

from __future__ import annotations

from farm_ir.schema import FarmScene


def run(scene: FarmScene, config, rng) -> FarmScene:
    """No-op this phase -- not wired into the orchestrator's DAG yet."""
    return scene

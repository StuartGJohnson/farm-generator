"""Expose Isaac Sim's USD/PhysX schema bindings without starting Kit."""

from __future__ import annotations

import os
from pathlib import Path
import sys


_READY = "FARM_GENERATOR_ISAAC_USD_READY"


def reexec_with_isaac_usd() -> None:
    """Re-execute Python with only Isaac's compatible USD libraries configured.

    Isaac Sim ships a USD build and compiled ``PhysxSchema`` module that must
    be used together. Merely importing ``isaacsim`` does not expose those
    paths, while starting ``SimulationApp`` needlessly initializes Kit and
    RTX. A re-exec is required because the dynamic loader reads its library
    search path when the process starts.
    """
    if os.environ.get(_READY) == "1":
        return

    python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    isaac_root = Path(sys.prefix) / "lib" / python_version / "site-packages" / "isaacsim"
    extension_cache = isaac_root / "extscache"
    usd_candidates = sorted(extension_cache.glob("omni.usd.libs-*"))
    physx_candidates = sorted(extension_cache.glob("omni.usd.schema.physx-*"))
    if not usd_candidates or not physx_candidates:
        raise RuntimeError(
            "Isaac Sim USD/PhysX extensions were not found under "
            f"{extension_cache}; activate the isaacsim61-cu13 environment"
        )

    usd_extension = usd_candidates[-1]
    physx_extension = physx_candidates[-1]
    project_root = Path(__file__).resolve().parents[1]

    def prepend(name: str, paths: list[Path]) -> None:
        existing = os.environ.get(name)
        values = [str(path) for path in paths]
        if existing:
            values.append(existing)
        os.environ[name] = os.pathsep.join(values)

    prepend("PYTHONPATH", [usd_extension, physx_extension, project_root])
    prepend(
        "LD_LIBRARY_PATH",
        [Path(sys.prefix) / "lib", usd_extension / "bin", physx_extension / "bin", isaac_root / "kit"],
    )
    prepend("PXR_PLUGINPATH_NAME", [physx_extension / "plugins" / "PhysxSchema" / "resources"])
    os.environ[_READY] = "1"
    os.execvpe(sys.executable, [sys.executable, *sys.argv], os.environ)


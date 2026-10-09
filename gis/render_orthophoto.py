"""Render a north-up orthographic RGB image with headless Isaac Sim.

Run this in the Isaac Sim environment, as a separate process from farm
generation because SimulationApp owns process lifetime and GPU state.
"""

from __future__ import annotations

import argparse
import math
import traceback
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("world", type=Path)
    parser.add_argument("image", type=Path)
    parser.add_argument("--bounds", type=float, nargs=4, required=True)
    parser.add_argument("--gsd", type=float, default=0.05)
    args = parser.parse_args()
    x0, y0, x1, y1 = args.bounds
    width = math.ceil((x1 - x0) / args.gsd)
    height = math.ceil((y1 - y0) / args.gsd)
    if width < 1 or height < 1 or width > 8192 or height > 8192:
        raise ValueError(f"Invalid orthophoto dimensions {width} x {height}")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "multi_gpu": False,
                         "width": width, "height": height})
    try:
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, UsdGeom

        if not omni.usd.get_context().open_stage(str(args.world.resolve())):
            raise RuntimeError(f"Could not open USD stage {args.world}")
        for _ in range(12):
            app.update()
        stage = omni.usd.get_context().get_stage()
        camera = UsdGeom.Camera.Define(stage, "/World/OrthophotoCamera")
        camera.CreateProjectionAttr("orthographic")
        camera.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2, (y0 + y1) / 2, 100.0))
        # USD camera apertures are in tenths of stage units for orthographic
        # projection. The default orientation views along -Z with +Y up.
        camera.CreateHorizontalApertureAttr((x1 - x0) * 10)
        camera.CreateVerticalApertureAttr((y1 - y0) * 10)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 500.0))
        product = rep.create.render_product(str(camera.GetPath()), (width, height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product])
        for _ in range(12):
            rep.orchestrator.step(rt_subframes=4, pause_timeline=True, delta_time=0.0)
        pixels = np.asarray(annotator.get_data())
        if pixels.shape[:2] != (height, width) or pixels[..., :3].std() < 2:
            raise RuntimeError(f"Invalid Isaac orthophoto render: {pixels.shape}")
        args.image.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(pixels[..., :3]).save(args.image)
        print(f"ORTHOPHOTO_RENDER_PASS {args.image} {width}x{height}", flush=True)
        annotator.detach([product])
        product.destroy()
    except BaseException:
        traceback.print_exc()
        app.close(exit_code=1)
        raise
    else:
        app.close()


if __name__ == "__main__":
    main()

"""Generate and render both liveries in a headless Isaac Sim studio scene."""

import argparse
from dataclasses import replace
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/tractor_default.yaml")
    parser.add_argument("--output-dir", default="debug_out/tractor/livery_preview")
    args = parser.parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "multi_gpu": False, "width": 1280, "height": 960})
    try:
        import numpy as np
        from PIL import Image
        import omni.usd
        import omni.replicator.core as rep
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade
        from tractor_generation.config import load_tractor_config
        from tractor_generation.usd import write_tractor_usda

        config = load_tractor_config(args.config)
        output = Path(args.output_dir).resolve()
        for livery in ("tiger_stripes", "cheetah_spots"):
            folder = output / livery
            asset = write_tractor_usda(replace(config, livery=livery), folder / "tractor.usda")
            omni.usd.get_context().open_stage(str(asset))
            for _ in range(10):
                app.update()
            stage = omni.usd.get_context().get_stage()
            # Save the studio separately; reusable tractor files contain no floor/lights.
            stage.GetRootLayer().Export(str(folder / "studio.usda"))
            omni.usd.get_context().open_stage(str(folder / "studio.usda"))
            stage = omni.usd.get_context().get_stage()
            ground = UsdGeom.Cube.Define(stage, "/Studio/Ground")
            ground.CreateSizeAttr(1)
            ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.06))
            ground.AddScaleOp().Set(Gf.Vec3f(200, 200, 0.1))
            mat = UsdShade.Material.Define(stage, "/Studio/GroundMaterial")
            shader = UsdShade.Shader.Define(stage, "/Studio/GroundMaterial/Surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.16, 0.19, 0.22))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(mat)
            dome = UsdLux.DomeLight.Define(stage, "/Studio/Fill")
            dome.CreateIntensityAttr(600)
            sun = UsdLux.DistantLight.Define(stage, "/Studio/Key")
            sun.CreateIntensityAttr(2200)
            sun.CreateAngleAttr(12)
            sun.AddRotateXYZOp().Set(Gf.Vec3f(25, -35, -30))
            scale = max(config.tractor_body_length / 3.5,
                        config.sensor_pod_height_above_ground / 2.3,
                        config.rear_track_width / 1.6)
            target = Gf.Vec3d(0, 0, config.sensor_pod_height_above_ground * 0.5)
            eye = target + Gf.Vec3d(5.4, -6.8, 4.0) * scale * 1.3
            camera = UsdGeom.Camera.Define(stage, "/Studio/Camera")
            camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())
            camera.CreateFocalLengthAttr(46)
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000))
            stage.GetRootLayer().Save()
            product = rep.create.render_product(str(camera.GetPath()), (1280, 960))
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([product])
            for _ in range(12):
                rep.orchestrator.step(rt_subframes=4, pause_timeline=True, delta_time=0.0)
            pixels = np.asarray(annotator.get_data())
            if pixels.shape[:2] != (960, 1280) or pixels[..., :3].std() < 2:
                raise RuntimeError(f"Invalid rendered image: {pixels.shape}")
            image_path = folder / "tractor.png"
            Image.fromarray(pixels).save(image_path)
            print(f"RENDER_PASS {livery}: {image_path}", flush=True)
            annotator.detach([product])
            product.destroy()
    except BaseException:
        # Fast shutdown exits the process, so print and preserve failures first.
        traceback.print_exc()
        app.close(exit_code=1)
        raise
    else:
        app.close()


if __name__ == "__main__":
    main()

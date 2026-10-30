"""Render the fence test scene from its authored camera in Isaac Sim 6.1."""

from pathlib import Path
import traceback


def main():
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "multi_gpu": False,
                         "width": 1600, "height": 900})
    try:
        import numpy as np
        from PIL import Image
        import omni.replicator.core as rep
        import omni.usd

        directory = Path(__file__).resolve().parent
        scene = directory / "fence_test_scene.usda"
        if not omni.usd.get_context().open_stage(str(scene)):
            raise RuntimeError(f"Could not open {scene}")
        for _ in range(12):
            app.update()
        product = rep.create.render_product("/World/InspectionCamera", (1600, 900))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product])
        for _ in range(12):
            rep.orchestrator.step(rt_subframes=4, pause_timeline=True,
                                  delta_time=0.0)
        pixels = np.asarray(annotator.get_data())
        if pixels.shape[:2] != (900, 1600) or pixels[..., :3].std() < 2:
            raise RuntimeError(f"Invalid render: {pixels.shape}")
        output = directory / "fence_preview.png"
        Image.fromarray(pixels[..., :3]).save(output)
        print(f"FENCE_RENDER_PASS {output}", flush=True)
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

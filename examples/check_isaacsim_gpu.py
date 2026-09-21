"""Headless SimulationApp and CUDA smoke check.

Run with host GPU access:
    conda run --no-capture-output -n isaacsim61-cu13 python examples/check_isaacsim_gpu.py
"""

import json

from isaacsim import SimulationApp


def main() -> None:
    app = SimulationApp({"headless": True, "multi_gpu": False})
    try:
        import torch
        import omni.usd
        from pxr import UsdGeom

        assert app.is_running(), "SimulationApp is not running"
        stage = omni.usd.get_context().get_stage()
        assert stage is not None, "No USD stage was created"
        UsdGeom.Cube.Define(stage, "/GpuSmokeCube")
        for _ in range(10):
            app.update()

        assert torch.cuda.is_available(), "CUDA is unavailable inside SimulationApp"
        values = torch.arange(1024, device="cuda", dtype=torch.float32)
        result = (values * values).sum().item()
        expected = sum(i * i for i in range(1024))
        assert result == expected, (result, expected)
        torch.cuda.synchronize()
        print("ISAACSIM_GPU_CHECK_PASS " + json.dumps({
            "headless": True,
            "app_updates": 10,
            "gpu": torch.cuda.get_device_name(0),
            "torch_cuda_version": torch.version.cuda,
            "cuda_sum_of_squares": result,
        }), flush=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()

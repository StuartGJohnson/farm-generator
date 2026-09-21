"""Minimal Isaac Sim startup/shutdown diagnostic.

This intentionally loads no farm or tractor assets. It isolates failures in
SimulationApp, renderer initialization, and shutdown from project code.
"""

from isaacsim import SimulationApp


def main() -> None:
    print("Starting SimulationApp (headless, minimal rendering)...", flush=True)
    app = SimulationApp(
        {
            "headless": True,
            "renderer": "MinimalRendering",
            "minimal_shading_mode": 4,
            "disable_viewport_updates": True,
            "multi_gpu": False,
            "sync_loads": False,
        }
    )
    print("SimulationApp started", flush=True)
    print("Calling SimulationApp.close(); fast shutdown terminates this Python process", flush=True)
    app.close()


if __name__ == "__main__":
    main()

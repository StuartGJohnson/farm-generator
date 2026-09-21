"""Fast plan/side-view tractor diagnostics independent of Isaac rendering."""

from pathlib import Path

from .config import TractorConfig


def save_tractor_diagnostic(config: TractorConfig, path: str | Path) -> Path:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    output = Path(path); output.parent.mkdir(parents=True, exist_ok=True)
    fig, (side, plan) = plt.subplots(1, 2, figsize=(13, 5), constrained_layout=True)
    side.add_patch(Rectangle((-config.tractor_body_length/2, config.tractor_body_ground_clearance),
                             config.tractor_body_length, config.tractor_body_height,
                             facecolor="#176b2c", edgecolor="black"))
    for x, r in ((config.front_axle_x, config.front_tire_dia/2), (config.rear_axle_x, config.rear_tire_dia/2)):
        side.add_patch(Circle((x, r), r, facecolor="#202020", edgecolor="black"))
    side.add_patch(Rectangle((-config.sensor_pod_length/2, config.sensor_pod_height_above_ground-config.sensor_pod_height/2),
                             config.sensor_pod_length, config.sensor_pod_height, facecolor="#303830"))
    side.set_title("side (+X forward)"); side.set_xlabel("longitudinal [m]"); side.set_ylabel("height [m]")
    for center_x, length, width in config.body_segments:
        plan.add_patch(Rectangle((center_x-length/2, -width/2), length, width,
                                 facecolor="#176b2c", edgecolor="black"))
    for x, track, width, dia in ((config.front_axle_x, config.front_track_width, config.front_tire_width, config.front_tire_dia),
                                  (config.rear_axle_x, config.rear_track_width, config.rear_tire_width, config.rear_tire_dia)):
        for y in (-track/2, track/2):
            plan.add_patch(Rectangle((x-dia/2, y-width/2), dia, width, facecolor="#202020"))
    plan.set_title("plan (+Y left)"); plan.set_xlabel("longitudinal [m]"); plan.set_ylabel("lateral [m]")
    for axes in (side, plan):
        axes.autoscale(); axes.set_aspect("equal"); axes.grid(alpha=.2)
    fig.suptitle(f"tractor diagnostic — {config.livery}, seed {config.random_seed}")
    fig.savefig(output, dpi=180); plt.close(fig)
    return output

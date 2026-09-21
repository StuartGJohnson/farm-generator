"""Programmatic Isaac Sim 6.1 OmniGraphs for tractor sensors."""

from __future__ import annotations

from tractor_generation.config import TractorConfig
from tractor_generation.names import camera_link


def create_ros2_camera_graphs(
    config: TractorConfig,
    tractor_path: str = "/World/Tractor",
    graph_path: str = "/World/TractorROS2Cameras",
) -> None:
    """Publish RGB images and camera info for every enabled camera.

    Call after ``SimulationApp`` has started and the tractor is on the stage.
    The graph is deliberately authored by Python rather than baked into the
    reusable tractor asset, keeping ROS 2 an optional bring-up concern.
    """
    import omni.graph.core as og
    from pxr import Sdf

    create_nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
    set_values = []
    connections = []
    width, height = config.camera_resolution

    for name in config.enabled_cameras:
        title = name.title()
        render = f"Create{title}RenderProduct"
        rgb = f"Publish{title}RGB"
        info = f"Publish{title}CameraInfo"
        camera_path = f"{tractor_path}/base_link/{camera_link(name)}/camera"
        create_nodes.extend(
            [
                (render, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                (rgb, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ]
        )
        set_values.extend(
            [
                (f"{render}.inputs:cameraPrim", [Sdf.Path(camera_path)]),
                (f"{render}.inputs:width", width),
                (f"{render}.inputs:height", height),
                (f"{rgb}.inputs:type", "rgb"),
                (f"{rgb}.inputs:topicName", f"tractor/{name}/image_rgb"),
                (f"{rgb}.inputs:frameId", camera_link(name)),
                (f"{info}.inputs:topicName", f"tractor/{name}/camera_info"),
                (f"{info}.inputs:frameId", camera_link(name)),
            ]
        )
        connections.append(
            (f"OnPlaybackTick.outputs:tick", f"{render}.inputs:execIn")
        )
        for publisher in (rgb, info):
            connections.extend(
                [
                    (f"{render}.outputs:execOut", f"{publisher}.inputs:execIn"),
                    (
                        f"{render}.outputs:renderProductPath",
                        f"{publisher}.inputs:renderProductPath",
                    ),
                ]
            )

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.SET_VALUES: set_values,
            og.Controller.Keys.CONNECT: connections,
        },
    )

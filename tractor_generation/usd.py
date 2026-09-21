"""Isaac Sim 6.1 / PhysX Vehicle 2 tractor USDA generation.

The caller must expose Isaac Sim's bundled USD and ``PhysxSchema`` libraries.
``examples/generate_tractor.py`` does this without starting ``SimulationApp``.
"""

from __future__ import annotations

import math
from pathlib import Path

from .config import TractorConfig
from .names import BASE_LINK, SENSOR_POD_LINK, WHEELS, camera_link

DENSITY = 1000.0


def write_tractor_usda(config: TractorConfig, path: str | Path) -> Path:
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    def add_collision_to_collision_group(stage, collision_path, group_path):
        """Add a collider without depending on a running Kit application."""
        group = stage.GetPrimAtPath(group_path)
        includes = Usd.CollectionAPI.Get(group, "colliders").GetIncludesRel()
        includes.AddTarget(collision_path)

    output = Path(path).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, "/Tractor")
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("farmGenerator:randomSeed", config.random_seed)
    root.GetPrim().SetCustomDataByKey("farmGenerator:livery", config.livery)

    physics_scene = UsdPhysics.Scene.Define(stage, "/Tractor/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    context = PhysxSchema.PhysxVehicleContextAPI.Apply(physics_scene.GetPrim())
    context.CreateUpdateModeAttr(PhysxSchema.Tokens.velocityChange)
    context.CreateVerticalAxisAttr(PhysxSchema.Tokens.posZ)
    context.CreateLongitudinalAxisAttr(PhysxSchema.Tokens.posX)

    def material(name, color, metallic=0.0, roughness=0.5):
        mat = UsdShade.Material.Define(stage, f"/Tractor/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"/Tractor/Looks/{name}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    body_mat = material("Body", (0.07, 0.32, 0.08), 0.15, 0.32)
    accent_mat = material("Accent", (0.98, 0.55, 0.03), 0.05, 0.38)
    black_mat = material("Rubber", (0.018, 0.022, 0.018), 0.0, 0.82)
    rim_mat = material("Rim", (0.68, 0.72, 0.70), 0.75, 0.22)
    lens_mat = material("Lens", (0.04, 0.16, 0.24), 0.25, 0.08)

    chassis_group_path = "/Tractor/Physics/ChassisCollisionGroup"
    wheel_group_path = "/Tractor/Physics/WheelCollisionGroup"
    query_group_path = "/Tractor/Physics/GroundQueryCollisionGroup"
    chassis_group = UsdPhysics.CollisionGroup.Define(stage, chassis_group_path)
    wheel_group = UsdPhysics.CollisionGroup.Define(stage, wheel_group_path)
    query_group = UsdPhysics.CollisionGroup.Define(stage, query_group_path)
    chassis_group.CreateFilteredGroupsRel().AddTarget(query_group_path)
    wheel_group.CreateFilteredGroupsRel().AddTarget(query_group_path)
    query_group.CreateFilteredGroupsRel().AddTarget(chassis_group_path)
    query_group.CreateFilteredGroupsRel().AddTarget(wheel_group_path)

    grass = UsdShade.Material.Define(stage, "/Tractor/Physics/DefaultGroundMaterial")
    grass_physics = UsdPhysics.MaterialAPI.Apply(grass.GetPrim())
    grass_physics.CreateStaticFrictionAttr(0.8)
    grass_physics.CreateDynamicFrictionAttr(0.7)
    grass_physics.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(grass.GetPrim())
    friction_table = PhysxSchema.PhysxVehicleTireFrictionTable.Define(stage, "/Tractor/Physics/TractorTireFriction")
    friction_table.CreateGroundMaterialsRel().AddTarget(grass.GetPath())
    friction_table.CreateFrictionValuesAttr([0.85])

    vehicle_path = f"/Tractor/{BASE_LINK}"
    vehicle = UsdGeom.Xform.Define(stage, vehicle_path)
    vehicle.AddTranslateOp().Set(Gf.Vec3d(0, 0, config.body_center_z))
    vehicle_prim = vehicle.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(vehicle_prim)
    # Vehicle 2 wheel/suspension frames below are expressed in the rigid-body
    # frame, not relative to the computed center of mass. Without this flag,
    # PhysX currently assumes the deprecated COM convention.
    vehicle_prim.SetCustomDataByKey(
        PhysxSchema.Tokens.referenceFrameIsCenterOfMass, False
    )
    pod_local_z = config.sensor_pod_height_above_ground - config.body_center_z
    components = [
        (
            DENSITY * length * width * config.tractor_body_height,
            (length, width, config.tractor_body_height),
            (center_x, 0.0, 0.0),
        )
        for center_x, length, width in config.body_segments
    ]
    components.append(
        (
            DENSITY * config.sensor_pod_length * config.sensor_pod_width * config.sensor_pod_height,
            (config.sensor_pod_length, config.sensor_pod_width, config.sensor_pod_height),
            (0.0, 0.0, pod_local_z),
        )
    )
    chassis_mass = sum(mass for mass, _, _ in components)
    center_of_mass = tuple(
        sum(mass * center[axis] for mass, _, center in components) / chassis_mass
        for axis in range(3)
    )
    inertia = [0.0, 0.0, 0.0]
    for mass, (length, width, height), center in components:
        dx, dy, dz = (center[i] - center_of_mass[i] for i in range(3))
        inertia[0] += mass * (width * width + height * height) / 12 + mass * (dy * dy + dz * dz)
        inertia[1] += mass * (length * length + height * height) / 12 + mass * (dx * dx + dz * dz)
        inertia[2] += mass * (length * length + width * width) / 12 + mass * (dx * dx + dy * dy)
    mass_api = UsdPhysics.MassAPI.Apply(vehicle_prim)
    mass_api.CreateMassAttr(chassis_mass)
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*center_of_mass))
    x, y, z = config.tractor_body_length, config.tractor_body_width, config.tractor_body_height
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    PhysxSchema.PhysxRigidBodyAPI.Apply(vehicle_prim).CreateDisableGravityAttr(True)

    vehicle_api = PhysxSchema.PhysxVehicleAPI.Apply(vehicle_prim)
    vehicle_api.CreateVehicleEnabledAttr(True)
    vehicle_api.CreateSubStepThresholdLongitudinalSpeedAttr(4.0)
    vehicle_api.CreateLowForwardSpeedSubStepCountAttr(5)
    vehicle_api.CreateHighForwardSpeedSubStepCountAttr(2)
    vehicle_api.CreateMinPassiveLongitudinalSlipDenominatorAttr(4.0)
    vehicle_api.CreateMinActiveLongitudinalSlipDenominatorAttr(0.1)
    vehicle_api.CreateMinLateralSlipDenominatorAttr(1.0)

    brakes = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehicle_prim, PhysxSchema.Tokens.brakes0)
    brakes.CreateMaxBrakeTorqueAttr(12000.0)
    handbrake = PhysxSchema.PhysxVehicleBrakesAPI.Apply(vehicle_prim, PhysxSchema.Tokens.brakes1)
    handbrake.CreateWheelsAttr([2, 3]); handbrake.CreateMaxBrakeTorqueAttr(14000.0)
    steering = PhysxSchema.PhysxVehicleAckermannSteeringAPI.Apply(vehicle_prim)
    steering.CreateWheel0Attr(1)  # right
    steering.CreateWheel1Attr(0)  # left
    steering.CreateMaxSteerAngleAttr(config.max_steer_angle_rad)
    steering.CreateWheelBaseAttr(config.wheelbase)
    steering.CreateTrackWidthAttr(config.front_track_width)
    steering.CreateStrengthAttr(1.0)
    differential = PhysxSchema.PhysxVehicleMultiWheelDifferentialAPI.Apply(vehicle_prim)
    differential.CreateWheelsAttr([2, 3])
    differential.CreateTorqueRatiosAttr([0.5, 0.5])
    drive = PhysxSchema.PhysxVehicleDriveBasicAPI.Apply(vehicle_prim)
    drive.CreatePeakTorqueAttr(9000.0)
    controller = PhysxSchema.PhysxVehicleControllerAPI.Apply(vehicle_prim)
    controller.CreateAcceleratorAttr(0.0); controller.CreateBrake0Attr(0.0)
    controller.CreateBrake1Attr(0.0); controller.CreateSteerAttr(0.0); controller.CreateTargetGearAttr(1)

    UsdGeom.Xform.Define(stage, f"{vehicle_path}/Body")
    for index, (center_x, length, width) in enumerate(config.body_segments):
        body = UsdGeom.Cube.Define(stage, f"{vehicle_path}/Body/segment_{index}")
        body.CreateSizeAttr(1.0)
        body.AddTranslateOp().Set(Gf.Vec3f(center_x, 0, 0))
        body.AddScaleOp().Set(Gf.Vec3f(length, width, z))
        UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(body_mat)
        UsdPhysics.CollisionAPI.Apply(body.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(body.GetPrim()).CreateContactOffsetAttr(0.02)
        add_collision_to_collision_group(stage, str(body.GetPath()), chassis_group_path)

    # Deterministic, simple livery geometry avoids external texture assets.
    if config.livery == "tiger_stripes":
        offsets = (-1.1, -0.45, 0.25, 0.95)
        widths = (0.16, 0.11, 0.15, 0.10)
        for index, (offset, width) in enumerate(zip(offsets, widths)):
            stripe_width = next(
                segment_width for center, length, segment_width in config.body_segments
                if center - length / 2 - 1e-9 <= offset <= center + length / 2 + 1e-9
            )
            stripe = UsdGeom.Cube.Define(stage, f"{vehicle_path}/Livery/stripe_{index}")
            stripe.CreateSizeAttr(1.0); stripe.AddTranslateOp().Set(Gf.Vec3f(offset, 0, z*0.505))
            stripe.AddScaleOp().Set(Gf.Vec3f(width, stripe_width*1.01, 0.02))
            UsdShade.MaterialBindingAPI.Apply(stripe.GetPrim()).Bind(accent_mat)
    else:
        for index, (sx, sy) in enumerate(((-1.0, .3), (-.35, -.35), (.25, .32), (.9, -.25))):
            spot = UsdGeom.Sphere.Define(stage, f"{vehicle_path}/Livery/spot_{index}")
            spot.CreateRadiusAttr(0.18); spot.AddTranslateOp().Set(Gf.Vec3f(sx, sy, z*.52))
            spot.AddScaleOp().Set(Gf.Vec3f(1.5, 1.0, .15))
            UsdShade.MaterialBindingAPI.Apply(spot.GetPrim()).Bind(accent_mat)

    pod = UsdGeom.Cube.Define(stage, f"{vehicle_path}/{SENSOR_POD_LINK}")
    pod.CreateSizeAttr(1.0); pod.AddTranslateOp().Set(Gf.Vec3f(0, 0, pod_local_z))
    pod.AddScaleOp().Set(Gf.Vec3f(config.sensor_pod_length, config.sensor_pod_width, config.sensor_pod_height))
    UsdShade.MaterialBindingAPI.Apply(pod.GetPrim()).Bind(black_mat)

    wheel_specs = (
        (config.front_axle_x, config.front_track_width/2, config.front_tire_dia/2, config.front_tire_width, config.front_tire_depth),
        (config.front_axle_x, -config.front_track_width/2, config.front_tire_dia/2, config.front_tire_width, config.front_tire_depth),
        (config.rear_axle_x, config.rear_track_width/2, config.rear_tire_dia/2, config.rear_tire_width, config.rear_tire_depth),
        (config.rear_axle_x, -config.rear_track_width/2, config.rear_tire_dia/2, config.rear_tire_width, config.rear_tire_depth),
    )
    spring_strength = (chassis_mass / 4 * 9.81) / 0.12
    suspension_travel = 0.28
    suspension_rest_length = suspension_travel - 0.12
    for index, (wx, wy, radius, width, tread_depth) in enumerate(wheel_specs):
        local_z = radius - config.body_center_z
        wheel_path = f"{vehicle_path}/{WHEELS[index]}"
        wheel_xform = UsdGeom.Xform.Define(stage, wheel_path)
        wheel_xform.AddTranslateOp().Set(Gf.Vec3f(wx, wy, local_z))
        # Match the zero-steer rest basis that Vehicle 2 writes when it takes
        # control.  The render children's local X axle then maps onto vehicle
        # Y both while stopped and while simulation is running.
        half_sqrt = math.sqrt(0.5)
        wheel_xform.AddOrientOp().Set(Gf.Quatf(half_sqrt, 0, 0, half_sqrt))
        # PhysX interprets and manages the direct collision child specially.
        # Its cylinder follows the Vehicle 2 side-axis convention (Y).  The
        # ordinary render children inherit the resulting managed-root pose,
        # whose local X maps to the visible axle in this configuration.
        attachment = PhysxSchema.PhysxVehicleWheelAttachmentAPI.Apply(wheel_xform.GetPrim())
        attachment.CreateCollisionGroupRel().AddTarget(query_group_path)
        attachment.CreateSuspensionTravelDirectionAttr(Gf.Vec3f(0, 0, -1))
        attachment.CreateSuspensionFramePositionAttr(Gf.Vec3f(wx, wy, local_z + suspension_rest_length))
        attachment.CreateSuspensionFrameOrientationAttr(Gf.Quatf(1, 0, 0, 0))
        attachment.CreateIndexAttr(index)
        wheel_mass = DENSITY * math.pi * radius * radius * width
        wheel_api = PhysxSchema.PhysxVehicleWheelAPI.Apply(wheel_xform.GetPrim())
        wheel_api.CreateRadiusAttr(radius); wheel_api.CreateWidthAttr(width)
        wheel_api.CreateMassAttr(wheel_mass); wheel_api.CreateMoiAttr(0.5 * wheel_mass * radius * radius)
        wheel_api.CreateDampingRateAttr(2.0)
        tire = PhysxSchema.PhysxVehicleTireAPI.Apply(wheel_xform.GetPrim())
        rest_load = chassis_mass * 9.81 / 4
        tire.CreateLateralStiffnessGraphAttr(Gf.Vec2f(2.0, 18.0 * rest_load))
        tire.CreateLongitudinalStiffnessAttr(6500.0)
        tire.CreateCamberStiffnessAttr(0.0)
        tire.CreateFrictionVsSlipGraphAttr([Gf.Vec2f(0, 1), Gf.Vec2f(.15, 1), Gf.Vec2f(1, .85)])
        tire.CreateFrictionTableRel().AddTarget(friction_table.GetPath())
        suspension = PhysxSchema.PhysxVehicleSuspensionAPI.Apply(wheel_xform.GetPrim())
        suspension.CreateSpringStrengthAttr(spring_strength)
        suspension.CreateSpringDamperRateAttr(2.0 * math.sqrt(spring_strength * chassis_mass / 4) * 0.65)
        suspension.CreateTravelDistanceAttr(suspension_travel)
        collision = UsdGeom.Cylinder.Define(stage, f"{wheel_path}/Collision")
        collision.CreatePurposeAttr(UsdGeom.Tokens.guide); collision.CreateAxisAttr(UsdGeom.Tokens.y)
        collision.CreateHeightAttr(width); collision.CreateRadiusAttr(radius)
        collision.CreateExtentAttr(UsdGeom.Cylinder.ComputeExtentFromPlugins(collision, 0))
        UsdPhysics.CollisionAPI.Apply(collision.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(collision.GetPrim()).CreateContactOffsetAttr(0.02)
        add_collision_to_collision_group(stage, str(collision.GetPath()), wheel_group_path)
        render = UsdGeom.Cylinder.Define(stage, f"{wheel_path}/Render")
        render.CreateAxisAttr(UsdGeom.Tokens.x); render.CreateHeightAttr(width); render.CreateRadiusAttr(radius)
        render.CreateExtentAttr(UsdGeom.Cylinder.ComputeExtentFromPlugins(render, 0))
        UsdShade.MaterialBindingAPI.Apply(render.GetPrim()).Bind(black_mat)
        rim = UsdGeom.Cylinder.Define(stage, f"{wheel_path}/Rim")
        rim.CreateAxisAttr(UsdGeom.Tokens.x); rim.CreateHeightAttr(width*1.03); rim.CreateRadiusAttr(max(.05, radius-tread_depth))
        rim.CreateExtentAttr(UsdGeom.Cylinder.ComputeExtentFromPlugins(rim, 0))
        UsdShade.MaterialBindingAPI.Apply(rim.GetPrim()).Bind(rim_mat)

    camera_positions = {
        "front": (config.sensor_pod_length/2, 0, pod_local_z),
        "rear": (-config.sensor_pod_length/2, 0, pod_local_z),
        "left": (0, config.sensor_pod_width/2, pod_local_z),
        "right": (0, -config.sensor_pod_width/2, pod_local_z),
    }
    directions = {
        "front": Gf.Vec3d(math.cos(math.radians(config.camera_pitch)), 0, -math.sin(math.radians(config.camera_pitch))),
        "rear": Gf.Vec3d(-math.cos(math.radians(config.camera_pitch)), 0, -math.sin(math.radians(config.camera_pitch))),
        "left": Gf.Vec3d(0, math.cos(math.radians(config.camera_pitch)), -math.sin(math.radians(config.camera_pitch))),
        "right": Gf.Vec3d(0, -math.cos(math.radians(config.camera_pitch)), -math.sin(math.radians(config.camera_pitch))),
    }
    aperture = 20.955
    focal_length = aperture / (2 * math.tan(math.radians(config.camera_hfov) / 2))
    for name in config.enabled_cameras:
        link_path = f"{vehicle_path}/{camera_link(name)}"
        link = UsdGeom.Xform.Define(stage, link_path)
        link.AddTranslateOp().Set(Gf.Vec3d(*camera_positions[name]))
        # A USD camera looks down local -Z with local +Y as image-up.  Aligning
        # only -Z leaves camera roll underdetermined (and produced a 90-degree
        # rolled image).  First aim the optical axis, then roll its current +Y
        # onto world-up projected perpendicular to the look direction.
        look = directions[name].GetNormalized()
        image_up = (Gf.Vec3d(0, 0, 1) - look * look[2]).GetNormalized()
        aim = Gf.Rotation(Gf.Vec3d(0, 0, -1), look)
        current_up = aim.TransformDir(Gf.Vec3d(0, 1, 0))
        roll = Gf.Rotation(current_up, image_up)
        orient = (aim * roll).GetQuat()
        link.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(orient.GetReal(), orient.GetImaginary())
        )
        camera = UsdGeom.Camera.Define(stage, f"{link_path}/camera")
        camera.CreateHorizontalApertureAttr(aperture); camera.CreateFocalLengthAttr(focal_length)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 1000.0))
        camera.GetPrim().CreateAttribute(
            "farmGenerator:resolution", Sdf.ValueTypeNames.Int2, custom=True
        ).Set(Gf.Vec2i(*config.camera_resolution))
        lens = UsdGeom.Cylinder.Define(stage, f"{link_path}/lens")
        lens.CreateAxisAttr(UsdGeom.Tokens.z); lens.CreateRadiusAttr(0.055); lens.CreateHeightAttr(0.015)
        lens.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.0075))
        UsdShade.MaterialBindingAPI.Apply(lens.GetPrim()).Bind(lens_mat)

    stage.GetRootLayer().Save()
    return output

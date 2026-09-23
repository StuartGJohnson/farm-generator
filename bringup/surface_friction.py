"""Connect a composed vehicle to the world's surface-material friction table."""


def configure_vehicle_surface_friction(stage, tractor_path="/World/Tractor", world_path="/World"):
    from pxr import PhysxSchema, Usd, UsdPhysics

    table_path = world_path + "/TireFrictionTable"
    table = PhysxSchema.PhysxVehicleTireFrictionTable.Get(stage, table_path)
    if not table:
        return 0  # Legacy worlds retain the tractor's standalone fallback.
    materials = table.GetGroundMaterialsRel().GetTargets()
    values = table.GetFrictionValuesAttr().Get()
    if not materials or values is None or len(materials) != len(values):
        raise ValueError(f"Invalid ground-material tire table: {table_path}")
    for path in materials:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.HasAPI(UsdPhysics.MaterialAPI):
            raise ValueError(f"Tire-table target is not a physics material: {path}")
    count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(tractor_path)):
        if prim.HasAPI(PhysxSchema.PhysxVehicleTireAPI):
            PhysxSchema.PhysxVehicleTireAPI(prim).CreateFrictionTableRel().SetTargets([table.GetPath()])
            count += 1
    # Vehicle suspension queries supply tire forces. Avoid a second source of
    # traction from wheel-cylinder rigid contacts; chassis contacts remain on.
    ground_group = UsdPhysics.CollisionGroup.Define(stage, world_path + "/GroundCollisionGroup")
    Usd.CollectionAPI.Get(ground_group.GetPrim(), "colliders").GetIncludesRel().AddTarget(world_path + "/Ground")
    wheel_group_path = tractor_path + "/Physics/WheelCollisionGroup"
    wheel_group = UsdPhysics.CollisionGroup.Get(stage, wheel_group_path)
    if wheel_group:
        ground_group.CreateFilteredGroupsRel().AddTarget(wheel_group_path)
        wheel_group.CreateFilteredGroupsRel().AddTarget(ground_group.GetPath())
    return count

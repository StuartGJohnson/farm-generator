"""Quantize upstream friction samples into USD physics-material face subsets."""

from dataclasses import dataclass
import numpy as np

from generation.surface_friction import face_surface_regions


@dataclass(frozen=True)
class UsdFrictionConfig:
    n_sample_fields: int = 10

    def __post_init__(self):
        if isinstance(self.n_sample_fields, bool) or not isinstance(self.n_sample_fields, int) or self.n_sample_fields < 1:
            raise ValueError("n_sample_fields must be a positive integer")


def quantize_friction(mesh, config=UsdFrictionConfig()):
    """Midpoint bins: delta_mu = full range/N, max error delta_mu/2."""
    fields = mesh.surface_friction
    if fields is None or mesh.face_friction is None:
        raise ValueError("ground mesh has no upstream surface friction samples")
    regions = np.asarray(mesh.face_surface_regions if mesh.face_surface_regions is not None else
                         face_surface_regions(mesh.points, mesh.triangles, mesh.face_classes))
    samples = np.asarray(mesh.face_friction)
    bins, face_bins = [], np.full(len(samples), -1, dtype=int)
    for region in ("crop", "road", "channel"):
        if region == "channel":
            low, span = fields.channel_friction, 0.0
        else:
            field = getattr(fields, region)
            low = field.friction_min
            span = field.friction_max - low
        count = config.n_sample_fields if span > 0 else 1
        step = span/count
        mask = regions == region
        assignments = np.clip(np.floor((samples[mask]-low)/step), 0, count-1).astype(int) if step else np.zeros(mask.sum(), dtype=int)
        start = len(bins)
        face_bins[mask] = start + assignments
        for i in range(count):
            bins.append((f"{region.capitalize()}_{i:02d}", region, low+(i+0.5)*step))
    assert np.all(face_bins >= 0)
    return bins, face_bins


def friction_usda(mesh, config=UsdFrictionConfig()):
    bins, assignments = quantize_friction(mesh, config)
    materials, subsets = [], []
    visuals = {"crop": "GrassMaterial", "road": "GravelRoadMaterial", "channel": "ChannelMaterial"}
    for i, (name, region, mu) in enumerate(bins):
        materials.append(f'''    def Material "{name}" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {{
        float physics:staticFriction = {mu:.9g}
        float physics:dynamicFriction = {mu:.9g}
        float physics:restitution = 0
    }}''')
        faces = ", ".join(map(str, np.flatnonzero(assignments == i)))
        if faces:
            subsets.append(f'''def GeomSubset "Friction_{name}" (
    prepend apiSchemas = ["MaterialBindingAPI"]
)
{{
    uniform token elementType = "face"
    uniform token familyName = "materialBind"
    int[] indices = [{faces}]
    rel material:binding = </World/Looks/{visuals[region]}>
    rel material:binding:physics = </World/PhysicsMaterials/{name}>
}}''')
    targets = ", ".join(f"</World/PhysicsMaterials/{name}>" for name, _, _ in bins)
    values = ", ".join(f"{mu:.9g}" for _, _, mu in bins)
    physics = 'def Scope "PhysicsMaterials"\n{\n' + "\n\n".join(materials) + '\n}\n\n'
    physics += f'''def PhysxVehicleTireFrictionTable "TireFrictionTable"
{{
    rel groundMaterials = [{targets}]
    float[] frictionValues = [{values}]
    float defaultFrictionValue = 1
}}
'''
    samples = ", ".join(f"{v:.9g}" for v in mesh.face_friction)
    quantized = ", ".join(f"{bins[i][2]:.9g}" for i in assignments)
    metadata = f'''uniform token subsetFamily:materialBind:familyType = "partition"
float[] primvars:surfaceFriction = [{samples}] (interpolation = "uniform")
float[] primvars:quantizedSurfaceFriction = [{quantized}] (interpolation = "uniform")'''
    return physics, "\n\n".join(subsets), metadata

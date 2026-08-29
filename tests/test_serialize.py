"""
tests/test_serialize.py

Round-trip tests for farm_ir/serialize.py -- the human-readable YAML/JSON
IR format that debug tooling writes to debug_out/, and that USD/Gazebo
exporters will eventually read as input (see CLAUDE.md "Purpose").
"""

from farm_ir.schema import FarmScene
from farm_ir.serialize import from_plain, read, to_plain, write
from generation.orchestrator import FarmGenerationConfig, generate_farm, load_farm, save_farm


def _small_config(seed: int) -> FarmGenerationConfig:
    return FarmGenerationConfig(bounds=(0.0, 0.0, 60.0, 60.0), seed=seed, max_faces=8)


def test_scene_round_trip_via_plain_dict():
    scene = generate_farm(_small_config(seed=1))
    plain = to_plain(scene)
    rebuilt = from_plain(plain, FarmScene)
    assert to_plain(rebuilt) == plain


def test_scene_yaml_file_round_trip(tmp_path):
    scene = generate_farm(_small_config(seed=2))
    path = str(tmp_path / "scene.yaml")
    write(scene, path)
    rebuilt = from_plain(read(path), FarmScene)
    assert to_plain(rebuilt) == to_plain(scene)


def test_scene_json_file_round_trip(tmp_path):
    scene = generate_farm(_small_config(seed=3))
    path = str(tmp_path / "scene.json")
    write(scene, path)
    rebuilt = from_plain(read(path), FarmScene)
    assert to_plain(rebuilt) == to_plain(scene)


def test_save_load_farm_round_trip(tmp_path):
    config = _small_config(seed=4)
    scene = generate_farm(config)
    path = str(tmp_path / "farm.yaml")
    save_farm(scene, config, path)
    rebuilt_scene, rebuilt_config = load_farm(path)
    assert to_plain(rebuilt_scene) == to_plain(scene)
    assert to_plain(rebuilt_config) == to_plain(config)


def test_polymorphic_parcel_fields_survive_round_trip(tmp_path):
    """CropArea-specific fields (planting/weeds/irrigation_type) must
    come back as CropArea, not the declared dict[str, Parcel] base type."""
    scene = generate_farm(_small_config(seed=5))
    path = str(tmp_path / "scene.yaml")
    write(scene, path)
    rebuilt = from_plain(read(path), FarmScene)

    assert rebuilt.parcels.keys() == scene.parcels.keys()
    for pid, parcel in scene.parcels.items():
        rebuilt_parcel = rebuilt.parcels[pid]
        assert type(rebuilt_parcel) is type(parcel)
        assert rebuilt_parcel.planting == parcel.planting
        assert rebuilt_parcel.irrigation_type == parcel.irrigation_type


def test_tuple_typed_fields_stay_tuples(tmp_path):
    """Point2/Point3 fields must reconstruct as tuples, not lists, so
    dataclass equality with the original scene holds."""
    scene = generate_farm(_small_config(seed=6))
    path = str(tmp_path / "scene.json")
    write(scene, path)
    rebuilt = from_plain(read(path), FarmScene)

    any_tree = next(iter(rebuilt.trees.values()))
    assert isinstance(any_tree.position, tuple)
    any_parcel = next(iter(rebuilt.parcels.values()))
    assert all(isinstance(v, tuple) for v in any_parcel.polygon)

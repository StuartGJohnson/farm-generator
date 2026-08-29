"""
farm_ir/serialize.py

Human-readable, round-trippable (de)serialization of the IR to YAML or
JSON. This is the on-disk format debug tooling writes to `debug_out/`
(see visualization/debug_view.py) and, later, what USD/Gazebo exporters
will read as input (see CLAUDE.md "Purpose" and "Explicitly out of scope
this phase" -- export itself is not built yet, but the IR needs a stable
on-disk form before it can be).

Deliberately generic and schema-driven, via `dataclasses`/`typing`
introspection, rather than a hand-written field list per class: the
schema (farm_ir/schema.py) is sizeable and will keep growing per its own
"add fields deliberately" policy, and a hand-written (de)serializer would
silently go stale the next time a field is added. Every dataclass
instance is tagged with a `__type__` discriminator on serialization so
polymorphic containers (e.g. `FarmScene.parcels: dict[str, Parcel]`,
whose values are actually `CropArea` or `FarmsteadCompound`) reconstruct
as their real subclass, not the declared field type -- this is the one
piece of behavior a fully generic `dataclasses.asdict`/library round trip
wouldn't get right on its own.

This module knows nothing about `generation/` -- `generation/
orchestrator.py`'s `save_farm`/`load_farm` compose a scene with its
`FarmGenerationConfig` (see CLAUDE.md "Config": "Serialize a
FarmGenerationConfig instance alongside a generated FarmScene for
reproducibility") using the generic `to_plain`/`from_plain`/`write`/
`read` primitives here.
"""

from __future__ import annotations

import dataclasses
import inspect
import json
import typing
from enum import Enum

import yaml

from farm_ir import schema as _schema

# Every dataclass actually defined in farm_ir/schema.py, keyed by class
# name -- used to resolve a serialized `__type__` discriminator back to
# the real (possibly subclass) type on load. Auto-discovered so this
# stays in sync automatically as the schema grows.
_TYPE_REGISTRY: dict[str, type] = {
    name: obj
    for name, obj in inspect.getmembers(_schema, inspect.isclass)
    if dataclasses.is_dataclass(obj) and obj.__module__ == _schema.__name__
}


def to_plain(obj):
    """Convert a dataclass instance (nested, dicts/lists/tuples of them,
    Enums, and plain scalars) into plain dict/list/str/float/int/bool/
    None -- the subset every YAML/JSON library can write directly."""
    if obj is None:
        return obj
    if isinstance(obj, Enum):
        # Checked BEFORE the str/int/bool branch: several enums in
        # schema.py mix in str (e.g. `class RoadSurface(str, Enum)`), so
        # `isinstance(obj, str)` would otherwise match first and let the
        # raw enum member through un-reduced -- yaml/json can't represent
        # that (its exact type isn't a registered representer), only its
        # plain `.value`.
        return obj.value
    if isinstance(obj, (str, int, float, bool)):
        return obj
    if dataclasses.is_dataclass(obj):
        result = {"__type__": type(obj).__name__}
        for f in dataclasses.fields(obj):
            result[f.name] = to_plain(getattr(obj, f.name))
        return result
    if isinstance(obj, dict):
        return {k: to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_plain(v) for v in obj]
    raise TypeError(f"farm_ir.serialize.to_plain: don't know how to serialize {type(obj)!r}")


def from_plain(data, expected_type):
    """
    Reconstruct a value of `expected_type` from plain data produced by
    `to_plain` (or a hand-edited YAML/JSON file matching that shape).
    `expected_type` is a real type or a `typing`/PEP 585 generic (e.g.
    `dict[str, Parcel]`, `Optional[float]`, `Point2` -- these are plain
    module-level aliases in schema.py, so `typing.get_type_hints`
    resolves them transparently even under `from __future__ import
    annotations`).

    For a dataclass field, the DECLARED type is only a fallback: if the
    serialized dict carries a `__type__` discriminator naming a
    registered (sub)class, that actual class's own fields are used
    instead -- this is what makes e.g. `CropArea`-specific fields survive
    round-tripping through a `dict[str, Parcel]`-typed container.
    """
    if data is None:
        return None

    origin = typing.get_origin(expected_type)

    if origin is typing.Union:
        # Optional[X] == Union[X, None]; data is already known not-None here.
        args = [a for a in typing.get_args(expected_type) if a is not type(None)]
        return from_plain(data, args[0])

    if origin is list:
        (item_type,) = typing.get_args(expected_type) or (object,)
        return [from_plain(v, item_type) for v in data]

    if origin is dict:
        _key_type, val_type = typing.get_args(expected_type)
        return {k: from_plain(v, val_type) for k, v in data.items()}

    if origin is tuple:
        item_types = typing.get_args(expected_type)
        if len(item_types) == 2 and item_types[1] is Ellipsis:
            return tuple(from_plain(v, item_types[0]) for v in data)
        return tuple(from_plain(v, t) for v, t in zip(data, item_types))

    if isinstance(expected_type, type) and issubclass(expected_type, Enum):
        return expected_type(data)

    if isinstance(expected_type, type) and dataclasses.is_dataclass(expected_type):
        actual_type = _TYPE_REGISTRY.get(data.get("__type__"), expected_type)
        hints = typing.get_type_hints(actual_type)
        kwargs = {
            f.name: from_plain(data[f.name], hints[f.name])
            for f in dataclasses.fields(actual_type)
            if f.name in data
        }
        return actual_type(**kwargs)

    # Plain scalar -- coerce to the declared type (YAML/JSON can hand back
    # e.g. an int where a float field is expected).
    if expected_type is float:
        return float(data)
    if expected_type is int:
        return int(data)
    if expected_type is str:
        return str(data)
    if expected_type is bool:
        return bool(data)
    return data


def write(obj, path: str) -> None:
    """Write `obj` (a dataclass, or a plain dict/list composed of them --
    see generation/orchestrator.py's save_farm) to `path` as YAML or
    JSON, chosen by extension (.yaml/.yml or .json)."""
    plain = to_plain(obj)
    if path.endswith((".yaml", ".yml")):
        with open(path, "w") as f:
            yaml.safe_dump(plain, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    elif path.endswith(".json"):
        with open(path, "w") as f:
            json.dump(plain, f, indent=2)
    else:
        raise ValueError(f"farm_ir.serialize.write: unsupported extension for {path!r} (use .yaml/.yml/.json)")


def read(path: str):
    """Read the plain dict/list previously written by `write` back from
    `path`. Callers reconstruct real types via `from_plain`."""
    if path.endswith((".yaml", ".yml")):
        with open(path) as f:
            return yaml.safe_load(f)
    elif path.endswith(".json"):
        with open(path) as f:
            return json.load(f)
    else:
        raise ValueError(f"farm_ir.serialize.read: unsupported extension for {path!r} (use .yaml/.yml/.json)")

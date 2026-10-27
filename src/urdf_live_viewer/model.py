# Copyright 2026 The urdf_live_viewer authors
# SPDX-License-Identifier: Apache-2.0

"""Expand a robot description, load its geometry with Pinocchio and describe it for the page."""

from __future__ import annotations

import base64
import os
import pathlib
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin

MOVABLE = {"revolute", "continuous", "prismatic"}


@dataclass
class Options:
    """Everything needed to turn files on disk into a model."""

    robot: pathlib.Path
    robot_args: dict[str, str] = field(default_factory=dict)
    srdf: pathlib.Path | None = None
    srdf_args: dict[str, str] = field(default_factory=dict)
    package_dirs: list[pathlib.Path] = field(default_factory=list)
    extra_watch: list[pathlib.Path] = field(default_factory=list)
    geometry: str = "collision"
    tip: str | None = None


class BuildError(RuntimeError):
    """The description could not be expanded or loaded."""


def expand(path: pathlib.Path, args: dict[str, str]) -> tuple[str, list[pathlib.Path]]:
    """Return the XML of a .urdf, .srdf or .xacro file and every file it read."""
    if path.suffix != ".xacro":
        return path.read_text(), [path]
    try:
        import xacro
    except ImportError:
        result = subprocess.run(["xacro", str(path), *(f"{k}:={v}" for k, v in args.items())],
                                capture_output=True, text=True)
        if result.returncode:
            raise BuildError((result.stderr.strip() or "xacro failed").splitlines()[-1])
        return result.stdout, [path]
    if hasattr(xacro, "all_includes"):
        xacro.all_includes.clear()
    try:
        doc = xacro.process_file(str(path), mappings=dict(args))
    except Exception as exc:  # xacro raises plain exceptions with a readable message
        raise BuildError(f"xacro: {exc}") from exc
    includes = [pathlib.Path(p) for p in getattr(xacro, "all_includes", [])]
    return doc.toxml(), [path, *includes]


def ros_package_dirs() -> list[pathlib.Path]:
    """Directories that hold ROS packages, so package:// URIs resolve without --package-dir."""
    dirs = []
    for prefix in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)):
        dirs.append(pathlib.Path(prefix) / "share")
    for entry in filter(None, os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep)):
        dirs.append(pathlib.Path(entry))
    return [d for d in dirs if d.is_dir()]


def enclosing_package_dir(path: pathlib.Path) -> pathlib.Path | None:
    """The folder that contains the package holding `path`, so source meshes win over installed ones."""
    for parent in path.resolve().parents:
        if (parent / "package.xml").is_file():
            return parent.parent
    return None


def se3_list(placement: pin.SE3) -> list[float]:
    m = np.eye(4)
    m[:3, :3] = placement.rotation
    m[:3, 3] = placement.translation
    return [round(float(v), 6) for v in m.flatten()]


def origin_list(origin: ET.Element | None) -> list[float]:
    xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
    rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
    return se3_list(pin.SE3(pin.rpy.rpyToMatrix(*rpy), np.array(xyz)))


def parse_srdf(xml: str) -> tuple[dict[str, dict[str, float]], set[frozenset[str]], list[str]]:
    root = ET.fromstring(xml)
    poses = {}
    for state in root.findall("group_state"):
        name = state.get("name")
        group = state.get("group")
        label = name if not group else f"{name} ({group})"
        poses[label] = {j.get("name"): float(j.get("value").split()[0]) for j in state.findall("joint")}
    disabled = {frozenset((d.get("link1"), d.get("link2"))) for d in root.findall("disable_collisions")}
    tips = [c.get("tip_link") for c in root.findall("group/chain") if c.get("tip_link")]
    return poses, disabled, tips


class CellModel:
    """The current description, rebuilt on demand; readers always see the last good build."""

    def __init__(self, options: Options) -> None:
        self.options = options
        self.package_dirs = [str(d) for d in self._package_dirs()]
        self.lock = threading.Lock()
        self.version = 0
        self.error = ""
        self.updated = ""
        self.payload = b"{}"
        self.watched: list[pathlib.Path] = [options.robot, *([options.srdf] if options.srdf else []), *options.extra_watch]
        self._model: pin.Model | None = None
        self._geom: pin.GeometryModel | None = None
        self._mesh_cache: dict[str, dict] = {}

    def _package_dirs(self) -> list[pathlib.Path]:
        dirs = list(self.options.package_dirs)
        for path in (self.options.robot, self.options.srdf):
            if path is not None and (own := enclosing_package_dir(path)) is not None:
                dirs.append(own)
        return [*dirs, *ros_package_dirs()]

    def rebuild(self) -> bool:
        started = time.monotonic()
        try:
            urdf, files = expand(self.options.robot, self.options.robot_args)
            poses, disabled, tips = {}, set(), []
            if self.options.srdf is not None:
                srdf, srdf_files = expand(self.options.srdf, self.options.srdf_args)
                poses, disabled, tips = parse_srdf(srdf)
                files += srdf_files
            model = pin.buildModelFromXML(urdf)
            kind = pin.GeometryType.VISUAL if self.options.geometry == "visual" else pin.GeometryType.COLLISION
            geom = pin.buildGeomFromUrdfString(model, urdf, kind, package_dirs=self.package_dirs)
            payload, moving = self._describe(urdf, model, geom, poses, tips)
            self._add_pairs(model, geom, payload["joints"], moving, disabled)
        except BuildError as exc:
            return self._fail(str(exc))
        except Exception as exc:  # noqa: BLE001 - shown on the page, the last good model stays up
            return self._fail(f"{type(exc).__name__}: {exc}")
        with self.lock:
            self._model, self._geom = model, geom
            self.payload = pin_json(payload)
            self.watched = sorted({*files, *self.options.extra_watch})
            self.version += 1
            self.error = ""
            self.updated = time.strftime("%H:%M:%S")
        print(f"[{self.updated}] built {payload['name']}: {len(payload['links'])} links, "
              f"{len(payload['geoms'])} shapes in {time.monotonic() - started:.1f}s", flush=True)
        return True

    def _fail(self, message: str) -> bool:
        with self.lock:
            self.error = message
            self.version += 1
        print(f"[{time.strftime('%H:%M:%S')}] build failed: {message}", flush=True)
        return False

    def _describe(self, urdf: str, model: pin.Model, geom: pin.GeometryModel, poses: dict,
                  tips: list[str]) -> tuple[dict, set[str]]:
        root = ET.fromstring(urdf)
        materials = {m.get("name"): m.find("color").get("rgba") for m in root.findall("material") if m.find("color") is not None}
        colors = {}
        for link in root.findall("link"):
            mat = link.find("visual/material")
            if mat is None:
                continue
            rgba = mat.find("color").get("rgba") if mat.find("color") is not None else materials.get(mat.get("name"))
            if rgba:
                colors[link.get("name")] = [float(v) for v in rgba.split()]

        joints = []
        for joint in root.findall("joint"):
            limit, axis, mimic = joint.find("limit"), joint.find("axis"), joint.find("mimic")
            jtype = joint.get("type")
            lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else None
            upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else None
            if jtype == "continuous" or (jtype in MOVABLE and (lower is None or upper is None or lower >= upper)):
                lower, upper = (-np.pi, np.pi) if jtype != "prismatic" else (-0.5, 0.5)
            joints.append({
                "name": joint.get("name"), "type": jtype if jtype in MOVABLE else "fixed",
                "parent": joint.find("parent").get("link"), "child": joint.find("child").get("link"),
                "T": origin_list(joint.find("origin")),
                "axis": [float(v) for v in axis.get("xyz").split()] if axis is not None else [1.0, 0.0, 0.0],
                "lower": lower, "upper": upper,
                "mimic": {"joint": mimic.get("joint"), "multiplier": float(mimic.get("multiplier", 1)),
                          "offset": float(mimic.get("offset", 0))} if mimic is not None else None,
            })

        children: dict[str, list[dict]] = {}
        for j in joints:
            children.setdefault(j["parent"], []).append(j)
        moving: set[str] = set()

        def mark(link: str, below_movable: bool) -> None:
            if below_movable:
                moving.add(link)
            for j in children.get(link, []):
                mark(j["child"], below_movable or j["type"] != "fixed")

        link_names = [link.get("name") for link in root.findall("link")]
        child_links = {j["child"] for j in joints}
        for top in (n for n in link_names if n not in child_links):
            mark(top, False)

        geoms, meshes = [], {}
        for obj in geom.geometryObjects:
            frame = model.frames[obj.parentFrame]
            shape, kind = obj.geometry, type(obj.geometry).__name__
            entry = {"link": frame.name, "T": se3_list(frame.placement.inverse() * obj.placement)}
            if kind == "Box":
                entry.update(kind="box", size=[round(2 * float(v), 5) for v in shape.halfSide])
            elif kind == "Cylinder":
                entry.update(kind="cylinder", radius=float(shape.radius), length=2 * float(shape.halfLength))
            elif kind == "Capsule":
                entry.update(kind="capsule", radius=float(shape.radius), length=2 * float(shape.halfLength))
            elif kind == "Sphere":
                entry.update(kind="sphere", radius=float(shape.radius))
            elif "BVH" in kind:
                meshes[obj.meshPath] = self._mesh(obj)
                entry.update(kind="mesh", mesh=obj.meshPath)
            else:
                print(f"  skipping {obj.name}: {kind} shapes are not drawn", flush=True)
                continue
            geoms.append(entry)

        with_shape = {g["link"] for g in geoms}
        candidates = [t for t in [self.options.tip, *tips] if t in link_names]
        tip = candidates[0] if candidates else deepest_link(link_names, joints, with_shape)
        return {
            "name": root.get("name", "robot"), "links": link_names, "joints": joints, "geoms": geoms,
            "meshes": meshes, "colors": colors, "moving": sorted(moving), "poses": poses, "tip": tip,
            "source": str(self.options.robot), "args": [f"{k}:={v}" for k, v in self.options.robot_args.items()],
            "geometry": self.options.geometry,
        }, moving

    def _mesh(self, obj: pin.GeometryObject) -> dict:
        path = pathlib.Path(obj.meshPath)
        key = f"{path}:{path.stat().st_mtime if path.exists() else 0}:{list(obj.meshScale)}"
        if key not in self._mesh_cache:
            shape = obj.geometry
            vertices = np.array(shape.vertices(), dtype="<f4") * np.asarray(obj.meshScale, dtype="<f4")
            triangles = np.array([[t[0], t[1], t[2]] for t in map(shape.tri_indices, range(shape.num_tris))], dtype="<u4")
            self._mesh_cache[key] = {"v": base64.b64encode(vertices.tobytes()).decode(),
                                     "i": base64.b64encode(triangles.tobytes()).decode()}
        return self._mesh_cache[key]

    @staticmethod
    def _add_pairs(model: pin.Model, geom: pin.GeometryModel, joints: list[dict], moving: set[str],
                   disabled: set[frozenset[str]]) -> None:
        adjacent = {frozenset((j["parent"], j["child"])) for j in joints}
        links = [model.frames[o.parentFrame].name for o in geom.geometryObjects]
        for s, static in enumerate(links):
            if static in moving:
                continue
            for m, mover in enumerate(links):
                pair = frozenset((static, mover))
                if mover in moving and pair not in disabled and pair not in adjacent:
                    geom.addCollisionPair(pin.CollisionPair(s, m))

    def clearance(self, values: dict[str, float]) -> list[dict]:
        """Closest moving part to every static link, at the given joint values."""
        with self.lock:
            if self._model is None:
                return []
            model, geom = self._model, self._geom
            q = pin.neutral(model)
            for name, value in values.items():
                if not model.existJointName(name):
                    continue
                joint = model.joints[model.getJointId(name)]
                if joint.nq == 2:
                    q[joint.idx_q:joint.idx_q + 2] = [np.cos(value), np.sin(value)]
                elif joint.nq == 1:
                    q[joint.idx_q] = value
            data, gdata = model.createData(), pin.GeometryData(geom)
            pin.computeDistances(model, data, geom, gdata, q)
            closest: dict[str, tuple[float, str]] = {}
            for k, pair in enumerate(geom.collisionPairs):
                static = model.frames[geom.geometryObjects[pair.first].parentFrame].name
                mover = model.frames[geom.geometryObjects[pair.second].parentFrame].name
                distance = float(gdata.distanceResults[k].min_distance)
                if static not in closest or distance < closest[static][0]:
                    closest[static] = (distance, mover)
        return sorted(({"object": o, "distance": d, "part": p} for o, (d, p) in closest.items()),
                      key=lambda r: r["distance"])


def deepest_link(links: list[str], joints: list[dict], with_shape: set[str]) -> str:
    """The link furthest from the root; among equals, a bare frame such as a TCP wins."""
    parent_of = {j["child"]: j["parent"] for j in joints}

    def depth(link: str) -> int:
        n = 0
        while link in parent_of:
            link, n = parent_of[link], n + 1
        return n

    return max(links, key=lambda link: (depth(link), link not in with_shape)) if links else ""


def pin_json(payload: dict) -> bytes:
    import json
    return json.dumps(payload, separators=(",", ":")).encode()

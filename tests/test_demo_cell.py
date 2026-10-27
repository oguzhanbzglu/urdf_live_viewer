# Copyright 2026 The urdf_live_viewer authors
# SPDX-License-Identifier: Apache-2.0

import json
import pathlib
import shutil

import pytest

from urdf_live_viewer.model import CellModel, Options

DEMO = pathlib.Path(__file__).resolve().parents[1] / "examples" / "demo_cell"


@pytest.fixture
def model(tmp_path):
    for f in DEMO.iterdir():
        shutil.copy(f, tmp_path / f.name)
    m = CellModel(Options(robot=tmp_path / "demo_cell.urdf.xacro", srdf=tmp_path / "demo_cell.srdf"))
    assert m.rebuild(), m.error
    return m


def payload(m):
    return json.loads(m.payload)


def test_builds_tree_and_shapes(model):
    data = payload(model)
    assert data["name"] == "demo_cell"
    assert {"world", "table", "obstacle", "tool_tip"} <= set(data["links"])
    assert {"shoulder", "forearm", "finger_left", "finger_right"} <= set(data["moving"])
    assert "table" not in data["moving"]
    assert data["tip"] == "tool_tip"
    kinds = {g["kind"] for g in data["geoms"]}
    assert kinds == {"box", "cylinder"}


def test_mimic_and_poses(model):
    data = payload(model)
    right = next(j for j in data["joints"] if j["name"] == "finger_right_joint")
    assert right["mimic"]["joint"] == "finger_left_joint"
    assert set(data["poses"]) == {"home (arm)", "over_obstacle (arm)", "reach_back (arm)"}


def test_clearance_skips_disabled_and_adjacent_pairs(model):
    rows = model.clearance({"pan": 0.0, "shoulder_lift": 0.0, "elbow": 0.0})
    objects = {r["object"] for r in rows}
    assert {"obstacle", "back_wall", "table"} <= objects
    assert "base" not in objects or all(r["part"] != "shoulder" for r in rows if r["object"] == "base")
    assert rows == sorted(rows, key=lambda r: r["distance"])


def test_reaching_back_hits_the_wall(model):
    rows = {r["object"]: r["distance"] for r in model.clearance({"pan": 3.14, "shoulder_lift": 0.0, "elbow": 0.0})}
    assert rows["back_wall"] <= 0.0


def test_xacro_argument_removes_obstacle(tmp_path):
    for f in DEMO.iterdir():
        shutil.copy(f, tmp_path / f.name)
    m = CellModel(Options(robot=tmp_path / "demo_cell.urdf.xacro", robot_args={"obstacle": "false"}))
    assert m.rebuild()
    assert "obstacle" not in payload(m)["links"]


def test_broken_save_keeps_last_good_model(model, tmp_path):
    good = model.payload
    path = model.options.robot
    path.write_text(path.read_text().replace("</robot>", "<oops></robot"))
    assert not model.rebuild()
    assert model.error
    assert model.payload == good


def test_watches_included_files(model):
    assert model.options.robot in model.watched

# urdf-live-viewer

View a URDF or xacro robot description in the browser, and see it redraw every time you save a file.

It is built for laying out robot cells: move the joints with sliders, read the pose of any link, and see
how close every moving part gets to the table, walls and fixtures, measured on the same collision
geometry MoveIt uses.

```bash
urdf-live-viewer my_cell.urdf.xacro --srdf my_cell.srdf
# open http://localhost:8765, edit the xacro in your editor, save, watch it update
```

## What it does

- **Redraws on save.** The description and every file xacro reads while expanding it are watched.
  Saving any of them rebuilds the model, and the page redraws within about a second, keeping your camera and joint
  values. A save that does not build shows the error and keeps the last good model on screen.
- **Moves the robot.** A slider for every revolute, continuous and prismatic joint. Mimic joints follow
  their leader. Named poses come from the SRDF `group_state`s.
- **Measures clearance.** Links behind a movable joint count as *moving*, everything else as *static*.
  For the current pose, the page lists the closest moving part to every static link, in centimetres,
  orange under 5 cm and red when touching. Pairs disabled in the SRDF and parent–child pairs are skipped.
  Distances come from [Pinocchio](https://github.com/stack-of-tasks/pinocchio) and Coal.
- **Inspects.** Click a part to see its link, joint and origin. Pick any link for a live pose readout.
  The static tab lists every primitive shape with its size, centre and axis-aligned extents. The links
  tab shows the kinematic tree.
- **Works with or without ROS.** `package://` URIs resolve from a sourced ROS 1 or ROS 2 workspace, from
  the package that contains the file you open, or from `--package-dir`.

## Install

From a sourced ROS 2 environment, which already has Pinocchio and xacro:

```bash
pip install --no-deps git+https://github.com/oguzhanbzglu/urdf_live_viewer.git
```

Without ROS, let pip install the dependencies too (`pin` is Pinocchio's wheel):

```bash
pip install git+https://github.com/oguzhanbzglu/urdf_live_viewer.git
```

The page loads three.js from a CDN, so the browser needs internet access the first time.

## Try it

The repository has a small demo cell made of primitive shapes, so it needs no ROS packages:

```bash
git clone https://github.com/oguzhanbzglu/urdf_live_viewer.git && cd urdf_live_viewer
pip install -e .
urdf-live-viewer examples/demo_cell/demo_cell.urdf.xacro --srdf examples/demo_cell/demo_cell.srdf --open
```

Then change `size="0.15 0.15 0.25"` of the obstacle in `demo_cell.urdf.xacro` and save. Try the
`reach_back` pose to see a collision with the wall turn red. Pass `obstacle:=false` to drop the box.

## Usage

```text
urdf-live-viewer ROBOT [name:=value ...] [options]

  ROBOT                     .urdf or .urdf.xacro
  name:=value               xacro arguments for ROBOT
  --srdf FILE               .srdf or .srdf.xacro: named poses and disabled collision pairs
  --srdf-arg name:=value    xacro argument for the SRDF (repeatable)
  --package-dir DIR         folder that contains ROS packages, for package:// URIs (repeatable)
  --watch PATH              extra file or folder to watch (repeatable)
  --geometry collision|visual   shapes to draw and measure (default: collision)
  --tip LINK                link for the pose readout (default: SRDF chain tip, else the deepest link)
  --host ADDR               default 127.0.0.1; use 0.0.0.0 to reach it from another machine
  --port N                  default 8765
  --poll SECONDS            how often files are checked (default: 0.5)
  --open                    open the page in the default browser
```

A ROS 2 example with xacro arguments on both files:

```bash
urdf-live-viewer src/my_cell/urdf/cell.urdf.xacro hardware:=mock use_scene:=true \
    --srdf src/my_cell/srdf/cell.srdf.xacro --srdf-arg use_scene:=true --tip tool0
```

Inside a Docker container with host networking the page is reachable from the host at
`http://localhost:8765`. With bridged networking, pass `--host 0.0.0.0` and publish the port.

## How it works

`urdf-live-viewer` expands the description with xacro, loads it with Pinocchio, and serves a JSON
description of the kinematic tree and the geometry to a three.js page. The page does the forward
kinematics for the sliders itself, and asks the server for distances, which Pinocchio computes on the
collision model. A background thread polls the modification times of every file xacro read and rebuilds
when one changes. The page polls a version counter and reloads the model when it changes.

Mesh geometry is drawn as the triangles Pinocchio loaded, so what you see is what is measured.

## Development

```bash
pip install -e '.[test]'
pytest
```

## License

Apache-2.0. See [LICENSE](LICENSE).

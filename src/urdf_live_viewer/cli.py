# Copyright 2026 The urdf_live_viewer authors
# SPDX-License-Identifier: Apache-2.0

"""Command line entry point."""

from __future__ import annotations

import argparse
import pathlib
import sys
import webbrowser

from . import __version__
from .model import CellModel, Options
from .server import serve

EPILOG = """examples:
  urdf-live-viewer robot.urdf
  urdf-live-viewer robot.urdf.xacro hardware:=mock --srdf robot.srdf
  urdf-live-viewer cell.urdf.xacro --srdf cell.srdf.xacro --srdf-arg scene:=true --tip tool0 --open
"""


def mappings(items: list[str], flag: str) -> dict[str, str]:
    out = {}
    for item in items:
        if ":=" not in item:
            sys.exit(f"urdf-live-viewer: {flag} expects name:=value, got '{item}'")
        name, value = item.split(":=", 1)
        out[name] = value
    return out


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="urdf-live-viewer",
        description="View a URDF or xacro robot description in the browser and redraw it every time a file is saved.",
        epilog=EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("robot", type=pathlib.Path, help="robot description: .urdf or .urdf.xacro")
    parser.add_argument("xacro_args", nargs="*", metavar="name:=value", help="xacro arguments for the robot description")
    parser.add_argument("--srdf", type=pathlib.Path, help="SRDF (or .srdf.xacro): named poses and disabled collision pairs")
    parser.add_argument("--srdf-arg", action="append", default=[], metavar="name:=value", help="xacro argument for the SRDF; repeatable")
    parser.add_argument("--package-dir", action="append", default=[], type=pathlib.Path, metavar="DIR",
                        help="folder that contains ROS packages, for package:// URIs; repeatable. "
                             "Sourced ROS workspaces are found automatically")
    parser.add_argument("--watch", action="append", default=[], type=pathlib.Path, metavar="PATH",
                        help="extra file or folder to watch; files read by xacro are watched automatically")
    parser.add_argument("--geometry", choices=["collision", "visual"], default="collision",
                        help="which geometry to draw and measure (default: collision)")
    parser.add_argument("--tip", metavar="LINK", help="link whose pose is shown in the readout (default: the deepest link)")
    parser.add_argument("--host", default="127.0.0.1", help="address to listen on; 0.0.0.0 to reach it from other machines")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--poll", type=float, default=0.5, help="seconds between file checks (default: 0.5)")
    parser.add_argument("--open", action="store_true", help="open the page in the default browser")
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    args = parser.parse_args(argv)

    if not args.robot.is_file():
        sys.exit(f"urdf-live-viewer: no such file: {args.robot}")
    if args.srdf is not None and not args.srdf.is_file():
        sys.exit(f"urdf-live-viewer: no such file: {args.srdf}")

    model = CellModel(Options(
        robot=args.robot.resolve(), robot_args=mappings(args.xacro_args, "robot argument"),
        srdf=args.srdf.resolve() if args.srdf else None, srdf_args=mappings(args.srdf_arg, "--srdf-arg"),
        package_dirs=[p.resolve() for p in args.package_dir], extra_watch=[p.resolve() for p in args.watch],
        geometry=args.geometry, tip=args.tip))
    model.rebuild()
    try:
        server = serve(model, args.host, args.port, args.poll)
    except OSError as exc:
        sys.exit(f"urdf-live-viewer: cannot listen on {args.host}:{args.port}: {exc.strerror}")
    url = f"http://{'localhost' if args.host in ('127.0.0.1', '0.0.0.0') else args.host}:{args.port}"
    print(f"Viewing {args.robot} on {url}  (Ctrl+C to stop)", flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()

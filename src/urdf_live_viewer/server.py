# Copyright 2026 The urdf_live_viewer authors
# SPDX-License-Identifier: Apache-2.0

"""HTTP server for the page, plus the file watcher that triggers rebuilds."""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources

from .model import CellModel


def watch(model: CellModel, period: float) -> None:
    def signature() -> dict[str, float]:
        stamps = {}
        for path in model.watched:
            files = sorted(p for p in path.rglob("*") if p.is_file()) if path.is_dir() else [path]
            for f in files:
                try:
                    stamps[str(f)] = f.stat().st_mtime
                except FileNotFoundError:
                    stamps[str(f)] = -1.0
        return stamps

    last = signature()
    while True:
        time.sleep(period)
        current = signature()
        if current != last:
            time.sleep(0.15)
            model.rebuild()
            last = signature()


def make_handler(model: CellModel):
    page = resources.files("urdf_live_viewer").joinpath("web/index.html")

    class Handler(BaseHTTPRequestHandler):
        def _send(self, body: bytes, kind: str, status: int = 200) -> None:
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802 - http.server API
            if self.path == "/":
                self._send(page.read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/model.json":
                with model.lock:
                    body = model.payload
                self._send(body, "application/json")
            elif self.path == "/version":
                with model.lock:
                    body = json.dumps({"version": model.version, "error": model.error, "updated": model.updated})
                self._send(body.encode(), "application/json")
            else:
                self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802 - http.server API
            if self.path != "/clearance":
                self.send_error(404)
                return
            try:
                values = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                rows = model.clearance({str(k): float(v) for k, v in values.items()})
            except (ValueError, TypeError):
                self._send(b'{"error": "expected a JSON object of joint values"}', "application/json", 400)
                return
            self._send(json.dumps(rows).encode(), "application/json")

        def log_message(self, *_args) -> None:
            pass

    return Handler


def serve(model: CellModel, host: str, port: int, poll: float) -> ThreadingHTTPServer:
    threading.Thread(target=watch, args=(model, poll), daemon=True).start()
    return ThreadingHTTPServer((host, port), make_handler(model))

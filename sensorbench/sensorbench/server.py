"""Small local HTTP API. No network services or API keys are required."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from . import __version__
from .storage import REPORT_ID_LENGTH, InputError, SessionConflict, Store, strict_json

MAX_BODY = 4 * 1024 * 1024


def make_server(database_path, host="127.0.0.1", port=8765):
    store = Store(database_path)

    class Handler(BaseHTTPRequestHandler):
        server_version = "SensorBench/" + __version__

        def send_bytes(self, status, body, content_type, headers=None):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            for name, value in (headers or {}).items():
                self.send_header(name, value)
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, status, obj):
            self.send_bytes(status, json.dumps(obj, ensure_ascii=False, allow_nan=False).encode("utf-8"), "application/json; charset=utf-8")

        def do_GET(self):
            parts = urlsplit(self.path)
            path = parts.path
            if path == "/":
                self.send_bytes(200, Path(__file__).with_name("static").joinpath("index.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/health":
                self.send_json(200, {"status": "ok", "app_version": __version__})
            elif path == "/api/runs":
                try:
                    query = parse_qs(parts.query)
                    limit = int(query.get("limit", ["100"])[0])
                    self.send_json(200, {"runs": store.list_runs(limit)})
                except (InputError, ValueError):
                    self.send_json(400, {"error": "limit must be an integer between 1 and 500"})
            elif path.startswith("/api/runs/"):
                elements = path.strip("/").split("/")
                if len(elements) not in (3, 4) or (len(elements) == 4 and elements[3] != "report"):
                    self.send_json(404, {"error": "Not found"})
                    return
                run_id = elements[2]
                detail = store.get_run(run_id)
                if detail is None:
                    self.send_json(404, {"error": "Run not found"})
                elif len(elements) == 4:
                    self.send_bytes(200, store.markdown_report(run_id).encode("utf-8"), "text/markdown; charset=utf-8",
                                    {"Content-Disposition": f'attachment; filename="sensorbench-{run_id[:REPORT_ID_LENGTH]}.md"'})
                else:
                    self.send_json(200, detail)
            elif path == "/favicon.ico":
                self.send_bytes(204, b"", "image/x-icon")
            else:
                self.send_json(404, {"error": "Not found"})

        def do_POST(self):
            if urlsplit(self.path).path != "/api/runs":
                self.send_json(404, {"error": "Not found"})
                return
            if self.headers.get("Transfer-Encoding"):
                self.send_json(400, {"error": "Transfer-Encoding is not supported"})
                return
            try:
                length = int(self.headers.get("Content-Length", "-1"))
            except ValueError:
                length = -1
            if length < 0:
                self.send_json(411, {"error": "A valid Content-Length is required"})
                return
            if length > MAX_BODY:
                self.send_json(413, {"error": "Request body exceeds 4 MiB"})
                return
            self.connection.settimeout(10)
            try:
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise InputError("Incomplete request body")
                # Drain the bounded body before a header-based rejection. Closing
                # a Windows socket with unread uploaded bytes can reset it before
                # the client receives the intended 403/415 response.
                origin = self.headers.get("Origin")
                host = self.headers.get("Host")
                if origin and origin != "http://" + (host or ""):
                    self.send_json(403, {"error": "Cross-origin ingestion is not allowed"})
                    return
                if self.headers.get_content_type() != "application/json":
                    self.send_json(415, {"error": "Use Content-Type: application/json"})
                    return
                result = store.ingest(strict_json(raw.decode("utf-8")))
                self.send_json(201 if result["created"] else 200, result)
            except SessionConflict as exc:
                self.send_json(409, {"error": str(exc)})
            except (InputError, UnicodeDecodeError, RecursionError) as exc:
                self.send_json(400, {"error": str(exc)})
            except TimeoutError:
                self.send_json(408, {"error": "Request body timed out"})

        def log_message(self, fmt, *args):
            # Keep device payloads out of access logs.
            return

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server

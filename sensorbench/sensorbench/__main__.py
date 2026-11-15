"""Run with python -m sensorbench from the project directory."""

import argparse
import json
from pathlib import Path
import sys

from .server import make_server
from .simulator import SCENARIOS, generate_samples
from .storage import InputError, Store, strict_json
from .capture import CAPTURE_PROFILES, capture_serial, list_ports, replay_file

DEFAULT_DB = Path("data") / "sensorbench.sqlite3"


def demo(store):
    results = []
    for scenario in SCENARIOS:
        payload = {"device_id": "sim-imu-01", "session_id": "demo-v2-" + scenario,
                   "firmware_version": "simulator-0.1", "source": "simulator", "test_profile": "guided_motion",
                   "scenario": scenario, "samples": generate_samples(scenario)}
        result = store.ingest(payload)
        results.append(result)
        run = result["run"]
        print(f"{scenario:15} {run['status'].upper():4} | {run['sample_count']} samples | {run['failed_checks']} failed checks | {run['run_id']}")
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description="SensorBench local sensor test bench")
    sub = parser.add_subparsers(dest="command", required=True)
    for command, help_text in (("demo", "Load four deterministic simulator sessions"), ("serve", "Start the local dashboard and API"),
                               ("ingest", "Import a JSON session payload"), ("report", "Export a traceable Markdown report"),
                               ("replay", "Replay recorded telemetry offline; defaults to simulator"),
                               ("capture", "Capture an explicitly chosen serial port as hardware")):
        arg = sub.add_parser(command, help=help_text)
        arg.add_argument("--db", type=Path, default=DEFAULT_DB)
        if command == "serve":
            arg.add_argument("--port", type=int, default=8765)
        elif command == "ingest":
            arg.add_argument("input", type=Path)
        elif command == "report":
            arg.add_argument("run_id")
            arg.add_argument("--output", type=Path)
        elif command in ("replay", "capture"):
            if command == "replay":
                arg.add_argument("input", type=Path)
                arg.add_argument("--source", choices=("simulator", "hardware"), default="simulator")
            else:
                arg.add_argument("--port", required=True)
                arg.add_argument("--seconds", type=float, default=30)
                arg.add_argument("--baudrate", type=int, default=115200)
            arg.add_argument("--device", required=True, help="Device identity recorded with the run")
            arg.add_argument("--session", help="Optional explicit immutable session identity")
            arg.add_argument("--profile", choices=CAPTURE_PROFILES, default="monitoring")
            arg.add_argument("--output", type=Path, help="New JSON payload path; paired .raw.log is also saved")
    sub.add_parser("ports", help="Enumerate optional serial ports without opening them")
    args = parser.parse_args(argv)
    try:
        if args.command == "serve":
            if not 0 <= args.port <= 65535:
                raise InputError("port must be between 0 and 65535")
            server = make_server(args.db, port=args.port)
            print(f"SensorBench: http://127.0.0.1:{server.server_port}", flush=True)
            print("Local dashboard. Press Ctrl+C to stop.", flush=True)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
            finally:
                server.server_close()
        elif args.command == "ports":
            print(json.dumps(list_ports(), indent=2))
        elif args.command == "replay":
            result = replay_file(args.input, device_id=args.device, session_id=args.session,
                                 source=args.source, profile=args.profile, db=args.db, output=args.output)
            print(json.dumps(result, indent=2))
        elif args.command == "capture":
            result = capture_serial(args.port, device_id=args.device, seconds=args.seconds, session_id=args.session,
                                    profile=args.profile, baudrate=args.baudrate, db=args.db, output=args.output)
            print(json.dumps(result, indent=2))
        else:
            store = Store(args.db)
            if args.command == "demo":
                demo(store)
            elif args.command == "ingest":
                payload = strict_json(args.input.read_text(encoding="utf-8-sig"))
                print(json.dumps(store.ingest(payload), indent=2))
            else:
                report = store.markdown_report(args.run_id)
                if report is None:
                    raise InputError("Run not found")
                if args.output:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(report, encoding="utf-8")
                    print(str(args.output))
                else:
                    print(report)
    except (InputError, OSError) as exc:
        parser.exit(2, f"Error: {exc}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

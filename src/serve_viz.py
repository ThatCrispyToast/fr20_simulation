"""
Launch the three.js bin-reachability viewer.

The viewer (viz/index.html + viz/app.js) is a static page that, in the browser,
fetches a run's best_versions.json and the FR20 URDF + STL meshes from resources/.
Browsers block those fetches from file:// URLs, so this serves the repo root over
HTTP, then opens the viewer pointed at a run.

    uv run python src/serve_viz.py                 # newest out/run_* folder
    uv run python src/serve_viz.py out/run_xxxx     # a specific run
    uv run python src/serve_viz.py --no-browser     # just serve; open the URL yourself
    uv run python src/serve_viz.py --port 8123

Everything is served read-only; Ctrl-C stops the server.
"""
import os
import sys
import glob
import argparse
import functools
import threading
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

_HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(_HERE)              # served root (holds viz/, out/, resources/)
OUT_DIR = os.path.join(REPO_ROOT, "out")


def latest_run():
    """Newest out/run_*/ folder that actually has a best_versions.json, or None."""
    runs = sorted(glob.glob(os.path.join(OUT_DIR, "run_*", "best_versions.json")),
                  key=os.path.getmtime, reverse=True)
    return os.path.dirname(runs[0]) if runs else None


def resolve_run(arg):
    """Map a CLI run argument (folder or json path, absolute or relative) to the
    best_versions.json path *relative to the repo root* that the viewer fetches."""
    if arg is None:
        run = latest_run()
        if run is None:
            sys.exit(f"No runs with best_versions.json under {OUT_DIR}. "
                     f"Run src/bin_reach.py first.")
        jpath = os.path.join(run, "best_versions.json")
    else:
        jpath = arg if arg.endswith(".json") else os.path.join(arg, "best_versions.json")
        if not os.path.isabs(jpath):
            jpath = os.path.join(os.getcwd(), jpath)
        if not os.path.isfile(jpath):
            sys.exit(f"No best_versions.json at {jpath}")
    rel = os.path.relpath(os.path.abspath(jpath), REPO_ROOT)
    if rel.startswith(".."):
        sys.exit(f"Run {jpath} is outside the repo ({REPO_ROOT}); cannot serve it.")
    return rel.replace(os.sep, "/")


def main():
    ap = argparse.ArgumentParser(description="Serve the three.js reachability viewer.")
    ap.add_argument("run", nargs="?", help="run folder or best_versions.json "
                                           "(default: newest out/run_*)")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true", help="don't auto-open a browser")
    args = ap.parse_args()

    rel_json = resolve_run(args.run)

    # Bind the static server to the repo root so /viz, /out and /resources resolve.
    handler = functools.partial(SimpleHTTPRequestHandler, directory=REPO_ROOT)
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    except OSError as e:
        sys.exit(f"Could not bind port {args.port} ({e}). Try --port <other>.")
    port = httpd.server_address[1]
    url = f"http://localhost:{port}/viz/index.html?run={rel_json}"

    print(f"Serving {REPO_ROOT} at http://localhost:{port}")
    print(f"Viewer:  {url}")
    print("Ctrl-C to stop.")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped.")
        httpd.shutdown()


if __name__ == "__main__":
    main()

// Runs the Python physics engine off the main thread so the page stays responsive.
import { loadPyodide } from "../vendor/pyodide/pyodide.mjs";

const FILES = ["__init__.py", "params.py", "tyre.py", "bumps.py", "vehicle.py", "ride.py", "solver.py", "api.py"];
let api = null;

async function init() {
  const base = new URL("..", import.meta.url);
  const py = await loadPyodide({ indexURL: new URL("vendor/pyodide/", base).href });
  py.FS.mkdirTree("/app/fslap");
  await Promise.all(FILES.map(async (f) => {
    const txt = await (await fetch(new URL("fslap/" + f, base))).text();
    py.FS.writeFile("/app/fslap/" + f, txt);
  }));
  const tracks = await (await fetch(new URL("data/tracks.json", base))).text();
  py.runPython("import sys; sys.path.insert(0, '/app')\nfrom fslap import api");
  api = py.pyimport("fslap.api");
  api.load_tracks(tracks);
  postMessage({ type: "ready", schema: JSON.parse(api.schema()), tracks: JSON.parse(tracks).tracks });
}

onmessage = (e) => {
  const { id, req } = e.data;
  try {
    const t0 = performance.now();
    const result = JSON.parse(api.run(JSON.stringify(req)));
    postMessage({ type: "result", id, result, ms: performance.now() - t0 });
  } catch (err) {
    postMessage({ type: "error", id, message: String(err) });
  }
};

init().catch((err) => postMessage({ type: "fatal", message: String(err) }));

// Runs the Python physics engine off the main thread so the page stays responsive.
import { loadPyodide } from "../vendor/pyodide/pyodide.mjs";

const FILES = ["__init__.py", "params.py", "tyre.py", "thermal.py", "bumps.py", "vehicle.py", "ride.py", "solver.py", "api.py"];
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

// Requests wait in a queue. Two things keep the page quick without changing any result:
// 1. If a newer request from the form is already waiting, older ones are skipped
//    (their answers would be thrown away anyway).
// 2. The last few answers are kept, so asking the same question twice is instant
//    (for example Setup A when only Setup B changed, or switching back to a tab).
const queue = [], cache = new Map(), CACHE_MAX = 12;
let busy = false;
const fromForm = (id) => !String(id).startsWith("x:");

onmessage = (e) => { queue.push(e.data); if (!busy) { busy = true; setTimeout(drain, 0); } };

function drain() {
  const job = queue.shift();
  if (!job) { busy = false; return; }
  const { id, req } = job;
  if (!(fromForm(id) && queue.some((q) => fromForm(q.id)))) {
    try {
      const t0 = performance.now(), key = JSON.stringify(req);
      let text = cache.get(key);
      if (text === undefined) {
        text = api.run(key);
        cache.set(key, text);
        if (cache.size > CACHE_MAX) cache.delete(cache.keys().next().value);
      }
      postMessage({ type: "result", id, result: JSON.parse(text), ms: performance.now() - t0 });
    } catch (err) {
      postMessage({ type: "error", id, message: String(err) });
    }
  }
  setTimeout(drain, 0);
}

init().catch((err) => postMessage({ type: "fatal", message: String(err) }));

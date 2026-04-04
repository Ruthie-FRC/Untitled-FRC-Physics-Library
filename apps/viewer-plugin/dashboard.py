from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def _timeline_payload(timeline) -> dict[str, Any]:
    frames: list[dict[str, Any]] = []
    for frame in timeline.frames():
        frames.append(
            {
                "tick": frame.tick,
                "time_s": frame.time_s,
                "contacts": list(frame.contacts),
                "bodies": [
                    {
                        "name": body.name,
                        "x_m": body.position_m[0],
                        "y_m": body.position_m[1],
                        "vx_mps": body.velocity_mps[0],
                        "vy_mps": body.velocity_mps[1],
                        "radius_m": body.radius_m,
                        "color": body.color,
                    }
                    for body in frame.bodies
                ],
            }
        )
    return {"frames": frames}


def write_sim_dashboard_html(
    *,
    timeline,
    field_width_m: float,
    field_height_m: float,
    output_html: Path,
    bindings_html_name: str,
) -> None:
    payload = _timeline_payload(timeline)
    payload_json = json.dumps(payload, separators=(",", ":"))

    html = f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>RenSim Dashboard</title>
  <style>
    :root {{
      --bg: #f4f8fb;
      --panel: #ffffff;
      --ink: #1f2b38;
      --muted: #5f7387;
      --accent: #0a6e8a;
      --line: #c8d7e5;
      --warn: #b33a3a;
      --mono: "IBM Plex Mono", "Courier New", monospace;
      --sans: "IBM Plex Sans", "Segoe UI", sans-serif;
    }}
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; font-family: var(--sans); background: radial-gradient(1200px 700px at 20% 0%, #eaf3fb 0%, var(--bg) 55%); color: var(--ink); }}
    .app {{ display: grid; grid-template-columns: 320px 1fr; min-height: 100vh; gap: 12px; padding: 12px; }}
    .sidebar {{ background: var(--panel); border: 1px solid var(--line); border-radius: 14px; padding: 14px; }}
    .panel {{ background: var(--panel); border: 1px solid var(--line); border-radius: 14px; padding: 14px; }}
    h1 {{ margin: 0 0 10px 0; font-size: 1.1rem; letter-spacing: 0.02em; }}
    h2 {{ margin: 14px 0 8px 0; font-size: 0.95rem; }}
    label {{ display: block; margin: 6px 0; font-size: 0.9rem; }}
    input[type=checkbox] {{ margin-right: 6px; }}
    select, input[type=range] {{ width: 100%; margin: 6px 0; }}
    .kv {{ font-family: var(--mono); color: var(--muted); font-size: 0.86rem; }}
    .canvas-wrap {{ width: 100%; overflow: auto; border: 1px solid var(--line); border-radius: 10px; background: #fff; }}
    canvas {{ display: block; width: 100%; height: auto; }}
    .row {{ display: flex; justify-content: space-between; gap: 8px; align-items: center; }}
    .btn {{ display: inline-block; padding: 8px 10px; border: 1px solid var(--line); border-radius: 8px; color: var(--ink); text-decoration: none; font-size: 0.88rem; }}
    .warn {{ color: var(--warn); }}
    @media (max-width: 900px) {{
      .app {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <div class=\"app\">
    <aside class=\"sidebar\">
      <h1>Simulation Settings</h1>
      <div class=\"row\"><span>Quality</span>
        <select id=\"quality\">
          <option value=\"low\">Low</option>
          <option value=\"medium\" selected>Medium</option>
          <option value=\"high\">High</option>
        </select>
      </div>
      <h2>Display Toggles</h2>
      <label><input id=\"tog_grid\" type=\"checkbox\" checked />Grid</label>
      <label><input id=\"tog_paths\" type=\"checkbox\" checked />Trajectories</label>
      <label><input id=\"tog_vectors\" type=\"checkbox\" checked />Velocity Vectors</label>
      <label><input id=\"tog_labels\" type=\"checkbox\" checked />Body Labels</label>
      <label><input id=\"tog_contacts\" type=\"checkbox\" checked />Contact Text</label>
      <label><input id=\"tog_hud\" type=\"checkbox\" checked />HUD Overlay</label>
      <h2>Playback</h2>
      <label>Tick <input id=\"tick\" type=\"range\" min=\"0\" max=\"0\" value=\"0\" /></label>
      <div id=\"meta\" class=\"kv\"></div>
      <h2>Controls</h2>
      <a class=\"btn\" href=\"{bindings_html_name}\">Open Driver/Co-Driver Bindings</a>
      <p class=\"kv\">Bindings are editable and stored in browser localStorage.</p>
    </aside>
    <main class=\"panel\">
      <div class=\"canvas-wrap\"><canvas id=\"arena\" width=\"1200\" height=\"640\"></canvas></div>
      <p class=\"kv\">Top-down world frame (w), SI units. Rendering reflects current sidebar settings.</p>
    </main>
  </div>
<script>
const DATA = {payload_json};
const FIELD_W = {field_width_m};
const FIELD_H = {field_height_m};
const canvas = document.getElementById('arena');
const ctx = canvas.getContext('2d');
const tickEl = document.getElementById('tick');
const metaEl = document.getElementById('meta');

const qualityScale = {{ low: 0.7, medium: 1.0, high: 1.4 }};

function q(id) {{ return document.getElementById(id); }}
function mToPxX(xm) {{ return (xm / FIELD_W) * canvas.width; }}
function mToPxY(ym) {{ return canvas.height - (ym / FIELD_H) * canvas.height; }}

function drawGrid() {{
  ctx.strokeStyle = '#dbe6f0';
  ctx.lineWidth = 1;
  for (let x = 0; x <= 16; x++) {{
    const px = (x / 16) * canvas.width;
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, canvas.height); ctx.stroke();
  }}
  for (let y = 0; y <= 8; y++) {{
    const py = (y / 8) * canvas.height;
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(canvas.width, py); ctx.stroke();
  }}
}}

function drawTick(index) {{
  const frame = DATA.frames[index];
  const scale = qualityScale[q('quality').value] || 1.0;
  ctx.save();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#f8fcff';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  if (q('tog_grid').checked) drawGrid();

  if (q('tog_paths').checked) {{
    for (const body of frame.bodies) {{
      ctx.strokeStyle = body.color + '99';
      ctx.lineWidth = 1.2 * scale;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i <= index; i++) {{
        const fb = DATA.frames[i].bodies.find(b => b.name === body.name);
        if (!fb) continue;
        const px = mToPxX(fb.x_m), py = mToPxY(fb.y_m);
        if (!started) {{ ctx.moveTo(px, py); started = true; }} else {{ ctx.lineTo(px, py); }}
      }}
      ctx.stroke();
    }}
  }}

  for (const body of frame.bodies) {{
    const cx = mToPxX(body.x_m), cy = mToPxY(body.y_m);
    const r = (body.radius_m / FIELD_W) * canvas.width;

    ctx.fillStyle = body.color;
    ctx.beginPath(); ctx.arc(cx, cy, Math.max(4, r), 0, Math.PI * 2); ctx.fill();

    if (q('tog_vectors').checked) {{
      const vx = body.vx_mps * 5 * scale;
      const vy = -body.vy_mps * 5 * scale;
      ctx.strokeStyle = '#2c3e50';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + vx, cy + vy); ctx.stroke();
    }}

    if (q('tog_labels').checked) {{
      ctx.fillStyle = '#1f2b38';
      ctx.font = `${{Math.round(12 * scale)}}px sans-serif`;
      ctx.fillText(body.name, cx + 6, cy - 6);
    }}
  }}

  if (q('tog_hud').checked) {{
    ctx.fillStyle = '#203040';
    ctx.font = `${{Math.round(12 * scale)}}px monospace`;
    ctx.fillText(`t=${{frame.time_s.toFixed(3)}}s tick=${{String(frame.tick).padStart(4,'0')}} contacts=${{frame.contacts.length}}`, 14, 22);
  }}

  if (q('tog_contacts').checked && frame.contacts.length > 0) {{
    ctx.fillStyle = '#b33a3a';
    ctx.font = `${{Math.round(11 * scale)}}px monospace`;
    ctx.fillText(`contact: ${{frame.contacts[0][0]}} <-> ${{frame.contacts[0][1]}}`, 14, canvas.height - 12);
  }}
  ctx.restore();

  metaEl.textContent = `frames=${{DATA.frames.length}} time=${{frame.time_s.toFixed(3)}}s contacts=${{frame.contacts.length}} quality=${{q('quality').value}}`;
}}

function bindUi() {{
  tickEl.max = String(Math.max(0, DATA.frames.length - 1));
  tickEl.value = tickEl.max;
  const rerender = () => drawTick(Number(tickEl.value));
  tickEl.addEventListener('input', rerender);
  ['quality','tog_grid','tog_paths','tog_vectors','tog_labels','tog_contacts','tog_hud'].forEach(id => q(id).addEventListener('change', rerender));
  rerender();
}}

bindUi();
</script>
</body>
</html>
"""
    output_html.write_text(html, encoding="utf-8")


def write_control_bindings_html(*, output_html: Path) -> None:
    html = """<!doctype html>
<html lang=\"en\"><head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>Driver and Co-Driver Bindings</title>
  <style>
    body { font-family: "IBM Plex Sans", "Segoe UI", sans-serif; margin: 0; background: #f4f8fb; color: #1f2b38; }
    .wrap { max-width: 920px; margin: 20px auto; background: #fff; border: 1px solid #c8d7e5; border-radius: 12px; padding: 16px; }
    table { width: 100%; border-collapse: collapse; }
    th, td { border-bottom: 1px solid #e2ebf3; padding: 8px; text-align: left; font-size: 0.92rem; }
    input { width: 100%; padding: 6px; border: 1px solid #b9cadb; border-radius: 6px; }
    .row { display: flex; gap: 8px; margin-top: 12px; }
    button, a { padding: 8px 10px; border: 1px solid #b9cadb; border-radius: 8px; text-decoration: none; color: #1f2b38; background: #fff; cursor: pointer; }
    .mono { font-family: "IBM Plex Mono", "Courier New", monospace; color: #52697d; font-size: 0.85rem; }
  </style>
</head><body>
<div class=\"wrap\">
  <h1>Driver and Co-Driver Control Bindings</h1>
  <p class=\"mono\">Edit any binding and click save. Bindings persist via localStorage for quick reassignment.</p>
  <table id=\"tbl\">
    <thead><tr><th>Role</th><th>Action</th><th>Button / Axis</th></tr></thead>
    <tbody></tbody>
  </table>
  <div class=\"row\">
    <button id=\"save\">Save Bindings</button>
    <button id=\"reset\">Reset Defaults</button>
    <a href=\"dashboard.html\">Back to Dashboard</a>
  </div>
</div>
<script>
const KEY = 'rensim.controlBindings.v1';
const defaults = [
  { role: 'driver', action: 'Drive X/Y', binding: 'left_stick_xy' },
  { role: 'driver', action: 'Rotate', binding: 'right_stick_x' },
  { role: 'driver', action: 'Intake', binding: 'RT' },
  { role: 'driver', action: 'Outtake', binding: 'LT' },
  { role: 'driver', action: 'Zero Gyro', binding: 'Y' },
  { role: 'codriver', action: 'Arm Up', binding: 'left_stick_y' },
  { role: 'codriver', action: 'Shoot', binding: 'RB' },
  { role: 'codriver', action: 'Climb', binding: 'A' }
];

function load() {
  const raw = localStorage.getItem(KEY);
  if (!raw) return defaults;
  try { return JSON.parse(raw); } catch { return defaults; }
}

function render(rows) {
  const tbody = document.querySelector('#tbl tbody');
  tbody.innerHTML = '';
  rows.forEach((row, idx) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${row.role}</td><td>${row.action}</td><td><input data-idx="${idx}" value="${row.binding}" /></td>`;
    tbody.appendChild(tr);
  });
}

function captureRows() {
  const rows = load().map(x => ({ ...x }));
  document.querySelectorAll('input[data-idx]').forEach(input => {
    const idx = Number(input.getAttribute('data-idx'));
    rows[idx].binding = input.value.trim();
  });
  return rows;
}

document.getElementById('save').addEventListener('click', () => {
  const rows = captureRows();
  localStorage.setItem(KEY, JSON.stringify(rows));
  alert('Bindings saved.');
});

document.getElementById('reset').addEventListener('click', () => {
  localStorage.setItem(KEY, JSON.stringify(defaults));
  render(defaults);
});

render(load());
</script>
</body></html>
"""
    output_html.write_text(html, encoding="utf-8")

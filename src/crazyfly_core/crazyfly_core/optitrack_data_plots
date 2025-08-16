import csv, re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from pathlib import Path

# ---------- Paths ----------
base = Path.home() / "Documents/GitHub/crazyflie_codebase"
stats_dir = base / "optitrack_data_stats"
raw_dir   = base / "optitrack_data_raw"

# ---------- Config ----------
Y_LEVELS = [0.0, 0.5, 1.0, 1.3]  # one figure per y
Y_TOL = 0.15                     # tolerance to bucket a y value
K = 2.0                          # k-sigma ellipse size (2σ)

# ---------- Helpers ----------
ts_re = re.compile(r'(\d{4}-\d{2}-\d{2}_\d{2}:\d{2}:\d{2})')
def ts(name: str):
    m = ts_re.search(name); return m.group(1) if m else None

def first_row_csv(path: Path):
    import csv
    with path.open(newline='') as f:
        row = next(csv.DictReader(f), None)
        return {k.strip().lower(): (v.strip() if v is not None else v) for k, v in row.items()} if row else None

def iter_rows_csv(path: Path):
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            yield {k.strip().lower(): (v.strip() if v is not None else v) for k, v in row.items()}

def y_bucket(y):
    nearest = min(Y_LEVELS, key=lambda v: abs(v - y))
    return nearest if abs(nearest - y) <= Y_TOL else None

# ---------- Pair files by timestamp ----------
stats_files = list(stats_dir.glob("optitrack_data_2025*.csv"))
raw_files   = list(raw_dir.glob("optitrack_test_2025*.csv"))

pairs = {}
for p in stats_files:
    t = ts(p.name)
    if t: pairs.setdefault(t, {})["stats"] = p
for p in raw_files:
    t = ts(p.name)
    if t: pairs.setdefault(t, {})["raw"] = p

pairs = {t: pr for t, pr in pairs.items() if "stats" in pr and "raw" in pr}
print(f"[INFO] Paired sessions: {len(pairs)}")

# ---------- Group data by y level ----------
groups = {y: {"true": [], "raw_x": [], "raw_z": [], "ells": []} for y in Y_LEVELS}
all_x, all_z = [], []

for t, pr in sorted(pairs.items()):
    sr = first_row_csv(pr["stats"])
    if not sr or not {"x","y","z"} <= set(sr.keys()):
        continue
    try:
        tx, ty, tz = float(sr["x"]), float(sr["y"]), float(sr["z"])
    except ValueError:
        continue

    yk = y_bucket(ty)
    if yk is None:
        continue

    xs, zs = [], []
    for r in iter_rows_csv(pr["raw"]):
        if "x" in r and "z" in r:
            try:
                xs.append(float(r["x"]))
                zs.append(float(r["z"]))
            except ValueError:
                pass
    if len(xs) < 2:
        continue

    xs, zs = np.array(xs), np.array(zs)
    dx, dz = xs - tx, zs - tz

    # covariance -> tilted ellipse
    cov = np.cov(np.vstack([dx, dz]))
    vals, vecs = np.linalg.eigh(cov)
    order = np.argsort(vals)[::-1]
    vals, vecs = vals[order], vecs[:, order]
    s1, s2 = np.sqrt(max(vals[0], 0.0)), np.sqrt(max(vals[1], 0.0))
    width, height = 2*K*s1, 2*K*s2
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))

    mx, mz = float(np.mean(xs)), float(np.mean(zs))  # center at measurement mean
    center = (mx, mz)  # change to (tx, tz) to center on the true grid point

    groups[yk]["true"].append((tx, tz))
    groups[yk]["raw_x"].extend(xs.tolist())
    groups[yk]["raw_z"].extend(zs.tolist())
    groups[yk]["ells"].append((center[0], center[1], width, height, angle, (tx, tz), (mx, mz)))

    all_x.extend(xs.tolist()); all_z.extend(zs.tolist())

# ---------- Global axis limits (same scale across pages) ----------
if all_x and all_z:
    xmin, xmax = min(all_x), max(all_x)
    zmin, zmax = min(all_z), max(all_z)
    pad_x = 0.05 * max(1e-9, xmax - xmin)
    pad_z = 0.05 * max(1e-9, zmax - zmin)
    XLIM = (xmin - pad_x, xmax + pad_x)
    ZLIM = (zmin - pad_z, zmax + pad_z)
else:
    XLIM = ZLIM = None

# ---------- One window per y (blocks until closed) ----------
for y in Y_LEVELS:
    g = groups[y]
    fig, ax = plt.subplots(figsize=(9, 9))

    txs = [p[0] for p in g["true"]]
    tzs = [p[1] for p in g["true"]]
    if txs:
        ax.scatter(txs, tzs, c='red', marker='s', s=25, label='True Grid')
    if g["raw_x"]:
        ax.scatter(g["raw_x"], g["raw_z"], s=6, alpha=0.15, label='Raw Samples')

    for cx, cz, w, h, ang, (tx, tz), (mx, mz) in g["ells"]:
        ell = Ellipse((cx, cz), width=w, height=h, angle=ang,
                      edgecolor='green', facecolor='none', lw=1.2, alpha=0.9)
        ax.add_patch(ell)
        ax.plot([tx, mx], [tz, mz], ls='--', lw=1, alpha=0.6)  # bias vector (mean→true)

    ax.set_title(f"Per-Point 2σ Error Ellipses — y = {y} m (points: {len(g['ells'])})")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Z (m)")
    ax.grid(True); ax.set_aspect('equal', adjustable='box')
    if XLIM: ax.set_xlim(XLIM)
    if ZLIM: ax.set_ylim(ZLIM)

    plt.show()       # <-- blocks until you close the window
    plt.close(fig)   # free memory before next page
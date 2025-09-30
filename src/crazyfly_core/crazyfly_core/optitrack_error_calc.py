import os, glob, csv, math

stats_dir = os.path.expanduser("~/Desktop/crazyflie_codebase/optitrack_data_stats")
# print("Stats dir:", stats_dir)
raw_dir   = os.path.expanduser("~/Desktop/crazyflie_codebase/optitrack_data_raw")
# print("raw dir:",raw_dir)

stats_files = sorted(glob.glob(os.path.join(stats_dir, "optitrack_data_*.csv")))
raw_files   = sorted(glob.glob(os.path.join(raw_dir,   "optitrack_test_*.csv")))

if not stats_files or not raw_files or len(stats_files) != len(raw_files):
    raise SystemExit("File count mismatch or none found. Check paths and naming.")

AXES_POS = ("x", "y", "z")
AXES_ANG = ("yaw", "pitch", "roll")
AXES_ALL = AXES_POS + AXES_ANG

def read_single_xyz(path):
    """Return {'x': float|None, 'y': float|None, 'z': float|None} from first stats row."""
    out = {k: None for k in AXES_POS}
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        row = next(r, None)
        if not row:
            return out
        for k in AXES_POS:
            val = row.get(k, "")
            if val not in (None, ""):
                try:
                    out[k] = float(val)
                except ValueError:
                    pass
    return out

def read_series(path, axes):
    """Return {axis: [floats]} for all requested axes from raw series file."""
    out = {k: [] for k in axes}
    with open(path, newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            for k in axes:
                val = row.get(k, "")
                if val not in (None, ""):
                    try:
                        out[k].append(float(val))
                    except ValueError:
                        pass
    return out

# Accumulators
total_error  = {k: 0.0 for k in AXES_ALL}  # sum of (desired - raw)
total_points = {k: 0   for k in AXES_ALL}

for stats_file, raw_file in zip(stats_files, raw_files):
    s = read_single_xyz(stats_file)            # desired for x,y,z (single values)
    for k in AXES_ANG:                         # desired=0 for yaw/pitch/roll
        s[k] = 0.0

    r = read_series(raw_file, AXES_ALL)        # ~511 values per axis in raw

    for k in AXES_ALL:
        series = r[k]
        desired = s.get(k, None)
        if desired is None or not series:
            continue
        total_error[k]  += math.fsum(desired - rv for rv in series)
        total_points[k] += len(series)

mean_error = {
    k: (total_error[k] / total_points[k]) if total_points[k] else None
    for k in AXES_ALL
}

# print("Total points:", total_points)
# print("Total summed error (desired - raw):", total_error)
print("Mean error per axis:", mean_error)

# CORRECTED: Max/Min ERROR calculation (not raw values)
max_abs_error = {k: -float("inf") for k in AXES_POS}
min_abs_error = {k: float("inf") for k in AXES_POS}
max_error_files = {k: None for k in AXES_POS}
min_error_files = {k: None for k in AXES_POS}

for stats_file, raw_file in zip(stats_files, raw_files):
    # Get desired values for this file
    desired_vals = read_single_xyz(stats_file)
    
    # Get raw measurement series
    raw_vals = read_series(raw_file, AXES_POS)
    
    for axis in AXES_POS:
        desired = desired_vals.get(axis, None)
        measurements = raw_vals[axis]
        
        if desired is None or not measurements:
            continue
            
        # Calculate errors for all measurements in this file
        errors = [abs(desired - measured) for measured in measurements]
        
        # Find max and min absolute errors in this file
        local_max_error = max(errors)
        local_min_error = min(errors)
        
        # Update global max
        if local_max_error > max_abs_error[axis]:
            max_abs_error[axis] = local_max_error
            max_error_files[axis] = raw_file
            
        # Update global min
        if local_min_error < min_abs_error[axis]:
            min_abs_error[axis] = local_min_error
            min_error_files[axis] = raw_file


print("----------------------------------------------------")
for axis in AXES_POS:
    print(f"Max {axis} error: {max_abs_error[axis]:.6f} (file: {os.path.basename(max_error_files[axis])})")
    print(f"Min {axis} error: {min_abs_error[axis]:.6f} (file: {os.path.basename(min_error_files[axis])})")
print("----------------------------------------------------")


#directory = os.path.expanduser('~/Desktop/crazyflie_codebase/cluster_data/cluster_hover')
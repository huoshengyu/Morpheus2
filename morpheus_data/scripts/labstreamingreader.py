#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# Morpheus2: ROS2 control and feedback for manual teleoperation research
# Copyright (C) 2026 Brian Huo
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# ---------------------------------------------------------------------------

import os
import pyxdf
import numpy as np
import pandas as pd

# ---- XDF path ----
xdf_path = "/root/catkin_ws/src/morpheus_data/data/sub-P001/ses-S001/eeg/sub-P001/ses-S001/eeg/sub-P001_ses-S001_task-Default_run-001_eeg.xdf"
print(f"Loading XDF: {xdf_path}")
data, header = pyxdf.load_xdf(xdf_path)

# ---- Prepare all streams ----
dfs = []
for i, stream in enumerate(data):
    name = stream["info"]["name"][0] if stream["info"]["name"] else f"stream_{i}"
    ts = np.array(stream["time_stamps"])
    x = np.array(stream["time_series"])

    if x.size == 0:
        print(f"[{i}] {name}: empty stream, skipped.")
        continue

    # Handle multidimensional streams
    n_ch = x.shape[1] if x.ndim > 1 else 1
    df = pd.DataFrame(x, columns=[f"{name}_ch{j+1}" for j in range(n_ch)])
    df["timestamp"] = ts
    df.set_index("timestamp", inplace=True)
    dfs.append(df)
    print(f"[{i}] {name}: {x.shape}")

# ---- Merge all by timestamp ----
if not dfs:
    raise ValueError("No valid streams found in the XDF file.")

# Merge using outer join on timestamps
merged = pd.concat(dfs, axis=1).sort_index()

# ---- Save to CSV ----
csv_path = os.path.splitext(xdf_path)[0] + "_ALL_STREAMS.csv"
merged.to_csv(csv_path, index_label="timestamp")

print(f"\nAll streams merged and saved to:\n{csv_path}")
print(f"Shape: {merged.shape}")

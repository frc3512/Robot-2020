#!/usr/bin/env python3
"""Finds latest versions of the CSVs for each subsystem, then plots the data."""
import matplotlib.pyplot as plt
import numpy as np
import os
import re


def num_lines(csv_group):
    with open(csv_group) as f:
        i = 0
        for i, l in enumerate(f):
            pass
    return i + 1


# Get list of files in current directory
files = [os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn]

# Maps subsystem name to tuple of csv_group and date
filtered = {}
file_rgx = re.compile(
    r"^\./(?P<name>[A-Za-z ]+)-(?P<date>\d{4}-\d{2}-\d{2}-\d{2}_\d{2}_\d{2})\.csv$"
)
for f in files:
    match = file_rgx.search(f)
    if not match:
        continue

    # If file is empty or only has header (that is, has no data), ignore it. We
    # ignore the case of one line of data because it might be truncated.
    if num_lines(f) <= 2:
        continue

    # If the file is a CSV with the correct name pattern, add it to the filtered
    # list. Files with newer dates override old ones in lexographic ordering.
    name = match.group("name")
    if name not in filtered or filtered[name] < name:
        filtered[name] = f

# Plot datasets
csv_group = "Drivetrain Positions"
plt.figure()
plt.title(csv_group)
filename = filtered[csv_group]

# Get labels from first row of file
with open(filename) as f:
    labels = [x.strip('"') for x in f.readline().rstrip().split(",")]

# Retrieve data from remaining rows of file
print(f"Plotting {filename}")
data = np.genfromtxt(filename, delimiter=",", skip_header=1, skip_footer=1)
plt.plot(data[:, 1], data[:, 2])
plt.plot(data[:, 4], data[:, 5])

# First label is x axis label (time). The remainder are dataset names.
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend(["Estimate", "Reference"])
plt.show()

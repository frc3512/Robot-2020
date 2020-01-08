#!/usr/bin/env python3
"""Finds latest versions of the CSVs for each subsystem, then plots the data."""

import matplotlib.pyplot as plt
import numpy as np
import os
import re


def num_lines(filename):
    with open(filename) as f:
        i = 0
        for i, l in enumerate(f):
            pass
    return i + 1


# Get list of files in current directory
files = [os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn]

# Maps subsystem name to tuple of filename and number suffix
filtered = {}

file_rgx = re.compile(r"^\./(?P<name>[A-Za-z]+)\.csv(?P<num>[0-9]*)$")
for f in files:
    match = file_rgx.search(f)
    if not match:
        continue

    # If file is empty or only has header (that is, has no data), ignore it. We
    # ignore the case of one line of data because it might be truncated.
    if num_lines(f) <= 2:
        continue

    # If the file is a CSV with the correct name pattern, add it to the filtered
    # list
    name = match.group("name")
    num = int(match.group("num"))
    if name not in filtered or filtered[name][1] < num:
        filtered[name] = (f, num)

for key in filtered.keys():
    plt.figure()
    plt.title(key)
    name = filtered[key][0]

    # Get labels from first row of file
    with open(name) as f:
        labels = f.readline().rstrip().split(",")

    # Retrieve data from remaining rows of file
    print(f"Plotting {name}")
    data = np.genfromtxt(name, delimiter=",", skip_header=1, skip_footer=1)
    plt.plot(data[:, 0], data[:, 1:])

    # First label is x axis label (time). The remainder are dataset names.
    plt.xlabel(labels[0])
    plt.legend(labels[1:])
plt.show()

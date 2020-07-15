#!/usr/bin/env python3
"""Finds latest versions of the CSVs for each subsystem, then plots the time
domain and X-Y data.

If provided, the first argument to this script is a filename regex that
restricts which CSVs are plotted to those that match the regex.
"""

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import re
import sys

plt.rcParams.update({'figure.max_open_warning': 0})

prefix = "./build/test-results/frcUserProgramTest/linuxx86-64/release"
postfix = r"-(?P<date>\d{4}-\d{2}-\d{2}-\d{2}_\d{2}_\d{2})\.csv"
unit_rgx = re.compile(r"^(?P<name>[\w\- ]+) \((?P<unit>.*?)\)$")


def num_lines(filename):
    with open(filename) as f:
        i = 0
        for i, l in enumerate(f):
            pass
    return i + 1


def get_file_list():
    # Get list of files in current directory
    files = [os.path.join(dp, f) for dp, dn, fn in os.walk(prefix) for f in fn]
    files += [os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn]

    # Ignore files not matching optional pattern
    if len(sys.argv) > 1:
        files = [f for f in files if re.search(sys.argv[1], f)]

    # Maps subsystem name to tuple of csv_group and date
    filtered = {}
    file_rgx = re.compile(r"^" + re.escape(prefix) + r"/(?P<name>[A-Za-z ]+)" +
                          postfix + r"$")
    for f in files:
        match = file_rgx.search(f)
        if not match:
            continue

        # If file is empty or only has header (that is, has no data), ignore it.
        # We ignore the case of one line of data because it might be truncated.
        if num_lines(f) <= 2:
            continue

        # If the file is a CSV with the correct name pattern, add it to the
        # filtered list. Files with newer dates override old ones in lexographic
        # ordering.
        name = match.group("name")
        date = match.group("date")
        if name not in filtered.keys() or filtered[name] < date:
            filtered[name] = date

    # Make filtered list of files
    files = []
    for name_stub in filtered.keys():
        files.append(
            os.path.join(prefix, name_stub) + "-" + filtered[name_stub] +
            ".csv")

    return files


def make_groups(files):
    # Group files by category (sets of files with states, inputs, or outputs
    # suffixes and the same name stub like "Flywheel")
    category_rgx = re.compile(
        r"^" + re.escape(prefix) +
        r"/(?P<category>[A-Za-z ]+) (states|inputs|outputs)" + postfix + r"$")
    file_groups = {}
    if files:
        print("Loading CSVs...")
    else:
        print("No data to plot.")

    # Sorting the file list puts files into the order ["inputs", "outputs",
    # "states". This means data series will be loaded in the order of "inputs,
    # outputs, references, states] (references are logged before states). This
    # produces the desired dataset layering on plots.
    for f in sorted(files):
        print(f"  {os.path.split(f)[1]}")

        match = category_rgx.search(f)

        if not match:
            # Couldn't find the file's category, so put it in its own category
            substr = os.path.split(f)[1]
            substr = substr[:substr.find("-")]
            file_groups[substr] = [f]
        else:
            category = match.group("category")

            # Create a new category if one doesn't exist, or add the file to the
            # existing category if it does
            if category not in file_groups.keys():
                file_groups[category] = [f]
            else:
                file_groups[category].append(f)
    return file_groups


file_groups = make_groups(get_file_list())
if file_groups:
    print("Plotting...")

# Within each group, make groups of datasets keyed on their unit, then plot each
# group on their own figure
for category, file_group in file_groups.items():
    unit_groups = {}
    name_groups = {}
    for filename in file_group:
        # Get labels from first row of file
        with open(filename) as f:
            labels = [x.strip('"') for x in f.readline().rstrip().split(",")]

        # Retrieve data from remaining rows of file. "skip_footer=1" skips the
        # last line because it may be incompletely written.
        data = np.genfromtxt(filename,
                             delimiter=",",
                             skip_header=1,
                             skip_footer=1)

        times = data[:, 0:1]

        # Skips label in first column because that's always "Time (s)"
        for i, label in enumerate(labels[1:]):
            match = unit_rgx.search(label)
            name = match.group("name")
            unit = match.group("unit")

            if unit not in unit_groups.keys():
                # Make a new unit group. Tuple entries are as follows:
                # 1. time data column
                # 2. list of data columns
                # 3. list of data labels
                unit_groups[unit] = (times, [], [])
            # "i + 1" skips the time data column
            unit_groups[unit][1].append(data[:, i + 1:i + 2])
            unit_groups[unit][2].append(name)

            # "i + 1" skips the time data column
            name_groups[name] = data[:, i + 1:i + 2]

    # Plot time domain datasets
    print(f'  [vs time] {category} ({", ".join(unit_groups.keys())})')
    for unit, data_tups in unit_groups.items():
        plt.figure()
        plt.title(f"{category} ({unit})")

        for i in range(len(data_tups[1])):
            plt.plot(data_tups[0], data_tups[1][i])

        # First label is x axis label (time). The remainder are dataset names.
        plt.xlabel("Time (s)")
        plt.ylabel(f"Data ({unit})")
        plt.legend(data_tups[2])

    # Plot X-Y datasets. If the file doesn't have all the required keys, skip
    # it.
    if not (set(["X reference", "Y reference", "X estimate", "Y estimate"]) -
            set(name_groups.keys())):
        print(f'  [y vs x] {category}')
        plt.figure()
        plt.title(f"{category} trajectory")

        plt.plot(name_groups["X reference"], name_groups["Y reference"])
        plt.plot(name_groups["X estimate"], name_groups["Y estimate"])

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.legend(["Reference", "Estimate"])

        # This equalizes the X and Y axes so the trajectories aren't warped
        plt.axis("equal")
plt.show()

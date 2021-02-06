#!/usr/bin/env python3

import argparse
import math
import os
import re
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MultipleLocator

from linear_filter import LinearFilter


def num_lines(filename):
    with open(filename) as f:
        i = 0
        for i, l in enumerate(f):
            pass
    return i + 1


def get_file_list(regex):
    # Get list of files in current directory
    files = [
        os.path.join(dp, f)
        for dp, dn, fn in os.walk(".")
        for f in fn
        if f.endswith(".csv") and "Schedule" in f
    ]

    # Ignore files not matching optional pattern
    if regex:
        files = [f for f in files if re.search(regex, f)]

    return files


def plot_broken_barh(ax, xranges, y):
    """
    Keyword arguments:
    xranges -- list of tuples of xstart and box width
    y -- y position of box's center
    """
    HEIGHT = 0.9
    ax.broken_barh(xranges, (y - HEIGHT / 2, HEIGHT))


def gantt(names, datasets):
    fig, ax = plt.subplots()

    ax.set_xlabel("Time (s)")
    ax.xaxis.set_major_locator(MultipleLocator(1))
    ax.xaxis.set_minor_locator(MultipleLocator(0.1))
    ax.set_yticks(range(1, len(names) + 1))
    ax.grid(True)
    ax.set_axisbelow(True)
    ax.set_yticklabels(names)
    plt.setp(ax.get_yticklabels(),
             rotation=45,
             ha="right",
             rotation_mode="anchor")

    for i, name in enumerate(names):
        plot_broken_barh(ax, datasets[name], i + 1)


def stackplot(names, epochs, datasets, taps):
    fig, ax = plt.subplots()

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Loop duration (ms)")
    ax.grid(True)
    ax.set_axisbelow(True)

    values = [datasets[name] for name in names]
    durations = np.array([values[0]])
    for i in range(1, len(values)):
        filter = LinearFilter.moving_average(taps)
        durations = np.concatenate(
            (durations, np.array([[filter.calculate(val) for val in values[i]]
                                 ])))

    ax.stackplot(epochs, durations, labels=names)
    ax.legend(names)


def plot(names, epochs, datasets, taps):
    fig, ax = plt.subplots()

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Loop duration (ms)")
    ax.grid(True)
    ax.set_axisbelow(True)

    for name in names:
        filter = LinearFilter.moving_average(taps)
        ax.plot(epochs, [filter.calculate(val) for val in datasets[name]],
                label=name)
    ax.legend(names)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-taps",
                        dest="taps",
                        type=int,
                        default=1,
                        help="Number of taps for filtering (default: 1)")
    parser.add_argument("regex", nargs="?")
    args = parser.parse_args()

    times = []
    starts = []
    durations = []
    names = []

    files = get_file_list(args.regex)
    if len(files):
        filename = files[0]
    else:
        print("No data to plot.")
    print("Loading CSVs...")
    print(f"  {filename}")
    with open(filename) as f:
        # Skip header
        next(f)

        for line in f:
            time, start, duration, name = line.rstrip().split(",")
            times.append(float(time))
            starts.append(float(start))
            durations.append(float(duration))
            names.append(name.strip('"'))

    dataset_names = []
    for name in names:
        if name not in dataset_names:
            dataset_names.append(name)

    # Dictionary from name to list of (x_start, x_duration) tuples
    datasets = {}
    for row in range(len(times)):
        name = names[row]
        if name not in datasets.keys():
            datasets[name] = [(starts[row], durations[row])]
        else:
            datasets[name].append((starts[row], durations[row]))
    gantt(dataset_names, datasets)

    # Dictionary from name to durations
    datasets = {}
    for name in names:
        datasets[name] = []
    epochs = []

    current_time = -1.0
    for row in range(len(times)):
        # If in a new epoch, pad unrecorded ones with zero durations
        if current_time != times[row]:
            current_time = times[row]
            epochs.append(current_time)

            # Give names that weren't recorded in this epoch a duration of zero
            for key in datasets.keys():
                while len(datasets[key]) < len(epochs):
                    datasets[key].append(0.0)

        # Either append data for a new epoch or replace the existing data
        name = names[row]
        if len(datasets[name]) < len(epochs):
            datasets[name].append(durations[row] * 1e3)
        else:
            datasets[name][-1] = durations[row] * 1e3

    # Give names that weren't recorded in this epoch a duration of zero
    for key in datasets.keys():
        while len(datasets[key]) < len(epochs):
            datasets[key].append(0.0)

    stackplot(dataset_names, epochs, datasets, args.taps)
    plot(dataset_names, epochs, datasets, args.taps)

    plt.show()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Deletes all but newest versions of CSV log files."""

import argparse
import json
import os
import re
import subprocess

import frcutils

parser = argparse.ArgumentParser(description="Deletes CSVs from the roboRIO")
parser.add_argument(
    "--all",
    action="store_true",
    help=
    "if specified, deletes all CSV files instead of leaving the most recent ones.",
)
args = parser.parse_args()

ip = frcutils.get_roborio_ip()

# Get list of files in log directory. If storage is mounted in /media, assume
# the logs are there.
dirs = (subprocess.check_output(["ssh", f"lvuser@{ip}", "ls", "/media"],
                                encoding="utf-8").rstrip().split("\n"))
if len(dirs) > 0:
    files = (subprocess.check_output(
        ["ssh", f"lvuser@{ip}", "ls", f"/media/{dirs[0]}"],
        encoding="utf-8").rstrip().split("\n"))
else:
    files = (subprocess.check_output(["ssh", f"lvuser@{ip}", "ls"],
                                     encoding="utf-8").rstrip().split("\n"))

# Maps subsystem name to tuple of csv_group and date and filters for CSV files
filtered = {}
file_rgx = re.compile(
    r"^(?P<name>[A-Za-z ]+)-(?P<date>\d{4}-\d{2}-\d{2}-\d{2}_\d{2}_\d{2})\.csv$"
)
files = [f for f in files if file_rgx.search(f)]

for f in files:
    match = file_rgx.search(f)
    # If the file is a CSV with the correct name pattern, add it to the filtered
    # list. Files with newer dates override old ones in lexographic ordering.
    name = match.group("name")
    date = match.group("date")
    if name not in filtered.keys() or filtered[name] < date:
        filtered[name] = date

# Makes list of files to delete
csvs_to_delete = []
if args.all:
    csvs_to_delete = files
else:
    for f in files:
        match = file_rgx.search(f)
        name = match.group("name")
        if name + "-" + filtered[name] + ".csv" != f:
            csvs_to_delete.append(f)

# Add quotes around filenames so rm doesn't split them apart
for i in range(len(csvs_to_delete)):
    # If storage is mounted in /media, delete the files there
    if len(dirs) > 0:
        csvs_to_delete[i] = f"'/media/{dirs[0]}/{csvs_to_delete[i]}'"
    else:
        csvs_to_delete[i] = f"'{csvs_to_delete[i]}'"

# Delete list of files from roboRIO
subprocess.run(["ssh", f"lvuser@{ip}", "rm", "-f", " ".join(csvs_to_delete)])

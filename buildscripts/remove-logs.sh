#!/bin/bash
find . -type f \( -name \*\.csv -o -name Robot\.log \) -exec rm -f {} \;

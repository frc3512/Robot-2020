#!/bin/bash
set -e

./make.py clean
mkdir -p build/linuxx86-64
~/git/ClangBuildAnalyzer/build/ClangBuildAnalyzer --start build/linuxx86-64
./make.py test -j4
~/git/ClangBuildAnalyzer/build/ClangBuildAnalyzer --stop build/linuxx86-64 Robot-2020-capture.bin
~/git/ClangBuildAnalyzer/build/ClangBuildAnalyzer --analyze Robot-2020-capture.bin > Robot-2020-analysis.txt

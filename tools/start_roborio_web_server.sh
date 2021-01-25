#!/bin/bash
set -e

# Restart the NI web server so the roboRIO can be reimaged
ssh admin@10.35.12.2 /usr/local/natinst/etc/init.d/systemWebServer start

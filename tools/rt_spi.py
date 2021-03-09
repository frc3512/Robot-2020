#!/usr/bin/env python3

import subprocess


def main():
    # Give NI SPI driver RT priority of 33
    subprocess.check_output([
        "ssh", "admin@10.35.12.2",
        "ps -ef | grep '\[spi0\]' | awk '{print $1}' | xargs chrt -f -p 33"
    ])


if __name__ == "__main__":
    main()

#!/bin/sh
mknod -m 666 /dev/ttyS51 c 4 115
socat PTY,link=/dev/ttyS51, TCP4:192.168.0.70:10002


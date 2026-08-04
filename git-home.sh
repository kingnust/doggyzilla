#!/bin/sh

exec git --git-dir=/home/pi/.dogzilla_new.git --work-tree=/home/pi "$@"

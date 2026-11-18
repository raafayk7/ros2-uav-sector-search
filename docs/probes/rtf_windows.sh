#!/bin/bash
# RTF as sim-time change over real-time change in three 30 s /stats windows
# (docker-notes gotcha 54). Run in the container: bash -l rtf_windows.sh LABEL
stat() { gz topic -e -t /stats -n 1 | awk '/^sim_time/{f="s"} /^real_time {/{f="r"} /^step_size/{f=""} /^ *sec:/{if(f=="s")s=$2; if(f=="r")r=$2} /^ *nsec:/{if(f=="s")sn=$2; if(f=="r")rn=$2} END{printf "%.3f %.3f\n", s+sn/1e9, r+rn/1e9}'; }
win() { a=$(stat); sleep 30; b=$(stat); echo "$a $b" | awk '{printf "%.3f\n", ($3-$1)/($4-$2)}'; }
echo "$1:"; for i in 1 2 3; do win; done

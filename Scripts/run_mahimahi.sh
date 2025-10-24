#!/bin/bash
rm -rf mm_time
date +%s%N >> mm_time
up_file=mahi_traces/mh-120-240
down_file=mahi_traces/mh-120-240
# up_file=/home/lei/mahimahi/traces/Verizon-LTE-driving.up
# down_file=/home/lei/mahimahi/traces/Verizon-LTE-driving.down
echo $up_file >> mm_time
echo $down_file >> mm_time
mm-link $up_file $down_file --meter-uplink --meter-downlink
#!/bin/bash

# Start the capture
datetime=$(date +%Y-%m-%d_%H:%M:%S)
filename="/home/youbot/log/capture_$datetime.pcap"
sudo tcpdump -w $filename -i eth0
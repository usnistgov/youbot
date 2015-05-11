#!/bin/bash

# Get the machine name
MACHNAME=$(hostname)
# Get the current date and time
DATETIME=$(date +_%Y-%m-%d_%H:%M:%S)
# Build the file postfix
POSTFIX=$MACHNAME$DATETIME
# Build the file name
FILENAME="/home/youbot/log/$POSTFIX.pcap"
tcpdump -w $FILENAME -i eth0
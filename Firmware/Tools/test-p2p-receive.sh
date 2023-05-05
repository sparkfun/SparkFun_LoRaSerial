#!/bin/bash
#   test-p2p-receive.sh
#   Lee Leahy
#   2023-05-06
#
#   Shell script to test data transmission with LoRaSerial
#
#   Parameters:
#       1: Loop count
#       2: Number of messages expected to receive
#       3: AirSpeed to use
#       4: COM port path
#
#   Example:
#       ./test-p2p-receive.sh 10 200 4800 /dev/ttyACM1 > Results/P2P_4800_RX.txt 2>&1
#

SYNC_FILE=p2p.sync
for ((i = 1; i <= $1; i++))
do
  # Separate the tests
  echo ""
  echo "----------------------------------------------------------------------"
  date
  echo "Test: $i"
  echo "----------------------------------------------------------------------"

  echo "Waiting for transmitter to create $SYNC_FILE"
  while [ -f "$SYNC_FILE" ]
  do
    sleep 1
  done
  echo ""

  # Perform the test
  ./Send_Receive  --receive=$2,p,$3,$4

  # Notify the transmit routine that the receive is complete
  if [ -f "$SYNC_FILE" ]; then
    echo ""
    echo "----------"
    echo ""
    date
    rm -f $SYNC_FILE
    date
    echo "Deleted $SYNC_FILE"
  fi
done

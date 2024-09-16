#!/bin/bash
#   test-p2p-transmit.sh
#   Lee Leahy
#   2023-05-06
#
#   Shell script to test data transmission with LoRaSerial
#
#   Parameters:
#       1: Loop count
#       2: Number of messages to send
#       3: AirSpeed to use
#       4: COM port path
#
#   Example:
#       ./test-p2p-transmit.sh 10 200 4800 /dev/ttyACM1 > Results/P2P_4800_RX.txt 2>&1
#
#   Note:
#
#       Start test-p2p-receive.sh first!
#

# Remove the sync file
SYNC_FILE=p2p.sync
if [ -f "$SYNC_FILE" ]; then
  rm $SYNC_FILE
  sleep 5
fi

for ((i = 1; i <= $1; i++))
do
  # Separate the tests
  echo ""
  echo "----------------------------------------------------------------------"
  date
  echo "Test: $i"
  echo "----------------------------------------------------------------------"
  echo ""

  # Create the sync file
  date
  echo "Creating $SYNC_FILE"
  echo ""
  touch $SYNC_FILE

  # Perform the test
  ./Send_Receive  --transmit=$2,p,$3,$4 < /dev/tty

  # Wait for the receiver to complete (delete the sync file)
  echo ""
  echo "----------"
  echo ""
  date
  echo "Waiting for receiver to delete $SYNC_FILE"
  while [ -f "$SYNC_FILE" ]
  do
    sleep 1
  done
done

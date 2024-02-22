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
#       4: COM port connected to LoRaSerial receiver
#       5: COM port connected to LoRaSerial transmitter
#
#   Example:
#       ./test-p2p.sh 10 2000 19200 /dev/ttyACM0 /dev/ttyACM1
#
#   Note:
#
#       Start test-p2p-receive.sh first!
#

# Remove the log files
rm Results/P2P_$3_RX.txt
rm Results/P2P_$3_TX.txt

# Analyze the test data
echo "----------------------------------------------------------------------" >> Results/P2P_$3_RX.txt
date >> Results/P2P_$3_RX.txt
echo "Data Analysis" >> Results/P2P_$3_RX.txt
echo "AirSpeed: $3" >> Results/P2P_$3_RX.txt
echo "----------------------------------------------------------------------" >> Results/P2P_$3_RX.txt
echo "" >> Results/P2P_$3_RX.txt
sync
./Send_Receive  --analyze=$2 >> Results/P2P_$3_RX.txt 2>&1
echo "" >> Results/P2P_$3_RX.txt

#Count the number of tests
for ((i = 1; i <= $1; i++))
do
  # Separate the tests
  echo "" >> Results/P2P_$3_RX.txt
  echo "----------------------------------------------------------------------" >> Results/P2P_$3_RX.txt
  date >> Results/P2P_$3_RX.txt
  echo "Test: $i" >> Results/P2P_$3_RX.txt
  echo "----------------------------------------------------------------------" >> Results/P2P_$3_RX.txt
  echo "" >> Results/P2P_$3_RX.txt

  echo "" >> Results/P2P_$3_TX.txt
  echo "----------------------------------------------------------------------" >> Results/P2P_$3_TX.txt
  date >> Results/P2P_$3_TX.txt
  echo "Test: $i" >> Results/P2P_$3_TX.txt
  echo "----------------------------------------------------------------------" >> Results/P2P_$3_TX.txt
  echo "" >> Results/P2P_$3_TX.txt

  # Perform the test
  sync
  ./Send_Receive  --receive=$2,p,$3,$4 >> Results/P2P_$3_RX.txt 2>&1 &
  ./Send_Receive  --transmit=$2,p,$3,$5 >> Results/P2P_$3_TX.txt 2>&1 &
  wait
  sync
  date >> Results/P2P_$3_RX.txt
  date >> Results/P2P_$3_TX.txt
  sleep 5
done

#!/bin/bash
WORK_DIR="/home/pi/lidar_scanner"
cd $WORK_DIR

sudo killall lidar_engine_v15 2>/dev/null
sudo killall python3 2>/dev/null
sleep 2
python3 calibrare_motoare.py

if [ $? -ne 0 ]; then
    echo "Eroare la calibrare! Sistemul se opreste."
    exit 1
fi
echo "Calibrare finalizata cu succes."
sudo ./lidar_engine_v15 > /dev/null 2>&1 &
PID_C=$!

wait $PID_C $PID_PY

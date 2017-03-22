#!/usr/bin/env bash
# Run for all data sets and for all sensors

./build/UnscentedKF $@    ./data/sample-laser-radar-measurement-data-1.txt ./tmp/data1_all.txt
./build/UnscentedKF -r $@ ./data/sample-laser-radar-measurement-data-1.txt ./tmp/data1_radar.txt
./build/UnscentedKF -l $@ ./data/sample-laser-radar-measurement-data-1.txt ./tmp/data1_lidar.txt
./build/UnscentedKF $@    ./data/sample-laser-radar-measurement-data-2.txt ./tmp/data2_all.txt
./build/UnscentedKF -r $@ ./data/sample-laser-radar-measurement-data-2.txt ./tmp/data2_radar.txt
./build/UnscentedKF -l $@ ./data/sample-laser-radar-measurement-data-2.txt ./tmp/data2_lidar.txt

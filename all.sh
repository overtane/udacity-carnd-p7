#!/usr/bin/env bash
# Run for all data sets and for all sensors

./build/UnscentedKF $@    ./data/sample-laser-radar-measurement-data-1.txt ./output/data1_fusion.txt
./build/UnscentedKF -r $@ ./data/sample-laser-radar-measurement-data-1.txt ./output/data1_radar.txt
./build/UnscentedKF -l $@ ./data/sample-laser-radar-measurement-data-1.txt ./output/data1_lidar.txt
./build/UnscentedKF $@    ./data/sample-laser-radar-measurement-data-2.txt ./output/data2_fusion.txt
./build/UnscentedKF -r $@ ./data/sample-laser-radar-measurement-data-2.txt ./output/data2_radar.txt
./build/UnscentedKF -l $@ ./data/sample-laser-radar-measurement-data-2.txt ./output/data2_lidar.txt

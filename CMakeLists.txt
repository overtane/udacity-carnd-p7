project(UnscentedKF)

cmake_minimum_required (VERSION 3.2)

add_definitions(-std=c++0x)

set(sources
	src/MeasurementFactory.cpp
	src/Measurement.cpp
	src/Sensor.cpp
	src/UnscentedKalmanFilter.cpp
	src/Tools.cpp
	src/main.cpp)

add_executable(UnscentedKF ${sources})

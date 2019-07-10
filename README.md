# What has been modified?

the official driver https://github.com/RoboSense-LiDAR/ros_rslidar

## first stage

only modify the code

keep the structure

-  `/rslidar_pointcloud/src/rawdata.cc` 215 --- 221: 

  stop subscribing to `raw_difop_packet` 

  stop publishing of `rslidar_device_info` 

-  `/rslidar_driver/src/rsdriver.cpp` 25---27: 

  stop publishing of tf

-  `/rslidar_driver/src/rsdriver.cpp` 121-124: 

  stop publishing of `raw_difop_packet` 

-  `/rslidar_driver/src/rsdriver.cpp` 95-99

  stop publishing of ros dynamic parameter

-  `/rslidar_pointcloud/src/convert.cc` 34-37

  stop publishing of ros dynamic pararmeters

keep the topic `/diagnostic` (in rsdriver.cpp 102, 128-130, 205-207)



## second stage

remove `/rslidar/point_cloud` 

modify `rslidar/src/CMakeLists.txt` and `CMakeLists.txt` 

add `rawdata.h` to `rsdriver.h` , so we can get the pointcloud directly

keep the "dynamic recofigure" and "diagnostics" part by comments in `rsdriver.h` and `rsdriver.cpp` 

-  dynamic configure

  `rsdriver.h` : 50-54

  `rsdriver.cpp` : 91-95, 233-237

- diagnostics

  `rsdriver.h` : 70-75

  `rsdriver.cpp` : 97-101, 109-116, 226-228



## RUN

```cmake
./build.sh
source devel_isolated/setup.bash
roslaunch rslidar_driver raccoon_rslidar.launch
```






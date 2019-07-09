# What has been modified?

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
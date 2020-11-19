# File player for MulRan data set

Maintainer: Giseop Kim (paulgkim@kaist.ac.kr)

This program is a file player for the [MulRan dataset](https://sites.google.com/view/mulran-pr/home). 


## Updates (2020. 11. 19)

- We released IMU and GPS data (consumer-level), as well as the originally delivered LiDAR and radar data. 
- We expect these data supports [GPS-aided radar researches](https://arxiv.org/pdf/2006.02108.pdf), LiDAR-IMU fusion such as [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) ([example video](https://youtu.be/Y6DXlC34qlc?t=479)), or radar-imu fusion.

## How to use 

```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch file_player file_player.launch
```
- Then, you need to select a sequence directory via GUI.
- For the correct load, first you need to place the GPS, IMU, LiDAR, and radar files in a directory following this structure (see this [guide video](https://youtu.be/uU-FC-GmHXA?t=45)) 
- IMU and GPS files (.csv) must be located at the same directory of "data_stamp.csv"


## Contributors
- Minwoo Jung: made the player system compatible with LIO-SAM input (i.e., supports ring information of a lidar scan)
 

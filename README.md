# Tello_Control

这是THU无人机赛课初赛提供的最小控制demo，供同学们参考。

代码改版自Tello官方[SDK](https://github.com/dji-sdk/Tello-Python):   需要进一步学习的同学请自行钻研。

## 1.安装

```
conda create -n tello python=2.7
conda activate tello

pip install pyyaml numpy matplotlib opencv-python

cd ~/catkin_ws/src/
git clone https://github.com/zoeyuchao/tello_control.git

cd tello_control/h264decoder 
mkdir build && cd build 
cmake .. 
make -j 
cp libh264decoder.so ../../
cd ~/catkin_ws
catkin_make
```

## 2.使用

1. 如果是第一次使用，需要增加可执行权限，否则会报找不到这个文件的错误，之后就可以直接使用，该命令不需要每次都执行：

```
chmod +x ~/catkin_ws/src/tello_control/tello_control.py
chmod +x ~/catkin_ws/src/tello_control/tello_state.py
```

2. 打开一个终端：

```
roscore
```

3. **新打开终端**，运行:

```
conda activate tello
rosrun tello_control tello_state.py
```
4. **新打开终端**，运行:

```
conda activate tello
rosrun tello_control tello_control.py
```

5. 报错 no module named rospkg

```
conda activate tello
conda install setuptools
pip install -U rosdep rosinstall_generator wstool rosinstall six vcstools
```
6. 报错ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type，在报错的文件里加上一句：

```
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
```

- tello_state.py的作用：

  - 发布/tello_state

  - 发布/tello_image

  - 不通过ROS控制tello【比赛不建议采用这种形式】

  - 订阅终端向/command发布的消息（用于快速调试）

     ```
      rostopic pub -1 /command std_msgs/String "takeoff"
     ```

- tello_control.py的作用：

  - 助教开发的最小控制程序，反馈控制实现识别定位毯并且飞到定位毯的中心位置。【注意：此程序仅适用于1号定位毯，其他定位毯需要修改程序】

  - 订阅

    - /tello_state
    - /tello_image

  - 向/command话题发布命令完成控制

## 3.Tips

- receiving video stream 端口11111（负责接收图像信息）
- receiving state 端口8890（负责接收状态信息）
- 控制指令见SDK2.0[文件](https://github.com/zoeyuchao/tello_control/blob/master/Tello_SDK_2.0_使用说明.pdf)






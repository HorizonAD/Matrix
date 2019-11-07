# matrix ros 驱动使用说明
## 1. 工程编译
1.1. 进入ros_ws文件夹执行catkin_make命令，如下
```shell
cd ros_ws/
catkin_make
```
1.2. 编译成功后执行命令
```shell
catkin_make install
```
1.3. 使用rosmsg和rostopic命令可以查看输出的信息，如果对这两个命令，请点击http://wiki.ros.org/kinetic/Installation/Ubuntu 自行查找。

1.4. 将matrix节点加入默认路径参数
```shell
source devel/setup.bash
```
1.5. 执行上述命令完成后即可运行matrix ros驱动程序
```shell
roslaunch matrix matrix.launch
```
## 2. RViz查看输出信息
### 2.1. 查看image信息
2.1.1. 打开rviz，命令如下
```shell
rviz
```
2.1.2. 在rviz主界面左侧Displays框下侧点击‘add’按钮，弹出框中选中‘image’条目，点击确定。展开Displays框中的image条目，在image topic中选择/matrix相关的条目，一般fullimageN 为出现图像的条目。
### 2.2. 查看障碍物信息
2.2.1. 打开rviz，命令如下
```shell
rviz
```
2.2.2. 在rviz主界面左侧Displays框下侧点击'add'按钮，弹出框中选中'MarkerArray'条目，点击确定。展开Displays框中的MarkerArray条目，在Marker Topic中选择/matrix/objs，在Global options中的Fix frame中填入velodyne即可
2.2.3. 将摄像头朝向障碍物方向，在有障碍物出现的时候会在rviz界面中间出现绿色的方块。
# 注意
1. 本示例适用于matrix 1.6, 1.7, mono 1-4路的输出，暂未支持多个IP的情况。   
2. 示例中的matrix设备的IP默认为192.168.1.10，请确认板子IP。
3. 运行本示例需要 ros环境，ros环境搭建请查看http://wiki.ros.org/kinetic/Installation/Ubuntu

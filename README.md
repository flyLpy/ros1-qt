# ros1-qt
雷达小车的上位机开发
一、
1.安装依赖包
sudo apt-get install expect
2.修改虚拟机环境变量
sudo gedit ~/.bashrc

将自己的qt_ros_test功能包的script实际路径加入
export PATH=$PATH:/home/lpy/catkin_qt5/src/qt_ros_test/script

编辑之后保存，使环境变量生效运行
source ~/.bashrc
3.配置launch文件
打开qt_ros_test.launch文件，将name和value 改为自己虚拟机用户名和密码
4.对虚拟机端的工作空间的 src 文件夹赋权限
进入工作空间的src
sudo chmod 777 -R ~/catkin_qt5/src
回到上一个目录cd ../
使用以下指令编译
catkin_make -DCATKIN_WHITELIST_PACKAGES="qt_ros_test"
编译过程中如果出现找不到libopencv_core.so.3.4.15按照下面方式安装
sudo apt update

sudo apt install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

sudo apt install python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

cd ~

wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.15.zip

unzip opencv.zip

cd opencv-3.4.15
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
编译之后的libopencv_core.so.3.4.15通常位于/usr/local/lib
进到工作空间cd ~/catkin_qt5 重新编译
catkin_make -DCATKIN_WHITELIST_PACKAGES="qt_ros_test"

二、
打开终端运行roscore

再打开一个新的终端运行 bash initssh.sh等待一会

设置环境变量
source ~/lpy/catkin_qt5/devel/setup.bash

运行程序
roslaunch qt_ros_test qt_ros_test.launch


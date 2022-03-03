# mpu6050DMP-ros-python on raspberry
This project sends datas from mpu6050 as rostopic.

The platform of this project is raspberry4B with ubuntu18.04

Make sure your device has smbus installed before using.

If not, please use:
```
pip install smbus

or

pip3 install smbus
```

After install smbus,please:
```
cd mpu6050DMP-ros-python

catkin_make
```

Now you can input this to run the code:
```
source ./devel/setup.bash
rosrun mpu2ros_python MPU6050_python.py
```

Open a new terminal,and input 
```
rostopic echo /mpu6050_data
```
to see the information

![image](https://user-images.githubusercontent.com/100181082/156544755-0cb33e2f-8ae1-4ab1-b3b1-54770043a13f.png)


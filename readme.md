#IMU helper 

##It is used to determine the IMU orientation

##用于判断IMU朝向

### How to use
1. record imu data ：Stand -> Forward 10m-> Left turn -> Forward 10m-> Stand, about 10-20s
2. git clone the repo in your workspace/src.
3. **catkin_make** and **source devel/setuo.sh**.
4. rosrun imu_helper imu_helper_node **imu_topic_name**.

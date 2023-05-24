#include <iostream>
#include <ros/ros.h>
#include <cstring>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace std;
// using namespace Eigen;

enum class Orientation{
    None,
    Up,
    Down,
    Front,
    Back,
    Left,
    Right
};

string Enum2String(Orientation a){
    string res;
    switch(a){
        case Orientation::None: res = "None"; break;
        case Orientation::Up: res = "Up"; break;
        case Orientation::Down: res = "Down"; break;
        case Orientation::Front: res = "Front"; break;
        case Orientation::Back: res = "Back"; break;
        case Orientation::Left: res = "Left"; break;
        case Orientation::Right: res = "Right"; break;
    }
    return res;
}

/**
 * @brief IMU 朝向判断类
 * 录制并且播放一段 启动->直行10m->左转90度->直行10m->结束录制
 * 时间在10-20秒左右
 */

class IMU_Orientation {
public:    
    IMU_Orientation(const string& topic_name,ros::NodeHandle& nh):dt(0),sum_dt(0),last_imu_stamp(0),count(-5),delta_p({0,0,0}),delta_v({0,0,0}),
                            linearized_ba({6.435665e-05,6.435665e-05,6.435665e-05}),linearized_bg({3.564031e-05,3.564031e-05,3.564031e-05}),
                            delta_q{Eigen::Quaterniond::Identity()},G(9.8) {
        z_ori = Orientation::None;
        x_ori = Orientation::None;
        y_ori = Orientation::None;
        imu_sub = nh.subscribe<sensor_msgs::Imu> (topic_name,2000, &IMU_Orientation::imu_callback, this, ros::TransportHints().tcpNoDelay());
    }

    void get_result(){
        if(y_ori != Orientation::None &&  x_ori == Orientation::None){
            // x轴朝向为左右
            if(delta_p[0] > 5){
                x_ori = Orientation::Left;
            }else if(delta_p[0] < -5){
                x_ori = Orientation::Right;
            }          
        }else if(x_ori != Orientation::None && y_ori == Orientation::None){
            // y轴朝向为左右
            if(delta_p[1] > 5){
                y_ori = Orientation::Left;
            }else if(delta_p[1] < -5){
                y_ori = Orientation::Right;
            }   
        }else{
            cout<<"bag error! cannot to computer IMU orientation!!!"<<endl;
            // return;
        }
        cout<<" ===== IMU orientation : "<<Enum2String(x_ori)<<"-"<<Enum2String(y_ori)<<"-"<<Enum2String(z_ori)<<" ====="<<endl;
    }
private:
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_raw){
        //忽略前5個
        if(count < 0){
            count++;
            last_imu_stamp = imu_raw->header.stamp.toNSec();
            return;
        }
        dt = double(imu_raw->header.stamp.toNSec() - last_imu_stamp) * pow(10,-9);    
        count++;
        if(count%10 == 0){
            cout<<"sum_dt:"<<sum_dt<<"    "
            <<"pose t:"<<delta_p[0]<<" "<<delta_p[1]<<" "<<delta_p[2]<<
                "   Quaterniond:"<<delta_q.w()<<" "<<delta_q.x()<<" "<<delta_q.y()<<" "<<delta_q.z()<<endl;
        }

        Eigen::Vector3d acc = {imu_raw->linear_acceleration.x,imu_raw->linear_acceleration.y,imu_raw->linear_acceleration.z};
        Eigen::Vector3d gyr = {imu_raw->angular_velocity.x,imu_raw->angular_velocity.y,imu_raw->angular_velocity.z};
        for(int i=0; i<3; i++) if(abs(acc[i]) < 0.009) acc[i] = 0;


        propagate(dt,acc,gyr);
        last_imu_stamp = imu_raw->header.stamp.toNSec();
        check_Orientation(acc,gyr);
    }

    void check_Orientation(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr){
        static double sum_z_acc = 0;
        if(z_ori == Orientation::None){
            // 判斷z軸方向
            if(count % 30 == 0){ //累加30個測量到的重力加速度
                if(sum_z_acc > G*20){
                    z_ori = Orientation::Up;
                }else if(sum_z_acc < -G*20){
                    z_ori = Orientation::Down;
                }else{
                    sum_z_acc = 0;
                }
            }else{
                sum_z_acc += acc[2];
            }
        }
        // cout<<sum_z_acc<<"    ,Z_Orientation:"<<z_ori<<endl;

        if(x_ori == Orientation::None || y_ori == Orientation::None){
            // 判斷x,y軸方向

            if(x_ori == Orientation::None && abs(delta_p[0]) > 5 && abs(delta_p[0]) > 15*abs(delta_p[1])){  //x轴的位移大于y轴15倍
                // x轴朝向为前后
                if(delta_p[0] > 5){
                    x_ori = Orientation::Front;
                }else if(delta_p[0] < -5){
                    x_ori = Orientation::Back;
                }

            }else if(y_ori == Orientation::None && abs(delta_p[1]) > 5 && abs(delta_p[1]) > 15*abs(delta_p[0])){  //y轴的位移大于x轴15倍
                // y轴朝向为前后
                if(delta_p[1] > 5){
                    y_ori = Orientation::Front;
                }else if(delta_p[1] < -5){
                    y_ori = Orientation::Back;
                }
            }
        }
    }

    /**
    * @brief   IMU预积分传播方程
    * @Description  积分计算两个关键帧之间IMU测量的变化量： 
    *               旋转delta_q 速度delta_v 位移delta_p
    *               加速度的biaslinearized_ba 陀螺仪的Bias linearized_bg
    *               同时维护更新预积分的Jacobian和Covariance,计算优化时必要的参数
    * @param[in]   _dt 时间间隔
    * @param[in]   _acc_1 线加速度
    * @param[in]   _gyr_1 角速度
    * @return  void
    */
    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;
        Eigen::Vector3d result_delta_p;
        Eigen::Quaterniond result_delta_q;
        Eigen::Vector3d result_delta_v;
        Eigen::Vector3d result_linearized_ba;
        Eigen::Vector3d result_linearized_bg;

        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1,G);
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;
        acc_0 = acc_1;
        gyr_0 = gyr_1;  
    }

        /**
    * @brief   IMU预积分中采用中值积分递推Jacobian和Covariance
    *          构造误差的线性化递推方程，得到Jacobian和Covariance递推公式-> Paper 式9、10、11
    * @return  void
    */
    void midPointIntegration(double _dt, 
                            const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                            const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                            const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                            const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                            Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                            Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian, float G)
    {
        //ROS_INFO("midpoint integration");
        Eigen::Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Eigen::Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
        result_delta_q = delta_q * Eigen::Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);
        Eigen::Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        un_acc[2]-=G;
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
        result_delta_v = delta_v + un_acc * _dt;
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;         
    }

    ros::Subscriber imu_sub;
    long last_imu_stamp;
    double dt, sum_dt;
    Eigen::Vector3d acc_0, gyr_0;
    Eigen::Vector3d acc_1, gyr_1;
    //预积分值
    Eigen::Vector3d delta_p;
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;
    Eigen::Vector3d linearized_ba, linearized_bg;
    int count;
    float G;
    //朝向
    Orientation z_ori,x_ori,y_ori;
};

int main(int argc,char** argv){
    ROS_INFO_STREAM(" ====== imu helper ======");
    ros::init(argc, argv, "imu_helper");
    ros::NodeHandle nh;
    IMU_Orientation IMU_Orient(argv[1],nh);
    ros::spin();

    cout<<"\n ====== IMU Orientation result ======"<<endl;
    IMU_Orient.get_result();
    return 0;
}
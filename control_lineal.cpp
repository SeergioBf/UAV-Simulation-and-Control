#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <vector>

float x, y, z;
void posicion_Callback(const geometry_msgs::Vector3::ConstPtr& pos){
    x = pos->x;
    y = pos->y;
    z = pos->z;
}

float xpunto, ypunto, zpunto;
void velocidad_Callback(const geometry_msgs::Vector3::ConstPtr& vel){
    xpunto = vel->x;
    ypunto = vel->y;
    zpunto = vel->z;
}

float xd, yd, zd;
void posicion_deseada_Callback(const geometry_msgs::Vector3::ConstPtr& posd){
    xd = posd->x;
    yd = posd->y;
    zd = posd->z;
}

float xdpunto, ydpunto, zdpunto;
void velocidad_deseada_Callback(const geometry_msgs::Vector3::ConstPtr& veld){
    xdpunto = veld->x;
    ydpunto = veld->y;
    zdpunto = veld->z;
}

float fi, theta, si;
void angulos_Callback(const geometry_msgs::Vector3::ConstPtr& ang){
    fi = ang->x;
    theta = ang->y;
    si = ang->z;
}

float sid;
void yawd_Callback(const geometry_msgs::Vector3::ConstPtr& yawd){
    sid = yawd->z;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "control_lineal");
    ros::NodeHandle nh;
    ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Vector3>("thrust", 10);
    ros::Publisher angulos_deseados_pub = nh.advertise<geometry_msgs::Vector3>("angulos_deseados", 10);
    ros::Subscriber posicion_sub = nh.subscribe("/posicion", 10, &posicion_Callback);
    ros::Subscriber velocidad_sub = nh.subscribe("/velocidad", 10, &velocidad_Callback);
    ros::Subscriber posicion_deseada_sub = nh.subscribe("/posicion_deseada", 10, &posicion_deseada_Callback);
    ros::Subscriber velocidad_deseada_sub = nh.subscribe("/velocidad_deseada", 10, &velocidad_deseada_Callback);
    ros::Subscriber angulos_sub = nh.subscribe("/angulos", 10, &angulos_Callback);
    ros::Subscriber yaw_sub = nh.subscribe("/yaw_deseada", 10, &yawd_Callback);

    ros::Publisher errorlineal_pub = nh.advertise<geometry_msgs::Vector3>("errorlineal", 10);
    ros::Rate loop_rate(100);
    geometry_msgs::Vector3 thrust_msg;
    geometry_msgs::Vector3 angulos_deseados_msg;
    geometry_msgs::Vector3 errorlineal_msg;

    float error_x;
    float error_xpunto;
    float error_y;
    float error_ypunto;
    float error_z;
    float error_zpunto;

    float kp_x = 1;
    float kd_x = 0.6;
    float kp_y = 1;
    float kd_y = 0.6;
    float kp_z = 1;
    float kd_z = 0.6;

    float u_vx;
    float u_vy;
    float u_vz;

    float z_2punto_des = 0;
    float Th;
    float fid;
    float thetad;

    while (ros::ok()){
        error_x = x - xd;
        error_xpunto = xpunto - xdpunto;
        error_y = y - yd;
        error_ypunto = ypunto - ydpunto;
        error_z = z - zd;
        error_zpunto = zpunto - zdpunto;

        u_vx = (-kp_x * error_x) - (kd_x * error_xpunto);
        u_vy = (-kp_y * error_y) - (kd_y * error_ypunto);
        u_vz = (-kp_z * error_z) - (kd_z * error_zpunto);

        z_2punto_des = z_2punto_des + 0.01 * zdpunto;

        Th = (2 / (cos(fi) * cos(theta))) * (z_2punto_des - 9.81 + u_vz);

        float sin_fid = (2 / Th) * ((sin(sid) * u_vx) - (cos(sid) * u_vy));
        float cos_fid = sqrt(1 - sin_fid * sin_fid);
        if (std::isfinite(sin_fid) && std::isfinite(cos_fid)) {
            fid = asin(sin_fid);
            if (std::isfinite(fid)) {
                thetad = asin((((2 / Th) * u_vx) - (sin(sid) * sin(fid))) / (cos(sid) * cos(fid)));
                if (!std::isfinite(thetad)) {
                    thetad = 0;
                }
            } else {
                fid = 0;
                thetad = 0;
            }
        } else {
            fid = 0;
            thetad = 0;
        }

        thrust_msg.x = Th;
        thrust_msg.y = 0;
        thrust_msg.z = 0;
        thrust_pub.publish(thrust_msg);

        angulos_deseados_msg.x = fid;
        angulos_deseados_msg.y = thetad;
        angulos_deseados_msg.z = 0;
        angulos_deseados_pub.publish(angulos_deseados_msg);

        errorlineal_msg.x = error_xpunto;
        errorlineal_msg.y = error_ypunto;
        errorlineal_msg.z = error_zpunto;
        errorlineal_pub.publish(errorlineal_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

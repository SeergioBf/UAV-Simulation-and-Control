#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <vector>

float fipunto_des, thetapunto_des, sipunto_des;
void velocidad_angular_deseada_Callback(const geometry_msgs::Vector3::ConstPtr& velangdes){
    fipunto_des = velangdes->x;
    thetapunto_des = velangdes->y;
    sipunto_des = velangdes->z;
}

float si_des;
void si_deseada_Callback(const geometry_msgs::Vector3::ConstPtr& sides){
    si_des = sides->z;
}

float fi, theta, si;
void angulos_Callback(const geometry_msgs::Vector3::ConstPtr& angles){
    fi = angles->x;
    theta = angles->y;
    si = angles->z;
}

float fipunto, thetapunto, sipunto;
void angulospunto_Callback(const geometry_msgs::Vector3::ConstPtr& anglespun){
    fipunto = anglespun->x;
    thetapunto = anglespun->y;
    sipunto = anglespun->z;
}

float fi_des, theta_des;
void angulos_deseados_Callback(const geometry_msgs::Vector3::ConstPtr& angdes){
    fi_des = angdes->x;
    theta_des = angdes->y;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "control_angular");
    ros::NodeHandle nh;
    ros::Publisher torques_pub = nh.advertise<geometry_msgs::Vector3>("torques", 10);
    ros::Subscriber velocidad_angular_deseada_sub = nh.subscribe("/velocidad_angular_deseada", 10, &velocidad_angular_deseada_Callback);
    ros::Subscriber angulos_sub = nh.subscribe("/angulos", 10, &angulos_Callback);
    ros::Subscriber angulos_deseados_sub = nh.subscribe("/angulos_deseados", 10, &angulos_deseados_Callback);
    ros::Subscriber angulospunto_sub = nh.subscribe("/angulospunto", 10, &angulospunto_Callback);
    ros::Subscriber yaw_deseada_sub = nh.subscribe("/yaw_deseada", 10, &si_deseada_Callback);
    //PLOTJUGGLER
    ros::Publisher errorangular_pub = nh.advertise<geometry_msgs::Vector3>("errorangular", 10);
    ros::Rate loop_rate(100);
    geometry_msgs::Vector3 torques_msg;
    //PLOTJUGGLER
    geometry_msgs::Vector3 errorangular_msg;

    //ERRORES ANGULARES
    float error_fi;
    float error_fi_punto;
    float error_theta;
    float error_theta_punto;
    float error_si;
    float error_si_punto;
    //GANANCIAS
    float kp_fi = 1.5;
    float kd_fi = 1.6;
    float kp_theta = 1.5;
    float kd_theta = 1.2;
    float kp_si = 1.7;
    float kd_si = 1.9;
    //CONTROLADORES
    float u_fi = 0;
    float u_theta = 0;
    float u_si = 0;
    //TORQUES
    float tau_fi = 0;
    float tau_theta = 0;
    float tau_si = 0;
    //INERCIAS
    float jxx = 0.0411;
    float jyy = 0.0478;
    float jzz = 0.0599;

    while (ros::ok()){
        //CALCULO DEL ERROR POR ANGULO
        error_fi = fi - fi_des; //aqui algo anda mal
        error_fi_punto = fipunto - fipunto_des; // aqui algo anda mal
        error_theta = theta - theta_des;
        error_theta_punto = thetapunto - thetapunto_des;
        error_si = si - si_des;
        error_si_punto = sipunto - sipunto_des;
        //CONTROLADORES
        u_fi = (-kp_fi*error_fi) - (kd_fi*error_fi_punto);
        u_theta = (-kp_theta*error_theta) - (kd_theta*error_theta_punto);
        u_si = (-kp_si*error_si) - (kd_si*error_si_punto);
        //CALCULO DE TORQUES
        tau_fi = jxx*((((jzz-jyy)/jxx)*thetapunto*sipunto) + u_fi);
        tau_theta = jyy*((((jxx-jzz)/jyy)*fipunto*sipunto) + u_theta);
        tau_si = jzz*((((jyy-jxx)/jzz)*fipunto*thetapunto) + u_si);

        torques_msg.x = tau_fi;
        torques_msg.y = tau_theta;
        torques_msg.z = tau_si;
        torques_pub.publish(torques_msg);

        //std::cout << "tau_fi " << error_fi << std::endl;
        //std::cout << "tau_theta " << fi << std::endl;
        //std::cout << "tau_si " << fi_des << std::endl;
        
        //GRAFICACION
        errorangular_msg.x = error_fi_punto;
        errorangular_msg.y = error_theta_punto;
        errorangular_msg.z = error_si_punto;
        errorangular_pub.publish(errorangular_msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

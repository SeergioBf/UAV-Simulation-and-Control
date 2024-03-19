#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <vector>

int main(int argc, char **argv){
    ros::init(argc, argv, "referencias");
    ros::NodeHandle nh;
    ros::Publisher posd_pub = nh.advertise<geometry_msgs::Vector3>("posicion_deseada", 10);
    ros::Publisher veld_pub = nh.advertise<geometry_msgs::Vector3>("velocidad_deseada", 10);
    ros::Publisher velangd_pub = nh.advertise<geometry_msgs::Vector3>("velocidad_angular_deseada", 10);
    ros::Publisher yawd_pub = nh.advertise<geometry_msgs::Vector3>("yaw_deseada", 10);
    ros::Rate loop_rate(100);
    geometry_msgs::Vector3 posd_msg;
    geometry_msgs::Vector3 veld_msg;
    geometry_msgs::Vector3 velangd_msg; 
    geometry_msgs::Vector3 yawd_msg;

    //VARIABLES AUXILIARES
    float t = 0;
    float cont = 0;
    //VELOCIDADES DESEADAS (TRES LINEALES Y 3 ANGULARES)
    float xpunto_des = 0;
    float ypunto_des = 0;
    float zpunto_des = -0.5;
    float sipunto_des = 0;
    float fipunto_des;
    float thetapunto_des;
    //POSICIONES DESEADAS (VELOCIDADES Y ANGULO DERIVAD@S) 
    float xd = 0;
    float yd = 0;
    float zd = 0;
    float sid = 0;

    while (ros::ok()){
        cont++;
        t = cont*0.01;
        if(t>=5 && t<=65){
            xpunto_des = 0.5*sin(0.1*(t-5));
            ypunto_des = 0.5*cos(0.1*(t-5));
            zpunto_des = 0;
            sipunto_des = 0.1;
        }
        else{
            xpunto_des = 0;
            ypunto_des = 0;
            zpunto_des = -0.5;
            sipunto_des = 0;
            xd = 0;
            yd = 0;
            zd = 0;
            sid = 0;
        }
        //INTEGRACION DE VELOCIDADES PARA OBTENER POSICIONES Y SI DESEAD@S 
        xd = xd + 0.01*xpunto_des;
        yd = yd + 0.01*ypunto_des;
        zd = zd + 0.01*zpunto_des;
        sid = sid + 0.01*sipunto_des;
        
        //MANDAR VALORES DE POSICION DESEADA
        posd_msg.x = xd;
        posd_msg.y = yd;
        posd_msg.z = zd;
        posd_pub.publish(posd_msg);
        //MANDAR VALORES DE VELOCIDAD DESEADA
        veld_msg.x = xpunto_des;
        veld_msg.y = ypunto_des;
        veld_msg.z = zpunto_des;
        veld_pub.publish(veld_msg);
        //MANDAR VALORES DE VELOCIDAD ANGULAR DESEADA
        velangd_msg.x = fipunto_des;
        velangd_msg.y = thetapunto_des;
        velangd_msg.z = sipunto_des;
        velangd_pub.publish(velangd_msg);
        //MANDAR SI-YAW DESEADA
        yawd_msg.x = 0;
        yawd_msg.y = 0;
        yawd_msg.z = sid;
        yawd_pub.publish(yawd_msg);

        //COMPROBACION DE VALORES (DES-COMENTAR PARA VER)
        /*
        std::cout << "t " << t << std::endl;
        std::cout << "xpunto_des " << xpunto_des << std::endl;
        std::cout << "ypunto_des " << ypunto_des << std::endl;
        std::cout << "zpunto_des " << zpunto_des << std::endl;
        std::cout << "sipunto_des " << sipunto_des << std::endl;
        */
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

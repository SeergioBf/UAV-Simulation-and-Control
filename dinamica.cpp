#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <vector>

Eigen::Vector3f posicion_real;
Eigen::Vector3f tau;
Eigen::Vector3f w_punto; //ACELERACION ANGULAR
Eigen::Vector3f w; //VELOCIDAD ANGULAR
Eigen::Vector3f attitude_punto;
Eigen::Vector3f attitude;
Eigen::Vector3f fuerza;
Eigen::Vector3f v_punto;
Eigen::Vector3f v;
Eigen::Vector3f p_punto;
Eigen::Vector3f p;
Eigen::Vector3f e3;
Eigen::Matrix3f J;
Eigen::Matrix3f R2;
Eigen::Matrix3f RotationMatrix(Eigen::Vector3f pos_att)
{
	float cos_phi = cos(pos_att(0));
	float sin_phi = sin(pos_att(0));
	float cos_theta = cos(pos_att(1));
	float sin_theta = sin(pos_att(1));
	float cos_psi = cos(pos_att(2));
	float sin_psi = sin(pos_att(2));
	Eigen::Matrix3f R;
	R << cos_psi * cos_theta, cos_psi * sin_phi * sin_theta - cos_phi * sin_psi, sin_psi * sin_phi + cos_psi * cos_phi * sin_theta,
			 cos_theta * sin_psi, cos_psi * cos_phi + sin_psi * sin_phi * sin_theta, cos_phi * sin_psi * sin_theta - cos_psi * sin_phi,
			 -sin_theta, cos_theta * sin_phi, cos_phi * cos_theta;
			 
	return R;
}
Eigen::Matrix3f w_antisimetrica(Eigen::Vector3f w){
	Eigen::Matrix3f antisimetrica;
	antisimetrica << 0, -w(2), w(1),
					w(2), 0, -w(0),
					-w(1), w(0), 0;
	return antisimetrica;
}

void torques_Callback(const geometry_msgs::Vector3::ConstPtr& torques){
    tau(0) = torques->x;//tau fi
    tau(1) = torques->y;//tau theta
    tau(2) = torques->z;//tau si
}
float Th = 0;
void thrust_Callback(const geometry_msgs::Vector3::ConstPtr& thrustC){
    Th = thrustC->x;
}


float fi = 0;
float theta = 0;  
float si = 0;
float gravedad = 9.81;

int main(int argc, char **argv){
    ros::init(argc, argv, "dinamica");
    ros::NodeHandle nh;
    ros::Publisher posicion_pub = nh.advertise<geometry_msgs::Vector3>("posicion", 10);
    ros::Publisher velocidad_pub = nh.advertise<geometry_msgs::Vector3>("velocidad", 10);
    ros::Publisher angulos_pub = nh.advertise<geometry_msgs::Vector3>("angulos", 10);
    ros::Publisher angulospunto_pub = nh.advertise<geometry_msgs::Vector3>("angulospunto", 10);
    ros::Subscriber torques_sub = nh.subscribe("/torques", 10, &torques_Callback);
    ros::Subscriber thrust_sub = nh.subscribe("/thrust", 10, &thrust_Callback);
    ros::Rate loop_rate(100);
    geometry_msgs::Vector3 posicion_msg;
    geometry_msgs::Vector3 velocidad_msg;
    geometry_msgs::Vector3 angulos_msg;
    geometry_msgs::Vector3 angulospunto_msg;
    J << 0.0411, 0, 0,
		    0, 0.0478, 0,
		    0, 0, 0.0599;
    posicion_real << -5, 0, 0;
    w_punto << 0, 0, 0;
    w << 0, 0, 0;
    attitude_punto << 0, 0, 0;
    attitude << 0, 0, 0;
    fuerza << 0, 0, 0;
    e3 << 0, 0, 1;
    v_punto << 0, 0, 0;
    v << 0, 0, 0;
    p_punto << 0, 0, 0;
    p << -5, 0, 0;

    while (ros::ok()){
        R2 << 1, (sin(fi)*tan(theta)), (cos(fi)*tan(theta)),
                0, cos(fi), -sin(fi),
                0, (sin(fi)/cos(theta)), (cos(fi)/cos(theta));
        //ANGULAR
        w_punto = J.inverse()*(tau-w_antisimetrica(w)*J*w);
        for(int i = 0; i<=2; i++){
            w(i) = w(i) + 0.01*w_punto(i);
        }
        
        attitude_punto = R2*w;
        for(int i = 0; i<=2; i++){
            attitude(i) = attitude(i) + 0.01*attitude_punto(i);
        }

        fi = attitude(0);
        theta = attitude(1);
        si = attitude(2);
        //LINEAL
        fuerza = (Th*e3) + (RotationMatrix(attitude).inverse()*(2*gravedad*e3));
        v_punto = (fuerza/2) - (-w_antisimetrica(w)*v);
        for(int i = 0; i<=2; i++){
            v(i) = v(i) + 0.01 * v_punto(i);
        } 
        p_punto = RotationMatrix(attitude)*v;
        for(int i = 0; i<=2; i++){
            p(i) = p(i) + 0.01 * p_punto(i);
        } 

        posicion_msg.x = p(0);
        posicion_msg.y = p(1);
        posicion_msg.z = p(2);
        posicion_pub.publish(posicion_msg);

        velocidad_msg.x = v(0);
        velocidad_msg.y = v(1);
        velocidad_msg.z = v(2);
        velocidad_pub.publish(velocidad_msg);

        angulos_msg.x = fi;
        angulos_msg.y = theta;
        angulos_msg.z = si;
        angulos_pub.publish(angulos_msg);

        angulospunto_msg.x = attitude_punto(0);
        angulospunto_msg.y = attitude_punto(1);
        angulospunto_msg.z = attitude_punto(2);
        angulospunto_pub.publish(angulospunto_msg);
        
        std::cout << "fi " << fi << std::endl;
        std::cout << "theta " << theta << std::endl;
        std::cout << "si " << si << std::endl;
       

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

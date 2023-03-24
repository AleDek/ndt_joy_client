#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "boost/thread.hpp"
//---mavros_msgs
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/PositionTarget.h>
//---
#include "utils.h"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
//---



#include <mutex>          // std::mutex
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include "geometry_msgs/Twist.h"

#include "ndt_core_interface/deploy.h"
#include "ndt_core_interface/close.h"
#include "ndt_core_interface/reset_bias.h"
#include "ndt_core_interface/disable_motors.h"
#include "ndt_core_interface/enable_motors.h"
#include "ndt_core_interface/enable_admittance.h"
#include "ndt_core_interface/interaction_mode.h"

#include "std_msgs/Int32.h"

using namespace Eigen;
using namespace std;



class SIMPLE_CLIENT {


    public:
        SIMPLE_CLIENT();
        void position_controller();
		void run();
		void localization_cb ( geometry_msgs::PoseStampedConstPtr msg );
		void mavros_state_cb( mavros_msgs::State mstate);

        void joy_cb( sensor_msgs::Joy j );
        void main_loop();

    private:

        ros::NodeHandle _nh;
        ros::Publisher _target_pub;
        ros::Subscriber _localization_sub;
        ros::Subscriber _joy_data_sub;
        ros::Subscriber _mavros_state_sub;

        bool _first_local_pos;
        bool _enable_joy;

        bool _enable_openarm;
        bool _enable_closearm;
        bool _enable_admittance;
        bool _enable_interaction;

        // --- Desired state
        Vector3d _cmd_p;
        double _cmd_yaw;       
        Vector3d _w_p;
        Vector3d _vel_joy;
        Vector4d _w_q;
        float _mes_yaw;
        mavros_msgs::State _mstate;
        double _vel_joy_yaw;

        int joy_ax0;
        int joy_ax1;
        int joy_ax2;
        int joy_ax3;

        int camera_0vel;
        int camera_1vel;
        int camera_2vel;
        int camera_yaw_vel;

};


SIMPLE_CLIENT::SIMPLE_CLIENT() {

     if( !_nh.getParam("joy_ax0", joy_ax0)) {
        joy_ax0 = 1.0;
    }

    if( !_nh.getParam("joy_ax1", joy_ax1)) {
        joy_ax1 = 1.0;
    }

    if( !_nh.getParam("joy_ax2", joy_ax2)) {
        joy_ax2 = 1.0;
    }

    if( !_nh.getParam("joy_ax3", joy_ax3)) {
        joy_ax3 = 1.0;
    }

	_target_pub = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

    _localization_sub   = _nh.subscribe( "/mavros/local_position/pose", 1, &SIMPLE_CLIENT::localization_cb, this);
    _mavros_state_sub   = _nh.subscribe( "/mavros/state", 1, &SIMPLE_CLIENT::mavros_state_cb, this);
    _joy_data_sub       = _nh.subscribe("/joy", 1, &SIMPLE_CLIENT::joy_cb, this);

    _vel_joy << 0.0, 0.0, 0.0;

    _first_local_pos = false;
    _enable_joy = false;

    _enable_openarm  = false;
    _enable_closearm = false;
    _enable_admittance = false;
    _enable_interaction = false;

    
}



void SIMPLE_CLIENT::mavros_state_cb( mavros_msgs::State mstate) {
    _mstate = mstate;
}


void SIMPLE_CLIENT::localization_cb ( geometry_msgs::PoseStampedConstPtr msg ) {
    _w_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
   
    Eigen::Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg->pose.orientation.w,  msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z) ) );
    _mes_yaw = rpy(2);

    Quaternionf q;
    q = AngleAxisf(0.0, Vector3f::UnitX())
        * AngleAxisf(0.0, Vector3f::UnitY())
        * AngleAxisf(_mes_yaw, Vector3f::UnitZ());
    Vector4d w_q ( q.w(), q.x(), q.y(), q.z() );
    _w_q = w_q / w_q.norm() ;

   
    _first_local_pos = true;    
}

void SIMPLE_CLIENT::position_controller(){

    ros::Rate r(100);

    mavros_msgs::PositionTarget ptarget;
    ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    ptarget.type_mask =
    mavros_msgs::PositionTarget::IGNORE_VX |
    mavros_msgs::PositionTarget::IGNORE_VY |
    mavros_msgs::PositionTarget::IGNORE_VZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::FORCE |
    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    _cmd_p = _w_p;
    _cmd_yaw = _mes_yaw;

    while (ros::ok()) {
        if( _mstate.mode != "OFFBOARD" ) {
            _cmd_p = _w_p;
            _cmd_yaw = _mes_yaw;
        }
        else {
            //---Position
        }

        //---Publish command
        ptarget.header.stamp = ros::Time::now();
        ptarget.position.x = _cmd_p[0];
        ptarget.position.y = _cmd_p[1];
        ptarget.position.z = _cmd_p[2];
        ptarget.yaw = _cmd_yaw;
        _target_pub.publish( ptarget );
        //---

        r.sleep();
    }
}


void SIMPLE_CLIENT::joy_cb( sensor_msgs::Joy j ) {

    if( j.buttons[0] == 1 ) _enable_joy = true;
    if( j.buttons[2] == 1 ) _enable_openarm = true;
    if( j.buttons[3] == 1 ) _enable_closearm = true;
    if( j.buttons[4] == 1 ) _enable_admittance = true;
    if( j.buttons[5] == 1 ) _enable_interaction = true;
    
    
    _vel_joy[0] = joy_ax0*j.axes[1]*0.2;
    _vel_joy[1] = joy_ax1*j.axes[0]*0.2;
    _vel_joy[2] = joy_ax2*j.axes[4]*0.2;
    _vel_joy_yaw = joy_ax3*j.axes[3]*0.2;

}

void SIMPLE_CLIENT::main_loop () {

    int enable_joy_cnt = 0;
    int enable_openarm_cnt = 0;
    int enable_closearm_cnt = 0;
    int enable_admittance_cnt = 0;
    int enable_interaction_cnt = 0;

    bool joy_ctrl = false;
    
    
    ros::Rate r(10);

	ros::ServiceClient close_arm_srv           = _nh.serviceClient<ndt_core_interface::deploy>("/NDT/close_arm");
  	ros::ServiceClient open_arm_srv            = _nh.serviceClient<ndt_core_interface::close>("/NDT/open_arm");
  	ros::ServiceClient reset_bias_srv          = _nh.serviceClient<ndt_core_interface::reset_bias>("/NDT/reset_bias");
  	ros::ServiceClient disable_motors_srv      = _nh.serviceClient<ndt_core_interface::disable_motors>("/NDT/disable_motors");
  	ros::ServiceClient enable_motors_srv       = _nh.serviceClient<ndt_core_interface::enable_motors>("/NDT/enable_motors");
  	ros::ServiceClient enable_admittance_srv   = _nh.serviceClient<ndt_core_interface::enable_admittance>("/NDT/enable_admittance_mode");
  	ros::ServiceClient disable_admittance_srv  = _nh.serviceClient<ndt_core_interface::enable_admittance>("/NDT/enable_admittance_mode");
  	ros::ServiceClient enable_interaction_srv  = _nh.serviceClient<ndt_core_interface::interaction_mode>("/NDT/enable_interaction_mode");
  	ros::ServiceClient disable_interaction_srv = _nh.serviceClient<ndt_core_interface::interaction_mode>("/NDT/enable_interaction_mode");

	ros::Publisher gelPump_pub = _nh.advertise<std_msgs::Int32>("/NDT/pump_time", 1);
	std_msgs::Int32 pumpTime;

	ndt_core_interface::deploy open_srv;
	ndt_core_interface::close close_srv;
	ndt_core_interface::disable_motors disable_m_srv;
	ndt_core_interface::enable_motors enable_m_srv;
	ndt_core_interface::enable_admittance enable_adm_srv;
	enable_adm_srv.request.enable = true;
	ndt_core_interface::enable_admittance disable_adm_srv;
	disable_adm_srv.request.enable = false;
	ndt_core_interface::reset_bias rnias_srv;

	ndt_core_interface::interaction_mode enable_int_srv;
	enable_int_srv.request.enable = true;
	ndt_core_interface::interaction_mode disable_int_srv;
	disable_int_srv.request.enable = false;

    while( ros::ok() ) {

        enable_joy_cnt++;
        enable_openarm_cnt++;
        enable_closearm_cnt++;

        //phase1_cnt++;
        //phase2_cnt++;

        if( _enable_joy == true && enable_joy_cnt > 50) {
            joy_ctrl = !joy_ctrl;
            enable_joy_cnt = 0;
            _enable_joy = false;
            cout << "JOY CTRL: " << joy_ctrl << endl;
        }

        if( _enable_openarm && enable_openarm_cnt > 50) {
            enable_openarm_cnt = 0;
            open_arm_srv.call( open_srv );
            _enable_openarm = false;
        }

        if( _enable_closearm && enable_closearm_cnt > 50) {
            enable_closearm_cnt = 0;
			close_arm_srv.call( close_srv );
            _enable_closearm = false;
        }


        /*
        if (_enable_phase1 == true && phase1_cnt > 50 ) {
            phase1_cnt = 0;
            phase1_ctrl = !phase1_ctrl;
            _enable_phase1 = false;
            if( phase1_ctrl ) {
                joy_ctrl = phase2_ctrl = false;
            }
            cout << "PHASE1 CTRL: " << phase1_ctrl << endl;

        }

        if (_enable_phase2 == true && phase2_cnt > 50 ) {
            phase2_cnt = 0;
            phase2_ctrl = !phase2_ctrl;
            _enable_phase2 = false;
            if( phase2_ctrl ) {
                joy_ctrl = phase1_ctrl = false;
            }
            cout << "PHASE2 CTRL: " << phase2_ctrl << endl;

        }
        */        

        if( joy_ctrl ) {     
            double cyaw = 0.0;       
            if( _mstate.mode == "OFFBOARD" ) {

                _cmd_p[0] += _vel_joy[0]*(1/10.0);
                _cmd_p[1] += _vel_joy[1]*(1/10.0);
                _cmd_p[2] += _vel_joy[2]*(1/10.0);

                cyaw = _cmd_yaw + _vel_joy_yaw*(1/10.0);

                while ( cyaw > M_PI ) cyaw -= 2*M_PI;
                while ( cyaw < -M_PI ) cyaw += 2*M_PI;

                _cmd_yaw = cyaw;

            }
        }
        /*

        if( phase1_ctrl ) {
            
            eyaw = _lp_yaw - M_PI/2.0;            
            if(_lp_yaw > 0.0 )        
                _ref_dyaw = camera_yaw_vel*-0.05*eyaw;
            else
                _ref_dyaw = camera_yaw_vel*0.05*eyaw;
            

            if( norm(eyaw) < 0.1 ) {

                ep << _lp[1], -_lp[0], 0.0;
                //ep << 0, 1, 0;
                
                cout << "lp2: " << _lp[2]  << endl;
                ep[2] =( _lp[2] > 1) ? (_lp[2]-1) : 0.0;
                ep[0] = 0.0;
                //ep[2] = 0.0;
                cout << "ep: " << ep.transpose() << endl;
                dvel = 0.1*ep;
                cout << "dvel: " << dvel.transpose() << endl;


                _cmd_p[0] += camera_0vel*dvel[0]*(1/10.0);
                _cmd_p[1] += camera_1vel*dvel[1]*(1/10.0);
                _cmd_p[2] += camera_2vel*dvel[2]*(1/10.0);
                //_ref_p = _cmd_p;
                cout << "_cmd_p: " << _cmd_p.transpose() << endl;
                //_cmd_p = _ref_p;
                
                //check this!
            }


        }

        if( phase2_ctrl ) { 

        }
        */

        r.sleep();
    }
}

void SIMPLE_CLIENT::run(){
    boost::thread position_controller_t( &SIMPLE_CLIENT::position_controller, this);
    boost::thread main_loop_t( &SIMPLE_CLIENT::main_loop, this );
    ros::spin();
}


int main(int argc, char** argv ) {
    ros::init(argc, argv, "ndt_joy_client");
    SIMPLE_CLIENT sc;
    sc.run();
    return 0;
}
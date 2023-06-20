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
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Int32.h"

using namespace Eigen;
using namespace std;

enum STATUS { POSITION, ADMITTANCE, INTERACTION};

class SIMPLE_CLIENT {


    public:
        SIMPLE_CLIENT();
        void position_controller();
		void run();
		void localization_cb ( geometry_msgs::PoseStampedConstPtr msg );
		void mavros_state_cb( mavros_msgs::State mstate);
        void joy_cb( sensor_msgs::JoyConstPtr j );
        void joy_ctrl();
        void raise(); 
        void main_loop();
        void sensorWrench_cb(geometry_msgs::WrenchStamped msg);
        void forceSetpointRise(double riseTime, double desX);

    private:

        ros::NodeHandle _nh;
        ros::Publisher _target_pub;
        ros::Publisher _desWrench_pub; // = n.advertise<geometry_msgs::WrenchStamped>("/NDT/des_interaction_force", 1);

        ros::Subscriber _localization_sub;
        ros::Subscriber _joy_data_sub;
        ros::Subscriber _mavros_state_sub;
        ros::Subscriber _wrench_sub;
        bool _first_local_pos;
        bool _enable_joy;
        bool _joy_ctrl;
        bool _joy_ctrl_active;
        bool _enable_openarm;
        bool _enable_closearm;
        bool _enable_admittance;
        bool _enable_interaction;
        bool _enable_home;
        bool _enable_pump;
        bool _first_wrench;
        double _currForce;

        STATUS _arm_status;
        // --- Desired state
        Vector3d _cmd_p;
        double _cmd_dyaw;       
        Vector3d _w_p;
        Vector3d _vel_joy;
        Vector4d _w_q;
        float _mes_yaw;
        mavros_msgs::State _mstate;
        double _vel_joy_dyaw;

        int joy_ax0;
        int joy_ax1;
        int joy_ax2;
        int joy_ax3;

        int camera_0vel;
        int camera_1vel;
        int camera_2vel;
        int camera_yaw_vel;

};

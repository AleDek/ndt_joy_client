
#include "ndt_joy_client.h"




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
    _wrench_sub         = _nh.subscribe("/NDT/Sensor_wrench", 1, &SIMPLE_CLIENT::sensorWrench_cb, this);           
    _desWrench_pub      = _nh.advertise<geometry_msgs::WrenchStamped>("/NDT/des_interaction_force", 1);

    _vel_joy << 0.0, 0.0, 0.0;

    _first_local_pos = false;
    _enable_joy = false;

    _enable_openarm  = false;
    _enable_closearm = false;
    _enable_admittance = false;
    _enable_interaction = false;
    _joy_ctrl_active =  false;
    _enable_pump = false;
    _enable_home = false;
    _first_wrench = false;
    _currForce = 0.0;
    
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
    mavros_msgs::PositionTarget::FORCE;

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    _cmd_p = _w_p;
    _cmd_yaw = _mes_yaw;

    while (ros::ok()) {
        if( _mstate.mode != "OFFBOARD" ) {
            _cmd_p = _w_p;
            _cmd_yaw = _mes_yaw;
        } // No control: follow localization

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


void SIMPLE_CLIENT::joy_cb( sensor_msgs::JoyConstPtr j ) {

    // LOGITECH F710
    // Switch on D(DirectInput) and Mode led OFF
    if( j->buttons[1] == 1 ) _enable_joy = true;
    if( j->buttons[0] == 1 ) _enable_openarm = true;
    if( j->buttons[2] == 1 ) _enable_closearm = true;
    if( j->buttons[3] == 1 ) _enable_admittance = true;
    if( j->buttons[9] == 1 ) _enable_interaction = true;
    if( j->buttons[5] == 1 ) _enable_pump = true;
    if( j->buttons[8] == 1 ) _enable_home = true;
    
    // tocheck
    _vel_joy[0] = joy_ax0*j->axes[1]*0.2;
    _vel_joy[1] = joy_ax1*j->axes[0]*0.2;
    _vel_joy[2] = joy_ax2*j->axes[3]*0.2;
    _vel_joy_dyaw = joy_ax3*j->axes[2]*0.2;
    
    // STEELSERIES
    /*if( j->buttons[0] == 1 ) _enable_joy = true;
    if( j->buttons[2] == 1 ) _enable_openarm = true;
    if( j->buttons[1] == 1 ) _enable_closearm = true;
    if( j->buttons[3] == 1 ) _enable_admittance = true;
    if( j->buttons[9] == 1 ) _enable_interaction = true;
    if( j->buttons[5] == 1 ) _enable_pump = true;
    if( j->buttons[18] == 1 ) _enable_home = true;
    
    _vel_joy[0] = joy_ax0*j->axes[1]*0.2;
    _vel_joy[1] = joy_ax1*j->axes[0]*0.2;
    _vel_joy[2] = joy_ax2*j->axes[5]*0.2;
    _vel_joy_dyaw = joy_ax3*j->axes[2]*0.2;*/

}



void SIMPLE_CLIENT::joy_ctrl () {

    ros::Rate r(100);
    
    _joy_ctrl_active = true;
    cout << "Activating joy control" << endl;
    
    while ( ros::ok() && _joy_ctrl ) {

        if( _mstate.mode == "OFFBOARD" ) {
            _cmd_p[0] += _vel_joy[0]*(1/100.0);
            _cmd_p[1] += _vel_joy[1]*(1/100.0);
            _cmd_p[2] += _vel_joy[2]*(1/100.0);
            _cmd_yaw += _vel_joy_dyaw*(1/100.0);
        }

        _joy_ctrl_active = true;
        r.sleep();
    }

    cout << "Deactivating joy control" << endl;
    _joy_ctrl_active = false; 
}


void SIMPLE_CLIENT::forceSetpointRise(double riseTime, double desX) {
	
    while(!_first_wrench ) usleep(0.1*1e6); 

	geometry_msgs::WrenchStamped desWrench;
	desWrench.wrench.force.x  = desWrench.wrench.force.y =  desWrench.wrench.force.z  = 0.0;
	desWrench.wrench.torque.x = desWrench.wrench.torque.y = desWrench.wrench.torque.z = 0.0;

	if(riseTime <= 0){
		desWrench.wrench.force.x = desX;
		_desWrench_pub.publish(desWrench);
		return;
	}

	double f0 = _currForce;
	double f = 0;
	double df = (desX - f0) / riseTime;
	auto rTime = ros::Duration();
	rTime.fromSec(riseTime);
	auto startTime = ros::Time::now();
	auto currTime = startTime;

	while(currTime - startTime < rTime){
		f = f0 + df * (currTime - startTime).toSec();
		
		desWrench.wrench.force.x = f;
		_desWrench_pub.publish(desWrench);

		currTime = ros::Time::now();
	}
}

void SIMPLE_CLIENT::sensorWrench_cb(geometry_msgs::WrenchStamped msg){
	_currForce = msg.wrench.force.z;
	_first_wrench = true;
}


void SIMPLE_CLIENT::main_loop () {

    int enable_joy_cnt = 0;

    int enable_openarm_cnt = 0;
    int enable_closearm_cnt = 0;
    int enable_admittance_cnt = 0;
    int enable_interaction_cnt = 0;
    int enable_home_cnt = 0;

    _joy_ctrl_active = false;
    _joy_ctrl = false;
    
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
    ros::ServiceClient home_arm_srv                = _nh.serviceClient<ndt_core_interface::close>("/NDT/home_arm");
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
    ndt_core_interface::close home_srv;


	ndt_core_interface::interaction_mode enable_int_srv;
	enable_int_srv.request.enable = true;
	ndt_core_interface::interaction_mode disable_int_srv;
	disable_int_srv.request.enable = false;

    _arm_status = POSITION;
	geometry_msgs::WrenchStamped desWrench;

    while( ros::ok() ) {

        enable_joy_cnt++;
        enable_openarm_cnt++;
        enable_closearm_cnt++;
        enable_admittance_cnt++;
        enable_interaction_cnt++;
        enable_home_cnt++;

        if( _enable_joy == true && enable_joy_cnt > 50) {
            _joy_ctrl = !_joy_ctrl;
            enable_joy_cnt = 0;
            _enable_joy = false;
        }

        if( _enable_openarm && enable_openarm_cnt > 50) {
            enable_openarm_cnt = 0;
            open_arm_srv.call( open_srv );
            _enable_openarm = false;
            _arm_status = POSITION;
        }

        if( _enable_closearm && enable_closearm_cnt > 50) {
            enable_closearm_cnt = 0;
			close_arm_srv.call( close_srv );
            _enable_closearm = false;
            _arm_status = POSITION;
        }

        if( _enable_pump ) {
            _enable_pump = false;
            pumpTime.data = 1;
            gelPump_pub.publish( pumpTime );
            sleep(1);
            pumpTime.data = 0;
            gelPump_pub.publish( pumpTime );
        }


        if( _enable_admittance && enable_admittance_cnt > 50) {
            if( _arm_status == POSITION )  {
                reset_bias_srv.call(rnias_srv);
                usleep(0.2*1e6);
                enable_admittance_cnt = 0;
                enable_admittance_srv.call( enable_adm_srv );
                _enable_admittance = false;
                _arm_status = ADMITTANCE;
            }
            else if ( _arm_status == INTERACTION ) {
                //force to zero, admittance
                _enable_admittance = false;
                enable_admittance_cnt = 0;
                forceSetpointRise(3.0, 0.0); 
                enable_admittance_srv.call( enable_adm_srv );
                _arm_status = ADMITTANCE;
            }
            else {
                enable_admittance_cnt = 0;
                enable_admittance_srv.call( disable_adm_srv );
                _enable_admittance = false;
                _arm_status = POSITION;
            }
        }

        if( _enable_interaction && enable_interaction_cnt > 50) {
            if( _arm_status == INTERACTION )  {
                
                _enable_interaction = false;
                disable_interaction_srv.call( disable_int_srv );
                _arm_status = POSITION;
                enable_interaction_cnt = 0;

            } //Disable interaction from an interaction state
            else if( _arm_status == ADMITTANCE ) {  
                _enable_interaction = false;
                enable_interaction_cnt = 0;

                //TO CHEEEEEEEEEEECK
                desWrench.wrench.force.x  = desWrench.wrench.force.y =  desWrench.wrench.force.z  = 0.0;
                desWrench.wrench.torque.x = desWrench.wrench.torque.y = desWrench.wrench.torque.z = 0.0;
                desWrench.wrench.force.x = _currForce;
	        	_desWrench_pub.publish(desWrench);

                enable_interaction_srv.call( enable_int_srv );
                forceSetpointRise(3.0, 2.5); 

                _arm_status = INTERACTION;
            } //Enable interaction from admittance state
        }

        if( _enable_home && enable_home_cnt > 50 ) {
            _enable_home = false;
            enable_home_cnt = 0;
			home_arm_srv.call(home_srv);          
            _arm_status = POSITION;
        }

        //Enable disable joy ctrl
        if( _joy_ctrl && !_joy_ctrl_active ) {     
            boost::thread joy_ctrl_t (&SIMPLE_CLIENT::joy_ctrl, this);
            usleep(0.2*1e6);
        }
        else if (!_joy_ctrl ) {
            if (_joy_ctrl_active == true ) {
                _joy_ctrl_active = false;
            }
        }

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
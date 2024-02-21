
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

    if( !_nh.getParam("wall_max_yaw", _wall_max_yaw)) {
        _wall_max_yaw = M_PI/2.0;
    }

    if( !_nh.getParam("wall_max_dist", _wall_max_dist)) {
        _wall_max_dist = 3.0;
    }

    if( !_nh.getParam("d_safe", _d_safe)) {
        _d_safe = 1.0;
    }

    if( !_nh.getParam("delta_approch", _delta_approch)) {
        _delta_approch = 0.3;
    }

    if( !_nh.getParam("Kp_x", _Kp_x)) {
        _Kp_x = 0.2;
    }

    if( !_nh.getParam("Kp_yaw", _Kp_yaw)) {
        _Kp_yaw = 0.3;
    }

    // if( !_nh.getParam("Kd_x", _Kd_x)) {
    //     _Kd_x = 0.0;
    // }

    // if( !_nh.getParam("Kd_yaw", _Kd_yaw)) {
    //     _Kd_yaw = 0.08;
    // }

    if( !_nh.getParam("eps_x", _eps_x)) {
        _eps_x = 0.001;
    }

    if( !_nh.getParam("eps_yaw", _eps_yaw)) {
        _eps_yaw = 0.001;
    }

    if( !_nh.getParam("K_ff_x_dot", _K_ff_x_dot)) {
        _K_ff_x_dot = 1.0;
    }

    if( !_nh.getParam("vx_cruise", _vx_cruise)) {
        _vx_cruise = 0.15;
    }

    if( !_nh.getParam("vx_max", _vx_max)) {
        _vx_max = 0.2;
    }

    if( !_nh.getParam("vyaw_max", _vyaw_max)) {
        _vyaw_max = 0.1;
    }


	_target_pub = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);

    _localization_sub   = _nh.subscribe( "/mavros/local_position/pose", 1, &SIMPLE_CLIENT::localization_cb, this);
    _mavros_state_sub   = _nh.subscribe( "/mavros/state", 1, &SIMPLE_CLIENT::mavros_state_cb, this);
    _joy_data_sub       = _nh.subscribe("/joy", 1, &SIMPLE_CLIENT::joy_cb, this);
    _wrench_sub         = _nh.subscribe("/NDT/Sensor_wrench", 1, &SIMPLE_CLIENT::sensorWrench_cb, this);           
    _desWrench_pub      = _nh.advertise<geometry_msgs::WrenchStamped>("/NDT/des_interaction_force", 1);


    _range_l_sub = _nh.subscribe("/NDT/tfluna_left", 1, &SIMPLE_CLIENT::range_l_cb, this);
    _range_r_sub = _nh.subscribe("/NDT/tfluna_right", 1, &SIMPLE_CLIENT::range_r_cb, this);
    // _wall_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("/NDT/wall_pose", 1);
    _wall_pose_filtered_pub = _nh.advertise<geometry_msgs::PoseStamped>("/NDT/wall_pose_filtered", 1);
    _wall_xb_sp_pub = _nh.advertise<std_msgs::Float32>("/NDT/wall_dist_sp", 1);

    const float samplingrate = 100.0; // Hz
    const float cutoff_frequency = 2.0; // Hz
    _butt_wall_x.setup (samplingrate, cutoff_frequency);
    _butt_wall_yaw.setup(samplingrate, cutoff_frequency);

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
    
    _first_range_l = false;
    _first_range_r = false;
    _range_mount_dist = 0.42;
}



void SIMPLE_CLIENT::mavros_state_cb( mavros_msgs::State mstate) {
    _mstate = mstate;
}


void SIMPLE_CLIENT::localization_cb ( geometry_msgs::PoseStampedConstPtr msg ) {
    _w_p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    _w_q = Eigen::Vector4d( msg->pose.orientation.w,  msg->pose.orientation.x,  msg->pose.orientation.y,  msg->pose.orientation.z);
    Eigen::Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat( _w_q ) );
    _mes_yaw = rpy(2);

    // Quaternionf q;
    // q = AngleAxisf(0.0, Vector3f::UnitX())
    //     * AngleAxisf(0.0, Vector3f::UnitY())
    //     * AngleAxisf(_mes_yaw, Vector3f::UnitZ());
    // Vector4d w_q ( q.w(), q.x(), q.y(), q.z() );
    // _w_q = w_q / w_q.norm() ;

    _first_local_pos = true;    
}

void SIMPLE_CLIENT::range_l_cb(const sensor_msgs::RangeConstPtr &range_left_msg){
    _range_l_val = range_left_msg->range;
    _first_range_l = true;
}

void SIMPLE_CLIENT::range_r_cb(const sensor_msgs::RangeConstPtr &range_right_msg){
    _range_r_val = range_right_msg->range;
    _first_range_r = true;
}

void SIMPLE_CLIENT::joy_cb( sensor_msgs::JoyConstPtr j ) {

    // LOGITECH F710 //Switch on D(DirectInput) and Mode led OFF
    if( j->buttons[1] == 1 ) _enable_joy = true;
    if( j->buttons[0] == 1 ) _enable_openarm = true;
    if( j->buttons[2] == 1 ) _enable_closearm = true;
    if( j->buttons[3] == 1 ) _enable_admittance = true;
    if( j->buttons[9] == 1 ) _enable_interaction = true;
    if( j->buttons[5] == 1 ) _enable_pump = true;
    if( j->buttons[8] == 1 ) _enable_home = true;
    // if( j->buttons[4] == 1 ) _enable_wall_ctrl = true;
    
    // tocheck
    _vel_joy[0] = joy_ax0*j->axes[3]*0.2;
    _vel_joy[1] = joy_ax1*j->axes[2]*0.2;
    _vel_joy[2] = joy_ax2*j->axes[1]*0.2;
    _vel_joy_dyaw = joy_ax3*j->axes[0]*0.2;
    
    // XBOX
    // if( j->buttons[0] == 1 ) _enable_joy = true;
    // if( j->buttons[2] == 1 ) _enable_openarm = true;
    // if( j->buttons[1] == 1 ) _enable_closearm = true;
    // if( j->buttons[3] == 1 ) _enable_admittance = true;
    // if( j->buttons[9] == 1 ) _enable_interaction = true;
    // if( j->buttons[5] == 1 ) _enable_pump = true;
    // if( j->buttons[18] == 1 ) _enable_home = true;
    
    // _vel_joy[0] = joy_ax0*j->axes[4]*0.2;
    // _vel_joy[1] = joy_ax1*j->axes[3]*0.2;
    // _vel_joy[2] = joy_ax2*j->axes[1]*0.2;
    // _vel_joy_dyaw = joy_ax3*j->axes[0]*0.2;

}

void SIMPLE_CLIENT::sensorWrench_cb(geometry_msgs::WrenchStamped msg){
	_currForce = msg.wrench.force.z;
	_first_wrench = true;
}



void SIMPLE_CLIENT::mavros_setpoint_publisher(){

    ros::Rate r(100);

    mavros_msgs::PositionTarget ptarget;
    ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    ptarget.type_mask =
    // mavros_msgs::PositionTarget::IGNORE_VX |
    // mavros_msgs::PositionTarget::IGNORE_VY |
    // mavros_msgs::PositionTarget::IGNORE_VZ |
    mavros_msgs::PositionTarget::IGNORE_AFX |
    mavros_msgs::PositionTarget::IGNORE_AFY |
    mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::FORCE;

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    

    _cmd_p = _w_p;
    _cmd_yaw = _mes_yaw;
    _cmd_v[0] = 0.00;
    _cmd_v[1] = 0.00;
    _cmd_v[2] = 0.00;
    _cmd_dyaw = 0.00;

    while (ros::ok()) {
        if( _mstate.mode != "OFFBOARD" ) { // No control: follow localization
            _cmd_p = _w_p;
            _cmd_yaw = _mes_yaw;
            _cmd_v[0] = 0.00;
            _cmd_v[1] = 0.00;
            _cmd_v[2] = 0.00;
            _cmd_dyaw = 0.00;
        } 
        //---Publish command
        ptarget.header.stamp = ros::Time::now();
        ptarget.position.x = _cmd_p[0];
        ptarget.position.y = _cmd_p[1];
        ptarget.position.z = _cmd_p[2];
        ptarget.yaw = _cmd_yaw;
        ptarget.velocity.x = _cmd_v[0];
        ptarget.velocity.y = _cmd_v[1];
        ptarget.velocity.z = _cmd_v[2];
        ptarget.yaw_rate = _cmd_dyaw;
        _target_pub.publish( ptarget );
        //---

        r.sleep();
    }
}


void SIMPLE_CLIENT::joy_ctrl () { //diventa ibrido, controllo da joy su y e z, ma setpoint di distanza dal muro con x e yaw

    ros::Rate r(100);

    while( !(_first_range_l && _first_range_r))
        usleep(0.1*1e6);
    ROS_INFO("First range l r arrived!");

    
    _joy_ctrl_active = false;
    ROS_INFO("wall control loop active");
    // geometry_msgs::PoseStamped wall_pose; //TODO debug only
    geometry_msgs::PoseStamped wall_pose_filtered; //TODO debug only
    std_msgs::Float32 wall_x_sp;
    wall_pose_filtered.header.frame_id = "base_link";
    wall_pose_filtered.pose.position.z = -0.30;
    wall_pose_filtered.pose.position.y = 0.00;
    // wall_pose_filtered = wall_pose; //TODO debug only
    double wall_xb_raw =0.00;
    double wall_yaw_raw =0.00;
    // double _wall_xb =0.00;
    // double _wall_yaw =0.00;
    double e_x = 0.00;
    double e_yaw = 0.00;
    double e_x_old = 0.00;
    double e_yaw_old = 0.00;
    // double Kp_x = 0.2;
    // double Kp_yaw = 0.3;
    // double eps_x = 0.001;
    // double eps_yaw = 0.001;
    Eigen::Vector3d u_b;
    Eigen::Vector3d u_w;
    double u_dyaw;
    Eigen::Matrix3d R_b_w;

    _wall_xb_sp = (_range_l_val + _range_r_val) / 2.00;
    _wall_xb_dot_sp = 0.00;
    _green_light_for_ctrl = false;

    while ( ros::ok() ) {
        //compute wall dist and yaw
        wall_xb_raw= (_range_l_val + _range_r_val) / 2.00; //TODO can be local to th
        wall_yaw_raw = atan((_range_r_val - _range_l_val) / _range_mount_dist);
        // Eigen::Vector4d wall_q_raw = utilities::rot2quat(utilities::XYZ2R(Eigen::Vector3d(0.0,0.0,_wall_yaw_raw))); //TODO debug only
        _wall_xb = _butt_wall_x.filter(wall_xb_raw);
        _wall_yaw = _butt_wall_yaw.filter( wall_yaw_raw);
        Eigen::Vector4d wall_q = utilities::rot2quat(utilities::XYZ2R(Eigen::Vector3d(0.0,0.0,_wall_yaw)));
        
        if(fabs(_wall_yaw) <_wall_max_yaw && fabs(_wall_xb) <_wall_max_dist) _green_light_for_ctrl = true;
        else _green_light_for_ctrl = false;
        
        if( _joy_ctrl && _green_light_for_ctrl) { //compute control only if drone is in offboard
            
            e_x = _wall_xb_sp - _wall_xb;
            if(fabs(e_x) < _eps_x) e_x = 0.00;
            e_yaw = -_wall_yaw;
            if(fabs(e_yaw) < _eps_yaw) e_yaw = 0.00;

            u_b << -(_Kp_x*e_x) -(_K_ff_x_dot*_wall_xb_dot_sp) ,_vel_joy[1], _vel_joy[2]; //wall ctrl + joy ctl on y_b and z_b
            u_dyaw = -(_Kp_yaw*e_yaw );

            //saturate velocity output
            if(u_b(0) > _vx_max) u_b(0) =_vx_max;  
            else if(u_b(0) < -_vx_max) u_b(0) = -_vx_max;  

            if(u_b(1) > _vx_max) u_b(1) =_vx_max;  
            else if(u_b(1) < -_vx_max) u_b(1) = -_vx_max;  

            if(u_b(2) > _vx_max) u_b(2) =_vx_max;  
            else if(u_b(2) < -_vx_max) u_b(2) = -_vx_max;  

            if(u_dyaw > _vyaw_max) u_dyaw =_vyaw_max;  
            else if(u_dyaw < -_vyaw_max) u_dyaw = -_vyaw_max;  

            //rotate in global frame
            R_b_w = utilities::QuatToMat(_w_q);
            u_w = R_b_w*u_b;

            // e_x_old =e_x;
            // e_yaw_old =e_yaw;
            
            _cmd_p[0] += u_w[0]*(1/100.0);
            _cmd_p[1] += u_w[1]*(1/100.0);
            _cmd_p[2] += u_w[2]*(1/100.0);
            _cmd_yaw += u_dyaw*(1/100.0);

            _cmd_v[0] = u_w[0]; 
            _cmd_v[1] = u_w[1];
            _cmd_v[2] = u_w[2];
            _cmd_dyaw = u_dyaw;

            _joy_ctrl_active = true;
        }
        else {  //if not in offboard reset integrals and outputs and overwrite sp 
            _wall_xb_sp = _wall_xb; 
            _wall_xb_dot_sp =0.00;
            _cmd_p = _w_p;
            _cmd_yaw = _mes_yaw;
            _cmd_v[0] = 0.00;
            _cmd_v[1] = 0.00;
            _cmd_v[2] = 0.00;
            _cmd_dyaw = 0.00;
            _joy_ctrl_active = false;
        } // No control: follow localization
        
        //publish wall pose (raw and filter, for debug)
        // wall_pose.pose.position.x = _wall_xb_raw; //TODO debug only
        // wall_pose.pose.orientation.w = wall_q_raw(0);
        // wall_pose.pose.orientation.x = wall_q_raw(1);
        // wall_pose.pose.orientation.y = wall_q_raw(2);
        // wall_pose.pose.orientation.z = wall_q_raw(3);
        // _wall_pose_pub.publish(wall_pose); //TODO debug only

        wall_pose_filtered.pose.position.x = _wall_xb; //TODO debug only
        wall_pose_filtered.pose.orientation.w = wall_q(0);
        wall_pose_filtered.pose.orientation.x = wall_q(1);
        wall_pose_filtered.pose.orientation.y = wall_q(2);
        wall_pose_filtered.pose.orientation.z = wall_q(3);
        wall_x_sp.data = _wall_xb_sp;
        _wall_pose_filtered_pub.publish(wall_pose_filtered); //TODO debug only
        _wall_xb_sp_pub.publish(wall_x_sp);

        // _joy_ctrl_active = true;
        r.sleep();
    }

    ROS_INFO("stop joy wall control loop");
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



void SIMPLE_CLIENT::traj_compute_scalar(double p_i, double p_f){
    ros::Rate rate(100.0);
    double dt = 1.0/100.0;
    Eigen::Matrix<double,6,6> A;
    double e = p_f-p_i;
    double T = fabs(e)/_vx_cruise;
    ROS_INFO("new traj, T=%f", T);
    double s_f = fabs(e); //arclength;
    Eigen::VectorXd b(6);
    b << 0.0, 0.0, 0.0, s_f, 0.0, 0.0; //qi qi_d qi_dd qf qf_d qf_dd : initial final pos vel acc
    A << 0,           0,           0,          0,        0,  1,
         0,           0,           0,          0,        1,  0,
         0,           0,           0,          1,        0,  0,
         pow(T,5),    pow(T,4),    pow(T,3),   pow(T,2), T,  1,
         5*pow(T,4),  4*pow(T,3),  3*pow(T,2), 2*T,      1,  0,
         20*pow(T,3), 12*pow(T,2), 6*T,        1,        0,  0;
 
    Eigen::VectorXd x = A.inverse()*b;
    double s, s_d, s_dd;
    double t = 0.0;
    int N = T/dt;
    int i =0;
    double t_0 =ros::Time::now().toSec();
    ROS_INFO("new traj, N=%d", N);
    while(ros::ok() && i <=N){
        //t = ros::Time::now().toSec()-t_0;
        s    = x(0)*pow(t,5)    +x(1)*pow(t,4)    +x(2)*pow(t,3)   +x(3)*pow(t,2) +x(4)*t +x(5);
        s_d  = x(0)*5*pow(t,4)  +x(1)*4*pow(t,3)  +x(2)*3*pow(t,2) +x(3)*2*t      +x(4);
        s_dd = x(0)*20*pow(t,3) +x(1)*12*pow(t,2) +x(2)*6*t        +x(3);
        
        _wall_xb_sp = p_i + s*e/s_f;
        _wall_xb_dot_sp = s_d*e/s_f; //not used, just position //TODO use ff ???
        // _a = s_dd*e/s_f;
        i++;
        t+=dt;
        rate.sleep();
    }   
}

void SIMPLE_CLIENT::main_loop () {

    int enable_joy_cnt = 0;
    // int enable_wall_ctrl_cnt = 0;
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
    ros::ServiceClient home_arm_srv            = _nh.serviceClient<ndt_core_interface::close>("/NDT/home_arm");
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
        // enable_wall_ctrl_cnt++;

        // if( _enable_wall_ctrl == true && enable_wall_ctrl_cnt > 50) {
        //     _wall_ctrl = !_wall_ctrl;
        //     enable_wall_ctrl_cnt = 0;
        //     _enable_wall_ctrl = false;
        // }

        if( _enable_joy == true && enable_joy_cnt > 50) { //now is hybrid joy + wall control
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
            // boost::thread joy_ctrl_t (&SIMPLE_CLIENT::joy_ctrl, this);
            if(_green_light_for_ctrl){
                ROS_INFO("Green Light, Activating joy/wall control");
                r.sleep();
                ROS_INFO("going to d_safe");
                traj_compute_scalar(_wall_xb_sp,_d_safe);
            } 
            else{
                ROS_WARN("out of range for activating control");
                _joy_ctrl = false;
            }
            // usleep(0.2*1e6);
        }
        else if (!_joy_ctrl ) {
            if (_joy_ctrl_active == true ) {
                _joy_ctrl_active = false;
                ROS_INFO("Deactivating joy/wall control");
            }
        }

        r.sleep();
    }
}

void SIMPLE_CLIENT::run(){
    boost::thread mavros_setpoint_publisher_t( &SIMPLE_CLIENT::mavros_setpoint_publisher, this);
    boost::thread joy_ctrl_t (&SIMPLE_CLIENT::joy_ctrl, this);
    boost::thread main_loop_t( &SIMPLE_CLIENT::main_loop, this );
    ros::spin();
}


int main(int argc, char** argv ) {
    ros::init(argc, argv, "ndt_joy_client");
    SIMPLE_CLIENT sc;
    sc.run();
    return 0;
}
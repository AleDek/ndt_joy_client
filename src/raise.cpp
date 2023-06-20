#include "ndt_joy_client.h"


// Todo in SIMPLE_CLIENT:
    // call raise as thread
    //
    // add ros subscriber to /NDT/Sensor_wrench (and callback)
    // flag _firstWrench true when read
    // add ros publisher to /NDT/des_interaction_force

void SIMPLE_CLIENT::raise(double desForce, double ms) {
    constexpr double rate = 100;    // Hz

    // wait for force reading
    while(!_firstWrench)
        ros::spinOnce();

    // apply force raise from read value to desired value
    double initForce = _wrench.wrench.force.z;       // current force (.x or .z ?)
    double currSetpoint = initForce;

    // generate ramp from initForce to currSetpoint
    double nsteps = (ms*rate/1000.0);
    double step = (desForce - initForce) / nsteps;

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.force.x = wrench_msg.force.y = wrench_msg.force.z = 0;
    wrench_msg.torque.x = wrench_msg.torque.y = wrench_msg.torque.z = 0;

    ros::Rate r(rate);

    for(int i=0; i<int(nsteps); ++i){
        currSetpoint += step;
        wrench_msg.force.z = currSetpoint;      // (.x or .z ?)

        // publish cmd
        wrench_msg.header.stamp = ros::Time::now();
        _desForcePub.publish(wrench_msg);

        r.sleep();
    }
}
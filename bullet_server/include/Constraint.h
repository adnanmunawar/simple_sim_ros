#ifndef CONSTRAINTS_HH
#define CONSTRAINTS_HH

#include "Body.h"
class Body;
class Constraint
{
    ros::NodeHandle nh_;
    btTypedConstraint* constraint_;

    btDiscreteDynamicsWorld* dynamics_world_;
    ros::Publisher* marker_array_pub_;
    std::map<std::string, float> command_;
    std::map<std::string, ros::Publisher> pubs_;
    std::map<std::string, ros::Subscriber> subs_;
    visualization_msgs::MarkerArray marker_array_;
    float max_motor_impulse_;
    void commandCallback(const std_msgs::Float64::ConstPtr msg, const std::string motor_name);
public:
    Constraint(
            const std::string name,
            unsigned int type,
            Body* body_a,
            Body* body_b,
            geometry_msgs::Point pivot_in_a,
            geometry_msgs::Point pivot_in_b,
            geometry_msgs::Vector3 axis_in_a,
            geometry_msgs::Vector3 axis_in_b,
            const double lower_lin_lim,
            const double upper_lin_lim,
            const double lower_ang_lim,
            const double upper_ang_lim,
            const float max_motor_impulse,
            btDiscreteDynamicsWorld* dynamics_world,
            ros::Publisher* marker_array_pub);
    ~Constraint();

    const std::string name_;
    Body* body_a_;
    Body* body_b_;
    void update();
};

#endif

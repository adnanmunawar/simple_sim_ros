#ifndef BODY_HH
#define BODY_HH

#include "bullet_server.h"
#include "Constraint.h"

class BulletServer;
class Constraint;
class Body
{
    BulletServer* parent_;
    tf::TransformBroadcaster* br_;
    // ros::Publisher* marker_pub_;
    ros::Publisher* marker_array_pub_;
    visualization_msgs::MarkerArray marker_array_;

    // TODO(lucasw) put this in a child class
    // for triangle meshes
    btVector3* vertices_;
    int* indices_;
    btTriangleIndexVertexArray* index_vertex_arrays_;

    std::map<std::string, Constraint*> constraints_;
    btCollisionShape* shape_;
    btDefaultMotionState* motion_state_;

    btDiscreteDynamicsWorld* dynamics_world_;
    int state_;
public:
    Body(BulletServer* parent,
         const std::string name,
         unsigned int type,
         const float mass,
         geometry_msgs::Pose pose,
         geometry_msgs::Twist twist,
         geometry_msgs::Vector3 scale,
         btDiscreteDynamicsWorld* dynamics_world,
         tf::TransformBroadcaster* br,
         ros::Publisher* marker_array_pub_);
    // heightfield
    Body(BulletServer* parent,
         const std::string name,
         // unsigned int type,
         // geometry_msgs::Pose pose,
         // geometry_msgs::Vector3 scale,
         cv::Mat& image,
         const float resolution,
         const float height_scale,
         const bool flip_quad_edges,
         btDiscreteDynamicsWorld* dynamics_world,
         tf::TransformBroadcaster* br,
         ros::Publisher* marker_array_pub);
    ~Body();

    // keep track of constraints attached to this body
    void addConstraint(Constraint* constraint);
    void removeConstraint(const Constraint* constraint);
    const std::string name_;
    btRigidBody* rigid_body_;
    void update();
};

#endif

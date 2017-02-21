
#ifndef BULLET_SERVER_HH
#define BULLET_SERVER_HH

#include <boost/functional/hash.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <bullet_server/AddBody.h>
#include <bullet_server/AddCompound.h>
#include <bullet_server/AddConstraint.h>
#include <bullet_server/AddHeightfield.h>
#include <bullet_server/AddImpulse.h>
#include <bullet_server/Anchor.h>
#include <bullet_server/Body.h>
#include <bullet_server/Constraint.h>
#include <bullet_server/Heightfield.h>
#include <bullet_server/Impulse.h>
#include <bullet_server/SoftBody.h>
#include <bullet_server/SoftConfig.h>
#include <bullet_server/Material.h>
#include <bullet_server/Node.h>
#include <bullet_server/Link.h>
#include <bullet_server/Face.h>
#include <bullet_server/Tetra.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "Constraint.h"
//#include "Body.h"
#include "SoftBody.h"

class Constraint;
class Body;
class SoftBody;

// TODO(lucasw) replace this with something better, numbers aren't random enough
// http://stackoverflow.com/questions/2535284/how-can-i-hash-a-string-to-an-int-using-c
static unsigned short hash(const char *str) {
    unsigned short hash = 5381;
    int c;

    while (c = *str++) {
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
    }

    return hash;
}

class BulletServer
{
  ros::NodeHandle nh_;
  ros::ServiceServer add_compound_;
  bool addCompound(bullet_server::AddCompound::Request& req,
                   bullet_server::AddCompound::Response& res);
  ros::Subscriber body_sub_;
  void bodyCallback(const bullet_server::Body::ConstPtr& msg);
  void softBodyCallback(const bullet_server::SoftBody::ConstPtr& msg);
  ros::Subscriber constraint_sub_;
  void constraintCallback(const bullet_server::Constraint::ConstPtr& msg);
  ros::Subscriber impulse_sub_;
  void impulseCallback(const bullet_server::Impulse::ConstPtr& msg);
  ros::Subscriber heightfield_sub_;
  void heightfieldCallback(const bullet_server::Heightfield::ConstPtr& msg);

  tf::TransformBroadcaster br_;
  float period_;
  // ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;

  bool rigid_only_;
  btBroadphaseInterface* broadphase_;

  btDefaultCollisionConfiguration* collision_configuration_;
  btSoftBodyRigidBodyCollisionConfiguration* soft_rigid_collision_configuration_;

  btCollisionDispatcher* dispatcher_;
  // only used with opencl
  // btSoftBodySolver* soft_body_solver_;
  btSequentialImpulseConstraintSolver* solver_;

  btDiscreteDynamicsWorld* dynamics_world_;

  // TODO(lucasw) make this configurable by addCompound
  // instead of hard coded
  btCollisionShape* ground_shape_;
  btDefaultMotionState* ground_motion_state_;
  btRigidBody* ground_rigid_body_;

  btSoftBodyWorldInfo soft_body_world_info_;
  std::map<std::string, SoftBody*> soft_bodies_;
  std::map<std::string, Constraint*> constraints_;

  int init();
public:
  BulletServer();
  ~BulletServer();
  void update();
  void removeConstraint(const Constraint* constraint, const bool remove_from_bodies);
  std::map<std::string, Body*> bodies_;
};


#endif


#ifndef SOFT_BODY_HH
#define SOFT_BODY_HH

#include "bullet_server.h"

class BulletServer;
class SoftBody
{
  BulletServer* parent_;
  btSoftRigidDynamicsWorld* dynamics_world_;
  tf::TransformBroadcaster* br_;
  ros::Publisher* marker_array_pub_;
  visualization_msgs::MarkerArray marker_array_;
  const std::string name_;
public:
  SoftBody(BulletServer* parent,
      const std::string name,
      btSoftBodyWorldInfo* soft_body_world_info,
      const std::vector<bullet_server::Node>& nodes,
      const std::vector<bullet_server::Link>& links,
      const std::vector<bullet_server::Face>& faces,
      const std::vector<bullet_server::Tetra>& tetras,
      const std::vector<bullet_server::Material>& materials,
      const std::vector<bullet_server::Anchor>& anchors,
      const bullet_server::SoftConfig& config,
      btSoftRigidDynamicsWorld* dynamics_world,
      tf::TransformBroadcaster* br,
      ros::Publisher* marker_array_pub);
  ~SoftBody();
  void update();
  btSoftBody* soft_body_;
};

#endif

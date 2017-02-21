#include "SoftBody.h"

SoftBody::SoftBody(BulletServer* parent,
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
    ros::Publisher* marker_array_pub) :
  dynamics_world_(dynamics_world),
  name_(name),
  br_(br),
  marker_array_pub_(marker_array_pub)
{
  btVector3* points = new btVector3[nodes.size()];
  btScalar* masses = new btScalar[nodes.size()];
  for (size_t i = 0; i < nodes.size(); ++i)
  {
    btVector3 pos(nodes[i].position.x, nodes[i].position.y, nodes[i].position.z);
    btScalar mass(nodes[i].mass);
    // this was segfaulting
    // soft_body_->appendNode(pos, mass);
    points[i] = pos;
    masses[i] = mass;
    // ROS_INFO_STREAM(pos << " " << mass);
  }

  soft_body_ = new btSoftBody(soft_body_world_info, nodes.size(), points, masses);
  delete[] points;
  delete[] masses;

  soft_body_->m_cfg.kVCF = config.kVCF;
  soft_body_->m_cfg.kDP = config.kDP;
  soft_body_->m_cfg.kDG = config.kDG;
  soft_body_->m_cfg.kLF = config.kLF;
  soft_body_->m_cfg.kPR = config.kPR;
  soft_body_->m_cfg.kVC = config.kVC;
  soft_body_->m_cfg.kDF = config.kDF;
  soft_body_->m_cfg.kMT = config.kMT;
  soft_body_->m_cfg.kCHR = config.kCHR;
  soft_body_->m_cfg.kKHR = config.kKHR;
  soft_body_->m_cfg.kSHR = config.kSHR;
  soft_body_->m_cfg.kAHR = config.kAHR;
  // TODO(lucasw) cluster stuff
  soft_body_->m_cfg.maxvolume = config.maxvolume;
  soft_body_->m_cfg.timescale = config.timescale;

  btSoftBody::Material* pm = NULL;
  for (size_t i = 0; i < materials.size(); ++i)
  {
    // btSoftBody::Material*
    pm = soft_body_->appendMaterial();
    pm->m_kLST = materials[i].kLST;
    pm->m_kAST = materials[i].kAST;
    pm->m_kVST = materials[i].kVST;
    // const int distance = 1;
    // soft_body_->generateBendingConstraints(distance, pm);
    ROS_INFO_STREAM(name_ << " " << pm->m_kLST);
  }

  for (size_t i = 0; i < links.size(); ++i)
  {
    // TODO(lucasw) need to provide an optional material index
    // for each link
    // With bcheckexist set to true redundant links ought
    // to be filtered out.
    soft_body_->appendLink(links[i].node_indices[0],
      links[i].node_indices[1], pm, true);
  }
  for (size_t i = 0; i < faces.size(); ++i)
  {
    soft_body_->appendFace(faces[i].node_indices[0],
      faces[i].node_indices[1],
      faces[i].node_indices[2], pm);
  }
  for (size_t i = 0; i < tetras.size(); ++i)
  {
    soft_body_->appendTetra(tetras[i].node_indices[0],
      tetras[i].node_indices[1],
      tetras[i].node_indices[2],
      tetras[i].node_indices[3], pm);
  }

#if 0
  for (size_t i = 0; i < materials.size(); ++i)
  {
    btSoftBody::Material* pm = soft_body_->appendMaterial();
    pm->m_kLST = materials[i].kLST;
    pm->m_kAST = materials[i].kAST;
    pm->m_kVST = materials[i].kVST;
    const int distance = 1;
    soft_body_->generateBendingConstraints(distance, pm);
    ROS_INFO_STREAM(name_ << " " << pm->m_kLST);
  }
#endif
  for (size_t i = 0; i < anchors.size(); ++i)
  {
    if (parent->bodies_.count(anchors[i].rigid_body_name) == 0)
    {
      ROS_ERROR_STREAM("no rigid body " << anchors[i].rigid_body_name
        << " to append anchor to");
      continue;
    }
    btRigidBody* rigid_body = parent->bodies_[anchors[i].rigid_body_name]->rigid_body_;
    const btVector3 local_pivot(
      anchors[i].local_pivot.x,
      anchors[i].local_pivot.y,
      anchors[i].local_pivot.z);

    soft_body_->appendAnchor(anchors[i].node_index, rigid_body,
      local_pivot, anchors[i].disable_collision_between_linked_bodies,
      anchors[i].influence);
  }

  {
    visualization_msgs::Marker marker;
    // TODO(lucasw) also have a LINES and TRIANGLE_LIST marker
    marker.type = visualization_msgs::Marker::POINTS;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.ns = "nodes";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash(name_.c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.45;
    marker.color.g = 0.4;
    marker.color.b = 0.65;
    marker_array_.markers.push_back(marker);
  }

  // link markers
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.ns = "links";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash((name_ + "lines").c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.3;
    marker.color.g = 0.67;
    marker.color.b = 0.65;
    marker_array_.markers.push_back(marker);
  }

  // anchor markers
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.ns = "anchors";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash((name_ + "anchors").c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.67;
    marker.color.g = 0.17;
    marker.color.b = 0.95;
    marker_array_.markers.push_back(marker);

    marker.ns = "anchor_pivots";
    marker.color.r = 0.67;
    marker.color.g = 0.87;
    marker.color.b = 0.45;
    marker_array_.markers.push_back(marker);
  }

  // tetra markers
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.ns = "tetras";
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    marker.id = hash((name_ + "tetras").c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.0;
    marker.color.g = 0.67;
    marker.color.b = 0.75;
    marker_array_.markers.push_back(marker);
  }
}

SoftBody::~SoftBody()
{
  if (soft_body_)
  {
    dynamics_world_->removeSoftBody(soft_body_);
    delete soft_body_;
  }
}

void SoftBody::update()
{
  // TODO(lucasw) getAabb

  // TODO(lucasw) instead of hardcoded markers indices, do something better

  btSoftBody::tNodeArray& nodes(soft_body_->m_nodes);
  marker_array_.markers[0].points.clear();
  for (size_t i = 0; i < nodes.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = nodes[i].m_x.getX();
    pt.y = nodes[i].m_x.getY();
    pt.z = nodes[i].m_x.getZ();
    marker_array_.markers[0].points.push_back(pt);
  }

  btSoftBody::tLinkArray& links(soft_body_->m_links);
  marker_array_.markers[1].points.clear();
  for (size_t i = 0; i < links.size(); ++i)
  {
    geometry_msgs::Point pt1;
    pt1.x = links[i].m_n[0]->m_x.getX();
    pt1.y = links[i].m_n[0]->m_x.getY();
    pt1.z = links[i].m_n[0]->m_x.getZ();
    marker_array_.markers[1].points.push_back(pt1);

    geometry_msgs::Point pt2;
    pt2.x = links[i].m_n[1]->m_x.getX();
    pt2.y = links[i].m_n[1]->m_x.getY();
    pt2.z = links[i].m_n[1]->m_x.getZ();
    marker_array_.markers[1].points.push_back(pt2);
  }

  btSoftBody::tTetraArray& tetras(soft_body_->m_tetras);
  marker_array_.markers[4].points.clear();
  for (size_t i = 0; i < tetras.size(); ++i)
  {
    // the indices ought to be ordered so that
    // the right hand rule will be an outward normal here.
    int tr[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    for (size_t j = 0; j < 4; ++j)
    {
      for (size_t k = 0; k < 3; ++k)
      {
        const btSoftBody::Node* node = tetras[i].m_n[tr[j][k]];
        geometry_msgs::Point pt;
        pt.x = node->m_x.getX();
        pt.y = node->m_x.getY();
        pt.z = node->m_x.getZ();
        marker_array_.markers[4].points.push_back(pt);
      }
    }
  }

  // TODO(lucasw) also do something with faces

  btSoftBody::tAnchorArray& anchors(soft_body_->m_anchors);
  marker_array_.markers[2].points.clear();
  marker_array_.markers[3].points.clear();
  for (size_t i = 0; i < anchors.size(); ++i)
  {
    geometry_msgs::Point pt1;
    pt1.x = anchors[i].m_node->m_x.getX();
    pt1.y = anchors[i].m_node->m_x.getY();
    pt1.z = anchors[i].m_node->m_x.getZ();
    marker_array_.markers[2].points.push_back(pt1);

    btTransform trans;
    anchors[i].m_body->getMotionState()->getWorldTransform(trans);

    btVector3 world_point = trans * anchors[i].m_local;
    geometry_msgs::Point pt2;
    pt2.x = world_point.getX();
    pt2.y = world_point.getY();
    pt2.z = world_point.getZ();
    marker_array_.markers[2].points.push_back(pt2);
    marker_array_.markers[3].points.push_back(pt2);

    geometry_msgs::Point pt3;
    pt3.x = trans.getOrigin().getX();
    pt3.y = trans.getOrigin().getY();
    pt3.z = trans.getOrigin().getZ();
    marker_array_.markers[3].points.push_back(pt3);
  }

  marker_array_pub_->publish(marker_array_);
}

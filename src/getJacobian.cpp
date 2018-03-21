 bool moveit::core::RobotState::getJacobian(const JointModelGroup* group, const LinkModel* link,
                                            const Eigen::Vector3d& reference_point_position, Eigen::MatrixXd& jacobian,
                                            bool use_quaternion_representation) const
 {
   BOOST_VERIFY(checkLinkTransforms());
 
   if (!group->isChain())
   {
     logError("The group '%s' is not a chain. Cannot compute Jacobian.", group->getName().c_str());
     return false;
   }
 
   if (!group->isLinkUpdated(link->getName()))
   {
     logError("Link name '%s' does not exist in the chain '%s' or is not a child for this chain",
              link->getName().c_str(), group->getName().c_str());
     return false;
   }
 
   const robot_model::JointModel* root_joint_model = group->getJointModels()[0];  // group->getJointRoots()[0];
   const robot_model::LinkModel* root_link_model = root_joint_model->getParentLinkModel();
   Eigen::Affine3d reference_transform =
       root_link_model ? getGlobalLinkTransform(root_link_model).inverse() : Eigen::Affine3d::Identity();
   int rows = use_quaternion_representation ? 7 : 6;
   int columns = group->getVariableCount();
   jacobian = Eigen::MatrixXd::Zero(rows, columns);
 
   Eigen::Affine3d link_transform = reference_transform * getGlobalLinkTransform(link);
   Eigen::Vector3d point_transform = link_transform * reference_point_position;
 
   /*
   logDebug("Point from reference origin expressed in world coordinates: %f %f %f",
            point_transform.x(),
            point_transform.y(),
            point_transform.z());
   */
 
   Eigen::Vector3d joint_axis;
   Eigen::Affine3d joint_transform;
 
   while (link)
   {
     /*
     logDebug("Link: %s, %f %f %f",link_state->getName().c_str(),
              link_state->getGlobalLinkTransform().translation().x(),
              link_state->getGlobalLinkTransform().translation().y(),
              link_state->getGlobalLinkTransform().translation().z());
     logDebug("Joint: %s",link_state->getParentJointState()->getName().c_str());
     */
     const JointModel* pjm = link->getParentJointModel();
     if (pjm->getVariableCount() > 0)
     {
       unsigned int joint_index = group->getVariableGroupIndex(pjm->getName());
       if (pjm->getType() == robot_model::JointModel::REVOLUTE)
       {
         joint_transform = reference_transform * getGlobalLinkTransform(link);
         joint_axis = joint_transform.rotation() * static_cast<const robot_model::RevoluteJointModel*>(pjm)->getAxis();
         jacobian.block<3, 1>(0, joint_index) =
             jacobian.block<3, 1>(0, joint_index) + joint_axis.cross(point_transform - joint_transform.translation());
         jacobian.block<3, 1>(3, joint_index) = jacobian.block<3, 1>(3, joint_index) + joint_axis;
       }
       else if (pjm->getType() == robot_model::JointModel::PRISMATIC)
       {
         joint_transform = reference_transform * getGlobalLinkTransform(link);
         joint_axis = joint_transform * static_cast<const robot_model::PrismaticJointModel*>(pjm)->getAxis();
         jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
       }
       else if (pjm->getType() == robot_model::JointModel::PLANAR)
       {
         joint_transform = reference_transform * getGlobalLinkTransform(link);
         joint_axis = joint_transform * Eigen::Vector3d(1.0, 0.0, 0.0);
         jacobian.block<3, 1>(0, joint_index) = jacobian.block<3, 1>(0, joint_index) + joint_axis;
         joint_axis = joint_transform * Eigen::Vector3d(0.0, 1.0, 0.0);
         jacobian.block<3, 1>(0, joint_index + 1) = jacobian.block<3, 1>(0, joint_index + 1) + joint_axis;
         joint_axis = joint_transform * Eigen::Vector3d(0.0, 0.0, 1.0);
         jacobian.block<3, 1>(0, joint_index + 2) = jacobian.block<3, 1>(0, joint_index + 2) +
                                                    joint_axis.cross(point_transform - joint_transform.translation());
         jacobian.block<3, 1>(3, joint_index + 2) = jacobian.block<3, 1>(3, joint_index + 2) + joint_axis;
       }
       else
         logError("Unknown type of joint in Jacobian computation");
     }
     if (pjm == root_joint_model)
       break;
     link = pjm->getParentLinkModel();
   }
   return true;
 }
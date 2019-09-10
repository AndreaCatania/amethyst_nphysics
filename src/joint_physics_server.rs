use amethyst_core::math::Isometry3;
use amethyst_phythyst::{
    objects::*,
    servers::{JointDesc, JointPhysicsServerTrait},
    PtReal,
};
use log::error;
use nphysics3d::{
    joint::FixedConstraint as NpFixedConstraint, object::BodyPartHandle as NpBodyPartHandle,
};

use crate::{
    conversors::*,
    joint::Joint,
    servers_storage::{BodiesStorageRead, JointsStorageWrite, ServersStorages},
    storage::StoreKey,
    RBodyNpServer,
};

pub struct JointNpServer<N: PtReal> {
    storages: ServersStorages<N>,
}

impl<N: PtReal> JointNpServer<N> {
    pub fn new(storages: ServersStorages<N>) -> Self {
        Self { storages }
    }
}

// This is a collection of function that can be used by other servers to perform some common
// operations on the joints.
impl<N: PtReal> JointNpServer<N> {
    pub fn drop_joint(
        joint_tag: PhysicsJointTag,
        joints: &mut JointsStorageWrite<'_, N>,
        bodies: &BodiesStorageRead<'_, N>,
    ) {
        let j_key = joint_tag_to_store_key(joint_tag);

        // Active the constraint bodies
        if let Some(joint) = joints.get_joint(j_key) {
            if joint.body_0.is_some() {
                RBodyNpServer::active_body(joint.body_0.unwrap().0, bodies);
            }
            if joint.body_1.is_some() {
                RBodyNpServer::active_body(joint.body_1.unwrap().0, bodies);
            }
        }

        joints.drop_joint(j_key);
    }

    pub fn update_internal_joint(
        joint_key: StoreKey,
        joints: &mut JointsStorageWrite<'_, N>,
        bodies: &BodiesStorageRead<'_, N>,
    ) {
        let mut notify_added = false;
        let mut notify_removed = false;
        {
            let joint = joints.get_joint(joint_key);
            if let Some(mut joint) = joint {
                if joint.np_joint.is_some() {
                    if joint.body_0.is_none() || joint.body_1.is_none() {
                        // -- Remove joint --

                        joint.np_joint = None;
                        notify_removed = true;
                    }
                } else if joint.body_0.is_some() && joint.body_1.is_some() {
                    // -- Create the joint --
                    let body_0 = bodies.get_body(joint.body_0.unwrap().0);
                    let body_1 = bodies.get_body(joint.body_1.unwrap().0);
                    fail_cond!(body_0.is_none() || body_1.is_none());

                    let body_0 = body_0.unwrap();
                    let body_1 = body_1.unwrap();

                    let body_0_trsf = body_0.body_transform();
                    let body_1_trsf = body_1.body_transform();

                    let anchor_0: Isometry3<N> = body_0_trsf.inverse() * joint.initial_isometry;
                    let anchor_1: Isometry3<N> = body_1_trsf.inverse() * joint.initial_isometry;

                    match joint.joint_desc {
                        JointDesc::Fixed => {
                            let np_joint = NpFixedConstraint::new(
                                joint.body_0.map(|v| NpBodyPartHandle(v.0, v.1)).unwrap(),
                                joint.body_1.map(|v| NpBodyPartHandle(v.0, v.1)).unwrap(),
                                anchor_0.translation.vector.into(),
                                anchor_0.rotation,
                                anchor_1.translation.vector.into(),
                                anchor_1.rotation,
                            );
                            joint.np_joint = Some(Box::new(np_joint));
                        }
                    }
                    notify_added = true;
                }
            }
        }
        if notify_added {
            joints.notify_joint_created(joint_key);
        } else if notify_removed {
            joints.notify_joint_removed(joint_key);
        }
    }
}

impl<N: PtReal> JointPhysicsServerTrait<N> for JointNpServer<N> {
    fn create_joint(
        &self,
        desc: &JointDesc,
        initial_position: &Isometry3<N>,
    ) -> PhysicsHandle<PhysicsJointTag> {
        let mut joints = self.storages.joints_w();
        let key = joints.insert(Joint::new(*desc, *initial_position));
        joints.get_joint(key).unwrap().self_key = Some(key);
        PhysicsHandle::new(store_key_to_joint_tag(key), self.storages.gc.clone())
    }

    fn insert_rigid_body(&self, joint_tag: PhysicsJointTag, body_tag: PhysicsRigidBodyTag) {
        let joint_key = joint_tag_to_store_key(joint_tag);
        let mut joints = self.storages.joints_w();

        {
            let joint = joints.get_joint(joint_key);
            if let Some(mut joint) = joint {
                if joint.body_0.is_none() {
                    joint.body_0 = Some((rigid_tag_to_store_key(body_tag), 0));
                } else if joint.body_1.is_none() {
                    joint.body_1 = Some((rigid_tag_to_store_key(body_tag), 0));
                } else {
                    error!("This joint is already joining two other bodies, and you can't add more. Remove one of them if you want to constraint this new joint.");
                    return;
                }
            } else {
                error!("Joint tag not found!");
            }
        }

        Self::update_internal_joint(joint_key, &mut joints, &self.storages.bodies_r());
    }

    fn remove_rigid_body(&self, joint_tag: PhysicsJointTag, body_tag: PhysicsRigidBodyTag) {
        let joint_key = joint_tag_to_store_key(joint_tag);
        let mut joints = self.storages.joints_w();
        let bodies = self.storages.bodies_r();

        {
            let joint = joints.get_joint(joint_key);
            if let Some(mut joint) = joint {
                if joint.body_0.is_some() {
                    RBodyNpServer::active_body(joint.body_0.unwrap().0, &bodies);
                }
                if joint.body_1.is_some() {
                    RBodyNpServer::active_body(joint.body_1.unwrap().0, &bodies);
                }

                if let Some(true) = joint
                    .body_0
                    .map(|v| v.0 == rigid_tag_to_store_key(body_tag))
                {
                    joint.body_0 = None;
                } else if let Some(true) = joint
                    .body_1
                    .map(|v| v.0 == rigid_tag_to_store_key(body_tag))
                {
                    joint.body_1 = None;
                } else {
                    error!("The body was not found in this joint");
                }
            } else {
                // Nothing, the joint could be already removed
            }
        }

        Self::update_internal_joint(joint_key, &mut joints, &bodies);
    }
}

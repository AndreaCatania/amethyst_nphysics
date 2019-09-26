use amethyst_physics::{
    servers::{JointDesc, JointPosition},
    PtReal,
};
use nphysics3d::{joint::JointConstraint as NpJointConstraint, object::BodySet as NpBodySet};

use crate::storage::StoreKey;

#[allow(missing_debug_implementations)]
pub struct Joint<N: PtReal, S: NpBodySet<N>> {
    pub self_key: Option<StoreKey>,
    pub joint_desc: JointDesc,
    pub initial_position: JointPosition<N>,
    pub np_joint: Option<Box<dyn NpJointConstraint<N, S>>>,
    pub body_0: Option<(StoreKey, usize)>, // Body key, Part id
    pub body_1: Option<(StoreKey, usize)>, // Body key, Part id
}

impl<N: PtReal, S: NpBodySet<N>> Joint<N, S> {
    pub(crate) fn new(joint_desc: JointDesc, initial_position: JointPosition<N>) -> Self {
        Joint {
            self_key: None,
            joint_desc,
            initial_position,
            np_joint: None,
            body_0: None,
            body_1: None,
        }
    }
}

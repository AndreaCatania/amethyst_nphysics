use amethyst_core::math::Isometry3;
use amethyst_phythyst::{servers::JointDesc, PtReal};
use nphysics3d::{joint::JointConstraint as NpJointConstraint, object::BodySet as NpBodySet};

use crate::storage::StoreKey;

#[allow(missing_debug_implementations)]
pub struct Joint<N: PtReal, S: NpBodySet<N>> {
    pub self_key: Option<StoreKey>,
    pub joint_desc: JointDesc,
    pub initial_isometry: Isometry3<N>,
    pub np_joint: Option<Box<dyn NpJointConstraint<N, S>>>,
    pub body_0: Option<(StoreKey, usize)>, // Body key, Part id
    pub body_1: Option<(StoreKey, usize)>, // Body key, Part id
}

impl<N: PtReal, S: NpBodySet<N>> Joint<N, S> {
    pub(crate) fn new(joint_desc: JointDesc, initial_isometry: Isometry3<N>) -> Self {
        Joint {
            self_key: None,
            joint_desc,
            initial_isometry,
            np_joint: None,
            body_0: None,
            body_1: None,
        }
    }
}

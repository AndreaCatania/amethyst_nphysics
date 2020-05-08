use amethyst_physics::PtReal;
use nphysics3d::{
    force_generator::ForceGenerator as NpForceGenerator,
    object::{ BodyHandle as NpBodyHandle},
};

use crate::storage::StoreKey;

#[allow(missing_debug_implementations)]
pub struct ForceGenerator<N: PtReal, Handle: NpBodyHandle> {
    pub self_key: Option<StoreKey>,
    pub np_force_generator: Box<dyn NpForceGenerator<N, Handle>>,
    pub world_key: StoreKey,
}

impl<N: PtReal, Handle: NpBodyHandle> ForceGenerator<N, Handle> {
    #[allow(dead_code)]
    pub(crate) fn new(
        np_force_generator: Box<dyn NpForceGenerator<N, Handle>>,
        world_key: StoreKey,
    ) -> Self {
        ForceGenerator {
            self_key: None,
            np_force_generator,
            world_key,
        }
    }
}

use amethyst_phythyst::PtReal;
use nphysics3d::{
    force_generator::ForceGenerator as NpForceGenerator, object::BodySet as NpBodySet,
};

use crate::storage::StoreKey;

#[allow(missing_debug_implementations)]
pub struct ForceGenerator<N: PtReal, S: NpBodySet<N>> {
    pub self_key: Option<StoreKey>,
    pub np_force_generator: Box<dyn NpForceGenerator<N, S>>,
    pub world_key: StoreKey,
}

impl<N: PtReal, S: NpBodySet<N>> ForceGenerator<N, S> {
    #[allow(dead_code)]
    pub(crate) fn new(
        np_force_generator: Box<dyn NpForceGenerator<N, S>>,
        world_key: StoreKey,
    ) -> Self {
        ForceGenerator {
            self_key: None,
            np_force_generator,
            world_key,
        }
    }
}

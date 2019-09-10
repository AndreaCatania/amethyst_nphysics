use amethyst_phythyst::PtReal;
use nphysics3d::{
    force_generator::{
        ForceGenerator as NpForceGenerator, ForceGeneratorSet as NpForceGeneratorSet,
    },
    object::BodySet as NpBodySet,
};

use crate::{
    force_generator::ForceGenerator,
    storage::{Storage, StorageGuard, StoreKey},
};

#[allow(missing_debug_implementations)]
pub struct ForceGeneratorStorage<N: PtReal, S: NpBodySet<N>> {
    storage: Storage<ForceGenerator<N, S>>,
}

impl<N: PtReal, S: NpBodySet<N>> ForceGeneratorStorage<N, S> {
    pub fn new() -> Self {
        ForceGeneratorStorage {
            storage: Storage::new(5, 5),
        }
    }
}

impl<N: PtReal, S: NpBodySet<N>> Default for ForceGeneratorStorage<N, S> {
    fn default() -> Self {
        ForceGeneratorStorage::new()
    }
}

impl<N: PtReal, S: NpBodySet<N>> ForceGeneratorStorage<N, S> {
    pub fn insert(&mut self, force_generator: ForceGenerator<N, S>) -> StoreKey {
        self.storage.insert(force_generator)
    }

    pub fn drop(&mut self, key: StoreKey) {
        self.storage.remove(key);
    }

    /// Returns a `Mutex` guarded force generator that can be used safely to get or set data.
    pub fn get_force_generator(
        &self,
        key: StoreKey,
    ) -> Option<StorageGuard<'_, ForceGenerator<N, S>>> {
        self.storage.get(key)
    }
}

impl<N: PtReal, S: NpBodySet<N> + 'static> NpForceGeneratorSet<N, S>
    for ForceGeneratorStorage<N, S>
{
    type ForceGenerator = dyn NpForceGenerator<N, S>;
    type Handle = StoreKey;

    fn get(&self, handle: Self::Handle) -> Option<&Self::ForceGenerator> {
        self.storage
            .unchecked_get(handle)
            .map(|v| v.np_force_generator.as_ref())
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::ForceGenerator> {
        self.storage
            .unchecked_get_mut(handle)
            .map(|v| v.np_force_generator.as_mut())
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.has(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::ForceGenerator)) {
        for (i, c) in self.storage.iter() {
            // Safe because NPhysics use this in single thread.
            unsafe { f(i, (*c.0.get()).np_force_generator.as_ref()) }
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::ForceGenerator)) {
        for (i, c) in self.storage.iter_mut() {
            // Safe because NPhysics use this in single thread.
            unsafe { f(i, (*c.0.get()).np_force_generator.as_mut()) }
        }
    }
}

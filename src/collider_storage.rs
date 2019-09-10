use amethyst_phythyst::PtReal;
use ncollide3d::pipeline::object::CollisionObjectSet as NpCollisionObjectSet;
use nphysics3d::object::{
    BodyHandle as NpBodyHandle, Collider as NpCollider,
    ColliderRemovalData as NpColliderRemovalData, ColliderSet,
};

use crate::storage::{Storage, StorageGuard, StoreKey};

#[allow(missing_debug_implementations)]
pub struct ColliderStorage<N: PtReal, BH: NpBodyHandle> {
    storage: Storage<NpCollider<N, BH>>,
    /// A list of inserted ID, this list is decremented only when the function `pop_inserted_event` is called
    inserted: Vec<StoreKey>,
    /// A list of removed ID, this list is decremented only when the function `pop_removal_event` is called
    removed: Vec<(StoreKey, NpColliderRemovalData<N, BH>)>,
}

impl<N: PtReal, BH: NpBodyHandle> ColliderStorage<N, BH> {
    pub fn new() -> Self {
        ColliderStorage {
            storage: Storage::new(50, 50),
            inserted: Vec::new(),
            removed: Vec::new(),
        }
    }
}

impl<N: PtReal, BH: NpBodyHandle> Default for ColliderStorage<N, BH> {
    fn default() -> Self {
        ColliderStorage::new()
    }
}

impl<N: PtReal, BH: NpBodyHandle> ColliderStorage<N, BH> {
    pub fn insert_collider(&mut self, collider: NpCollider<N, BH>) -> StoreKey {
        let key = self.storage.insert(collider);
        self.inserted.push(key);
        key
    }

    pub fn drop_collider(&mut self, key: StoreKey) {
        let res = self.storage.remove(key);
        if let Some(data) = res {
            if let Some(d) = data.removal_data() {
                self.removed.push((key, d));
            }
        }
    }

    /// Returns a `Mutex` guarded collider that can be used safely to get or set data.
    pub fn get_collider(&self, key: StoreKey) -> Option<StorageGuard<'_, NpCollider<N, BH>>> {
        self.storage.get(key)
    }
}

impl<N: PtReal, BH: NpBodyHandle> NpCollisionObjectSet<N> for ColliderStorage<N, BH> {
    type CollisionObject = NpCollider<N, BH>;
    type CollisionObjectHandle = StoreKey;

    fn collision_object(
        &self,
        handle: Self::CollisionObjectHandle,
    ) -> Option<&Self::CollisionObject> {
        self.storage.unchecked_get(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::CollisionObjectHandle, &Self::CollisionObject)) {
        for (i, c) in self.storage.iter() {
            // Safe because NPhysics use this in single thread.
            unsafe { f(i, &*c.0.get()) }
        }
    }
}

impl<N: PtReal, BH: NpBodyHandle> ColliderSet<N, BH> for ColliderStorage<N, BH> {
    type Handle = StoreKey;

    fn get(&self, handle: Self::Handle) -> Option<&NpCollider<N, BH>> {
        self.storage.unchecked_get(handle)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut NpCollider<N, BH>> {
        self.storage.unchecked_get_mut(handle)
    }

    fn get_pair_mut(
        &mut self,
        handle1: Self::Handle,
        handle2: Self::Handle,
    ) -> (
        Option<&mut NpCollider<N, BH>>,
        Option<&mut NpCollider<N, BH>>,
    ) {
        assert_ne!(handle1, handle2, "Both body handles must not be equal.");
        let b1 = self.get_mut(handle1).map(|b| b as *mut NpCollider<N, BH>);
        let b2 = self.get_mut(handle2).map(|b| b as *mut NpCollider<N, BH>);
        unsafe { (b1.map(|b| &mut *b), b2.map(|b| &mut *b)) }
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.has(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &NpCollider<N, BH>)) {
        for (i, c) in self.storage.iter() {
            // Safe because NPhysics use this in single thread.
            unsafe { f(i, &*c.0.get()) }
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut NpCollider<N, BH>)) {
        for (i, c) in self.storage.iter_mut() {
            // Safe because NPhysics use this in single thread.
            unsafe { f(i, &mut *c.0.get()) }
        }
    }

    fn pop_insertion_event(&mut self) -> Option<Self::Handle> {
        self.inserted.pop()
    }

    fn pop_removal_event(&mut self) -> Option<(Self::Handle, NpColliderRemovalData<N, BH>)> {
        self.removed.pop()
    }

    fn remove(&mut self, to_remove: Self::Handle) -> Option<&mut NpColliderRemovalData<N, BH>> {
        let collider = self.storage.remove(to_remove)?;
        if let Some(data) = collider.removal_data() {
            self.removed.push((to_remove, data));
            self.removed.last_mut().map(|r| &mut r.1)
        } else {
            None
        }
    }
}

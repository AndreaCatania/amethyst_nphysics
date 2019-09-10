use std::{cell::UnsafeCell, sync::Mutex};

use amethyst_phythyst::PtReal;
use generational_arena::{Iter, IterMut};
use nphysics3d::object::{Body as NpBody, BodySet};

use crate::{
    body::Body,
    storage::{Storage, StorageGuard, StoreKey},
};

#[allow(missing_debug_implementations)]
pub struct BodyStorage<N: PtReal> {
    storage: Storage<Body<N>>,
    /// A list of removed ID, this list is decremented only when the function `pop_removal_event` is called
    removed: Vec<StoreKey>,
}

impl<N: PtReal> BodyStorage<N> {
    pub fn new() -> Self {
        BodyStorage {
            storage: Storage::new(50, 50),
            removed: Vec::new(),
        }
    }
}

impl<N: PtReal> Default for BodyStorage<N> {
    fn default() -> Self {
        BodyStorage::new()
    }
}

impl<N: PtReal> BodyStorage<N> {
    pub fn insert_body(&mut self, body: Body<N>) -> StoreKey {
        self.storage.insert(body)
    }

    pub fn drop_body(&mut self, key: StoreKey) {
        self.storage.remove(key);
        self.removed.push(key);
    }

    /// Returns a `Mutex` guarded body that can be used safely to get or set data.
    pub fn get_body(&self, key: StoreKey) -> Option<StorageGuard<'_, Body<N>>> {
        self.storage.get(key)
    }

    pub fn iter(&self) -> Iter<'_, (UnsafeCell<Body<N>>, Mutex<()>)> {
        self.storage.iter()
    }

    pub fn iter_mut(&mut self) -> IterMut<'_, (UnsafeCell<Body<N>>, Mutex<()>)> {
        self.storage.iter_mut()
    }
}

impl<N: PtReal> BodySet<N> for BodyStorage<N> {
    type Body = dyn NpBody<N>;
    type Handle = StoreKey;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body> {
        self.storage
            .unchecked_get(handle)
            .map(|v| v.np_body.as_ref())
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        self.storage
            .unchecked_get_mut(handle)
            .map(|v| v.np_body.as_mut())
    }

    fn get_pair_mut(
        &mut self,
        handle1: Self::Handle,
        handle2: Self::Handle,
    ) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        assert_ne!(handle1, handle2, "Both body handles must not be equal.");
        let b1 = self.get_mut(handle1).map(|b| b as *mut dyn NpBody<N>);
        let b2 = self.get_mut(handle2).map(|b| b as *mut dyn NpBody<N>);
        unsafe { (b1.map(|b| &mut *b), b2.map(|b| &mut *b)) }
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.has(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::Body)) {
        for (h, b) in self.storage.iter() {
            // Safe because NPhysics use this in single thread.
            unsafe { f(h, (*b.0.get()).np_body.as_ref()) }
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::Body)) {
        for (h, b) in self.storage.iter_mut() {
            // Safe because NPhysics use this in single thread.
            unsafe { f(h, (*b.0.get()).np_body.as_mut()) }
        }
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        self.removed.pop()
    }
}

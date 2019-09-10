use amethyst_phythyst::PtReal;
use nphysics3d::{
    joint::{JointConstraint as NpJointConstraint, JointConstraintSet as NpJointConstraintSet},
    object::{BodyPartHandle as NpBodyPartHandle, BodySet as NpBodySet},
};

use crate::{
    joint::Joint,
    storage::{Storage, StorageGuard, StoreKey},
};

#[allow(missing_debug_implementations)]
pub struct JointStorage<N: PtReal, S: NpBodySet<N>> {
    storage: Storage<Joint<N, S>>,
    /// A list of inserted ID, this list is decremented only when the function `pop_inserted_event` is called
    inserted: Vec<(
        StoreKey,
        NpBodyPartHandle<S::Handle>,
        NpBodyPartHandle<S::Handle>,
    )>,
    /// A list of removed ID, this list is decremented only when the function `pop_removal_event` is called
    removed: Vec<(
        StoreKey,
        NpBodyPartHandle<S::Handle>,
        NpBodyPartHandle<S::Handle>,
    )>,
}

impl<N: PtReal, S: NpBodySet<N>> JointStorage<N, S> {
    pub fn new() -> Self {
        JointStorage {
            storage: Storage::new(5, 15),
            inserted: Vec::new(),
            removed: Vec::new(),
        }
    }
}

impl<N: PtReal, S: NpBodySet<N>> Default for JointStorage<N, S> {
    fn default() -> Self {
        JointStorage::new()
    }
}

impl<N: PtReal, S: NpBodySet<N>> JointStorage<N, S> {
    pub fn insert(&mut self, joint: Joint<N, S>) -> StoreKey {
        let notify_joint_created = joint.np_joint.is_some();
        let key = self.storage.insert(joint);
        if notify_joint_created {
            self.notify_joint_created(key);
        }
        key
    }

    /// Notify that a NPhysics joint is just created.
    ///
    /// This function must be called each time a NPhysics joint is created.
    ///
    /// Usually the NPhysics joint is not created along with the `Joint` object.
    pub fn notify_joint_created(&mut self, key: StoreKey) {
        let j = self.storage.get(key);
        if let Some(j) = j {
            if let Some(j) = &j.np_joint {
                let (part1, part2) = j.anchors();
                self.inserted.push((key, part1, part2));
            }
        }
    }

    pub fn drop_joint(&mut self, key: StoreKey) {
        let res = self.storage.remove(key);
        if let Some(data) = res {
            if let Some(joint) = &data.np_joint {
                let (part1, part2) = joint.anchors();
                self.removed.push((key, part1, part2));
            }
        }
    }

    /// Notify that a NPhysics joint is just removed.
    ///
    /// This function must be called each time a NPhysics joint is removed.
    ///
    /// An NPhysics joint can be removed anytime.
    pub fn notify_joint_removed(&mut self, key: StoreKey) {
        let j = self.storage.get(key);
        if let Some(j) = j {
            if let Some(j) = &j.np_joint {
                let (part1, part2) = j.anchors();
                self.removed.push((key, part1, part2));
            }
        }
    }

    /// Returns a `Mutex` guarded joint that can be used safely to get or set data.
    pub fn get_joint(&self, key: StoreKey) -> Option<StorageGuard<'_, Joint<N, S>>> {
        self.storage.get(key)
    }
}

impl<N: PtReal, S: NpBodySet<N> + 'static> NpJointConstraintSet<N, S> for JointStorage<N, S> {
    type JointConstraint = dyn NpJointConstraint<N, S>;
    type Handle = StoreKey;

    fn get(&self, handle: Self::Handle) -> Option<&Self::JointConstraint> {
        if let Some(j) = self.storage.unchecked_get(handle) {
            j.np_joint.as_ref().map(|v| v.as_ref())
        } else {
            None
        }
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::JointConstraint> {
        if let Some(j) = self.storage.unchecked_get_mut(handle) {
            j.np_joint.as_mut().map(|v| v.as_mut())
        } else {
            None
        }
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.has(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::JointConstraint)) {
        for (i, c) in self.storage.iter() {
            // Safe because NPhysics use this in single thread.
            unsafe {
                if let Some(joint) = (*c.0.get()).np_joint.as_ref() {
                    f(i, joint.as_ref())
                }
            }
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::JointConstraint)) {
        for (i, c) in self.storage.iter_mut() {
            // Safe because NPhysics use this in single thread.
            unsafe {
                if let Some(joint) = (*c.0.get()).np_joint.as_mut() {
                    f(i, joint.as_mut())
                }
            }
        }
    }

    fn pop_insertion_event(
        &mut self,
    ) -> Option<(
        Self::Handle,
        NpBodyPartHandle<S::Handle>,
        NpBodyPartHandle<S::Handle>,
    )> {
        self.inserted.pop()
    }

    fn pop_removal_event(
        &mut self,
    ) -> Option<(
        Self::Handle,
        NpBodyPartHandle<S::Handle>,
        NpBodyPartHandle<S::Handle>,
    )> {
        self.removed.pop()
    }

    fn remove(&mut self, to_remove: Self::Handle) {
        self.storage.remove(to_remove);
    }
}

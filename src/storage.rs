use std::{
    cell::UnsafeCell,
    sync::{Mutex, MutexGuard},
};

use generational_arena::{Arena, Index, Iter, IterMut};

pub type StoreKey = Index;

/// This struct is used to store the physics resources, and return an opaque handle that allow to
/// return a reference to them.
///
/// Each value is protected by a `Mutex` so each thread can perform operation on multiple elements
/// without locking the entire storage.
///
/// The actual data are not stored inside the `Mutex` because *NPhysics* can't deal with the mutex and
/// expects the raw reference.
#[derive(Debug)]
pub struct Storage<T> {
    memory: Arena<(UnsafeCell<T>, Mutex<()>)>,
    growing_size: usize,
}

impl<T> Storage<T> {
    /// Create a storage with an initial capacity
    /// The parameter `growing_size` is used to grow the internal storage by a certain amount when it
    /// hits maximum capacity.
    /// The `growing_size` must be big enough to avoid too much reallocation
    pub fn new(initial_capacity: usize, growing_size: usize) -> Storage<T> {
        Storage {
            memory: Arena::with_capacity(initial_capacity),
            growing_size,
        }
    }

    /// Takes an object and returns an opaque id.
    /// This function takes also the ownership, so to drop an object you need to call the `remove`
    /// function with the ID of the object to delete.
    pub fn insert(&mut self, object: T) -> StoreKey {
        // Reserve the memory if no more space
        if self.memory.len() == self.memory.capacity() {
            self.memory.reserve(self.growing_size);
        }

        self.memory
            .insert((UnsafeCell::new(object), Mutex::new(())))
    }

    /// Returns true if the store key is associated to something
    pub fn has(&self, key: StoreKey) -> bool {
        self.memory.contains(key)
    }

    /// This is the default get function that must be used in order to obtain access to the stored object.
    ///
    /// Since the storage is using a `Mutex` to prevent data races, only this function is enough to
    /// read or to write the stored data.
    pub fn get(&self, key: StoreKey) -> Option<StorageGuard<'_, T>> {
        unsafe {
            self.memory.get(key).map(|v| StorageGuard {
                data: &mut *v.0.get(),
                _guard: v.1.lock().unwrap(),
            })
        }
    }

    /// This function is safe only when it's used by *NPhysics* set storages.
    ///
    /// The reason is that *NPhysics* runs in single thread, and during this process no one can access
    /// to the storage because it's fully locked by RwLock which own this storage.
    /// So the borrow checker is it able to correctly prevent data races.
    pub fn unchecked_get(&self, key: StoreKey) -> Option<&T> {
        unsafe { self.memory.get(key).map(|v| &*v.0.get()) }
    }

    /// This function is safe only when it's used by *NPhysics* set storages.
    ///
    /// The reason is that *NPhysics* runs in single thread, and during this process no one can access
    /// to the storage because it's fully locked by RwLock which own this storage.
    /// So the borrow checker is it able to correctly prevent data races.
    pub fn unchecked_get_mut(&mut self, key: StoreKey) -> Option<&mut T> {
        unsafe { self.memory.get(key).map(|v| &mut *v.0.get()) }
    }

    /// Remove an object and release the key for future use.
    ///
    /// Returns `Some` with the removed object, or `None` if nothing was removed.
    pub fn remove(&mut self, key: StoreKey) -> Option<T> {
        self.memory.remove(key).map(|v| v.0.into_inner())
    }

    /// Returns an iterator to the data.
    // TODO consider to create a for each, similar to NPhysics set trait, instead?
    pub fn iter(&self) -> Iter<'_, (UnsafeCell<T>, Mutex<()>)> {
        self.memory.iter()
    }

    /// Returns a mutable iterator to the data.
    // TODO consider to create a for each, similar to NPhysics set trait, instead?
    pub fn iter_mut(&mut self) -> IterMut<'_, (UnsafeCell<T>, Mutex<()>)> {
        self.memory.iter_mut()
    }
}

impl<T> Default for Storage<T> {
    fn default() -> Self {
        Storage::new(10, 10)
    }
}

// Safe to be sent trough threads thanks to the `Mutex`
unsafe impl<T> Sync for Storage<T> {}

/// The `StorageGuard` is used to returns an object that contains the requested data plus the MutexGuard
/// which is used to track the lifetime of the data reference.
///
/// The reason of the extra type, is because the `Mutex` doesn't own directly the data.
#[allow(missing_debug_implementations)]
pub struct StorageGuard<'a, T> {
    data: &'a mut T,
    _guard: MutexGuard<'a, ()>,
}

impl<T> std::ops::Deref for StorageGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.data
    }
}

impl<T> std::ops::DerefMut for StorageGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.data
    }
}

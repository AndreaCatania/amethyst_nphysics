use std::sync::{Arc, RwLock, RwLockReadGuard, RwLockWriteGuard};

use amethyst_phythyst::{objects::*, PtReal};

use crate::{
    body_storage::BodyStorage,
    collider_storage::ColliderStorage,
    force_generator_storage::ForceGeneratorStorage,
    joint_storage::JointStorage,
    shape::RigidShape,
    storage::{Storage, StoreKey},
};

pub type ServersStorages<N> = Arc<ServersStorage<N>>;

pub type BodiesStorageWrite<'a, N> = RwLockWriteGuard<'a, BodyStorage<N>>;
pub type BodiesStorageRead<'a, N> = RwLockReadGuard<'a, BodyStorage<N>>;
pub type CollidersStorageWrite<'a, N> = RwLockWriteGuard<'a, ColliderStorage<N, StoreKey>>;
pub type CollidersStorageRead<'a, N> = RwLockReadGuard<'a, ColliderStorage<N, StoreKey>>;
pub type JointsStorageWrite<'a, N> = RwLockWriteGuard<'a, JointStorage<N, BodyStorage<N>>>;
pub type JointsStorageRead<'a, N> = RwLockReadGuard<'a, JointStorage<N, BodyStorage<N>>>;
pub type ForceGeneratorsStorageWrite<'a, N> =
    RwLockWriteGuard<'a, ForceGeneratorStorage<N, BodyStorage<N>>>;
pub type ForceGeneratorsStorageRead<'a, N> =
    RwLockReadGuard<'a, ForceGeneratorStorage<N, BodyStorage<N>>>;
pub type ShapesStorageWrite<'a, N> = RwLockWriteGuard<'a, Storage<Box<RigidShape<N>>>>;
pub type ShapesStorageRead<'a, N> = RwLockReadGuard<'a, Storage<Box<RigidShape<N>>>>;

/// This struct is responsible to hold all the storages
///
/// ## Multi threading issue
/// This a simplified version of the actual way to handle the storages:
/// https://play.rust-lang.org/?version=stable&mode=debug&edition=2018&code=use%20std%3A%3Async%3A%3A%7B%0A%20%20%20%20RwLock%2C%0A%20%20%20%20Arc%2C%0A%7D%3B%0A%0Astruct%20WorldStorage%7B%0A%20%20%20%20%20pub%20worlds%3A%20Vec%3CArc%3CRwLock%3CWorld%3E%3E%3E%2C%20%20%20%0A%7D%0A%0Astruct%20World%7B%0A%20%20%20%20pub%20bodies%3A%20Vec%3CBox%3CBody%3E%3E%2C%0A%7D%0A%0A%23%5Bderive(Debug)%5D%0Astruct%20Body%7B%0A%20%20%20%20pub%20i%3A%20i32%2C%0A%20%20%20%20pub%20weight%3A%20f32%2C%0A%7D%0A%0Afn%20main()%7B%0A%0A%20%20%20%20%2F%2F%20Create%20Storage%2C%20World%20and%203%20bodies%20owned%20by%20the%20world%0A%20%20%20%20let%20mut%20storage%20%3D%20WorldStorage%7B%0A%20%20%20%20%20%20%20%20worlds%3A%20vec!()%2C%0A%20%20%20%20%7D%3B%0A%20%20%20%20%0A%20%20%20%20%7B%0A%20%20%20%20%20%20%20%20%2F%2F%20These%20are%203%20independent%20bodies%0A%20%20%20%20%20%20%20%20let%20body_1%20%3D%20Box%3A%3Anew(Body%7Bi%3A%201%2C%20weight%3A%2010.0%7D)%3B%0A%20%20%20%20%20%20%20%20let%20body_2%20%3D%20Box%3A%3Anew(Body%7Bi%3A%202%2C%20weight%3A%2010.0%7D)%3B%0A%20%20%20%20%20%20%20%20let%20body_3%20%3D%20Box%3A%3Anew(Body%7Bi%3A%203%2C%20weight%3A%2010.0%7D)%3B%0A%20%20%20%20%20%20%20%20%0A%20%20%20%20%20%20%20%20storage.worlds.push(Arc%3A%3Anew(RwLock%3A%3Anew(World%7Bbodies%3A%20vec!(body_1%2C%20body_2%2C%20body_3)%2C%7D)))%3B%0A%20%20%20%20%7D%0A%20%20%20%20%0A%20%20%20%20mutate_parallel(%26storage%2C%200)%3B%0A%20%20%20%20mutate_parallel(%26storage%2C%201)%3B%0A%20%20%20%20mutate_parallel(%26storage%2C%202)%3B%0A%20%20%20%20%0A%20%20%20%20%0A%20%20%20%20let%20world%20%3D%20storage.worlds%5B0%5D.read().unwrap()%3B%0A%20%20%20%20for%20b%20in%20%26world.bodies%20%7B%0A%20%20%20%20%20%20%20%20dbg!(b)%3B%0A%20%20%20%20%7D%0A%7D%0A%0Afn%20mutate_parallel(storage%3A%20%26WorldStorage%2C%20body_id%3A%20usize)%7B%0A%20%20%20%20let%20mut%20world%20%3D%20storage.worlds%5B0%5D.write().unwrap()%3B%20%20%20%0A%20%20%20%20%0A%20%20%20%20world.bodies%5Bbody_id%5D.weight%20%3D%2044.0%3B%0A%7D%0A
/// The world internal storage is not thread safe, this mean that is mandatory have a mutable World
/// in order to retrieve a **Mutable** body.
///
/// The problem is that taking a mutable World using the `RwLock::write()` function make all others
/// threads to wait the unlock of the world.
///
/// Since each call like apply_force, or set_velocity, or set_friction need a mutable body that
/// can be taken only if the world is mutable.
/// Again to take the World mutable I have to use `RwLock::write()` that synchronize the execution.
///
/// A solution to this problem would be support add multithreading support on NPhysics
#[allow(missing_debug_implementations)]
pub struct ServersStorage<N: PtReal> {
    pub(crate) gc: Arc<RwLock<PhysicsGarbageCollector>>,
    bodies: Arc<RwLock<BodyStorage<N>>>,
    colliders: Arc<RwLock<ColliderStorage<N, StoreKey>>>,
    joints: Arc<RwLock<JointStorage<N, BodyStorage<N>>>>,
    force_generators: Arc<RwLock<ForceGeneratorStorage<N, BodyStorage<N>>>>,
    shapes: Arc<RwLock<Storage<Box<RigidShape<N>>>>>,
}

impl<N: PtReal> ServersStorage<N> {
    pub fn new() -> ServersStorages<N> {
        Arc::new(ServersStorage {
            gc: Arc::new(RwLock::new(PhysicsGarbageCollector::default())),
            bodies: Arc::new(RwLock::new(BodyStorage::default())),
            colliders: Arc::new(RwLock::new(ColliderStorage::default())),
            joints: Arc::new(RwLock::new(JointStorage::default())),
            force_generators: Arc::new(RwLock::new(ForceGeneratorStorage::default())),
            shapes: Arc::new(RwLock::new(Storage::new(50, 50))),
        })
    }
}

impl<N: PtReal> ServersStorage<N> {
    pub fn bodies_w(&self) -> BodiesStorageWrite<'_, N> {
        self.bodies.write().unwrap()
    }

    pub fn bodies_r(&self) -> BodiesStorageRead<'_, N> {
        self.bodies.read().unwrap()
    }

    pub fn colliders_w(&self) -> CollidersStorageWrite<'_, N> {
        self.colliders.write().unwrap()
    }

    pub fn colliders_r(&self) -> CollidersStorageRead<'_, N> {
        self.colliders.read().unwrap()
    }

    pub fn joints_w(&self) -> JointsStorageWrite<'_, N> {
        self.joints.write().unwrap()
    }

    pub fn joints_r(&self) -> JointsStorageRead<'_, N> {
        self.joints.read().unwrap()
    }

    pub fn force_generator_w(&self) -> ForceGeneratorsStorageWrite<'_, N> {
        self.force_generators.write().unwrap()
    }

    pub fn force_generator_r(&self) -> ForceGeneratorsStorageRead<'_, N> {
        self.force_generators.read().unwrap()
    }

    pub fn shapes_w(&self) -> ShapesStorageWrite<'_, N> {
        self.shapes.write().unwrap()
    }

    pub fn shapes_r(&self) -> ShapesStorageRead<'_, N> {
        self.shapes.read().unwrap()
    }
}

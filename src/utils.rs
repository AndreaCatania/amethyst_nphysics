use crate::storage::StoreKey;
use amethyst_core::ecs::Entity;
use amethyst_physics::{PtReal, servers::ContactEvent};

#[derive(Copy, Clone, Debug)]
pub(crate) enum ObjectType {
    RigidBody,
    Area,
}

#[derive(Clone, Debug)]
pub(crate) struct UserData {
    object_type: ObjectType,
    store_key: StoreKey,
    entity: Option<Entity>,
}

impl UserData {
    pub(crate) fn new(
        object_type: ObjectType,
        store_key: StoreKey,
        entity: Option<Entity>,
    ) -> Self {
        UserData {
            object_type,
            store_key,
            entity,
        }
    }
}

impl UserData {
    pub fn object_type(&self) -> ObjectType {
        self.object_type
    }
    pub fn store_key(&self) -> StoreKey {
        self.store_key
    }
    pub fn entity(&self) -> Option<Entity> {
        self.entity
    }
}

/// Contact data is used to store the information of the contacts
/// per each collider.
///
/// These information are not stored inside the body to optimize the collection
/// process.
pub struct ContactData<N: PtReal> {
    pub collider_handle: StoreKey,
    pub contacts: Vec<ContactEvent<N>>,
}
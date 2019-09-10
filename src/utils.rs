use crate::storage::StoreKey;
use amethyst_core::ecs::Entity;

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

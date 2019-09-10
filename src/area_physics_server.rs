use amethyst_core::ecs::Entity;
use amethyst_core::math::{zero, Isometry3};
use amethyst_phythyst::{
    objects::*,
    servers::{AreaPhysicsServerTrait, OverlapEvent},
    PtReal,
};
use log::error;
use nphysics3d::object::{
    BodyPartHandle as NpBodyPartHandle, BodyStatus as NpBodyStatus, Collider as NpCollider,
    ColliderDesc as NpColliderDesc, RigidBody as NpRigidBody, RigidBodyDesc as NpRigidBodyDesc,
};

use crate::{
    body::{Body, BodyData},
    conversors::*,
    servers_storage::*,
    shape::RigidShape,
    storage::StoreKey,
    utils::*,
};

pub struct AreaNpServer<N: PtReal> {
    storages: ServersStorages<N>,
}

impl<N: PtReal> AreaNpServer<N> {
    pub fn new(storages: ServersStorages<N>) -> Self {
        AreaNpServer { storages }
    }
}

// This is a collection of functions that can be used by other servers to perform some common
// operation on the areas.
impl<N: PtReal> AreaNpServer<N> {
    pub fn drop_area(
        area_tag: PhysicsAreaTag,
        bodies_storage: &mut BodiesStorageWrite<'_, N>,
        colliders_storage: &mut CollidersStorageWrite<'_, N>,
        shapes_storage: &ShapesStorageRead<'_, N>,
    ) {
        let area_key = area_tag_to_store_key(area_tag);
        if let Some(mut area) = bodies_storage.get_body(area_key) {
            Self::remove_shape(&mut *area, shapes_storage, colliders_storage);
        }
        bodies_storage.drop_body(area_key);
    }

    /// Set shape.
    /// Take care to register the shape and set the collider to the body.
    pub fn install_shape(
        area: &mut Body<N>,
        shape: &mut RigidShape<N>,
        collider_desc: &NpColliderDesc<N>,
        colliders: &mut CollidersStorageWrite<'_, N>,
    ) {
        Self::install_collider(area, collider_desc, colliders);

        // Collider registration
        shape.register_body(area.self_key.unwrap());
        area.shape_key = shape.self_key;
    }

    /// Remove shape.
    /// Take care to unregister the shape and then drop the internal collider.
    pub fn remove_shape(
        area: &mut Body<N>,
        shapes: &ShapesStorageRead<'_, N>,
        colliders: &mut CollidersStorageWrite<'_, N>,
    ) {
        if let Some(shape_key) = area.shape_key {
            if let Some(mut shape) = shapes.get(shape_key) {
                shape.unregister_body(area.self_key.unwrap());
            } else {
                error!("An area is associated with a shape, but the shape doesn't exist!");
            }
            area.shape_key = None;
        }
        Self::drop_collider(area, colliders);
    }

    pub fn install_collider(
        area: &mut Body<N>,
        collider_desc: &NpColliderDesc<N>,
        colliders: &mut CollidersStorageWrite<'_, N>,
    ) {
        let mut collider = collider_desc.build(NpBodyPartHandle(area.self_key.unwrap(), 0));
        AreaNpServer::update_user_data(&mut collider, area);

        let key = colliders.insert_collider(collider);
        area.collider_key = Some(key);
    }

    /// Just drop the internal collider of the passed area.
    pub fn drop_collider(area: &mut Body<N>, colliders: &mut CollidersStorageWrite<'_, N>) {
        if let Some(collider_key) = area.collider_key {
            colliders.drop_collider(collider_key);
            area.collider_key = None;
        }
    }

    pub fn update_user_data(collider: &mut NpCollider<N, StoreKey>, area: &Body<N>) {
        collider.set_user_data(Some(Box::new(UserData::new(
            ObjectType::Area,
            area.self_key.unwrap(),
            area.entity,
        ))));
    }

    pub fn extract_collider_desc(
        _np_rigid_body: &NpRigidBody<N>, // Even if not used still here because in future this will be used.
        _shape: &RigidShape<N>, // Even if not used still here because in future this will be used.
        np_collider_desc: &mut NpColliderDesc<N>,
    ) {
        np_collider_desc.set_density(zero());
        np_collider_desc.set_is_sensor(true);
    }
}

impl<N> AreaPhysicsServerTrait<N> for AreaNpServer<N>
where
    N: PtReal,
{
    fn create_area(&self) -> PhysicsHandle<PhysicsAreaTag> {
        let mut bodies_storage = self.storages.bodies_w();

        // Create Rigid body
        let np_rigid_body = NpRigidBodyDesc::new()
            .set_status(NpBodyStatus::Static)
            .set_mass(N::from(0.0f32))
            .build();

        let a_key =
            bodies_storage.insert_body(Body::new_area(Box::new(np_rigid_body), zero(), zero()));
        let mut area = bodies_storage.get_body(a_key).unwrap();
        area.self_key = Some(a_key);

        PhysicsHandle::new(store_key_to_area_tag(a_key), self.storages.gc.clone())
    }

    fn set_entity(&self, area_tag: PhysicsAreaTag, entity: Option<Entity>) {
        let area_key = area_tag_to_store_key(area_tag);
        let bodies = self.storages.bodies_r();

        let area = bodies.get_body(area_key);
        if let Some(mut area) = area {
            fail_cond!(!matches!(area.body_data, BodyData::Area(_)));
            area.entity = entity;

            if let Some(collider_key) = area.collider_key {
                let colliders = self.storages.colliders_r();

                let collider = colliders.get_collider(collider_key);
                if let Some(mut collider) = collider {
                    AreaNpServer::update_user_data(&mut *collider, &*area);
                } else {
                    error!("A body is assigned to a collider, but the collider doesn't exist!")
                }
            }
        }
    }

    fn entity(&self, area_tag: PhysicsAreaTag) -> Option<Entity> {
        let area_key = area_tag_to_store_key(area_tag);
        let bodies = self.storages.bodies_r();

        let area = bodies.get_body(area_key);
        if let Some(area) = area {
            area.entity
        } else {
            None
        }
    }

    fn set_shape(&self, area_tag: PhysicsAreaTag, shape_tag: Option<PhysicsShapeTag>) {
        let area_key = area_tag_to_store_key(area_tag);
        let shape_key = shape_tag.map(shape_tag_to_store_key);

        let bodies = self.storages.bodies_r();

        let area = bodies.get_body(area_key);

        if let Some(mut area) = area {
            if area.shape_key != shape_key {
                let shapes = self.storages.shapes_r();
                let mut colliders = self.storages.colliders_w();

                // Remove the old shape
                if let Some(_b_shape_key) = area.shape_key {
                    AreaNpServer::remove_shape(&mut *area, &shapes, &mut colliders);
                }

                if let Some(shape_key) = shape_key {
                    // Assign the new shape

                    if let Some(mut shape) = shapes.get(shape_key) {
                        // Create and attach the collider
                        let collider_desc =
                            NpColliderDesc::new(shape.shape_handle().clone()).sensor(true);

                        AreaNpServer::install_shape(
                            &mut *area,
                            &mut *shape,
                            &collider_desc,
                            &mut colliders,
                        );
                    } else {
                        error!("During the area creation, was not possible to find the shape to assign");
                    }
                } else {
                    // Nothing, previous shape already removed.
                }
            }
        } else {
            error!("Area not found");
        }
    }

    fn shape(&self, area_tag: PhysicsAreaTag) -> Option<PhysicsShapeTag> {
        let area_key = area_tag_to_store_key(area_tag);
        let bodies = self.storages.bodies_r();

        let area = bodies.get_body(area_key);
        if let Some(area) = area {
            area.shape_key.map(store_key_to_shape_tag)
        } else {
            None
        }
    }

    fn set_body_transform(&self, area_tag: PhysicsAreaTag, transf: &Isometry3<N>) {
        let body_key = area_tag_to_store_key(area_tag);
        let bodies = self.storages.bodies_r();

        let area = bodies.get_body(body_key);
        if let Some(mut area) = area {
            area.set_body_transform(transf);
        }
    }

    fn body_transform(&self, area_tag: PhysicsAreaTag) -> Isometry3<N> {
        let area_key = area_tag_to_store_key(area_tag);
        let bodies = self.storages.bodies_r();

        let area = bodies.get_body(area_key);
        if let Some(area) = area {
            *area.body_transform()
        } else {
            Isometry3::identity()
        }
    }

    fn overlap_events(&self, area_tag: PhysicsAreaTag) -> Vec<OverlapEvent> {
        let area_key = area_tag_to_store_key(area_tag);
        let bodies = self.storages.bodies_r();

        let area = bodies.get_body(area_key);
        if let Some(area) = area {
            if let BodyData::Area(e) = &area.body_data {
                return e.to_vec();
            }
        }
        Vec::new()
    }
}

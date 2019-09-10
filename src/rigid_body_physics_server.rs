use amethyst_core::ecs::Entity;
use amethyst_core::math::{one, zero, Isometry3, Point, Vector3};
use amethyst_phythyst::{objects::*, servers::*, PtReal};
use log::error;
use nphysics3d::{
    math::{Force, ForceType},
    object::{
        BodyPartHandle as NpBodyPartHandle, Collider as NpCollider, ColliderDesc as NpColliderDesc,
        RigidBody as NpRigidBody, RigidBodyDesc as NpRigidBodyDesc,
    },
};

use crate::{
    body::{Body, BodyData},
    conversors::*,
    servers_storage::*,
    shape::RigidShape,
    storage::StoreKey,
    utils::*,
};

pub struct RBodyNpServer<N: PtReal> {
    storages: ServersStorages<N>,
}

impl<N: PtReal> RBodyNpServer<N> {
    pub fn new(storages: ServersStorages<N>) -> Self {
        RBodyNpServer { storages }
    }
}

// This is a collection of function that can be used by other servers to perform some common
// operations on the areas.
impl<N: PtReal> RBodyNpServer<N> {
    pub fn drop_body(
        body_tag: PhysicsRigidBodyTag,
        bodies_storage: &mut BodiesStorageWrite<'_, N>,
        colliders_storage: &mut CollidersStorageWrite<'_, N>,
        shapes_storage: &ShapesStorageRead<'_, N>,
    ) {
        let body_key = rigid_tag_to_store_key(body_tag);
        if let Some(mut body) = bodies_storage.get_body(body_key) {
            Self::remove_shape(&mut *body, shapes_storage, colliders_storage);
        }
        bodies_storage.drop_body(body_key);
    }

    /// Set shape.
    /// Take care to register the shape and set the collider to the body.
    pub fn install_shape(
        body: &mut Body<N>,
        shape: &mut RigidShape<N>,
        mut collider_desc: NpColliderDesc<N>,
        colliders: &mut CollidersStorageWrite<'_, N>,
    ) {
        if shape.is_concave() {
            collider_desc.set_density(zero());
        } else {
            collider_desc.set_density(one());
        }

        collider_desc.set_material(body.material_handle.clone());

        Self::install_collider(body, &collider_desc, colliders);

        // Collider registration
        shape.register_body(body.self_key.unwrap());
        body.shape_key = shape.self_key;
    }

    /// Remove shape.
    /// Take care to unregister the shape and then drop the internal collider.
    pub fn remove_shape(
        body: &mut Body<N>,
        shapes: &ShapesStorageRead<'_, N>,
        colliders: &mut CollidersStorageWrite<'_, N>,
    ) {
        if let Some(shape_key) = body.shape_key {
            if let Some(mut shape) = shapes.get(shape_key) {
                shape.unregister_body(body.self_key.unwrap());
            } else {
                error!("A body is associated with a shape, but the shape doesn't exist!");
            }
            body.shape_key = None;
        }
        Self::drop_collider(body, colliders);
    }

    /// Set collider to the body
    pub fn install_collider(
        body: &mut Body<N>,
        collider_desc: &NpColliderDesc<N>,
        colliders: &mut CollidersStorageWrite<'_, N>,
    ) {
        let mut collider = collider_desc.build(NpBodyPartHandle(body.self_key.unwrap(), 0));

        RBodyNpServer::update_user_data(&mut collider, body);

        let key = colliders.insert_collider(collider);
        body.collider_key = Some(key);
    }

    /// Just drop the internal collider of the passed body.
    pub fn drop_collider(body: &mut Body<N>, colliders: &mut CollidersStorageWrite<'_, N>) {
        if let Some(collider_key) = body.collider_key {
            colliders.drop_collider(collider_key);
            body.collider_key = None;
        }
    }

    pub fn update_user_data(collider: &mut NpCollider<N, StoreKey>, body: &Body<N>) {
        collider.set_user_data(Some(Box::new(UserData::new(
            ObjectType::RigidBody,
            body.self_key.unwrap(),
            body.entity,
        ))));
    }

    /// Extract collider description from a rigid body
    pub fn extract_collider_desc(
        _np_rigid_body: &NpRigidBody<N>, // This is not yet used, but in future it will be.
        shape: &RigidShape<N>,
        np_collider_desc: &mut NpColliderDesc<N>,
    ) {
        if shape.is_concave() {
            np_collider_desc.set_density(zero());
        } else {
            np_collider_desc.set_density(one());
        }
    }

    pub fn active_body(body_key: StoreKey, bodies: &BodiesStorageRead<'_, N>) {
        if let Some(mut body) = bodies.get_body(body_key) {
            body.activate();
        }
    }
}

impl<N> RBodyPhysicsServerTrait<N> for RBodyNpServer<N>
where
    N: PtReal,
{
    fn create_body(&self, body_desc: &RigidBodyDesc<N>) -> PhysicsHandle<PhysicsRigidBodyTag> {
        let mut bodies_storage = self.storages.bodies_w();

        // Create Rigid body
        let np_rigid_body = NpRigidBodyDesc::new()
            .set_status(body_mode_conversor::to_physics(body_desc.mode))
            .set_mass(body_desc.mass)
            .build();

        let b_key = bodies_storage.insert_body(Body::new_rigid_body(
            Box::new(np_rigid_body),
            body_desc.friction,
            body_desc.bounciness,
        ));
        let mut body = bodies_storage.get_body(b_key).unwrap();
        body.self_key = Some(b_key);

        PhysicsHandle::new(store_key_to_rigid_tag(b_key), self.storages.gc.clone())
    }

    fn set_entity(&self, body_tag: PhysicsRigidBodyTag, entity: Option<Entity>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            fail_cond!(!matches!(body.body_data, BodyData::Rigid));
            body.entity = entity;

            if let Some(collider_key) = body.collider_key {
                let colliders = self.storages.colliders_r();
                let collider = colliders.get_collider(collider_key);
                if let Some(mut collider) = collider {
                    RBodyNpServer::update_user_data(&mut *collider, &*body);
                } else {
                    error!("A body is assigned to a collider, but the collider doesn't exist!")
                }
            }
        }
    }

    fn entity(&self, body_tag: PhysicsRigidBodyTag) -> Option<Entity> {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(body) = body {
            body.entity
        } else {
            None
        }
    }

    fn set_shape(&self, body_tag: PhysicsRigidBodyTag, shape_tag: Option<PhysicsShapeTag>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let shape_key = shape_tag.map(shape_tag_to_store_key);

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            if body.shape_key == shape_key {
                return;
            }

            let mut colliders = self.storages.colliders_w();
            let shapes = self.storages.shapes_r();

            // Remove the old shape
            if let Some(_b_shape_key) = body.shape_key {
                RBodyNpServer::remove_shape(&mut *body, &shapes, &mut colliders);
            }

            // Assign the new shape if shape_tag is Some
            if let Some(shape_key) = shape_key {
                let shape = shapes.get(shape_key);
                if let Some(mut shape) = shape {
                    // Create and attach the collider
                    let collider_desc = NpColliderDesc::new(shape.shape_handle().clone());

                    RBodyNpServer::install_shape(
                        &mut *body,
                        &mut *shape,
                        collider_desc,
                        &mut colliders,
                    );
                } else {
                    error!("During the rigid body creation, was not possible to find the shape to assign");
                }
            }
        } else {
            error!("Body not found");
        }
    }

    fn shape(&self, body_tag: PhysicsRigidBodyTag) -> Option<PhysicsShapeTag> {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(body) = body {
            body.shape_key.map(store_key_to_shape_tag)
        } else {
            None
        }
    }

    fn set_body_transform(&self, body_tag: PhysicsRigidBodyTag, transf: &Isometry3<N>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.set_body_transform(transf);
        }
    }

    fn body_transform(&self, body_tag: PhysicsRigidBodyTag) -> Isometry3<N> {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(body) = body {
            *body.body_transform()
        } else {
            Isometry3::identity()
        }
    }

    fn set_mode(&self, body_tag: PhysicsRigidBodyTag, mode: BodyMode) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body
                .set_status(body_mode_conversor::to_physics(mode));
        }
    }

    fn mode(&self, body_tag: PhysicsRigidBodyTag) -> BodyMode {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(body) = body {
            body_mode_conversor::from_physics(body.np_body.status())
        } else {
            error!("Rigid Body not foud");
            BodyMode::Disabled
        }
    }

    fn set_friction(&self, _body_tag: PhysicsRigidBodyTag, _friction: N) {
        unimplemented!("Make sure to have a sharable material instead");
    }

    fn friction(&self, _body_tag: PhysicsRigidBodyTag) -> N {
        unimplemented!();
    }

    fn set_bounciness(&self, _body_tag: PhysicsRigidBodyTag, _bounciness: N) {
        unimplemented!("Make sure to have a sharable material instead");
    }

    fn bounciness(&self, _body_tag: PhysicsRigidBodyTag) -> N {
        unimplemented!();
    }

    fn clear_forces(&self, body_tag: PhysicsRigidBodyTag) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body.clear_forces();
        }
    }

    fn apply_force(&self, body_tag: PhysicsRigidBodyTag, force: &Vector3<N>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body
                .apply_force(0, &Force::linear(*force), ForceType::Force, true);
        }
    }

    fn apply_torque(&self, body_tag: PhysicsRigidBodyTag, force: &Vector3<N>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body
                .apply_force(0, &Force::torque(*force), ForceType::Force, true);
        }
    }

    fn apply_force_at_position(
        &self,
        body_tag: PhysicsRigidBodyTag,
        force: &Vector3<N>,
        position: &Vector3<N>,
    ) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body.apply_force_at_local_point(
                0,
                force,
                &Point::from(*position),
                ForceType::Force,
                true,
            );
        }
    }

    fn apply_impulse(&self, body_tag: PhysicsRigidBodyTag, impulse: &Vector3<N>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body
                .apply_force(0, &Force::linear(*impulse), ForceType::Impulse, true);
        }
    }

    fn apply_angular_impulse(&self, body_tag: PhysicsRigidBodyTag, impulse: &Vector3<N>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body
                .apply_force(0, &Force::torque(*impulse), ForceType::Impulse, true);
        }
    }

    fn apply_impulse_at_position(
        &self,
        body_tag: PhysicsRigidBodyTag,
        impulse: &Vector3<N>,
        position: &Vector3<N>,
    ) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            body.np_body.apply_force_at_point(
                0,
                impulse,
                &Point::from(*position),
                ForceType::Impulse,
                true,
            );
        }
    }

    fn set_linear_velocity(&self, body_tag: PhysicsRigidBodyTag, velocity: &Vector3<N>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            if let Some(rb_body) = body.rigid_body_mut() {
                rb_body.set_linear_velocity(*velocity);
            } else {
                error!("The tag is not associated to any RigidBody");
            }
        }
    }

    fn linear_velocity(&self, body_tag: PhysicsRigidBodyTag) -> Vector3<N> {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(body) = body {
            if let Some(rb_body) = body.rigid_body() {
                return rb_body.velocity().linear;
            } else {
                error!("The tag is not associated to any RigidBody");
            }
        }
        Vector3::zeros()
    }

    fn set_angular_velocity(&self, body_tag: PhysicsRigidBodyTag, velocity: &Vector3<N>) {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(mut body) = body {
            if let Some(rb_body) = body.rigid_body_mut() {
                rb_body.set_angular_velocity(*velocity);
            } else {
                error!("The tag is not associated to any RigidBody");
            }
        }
    }

    fn angular_velocity(&self, body_tag: PhysicsRigidBodyTag) -> Vector3<N> {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(body) = body {
            if let Some(rb_body) = body.rigid_body() {
                return rb_body.velocity().angular;
            } else {
                error!("The tag is not associated to any RigidBody");
            }
        }
        Vector3::zeros()
    }

    fn linear_velocity_at_position(
        &self,
        body_tag: PhysicsRigidBodyTag,
        position: &Vector3<N>,
    ) -> Vector3<N> {
        let body_key = rigid_tag_to_store_key(body_tag);
        let bodies = self.storages.bodies_r();

        let body = bodies.get_body(body_key);
        if let Some(body) = body {
            if let Some(rb_body) = body.rigid_body() {
                return rb_body.velocity().shift(&position).linear;
            } else {
                error!("The tag is not associated to any RigidBody");
            }
        }
        Vector3::zeros()
    }
}

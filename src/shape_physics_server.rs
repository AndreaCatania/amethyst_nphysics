use amethyst_phythyst::{
    objects::*,
    servers::{ShapeDesc, ShapePhysicsServerTrait},
    PtReal,
};
use log::error;
use nphysics3d::object::ColliderDesc as NpColliderDesc;

use crate::{
    area_physics_server::AreaNpServer, body::BodyData, conversors::*,
    rigid_body_physics_server::RBodyNpServer, servers_storage::*, shape::RigidShape,
    storage::StoreKey,
};

pub struct ShapeNpServer<N: PtReal> {
    storages: ServersStorages<N>,
}

impl<N: PtReal> ShapeNpServer<N> {
    pub fn new(storages: ServersStorages<N>) -> Self {
        ShapeNpServer { storages }
    }

    /// Drop a shape, return false if it can't be removed right now or it something failed.
    pub fn drop_shape(
        shape_tag: PhysicsShapeTag,
        shapes_storage: &mut ShapesStorageWrite<'_, N>,
    ) -> bool {
        let shape_key = shape_tag_to_store_key(shape_tag);

        let safe_to_drop = !ShapeNpServer::has_dependency(shape_key, shapes_storage);

        if !safe_to_drop {
            if let Some(mut shape) = shapes_storage.get(shape_key) {
                if !shape.marked_for_drop {
                    shape.marked_for_drop = true;
                    fail!("A shape is marked for drop while still in use. Consider to store the PhysicsHandle<PhysicsShapeTag> to not waste resources.", false);
                }
            }
            false
        } else {
            shapes_storage.remove(shape_key);
            true
        }
    }

    /// Returns `true` if this shape is still in use.
    // It's using ShapeStorageWrite because this function is used during shape dropping, and at that
    // stage only the writing storage is available.
    pub fn has_dependency(
        shape_key: StoreKey,
        shapes_storage: &mut ShapesStorageWrite<'_, N>,
    ) -> bool {
        if let Some(shape) = shapes_storage.get(shape_key) {
            if shape.bodies().is_empty() {
                return true;
            }
        }
        false
    }
}

impl<N: PtReal> ShapePhysicsServerTrait<N> for ShapeNpServer<N> {
    fn create_shape(&self, shape_desc: &ShapeDesc<N>) -> PhysicsHandle<PhysicsShapeTag> {
        let shape = Box::new(RigidShape::new(shape_desc));

        let mut shapes_storage = self.storages.shapes_w();
        let shape_key = shapes_storage.insert(shape);

        let mut shape = shapes_storage.get(shape_key).unwrap();
        shape.self_key = Some(shape_key);

        PhysicsHandle::new(store_key_to_shape_tag(shape_key), self.storages.gc.clone())
    }

    fn update_shape(&self, shape_tag: PhysicsShapeTag, shape_desc: &ShapeDesc<N>) {
        let bodies = self.storages.bodies_r();
        let mut colliders = self.storages.colliders_w();
        let shapes = self.storages.shapes_r();

        let shape_key = shape_tag_to_store_key(shape_tag);
        let shape = shapes.get(shape_key);
        if let Some(mut shape) = shape {
            shape.update(shape_desc);

            let b_keys = shape.bodies();
            for body_key in b_keys {
                let body = bodies.get_body(*body_key);
                if let Some(mut body) = body {
                    let mut collider_desc = NpColliderDesc::new(shape.shape_handle().clone());

                    match &body.body_data {
                        BodyData::Rigid => {
                            RBodyNpServer::drop_collider(&mut *body, &mut colliders);
                            RBodyNpServer::extract_collider_desc(
                                body.rigid_body().unwrap(),
                                &*shape,
                                &mut collider_desc,
                            );
                            RBodyNpServer::install_collider(
                                &mut *body,
                                &collider_desc,
                                &mut colliders,
                            );
                        }
                        BodyData::Area(_e) => {
                            AreaNpServer::drop_collider(&mut *body, &mut colliders);
                            AreaNpServer::extract_collider_desc(
                                body.rigid_body().unwrap(),
                                &*shape,
                                &mut collider_desc,
                            );
                            AreaNpServer::install_collider(
                                &mut *body,
                                &collider_desc,
                                &mut colliders,
                            );
                        }
                    }
                }
            }
        } else {
            error!("Shape not found!");
        }
    }
}

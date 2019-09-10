use amethyst_core::{ecs::Entity, math::Isometry3};
use amethyst_phythyst::{servers::OverlapEvent, PtReal};
use nphysics3d::{
    material::{BasicMaterial, MaterialHandle},
    object::{Body as NpBody, RigidBody as NpRigidBody},
};

use crate::storage::StoreKey;

/// Store information about a body
///
/// A body is:
/// - Rigid - RigidBody(Disabled, Dynamic, Static, Kinematic)
/// - Area - RigidBody(Static)
#[allow(missing_debug_implementations)]
pub struct Body<N: PtReal> {
    pub self_key: Option<StoreKey>,
    pub np_body: Box<dyn NpBody<N>>,
    pub body_data: BodyData,
    pub collider_key: Option<StoreKey>,
    pub shape_key: Option<StoreKey>,
    pub entity: Option<Entity>,
    pub material_handle: MaterialHandle<N>, // TODO share this material across many bodies
}

impl<N: PtReal> Body<N> {
    /// Creates a Rigid Body `Body`
    pub(crate) fn new_rigid_body(
        np_rigid_body: Box<NpRigidBody<N>>,
        friction: N,
        bounciness: N,
    ) -> Self {
        Body {
            self_key: None,
            np_body: np_rigid_body,
            body_data: BodyData::Rigid,
            collider_key: None,
            shape_key: None,
            entity: None,
            material_handle: MaterialHandle::new(BasicMaterial::new(bounciness, friction)),
        }
    }

    /// Creates an Area `Body`
    pub(crate) fn new_area(np_rigid_body: Box<NpRigidBody<N>>, friction: N, bounciness: N) -> Self {
        Body {
            self_key: None,
            np_body: np_rigid_body,
            body_data: BodyData::Area(Vec::new()),
            collider_key: None,
            shape_key: None,
            entity: None,
            material_handle: MaterialHandle::new(BasicMaterial::new(bounciness, friction)),
        }
    }

    /// Returns some with a rigid body reference if this body is a RigidBody.
    ///
    /// Note that the area is a RigidBody.
    pub fn rigid_body(&self) -> Option<&NpRigidBody<N>> {
        self.np_body.downcast_ref::<NpRigidBody<N>>()
    }

    /// Returns some with a rigid body mut reference if this body is a RigidBody.
    ///
    /// Note that the area is a RigidBody.
    pub fn rigid_body_mut(&mut self) -> Option<&mut NpRigidBody<N>> {
        self.np_body.downcast_mut::<NpRigidBody<N>>()
    }

    pub fn activate(&mut self) {
        self.np_body.activate();
    }

    /// Set body transform.
    pub fn set_body_transform(&mut self, transf: &Isometry3<N>) {
        match self.body_data {
            BodyData::Rigid | BodyData::Area(_) => {
                if let Some(body) = self.rigid_body_mut() {
                    body.set_position(*transf);
                } else {
                    panic!("Failed to cast the body, to a Rigid Body!");
                }
            }
        }
    }

    /// Get body transform.
    pub fn body_transform(&self) -> &Isometry3<N> {
        match self.body_data {
            BodyData::Rigid | BodyData::Area(_) => {
                if let Some(body) = self.rigid_body() {
                    body.position()
                } else {
                    panic!("Failed to cast the body, to a Rigid Body!");
                }
            }
        }
    }
}

/// Here are stored extra body information, depending on the body type
#[derive(Debug, PartialEq)]
pub enum BodyData {
    Rigid,
    Area(Vec<OverlapEvent>),
}

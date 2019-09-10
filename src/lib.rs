//! # IMPORTANT:
//! This library is not meant to stay inside the amethyst project.
//!
//! Actually this is here only to make it more simple to develop.
//! The idea is to move this outside once it's almost done.

//! # NPhysics backend for Phythyst.
//! To use this backend you have to specify the `NPhysicsBackend` type in the `PhysicsBundle`.
//!
//! Follow the `Phythyst` instructions know more.
//!

// ## Naming
// Since NPhysics doesn't use any prefix to identify its structures this implementation take care
// to append the prefix `Np` to any struct that come from NPhysics.
// In this way is possible to have a clear distinction of what is what.
// Example: `RigidBody` and `NpRigidBody`.

#![warn(
    missing_debug_implementations,
    rust_2018_idioms,
    rust_2018_compatibility
)]
#![warn(clippy::all)]

use area_physics_server::AreaNpServer;
use joint_physics_server::JointNpServer;
use rigid_body_physics_server::RBodyNpServer;
use shape_physics_server::ShapeNpServer;
use world_physics_server::WorldNpServer;

use amethyst_phythyst::{servers::PhysicsWorld, PtReal};

/// NPhysics backend can be specified as type of the PhysicsBundle to use NPhysics engine.
#[allow(missing_debug_implementations)]
pub struct NPhysicsBackend;

/// NPhysics Backend
impl<N> amethyst_phythyst::PhysicsBackend<N> for NPhysicsBackend
where
    N: PtReal,
{
    fn create_world() -> PhysicsWorld<N> {
        let storages = servers_storage::ServersStorage::new();

        PhysicsWorld::new(
            Box::new(WorldNpServer::new(storages.clone())),
            Box::new(RBodyNpServer::new(storages.clone())),
            Box::new(AreaNpServer::new(storages.clone())),
            Box::new(ShapeNpServer::new(storages.clone())),
            Box::new(JointNpServer::new(storages.clone())),
        )
    }
}

#[macro_use]
mod conditional_macros;
mod area_physics_server;
mod body;
mod body_storage;
mod collider_storage;
mod conversors;
mod force_generator;
mod force_generator_storage;
mod joint;
mod joint_physics_server;
mod joint_storage;
mod rigid_body_physics_server;
pub mod servers_storage;
mod shape;
mod shape_physics_server;
mod storage;
mod utils;
mod world_physics_server;

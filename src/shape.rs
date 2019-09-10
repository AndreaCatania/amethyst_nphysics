use amethyst_core::math::{convert, Unit, Vector3};
use amethyst_phythyst::{servers::ShapeDesc, PtReal};
use ncollide3d::shape::{
    Ball as NcBall, Capsule as NcCapsule, Compound as NcCompound, ConvexHull as NcConvexHull,
    Cuboid as NcCuboid, Plane as NcPlane, ShapeHandle as NcShapeHandle, TriMesh as NcTriMesh,
};

use crate::storage::StoreKey;

#[allow(missing_debug_implementations)]
pub struct RigidShape<N: PtReal> {
    pub self_key: Option<StoreKey>,
    shape_desc: ShapeDesc<N>,
    shape_handle: NcShapeHandle<N>,
    bodies: Vec<StoreKey>,
    /// This is used to know if the shape will be soon dropped since no one own it anymore.
    ///
    /// When the shape is no more owned but still in use by a rigid body or an area is safer not delete it.
    /// Rather the program mark it for removal and when nobody will use it anymore it will be safely
    /// dropped.
    pub marked_for_drop: bool,
}

impl<N: PtReal> RigidShape<N> {
    pub fn new(shape_desc: &ShapeDesc<N>) -> Self {
        RigidShape {
            self_key: None,
            shape_desc: shape_desc.clone(), // Consider to not save this in order to save memory when Convex and TriMeshes are used.
            shape_handle: RigidShape::generate_handle(shape_desc),
            bodies: Vec::new(),
            marked_for_drop: false,
        }
    }

    pub fn update(&mut self, shape_desc: &ShapeDesc<N>) {
        self.shape_desc = shape_desc.clone();
        self.shape_handle = RigidShape::generate_handle(shape_desc);
    }

    pub fn shape_handle(&self) -> &NcShapeHandle<N> {
        &self.shape_handle
    }

    pub fn register_body(&mut self, body: StoreKey) {
        self.bodies.push(body);
    }

    pub fn unregister_body(&mut self, body: StoreKey) {
        self.bodies.retain(|&b| b != body);
    }

    pub fn bodies(&self) -> &Vec<StoreKey> {
        &self.bodies
    }

    pub fn is_concave(&self) -> bool {
        match &self.shape_desc {
            ShapeDesc::TriMesh { .. } => true,
            _ => false,
        }
    }
}

impl<N: PtReal> RigidShape<N> {
    fn generate_handle(shape_desc: &ShapeDesc<N>) -> NcShapeHandle<N> {
        match shape_desc {
            ShapeDesc::Sphere { radius } => NcShapeHandle::new(NcBall::new(*radius)),
            ShapeDesc::Cube { half_extents } => NcShapeHandle::new(NcCuboid::new(*half_extents)),
            ShapeDesc::Capsule {
                half_height,
                radius,
            } => NcShapeHandle::new(NcCapsule::new(*half_height, *radius)),
            ShapeDesc::Cylinder { .. } => {
                unimplemented!();
                //NcShapeHandle::new(NcCylinder::new(*half_height, *radius))
            }
            ShapeDesc::Plane => NcShapeHandle::new(NcPlane::new(Unit::new_normalize(
                Vector3::new(convert(0.0), convert(1.0), convert(0.0)),
            ))),
            ShapeDesc::Convex { points } => NcShapeHandle::new(
                NcConvexHull::try_from_points(&points)
                    .expect("Was not possible to construct the ConvexHull from the passed points."),
            ),
            ShapeDesc::TriMesh { points, indices } => {
                NcShapeHandle::new(NcTriMesh::new(points.clone(), indices.clone(), None))
            }
            ShapeDesc::Compound { shapes } => {
                let computed_shapes = shapes
                    .iter()
                    .map(|v| (v.0, RigidShape::generate_handle(&v.1)))
                    .collect();
                NcShapeHandle::new(NcCompound::new(computed_shapes))
            }
        }
    }
}

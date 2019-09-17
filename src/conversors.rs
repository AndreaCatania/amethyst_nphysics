use amethyst_physics::objects::{
    PhysicsAreaTag, PhysicsJointTag, PhysicsRigidBodyTag, PhysicsShapeTag,
};

use crate::storage::StoreKey;

pub mod body_mode_conversor {
    use amethyst_physics::servers::BodyMode;

    use nphysics3d::object::BodyStatus as NpBodyStatus;

    pub fn to_physics(m: BodyMode) -> NpBodyStatus {
        match m {
            BodyMode::Disabled => NpBodyStatus::Disabled,
            BodyMode::Static => NpBodyStatus::Static,
            BodyMode::Dynamic => NpBodyStatus::Dynamic,
            BodyMode::Kinematic => NpBodyStatus::Kinematic,
        }
    }

    pub fn from_physics(s: NpBodyStatus) -> BodyMode {
        match s {
            NpBodyStatus::Disabled => BodyMode::Disabled,
            NpBodyStatus::Static => BodyMode::Static,
            NpBodyStatus::Dynamic => BodyMode::Dynamic,
            NpBodyStatus::Kinematic => BodyMode::Kinematic,
        }
    }
}

macro_rules! opaque_conversors {
    ($t:ident, $to:ident, $from:ident, $test_mod:ident) => {
        pub fn $to(tag: $t) -> StoreKey {
            match tag {
                $t::UsizeU64(a, b) => StoreKey::new(a, b),
                _ => {
                    // If happens, something is strange
                    panic!();
                }
            }
        }

        pub fn $from(key: StoreKey) -> $t {
            unsafe { key.map(|index, generation| $t::new_usizeu64(index, generation)) }
        }

        #[cfg(test)]
        mod $test_mod {
            use crate::conversors::*;

            #[test]
            fn test() {
                let tag = unsafe { $t::new_usizeu64(1, 10) };
                let key = $to(tag);
                assert_eq!(tag, $from(key));
            }
        }
    };
}

opaque_conversors!(
    PhysicsRigidBodyTag,
    rigid_tag_to_store_key,
    store_key_to_rigid_tag,
    test_conversors_physics_rigid_body_tag
);
opaque_conversors!(
    PhysicsAreaTag,
    area_tag_to_store_key,
    store_key_to_area_tag,
    test_conversors_physics_area_tag
);
opaque_conversors!(
    PhysicsShapeTag,
    shape_tag_to_store_key,
    store_key_to_shape_tag,
    test_conversors_physics_shape_tag
);
opaque_conversors!(
    PhysicsJointTag,
    joint_tag_to_store_key,
    store_key_to_joint_tag,
    test_conversors_physics_joint_tag
);

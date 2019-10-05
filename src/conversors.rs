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

pub mod collision_group_conversor {
    use amethyst_physics::objects::CollisionGroup;
    use ncollide3d::pipeline::object::CollisionGroups as NcCollisionGroups;

    pub fn to_nphysics(
        belong_to: &[CollisionGroup],
        collide_with: &[CollisionGroup],
    ) -> NcCollisionGroups {
        let mut membership: Vec<usize> = belong_to.iter().map(|v| v.get().into()).collect();
        membership.sort();
        membership.dedup();
        let mut white_list: Vec<usize> = collide_with.iter().map(|v| v.get().into()).collect();
        white_list.sort();
        white_list.dedup();
        let mut black_list = Vec::<usize>::new();
        for e in &membership {
            if let Err(..) = white_list.binary_search(e) {
                black_list.push(*e);
            }
        }
        let mut collision_groups = NcCollisionGroups::new();
        collision_groups.set_membership(membership.as_slice());
        collision_groups.set_whitelist(white_list.as_slice());
        collision_groups.set_blacklist(black_list.as_slice());

        collision_groups
    }

    pub fn from_nphysics(groups: &NcCollisionGroups) -> (Vec<CollisionGroup>, Vec<CollisionGroup>) {
        let mut belong_to = Vec::<CollisionGroup>::with_capacity(NcCollisionGroups::max_group_id());
        let mut collide_with =
            Vec::<CollisionGroup>::with_capacity(NcCollisionGroups::max_group_id());

        for group in 0..NcCollisionGroups::max_group_id() {
            if groups.is_member_of(group) {
                belong_to.push(CollisionGroup::new(group as u8));
            }
            if groups.is_group_whitelisted(group) {
                collide_with.push(CollisionGroup::new(group as u8));
            }
        }
        (belong_to, collide_with)
    }
}

macro_rules! opaque_conversors {
    ($t:ident, $to:ident, $from:ident, $test_mod:ident) => {
        pub fn $to(tag: $t) -> StoreKey {
            match tag {
                $t::UsizeU64(a, b) => StoreKey::from_raw_parts(a, b),
                _ => {
                    // If happens, something is strange
                    panic!();
                }
            }
        }

        pub fn $from(key: StoreKey) -> $t {
            let (index, generation) = key.into_raw_parts();
            unsafe { $t::new_usizeu64(index, generation) }
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

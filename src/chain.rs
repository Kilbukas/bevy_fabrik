use bevy::{
    ecs::{
        component::{ComponentHooks, ComponentId, StorageType},
        query::QueryEntityError,
        system::QueryLens,
        world::DeferredWorld,
    },
    math::{Affine3A, Vec3A},
    prelude::*,
};

use crate::constraint::*;
use crate::util::QuatExt;

const DEFAULT_TARGET_SQ_THRESHOLD: f32 = 0.001;
const DEFAULT_MAX_ITTERATIONS: usize = 10;

/// The bone axis in local space.
const BONE_AXIS: Vec3 = Vec3::Y;

/// Wrapper for the [QueryEntityError] specifically for IK chain errors.
/// Used to work around the [QueryEntityError] lifetime introduced in Bevy 0.15.
#[derive(Clone, Copy)]
pub enum IkChainError {
    IkComponentQueryError,
}

impl core::error::Error for IkChainError {}

impl core::fmt::Display for IkChainError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Failed to query required IK chain components")
    }
}

impl core::fmt::Debug for IkChainError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "IkChainError")
    }
}

impl From<QueryEntityError<'_>> for IkChainError {
    fn from(_: QueryEntityError) -> Self {
        Self::IkComponentQueryError
    }
}

/// Per-joint working data.
struct JointData {
    position: Vec3,
    rotation: Quat,
    length: f32,
    forward_axis: Vec3,
    up_axis: Vec3,
}

/// Main component that defines an IK chain. Add this to an end entity
/// of a chain of entities to solve the chain's inverse kinematics.
#[derive(Reflect)]
pub struct IkChain {
    /// If false, the chain will not be solved.
    pub enabled: bool,
    /// The target position for the chain to reach.
    pub target: Vec3,
    /// The squared distance threshold for the chain to stop solving.
    pub threshold: f32,
    /// The maximum number of iterations to solve the chain.
    pub max_iterations: usize,
    pub(crate) joint_count: usize,
    pub(crate) joint_lengths: Vec<f32>,
}

impl Component for IkChain {
    const STORAGE_TYPE: StorageType = StorageType::Table;

    fn register_component_hooks(hooks: &mut ComponentHooks) {
        hooks.on_insert(ik_chain_insert_hook);
    }
}

impl IkChain {
    /// Creates a new IK chain with the given number of joints.
    ///
    /// The parent entities need to already exist in the [`World`] and
    /// have [`GlobalTransform`] components correctly set up.
    pub fn new(joint_count: usize) -> Self {
        Self {
            enabled: true,
            target: Vec3::ZERO,
            threshold: DEFAULT_TARGET_SQ_THRESHOLD,
            max_iterations: DEFAULT_MAX_ITTERATIONS,
            joint_count,
            joint_lengths: vec![],
        }
    }

    pub(crate) fn solve(
        &self,
        entity: &Entity,
        constraint_query: &mut Query<(Option<&SwingConstraint>, Option<&TwistConstraint>)>,
        transform_query: &mut Query<(Option<&Parent>, &mut Transform, &GlobalTransform)>,
    ) -> Result<(), IkChainError> {
        if !self.enabled {
            return Ok(());
        }

        let dist = transform_query
            .get(*entity)
            .map(|(_, _, gt)| gt.translation().distance_squared(self.target))?;

        if dist < self.threshold {
            return Ok(());
        }

        let mut constraint_lens: QueryLens<(
            Option<&Parent>,
            &GlobalTransform,
            Option<&SwingConstraint>,
            Option<&TwistConstraint>,
        )> = constraint_query.join_filtered(transform_query);
        let lens_query = constraint_lens.query();

        // Collect joint data. Index 0 = end effector (tip), last = root.
        let mut joints: Vec<JointData> = Vec::with_capacity(self.joint_count);
        let mut constraint_refs: Vec<Vec<&dyn Constraint>> = Vec::with_capacity(self.joint_count);
        let mut current_joint = *entity;

        for i in 0..self.joint_count {
            let (parent, global_transform, swing_constraint, twist_constraint) =
                lens_query.get(current_joint)?;

            let transform = global_transform.compute_transform();

            let mut cs: Vec<&dyn Constraint> = Vec::new();
            if let Some(sc) = swing_constraint {
                cs.push(sc);
            }
            if let Some(tc) = twist_constraint {
                cs.push(tc);
            }

            // Compute the forward axis: direction from parent to this joint in
            // parent's local space.
            let forward_axis = if i < self.joint_count - 1 {
                let local_transform = transform_query.get(current_joint).map(|(_, t, _)| t)?;
                local_transform.translation.normalize_or(BONE_AXIS)
            } else {
                BONE_AXIS
            };

            joints.push(JointData {
                position: global_transform.translation(),
                rotation: transform.rotation,
                length: if i < self.joint_lengths.len() {
                    self.joint_lengths[i]
                } else {
                    0.0
                },
                forward_axis,
                up_axis: Vec3::Z,
            });
            constraint_refs.push(cs);

            // Move to parent
            if i < self.joint_count - 1 {
                current_joint = **parent.unwrap();
            }
        }

        // FABRIK iterations
        for _ in 0..self.max_iterations {
            if self.target.distance_squared(joints[0].position) < self.threshold {
                break;
            }

            self.backward_pass(&mut joints);
            self.forward_pass(&mut joints, &constraint_refs);
        }

        // Apply the solved positions and rotations back to the transforms.
        // joints[0] = end effector, joints[last] = root.
        let mut current_joint = *entity;
        for i in 0..self.joint_count - 1 {
            let (parent, mut transform, _) = transform_query.get_mut(current_joint)?;

            // Convert world-space position/rotation to local space relative to parent.
            // Parent in the chain is joints[i+1].
            let parent_gt: GlobalTransform = Affine3A {
                matrix3: bevy::math::Mat3A::from_quat(joints[i + 1].rotation),
                translation: Vec3A::from(joints[i + 1].position),
            }
            .into();

            let child_gt: GlobalTransform = Affine3A {
                matrix3: bevy::math::Mat3A::from_quat(joints[i].rotation),
                translation: Vec3A::from(joints[i].position),
            }
            .into();

            let local = child_gt.reparented_to(&parent_gt);
            transform.translation = local.translation;
            transform.rotation = local.rotation;

            current_joint = **parent.unwrap();
        }

        Ok(())
    }

    fn backward_pass(&self, joints: &mut [JointData]) {
        let root_idx = joints.len() - 1;

        // Save root position to restore later
        let root_origin = joints[root_idx].position;

        // Set end effector to target
        joints[0].position = self.target;

        // Reposition each joint toward root
        for i in 1..joints.len() {
            let direction = (joints[i].position - joints[i - 1].position).normalize();
            joints[i].position = joints[i - 1].position + direction * joints[i - 1].length;
        }

        // Restore root to its original position
        joints[root_idx].position = root_origin;
    }

    fn forward_pass(&self, joints: &mut [JointData], constraints: &[Vec<&dyn Constraint>]) {
        let root_idx = joints.len() - 1;

        joints[root_idx - 1].position = joints[root_idx].position
            + joints[root_idx].rotation * BONE_AXIS * joints[root_idx - 1].length;

        for i in (1..root_idx).rev() {
            let direction = (joints[i - 1].position - joints[i].position).normalize();

            let (lower, upper) = joints.split_at_mut(i + 1);
            let parent_rotation = upper[0].rotation;
            let joint = &mut lower[i];
            apply_constraints(joint, parent_rotation, direction, &constraints[i]);

            joints[i - 1].position =
                joints[i].position + joints[i].rotation * BONE_AXIS * joints[i - 1].length;
        }

        let tip_dir = (joints[0].position - joints[1].position).normalize();
        if tip_dir.length_squared() > 0.0 {
            joints[0].rotation = Quat::look_rotation_y(tip_dir, joints[1].rotation * Vec3::Z);
        }
    }
}

fn apply_constraints(
    joint: &mut JointData,
    parent_rotation: Quat,
    direction: Vec3,
    constraints: &[&dyn Constraint],
) {
    if constraints.is_empty() {
        joint.rotation = Quat::look_rotation_y(direction, Vec3::Z);
        return;
    }

    let constraint_forward = parent_rotation * joint.forward_axis;
    let constraint_up = parent_rotation * joint.up_axis;
    let rotation_global = Quat::look_rotation_y(constraint_forward, constraint_up);

    let target_rotation = Quat::look_rotation_y(direction, Vec3::Z);

    // Local rotation relative to the constraint reference frame
    let rotation_local = rotation_global.inverse() * target_rotation;

    let (mut twist, mut swing) = rotation_local.decompose(BONE_AXIS);
    constraints.iter().for_each(|constraint| {
        (swing, twist) = constraint.apply(swing, twist);
    });

    // Reconstruct world rotation
    joint.rotation = rotation_global * swing * twist;
}

fn ik_chain_insert_hook(mut world: DeferredWorld, entity: Entity, _component_id: ComponentId) {
    let joint_count = { world.get::<IkChain>(entity).unwrap().joint_count };

    if joint_count < 2 {
        panic!("IK chain must have at least 2 joints.");
    }

    let mut tail = entity;
    let mut lengths = Vec::<f32>::with_capacity(joint_count - 1);

    for _ in 1..joint_count {
        let joint = {
            let Some(joint) = world.get::<Parent>(tail) else {
                panic!(
                    "Parent not found for entity {:?} while building IK chain (length {:?})",
                    tail, joint_count
                );
            };
            **joint
        };

        let joint_pos = {
            let Some(transform) = world.get::<GlobalTransform>(joint) else {
                panic!("Parent transform not found for entity {:?} while building IK chain (length {:?})", joint, joint_count);
            };
            transform.translation()
        };

        let tail_pos = {
            let Some(transform) = world.get::<GlobalTransform>(tail) else {
                panic!(
                    "Transform not found for entity {:?} while building IK chain (length {:?})",
                    tail, joint_count
                );
            };
            transform.translation()
        };

        let diff = tail_pos - joint_pos;
        lengths.push(diff.length());
        tail = joint;
    }
    let Some(mut chain) = world.get_mut::<IkChain>(entity) else {
        panic!(
            "IkChain component not found for entity {:?} while building IK chain.",
            entity
        );
    };

    chain.joint_lengths = lengths;
}

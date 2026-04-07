use bevy::prelude::*;
use bevy::transform::TransformSystem;
use std::f32::consts::PI;

use bevy_fabrik::*;

const SEGMENT_LENGTH: f32 = 0.4;

/// Per-segment constraint configuration.
struct SegmentDef {
    color: Color,
    swing: Option<f32>,
    twist: Option<f32>,
}

///   SWING ONLY (red)    – 3 segments, swing 15°, no twist constraint
///   TWIST ONLY (yellow) – 3 segments, no swing constraint, twist 10°
///   BOTH (blue)         – 3 segments, swing 20°, twist 15°
fn segment_defs() -> Vec<SegmentDef> {
    let deg = PI / 180.0;
    vec![
        // --- SWING ONLY (red) – limited bend, free roll ---
        SegmentDef {
            color: Color::srgb(1.0, 0.2, 0.2),
            swing: Some(15.0 * deg),
            twist: None,
        },
        SegmentDef {
            color: Color::srgb(1.0, 0.2, 0.2),
            swing: Some(15.0 * deg),
            twist: None,
        },
        SegmentDef {
            color: Color::srgb(1.0, 0.2, 0.2),
            swing: Some(15.0 * deg),
            twist: None,
        },
        // --- TWIST ONLY (yellow) – free bend, limited roll ---
        SegmentDef {
            color: Color::srgb(1.0, 0.9, 0.1),
            swing: None,
            twist: Some(10.0 * deg),
        },
        SegmentDef {
            color: Color::srgb(1.0, 0.9, 0.1),
            swing: None,
            twist: Some(10.0 * deg),
        },
        SegmentDef {
            color: Color::srgb(1.0, 0.9, 0.1),
            swing: None,
            twist: Some(10.0 * deg),
        },
        // --- BOTH (blue) – limited bend + limited roll ---
        SegmentDef {
            color: Color::srgb(0.2, 0.5, 1.0),
            swing: Some(20.0 * deg),
            twist: Some(15.0 * deg),
        },
        SegmentDef {
            color: Color::srgb(0.2, 0.5, 1.0),
            swing: Some(20.0 * deg),
            twist: Some(15.0 * deg),
        },
        SegmentDef {
            color: Color::srgb(0.2, 0.5, 1.0),
            swing: Some(20.0 * deg),
            twist: Some(15.0 * deg),
        },
    ]
}

#[derive(Component)]
struct IkTarget;

#[derive(Component)]
struct IkTailMarker;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(InverseKinematicsPlugin)
        .add_systems(Startup, setup)
        .add_systems(
            PostStartup,
            ik_setup.after(TransformSystem::TransformPropagate),
        )
        .add_systems(Update, (move_target, update_chain_target))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let defs = segment_defs();

    // Plank for better constraint visualization.
    let joint_mesh = meshes.add(Cuboid::new(0.06, SEGMENT_LENGTH * 0.9, 0.18));

    let root = commands
        .spawn((Transform::default(), Visibility::default()))
        .id();

    // Build chain from root to tip using per-segment definitions.
    let mut parent = root;
    for def in &defs {
        let mat = materials.add(StandardMaterial {
            base_color: def.color,
            ..default()
        });

        let mut entity_commands = commands.spawn((
            Mesh3d(joint_mesh.clone()),
            MeshMaterial3d(mat),
            Transform::from_translation(Vec3::Y * SEGMENT_LENGTH),
        ));

        if let Some(swing) = def.swing {
            entity_commands.insert(SwingConstraint(swing));
        }
        if let Some(twist) = def.twist {
            entity_commands.insert(TwistConstraint(twist));
        }

        let joint = entity_commands.id();
        commands.entity(parent).add_child(joint);
        parent = joint;
    }

    // Invisible tail (end effector)
    let tail = commands
        .spawn((
            Transform::from_translation(Vec3::Y * SEGMENT_LENGTH),
            Visibility::default(),
            IkTailMarker,
        ))
        .id();
    commands.entity(parent).add_child(tail);

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-3.0, 3.5, 6.0).looking_at(Vec3::Y * 1.5, Vec3::Y),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Sphere { radius: 0.12 })),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 0.0))),
        IkTarget,
    ));

    commands.spawn((
        Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
        DirectionalLight {
            illuminance: 1_500.,
            ..default()
        },
    ));
}

fn ik_setup(mut commands: Commands, query: Query<Entity, With<IkTailMarker>>) {
    let defs = segment_defs();
    let joint_count = defs.len() + 2; // segments + root + tail
    if let Ok(tail) = query.get_single() {
        commands.entity(tail).insert(IkChain::new(joint_count));
    }
}

fn move_target(time: Res<Time>, mut query: Query<&mut Transform, With<IkTarget>>) {
    for mut transform in query.iter_mut() {
        let t = time.elapsed().as_millis() as f32;

        transform.translation.y = (t * 0.002).sin() * 1.0 + 1.5;
        transform.translation.x = (t * 0.001).cos() * 1.5;
        transform.translation.z = (t * 0.001).sin() * 1.5;
    }
}

fn update_chain_target(
    mut chain_query: Query<&mut IkChain>,
    target_query: Query<&GlobalTransform, With<IkTarget>>,
) {
    if let Ok(target) = target_query.get_single() {
        for mut chain in chain_query.iter_mut() {
            chain.target = target.translation();
        }
    }
}

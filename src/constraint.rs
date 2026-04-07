use bevy::prelude::*;

use crate::util::*;

// This will likely be useful more directly when Bevy supports trait queries.
pub(crate) trait Constraint {
    fn apply(&self, swing: Quat, twist: Quat) -> (Quat, Quat);
}

/// Constrains the swing of a joint rotation by a given angle in radians.
#[derive(Component, Clone, Reflect)]
pub struct SwingConstraint(pub f32);

impl Constraint for SwingConstraint {
    fn apply(&self, swing: Quat, twist: Quat) -> (Quat, Quat) {
        (Quat::constrain(swing, self.0), twist)
    }
}

/// Constrains the twist of a joint rotation by a given angle in radians.
#[derive(Component, Clone, Reflect)]
pub struct TwistConstraint(pub f32);

impl Constraint for TwistConstraint {
    fn apply(&self, swing: Quat, twist: Quat) -> (Quat, Quat) {
        (swing, Quat::constrain(twist, self.0))
    }
}

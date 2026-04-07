use bevy::prelude::*;

pub trait QuatExt {
    fn decompose(&self, dir: Vec3) -> (Quat, Quat);
    fn constrain(rotation: Quat, angle: f32) -> Quat;
}

impl QuatExt for Quat {
    fn decompose(&self, dir: Vec3) -> (Quat, Quat) {
        let projected = self.xyz().project_onto(dir);

        let twist = Quat::from_xyzw(projected.x, projected.y, projected.z, self.w).normalize();
        let swing = *self * twist.inverse();

        (twist, swing)
    }

    fn constrain(rotation: Quat, angle: f32) -> Quat {
        let magnitude = f32::sin(0.5 * angle);
        let sqr_magnitude = magnitude * magnitude;

        let mut vector = rotation.xyz();

        if vector.length_squared() > sqr_magnitude {
            vector = vector.normalize() * magnitude;

            // When w is very close to zero (rotation near 180deg), signum(0.0)
            // returns 0.0 which would produce a non-unit quaternion. Treat
            // near-zero w as positive to avoid this.
            let w_sign = if rotation.w >= 0.0 { 1.0 } else { -1.0 };

            return Quat::from_xyzw(
                vector.x,
                vector.y,
                vector.z,
                f32::sqrt(1.0 - sqr_magnitude) * w_sign,
            );
        }
        rotation
    }
}

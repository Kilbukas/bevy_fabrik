use bevy::prelude::*;

pub trait QuatExt {
    fn decompose(&self, dir: Vec3) -> (Quat, Quat);
    fn constrain(rotation: Quat, angle: f32) -> Quat;
    fn look_rotation_y(forward: Vec3, up: Vec3) -> Quat;
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

    fn look_rotation_y(forward: Vec3, up: Vec3) -> Quat {
        let y = forward.normalize_or(Vec3::Y);
        let mut x = y.cross(up).normalize_or_zero();
        if x.length_squared() < 1e-6 {
            // forward and up are parallel - pick an arbitrary perpendicular
            let arbitrary = if y.dot(Vec3::X).abs() < 0.9 {
                Vec3::X
            } else {
                Vec3::Z
            };
            x = y.cross(arbitrary).normalize();
        }
        let z = x.cross(y);
        Quat::from_mat3(&Mat3::from_cols(x, y, z))
    }
}

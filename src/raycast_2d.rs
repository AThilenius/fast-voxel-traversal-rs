use glam::{IVec2, Vec2};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct BoundingVolume2 {
    pub size: (i32, i32),
}

impl BoundingVolume2 {
    pub fn traverse_ray(&self, ray: Ray2) -> VoxelRay2Iterator {
        VoxelRay2Iterator::new(self.clone(), ray)
    }

    #[inline(always)]
    pub(crate) fn contains_point(&self, point: IVec2) -> bool {
        point.cmpge(IVec2::ZERO).all() && point.cmplt(self.size.into()).all()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Ray2 {
    pub origin: (f32, f32),
    pub direction: (f32, f32),
    pub length: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct Ray2hit {
    pub distance: f32,
    pub voxel: (i32, i32),
    pub normal: Option<(i32, i32)>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct VoxelRay2Iterator {
    volume: BoundingVolume2,
    max_d: f32,
    i: IVec2,
    step: IVec2,
    delta: Vec2,
    dist: Vec2,
    t_max: Vec2,
    t: f32,
    norm: Option<IVec2>,
    done: bool,
}

// Based on https://github.com/fenomas/fast-voxel-raycast/blob/master/index.js
impl VoxelRay2Iterator {
    pub fn new(volume: BoundingVolume2, ray: Ray2) -> Self {
        let mut p = Vec2::from(ray.origin);

        // Normalize direction vector
        let d = Vec2::from(ray.direction).normalize();

        // How long we have traveled thus far (modified by initial 'jump to volume bounds').
        let mut t = 0.0;

        // If the point it outside the chunk, AABB test to 'jump ahead'.
        if !volume.contains_point(p.floor().as_ivec2()) {
            // First AABB test the chunk bounds
            let aabb = test_aabb_of_chunk(volume, p, d, ray.length);

            // Chunk AABB test failed, no way we could intersect a voxel.
            if aabb.is_none() {
                return Self {
                    done: true,
                    ..Default::default()
                };
            }

            let aabb = aabb.unwrap();

            // Back the hit off at least 1 voxel
            p = aabb - d * 2.0;

            // Set t to the already traveled distance.
            t += (p - aabb).length() - 2.0;
        }

        // Max distance we can travel. This is either the ray length, or the current `t` plus the
        // corner to corner length of the voxel volume.
        let max_d = f32::min(
            ray.length,
            t + IVec2::from(volume.size).as_vec2().length() + 2.0,
        );

        // The starting voxel for the raycast.
        let i = p.floor().as_ivec2();

        // The directionVec we are stepping for each component.
        let step = d.signum().as_ivec2();

        // Just abs(Vec2::ONE / d) but acounts for zeros in the distance vector.
        let delta = (Vec2::new(
            if d.x.abs() < f32::EPSILON {
                f32::INFINITY
            } else {
                1.0 / d.x
            },
            if d.y.abs() < f32::EPSILON {
                f32::INFINITY
            } else {
                1.0 / d.y
            },
        ))
        .abs();

        let dist = Vec2::new(
            if step.x > 0 {
                i.x as f32 + 1.0 - p.x
            } else {
                p.x - i.x as f32
            },
            if step.y > 0 {
                i.y as f32 + 1.0 - p.y
            } else {
                p.y - i.y as f32
            },
        );

        // The nearest voxel boundary.
        let t_max = Vec2::new(
            if delta.x < f32::INFINITY {
                delta.x * dist.x
            } else {
                f32::INFINITY
            },
            if delta.y < f32::INFINITY {
                delta.y * dist.y
            } else {
                f32::INFINITY
            },
        );

        Self {
            volume,
            max_d,
            i,
            step,
            delta,
            dist,
            t_max,
            t,
            norm: None,
            done: false,
        }
    }
}

impl Iterator for VoxelRay2Iterator {
    type Item = Ray2hit;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }

        while self.t <= self.max_d {
            // Test if the current traverse is within the volume.
            let mut hit = None;
            if self.volume.contains_point(self.i) {
                hit = Some(Ray2hit {
                    distance: self.t,
                    voxel: self.i.into(),
                    normal: self.norm.map(|n| n.into()),
                });
            }

            // Select the smallest t_max
            if self.t_max.x < self.t_max.y {
                self.i.x += self.step.x;
                self.t = self.t_max.x;
                self.t_max.x += self.delta.x;
                self.norm = Some(IVec2::new(-self.step.x, 0));
            } else {
                self.i.y += self.step.y;
                self.t = self.t_max.y;
                self.t_max.y += self.delta.y;
                self.norm = Some(IVec2::new(0, -self.step.y));
            }

            if hit.is_some() {
                return hit;
            }
        }

        self.done = true;
        return None;
    }
}

fn test_aabb_of_chunk(
    volume: BoundingVolume2,
    from: Vec2,
    direction: Vec2,
    distance: f32,
) -> Option<Vec2> {
    let min = Vec2::ZERO;
    let max = IVec2::from(volume.size).as_vec2();
    let mut t = Vec2::ZERO;

    for i in 0..2 {
        if direction[i] > 0.0 {
            t[i] = (min[i] - from[i]) / direction[i];
        } else {
            t[i] = (max[i] - from[i]) / direction[i];
        }
    }

    let mi = if t[0] > t[1] { 0 } else { 1 };

    if t[mi] >= 0.0 && t[mi] <= distance {
        // The intersect point (distance along the ray).
        let pt = from + direction * t[mi];

        // The other value that need to be checked
        let o1 = (mi + 1) % 2;

        if pt[o1] >= min[o1] && pt[o1] <= max[o1] {
            return Some(pt);
        }
    }

    // AABB test failed.
    return None;
}

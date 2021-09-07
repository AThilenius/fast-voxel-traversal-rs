use glam::{IVec3, Vec3};

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct BoundingVolume3 {
    pub size: IVec3,
}

impl BoundingVolume3 {
    pub fn traverse_ray(&self, ray: Ray3) -> VoxelRay3Iterator {
        VoxelRay3Iterator::new(self.clone(), ray)
    }

    #[inline(always)]
    pub(crate) fn contains_point(&self, point: IVec3) -> bool {
        point.cmpge(IVec3::ZERO).all() && point.cmplt(self.size).all()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Ray3 {
    pub origin: Vec3,
    pub direction: Vec3,
    pub length: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct Ray3hit {
    pub distance: f32,
    pub voxel: IVec3,
    pub normal: Option<IVec3>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct VoxelRay3Iterator {
    volume: BoundingVolume3,
    max_d: f32,
    i: IVec3,
    step: IVec3,
    delta: Vec3,
    dist: Vec3,
    t_max: Vec3,
    t: f32,
    norm: Option<IVec3>,
    done: bool,
}

// Based on https://github.com/fenomas/fast-voxel-raycast/blob/master/index.js
impl VoxelRay3Iterator {
    pub fn new(volume: BoundingVolume3, ray: Ray3) -> Self {
        let mut p = ray.origin;

        // Normalize direction vector
        let d = ray.direction.normalize();

        // How long we have traveled thus far (modified by initial 'jump to volume bounds').
        let mut t = 0.0;

        // If the point it outside the chunk, AABB test to 'jump ahead'.
        if !volume.contains_point(p.floor().as_ivec3()) {
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
        let max_d = f32::min(ray.length, t + volume.size.as_vec3().length() + 2.0);

        // The starting voxel for the raycast.
        let i = p.floor().as_ivec3();

        // The directionVec we are stepping for each component.
        let step = d.signum().as_ivec3();

        // Just abs(Vec3::ONE / d) but acounts for zeros in the distance vector.
        let delta = (Vec3::new(
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
            if d.z.abs() < f32::EPSILON {
                f32::INFINITY
            } else {
                1.0 / d.z
            },
        ))
        .abs();

        let dist = Vec3::new(
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
            if step.z > 0 {
                i.z as f32 + 1.0 - p.z
            } else {
                p.z - i.z as f32
            },
        );

        // The nearest voxel boundary.
        let t_max = Vec3::new(
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
            if delta.z < f32::INFINITY {
                delta.z * dist.z
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

impl Iterator for VoxelRay3Iterator {
    type Item = Ray3hit;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }

        while self.t <= self.max_d {
            // Test if the current traverse is within the volume.
            let mut hit = None;
            if self.volume.contains_point(self.i) {
                hit = Some(Ray3hit {
                    distance: self.t,
                    voxel: self.i,
                    normal: self.norm,
                });
            }

            // Select the smallest t_max
            if self.t_max.x < self.t_max.y {
                if self.t_max.x < self.t_max.z {
                    self.i.x += self.step.x;
                    self.t = self.t_max.x;
                    self.t_max.x += self.delta.x;
                    self.norm = Some(IVec3::new(-self.step.x, 0, 0));
                } else {
                    self.i.z += self.step.z;
                    self.t = self.t_max.z;
                    self.t_max.z += self.delta.z;
                    self.norm = Some(IVec3::new(0, 0, -self.step.z));
                }
            } else {
                if self.t_max.y < self.t_max.z {
                    self.i.y += self.step.y;
                    self.t = self.t_max.y;
                    self.t_max.y += self.delta.y;
                    self.norm = Some(IVec3::new(0, -self.step.y, 0));
                } else {
                    self.i.z += self.step.z;
                    self.t = self.t_max.z;
                    self.t_max.z += self.delta.z;
                    self.norm = Some(IVec3::new(0, 0, -self.step.z));
                }
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
    volume: BoundingVolume3,
    from: Vec3,
    direction: Vec3,
    distance: f32,
) -> Option<Vec3> {
    let min = Vec3::ZERO;
    let max = volume.size.as_vec3();
    let mut t = Vec3::ZERO;

    for i in 0..3 {
        if direction[i] > 0.0 {
            t[i] = (min[i] - from[i]) / direction[i];
        } else {
            t[i] = (max[i] - from[i]) / direction[i];
        }
    }

    let mi = if t[0] > t[1] {
        if t[0] > t[2] {
            0
        } else {
            2
        }
    } else {
        if t[1] > t[2] {
            1
        } else {
            2
        }
    };

    if t[mi] >= 0.0 && t[mi] <= distance {
        // The intersect point (distance along the ray).
        let pt = from + direction * t[mi];

        // The other two value that need to be checked
        let o1 = (mi + 1) % 3;
        let o2 = (mi + 2) % 3;

        if pt[o1] >= min[o1] && pt[o1] <= max[o1] && pt[o2] >= min[o2] && pt[o2] <= max[o2] {
            return Some(pt);
        }
    }

    // AABB test failed.
    return None;
}

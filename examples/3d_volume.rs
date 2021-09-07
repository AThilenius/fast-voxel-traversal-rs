use fast_voxel_traversal::raycast_3d::*;

fn main() {
    // Create a bounding volume. Volmes always start at (0, 0, 0) with a given size. This doesn't
    // actually allocate a volume, it just stores the size. If you need to query a voxel volume that
    // doesn't start a (0, 0, 0), simply subtract the offset from the ray's origin, then add the
    // offset back in during each traversal iteration.
    let volume = BoundingVolume3 { size: (8, 8, 8) };

    // Create a ray that we will intersect with our voxels.
    let ray = Ray3 {
        origin: (-10.0, -10.0, -10.0),
        direction: (1.0, 1.0, 1.0),
        length: 100.0,
    };

    // Traverse the ray through the volume.
    for hit in volume.traverse_ray(ray) {
        // The position of the voxel that was traversed. This will always be a voxel within the
        // bounding volume.
        let _position = hit.voxel;

        // The normal of the face that was 'entered' when the ray traversed the voxel. This can be
        // `None` for the first voxel if the ray started within a voxel.
        let _normal = hit.normal;

        // The distance from the ray origin this hit occured.
        let _distance = hit.distance;

        println!("{:?}", hit);
    }

    let ray_doesnt_intersect = Ray3 {
        origin: (-1.0, -1.0, -1.0),
        direction: (1.0, 0.0, 0.0),
        length: f32::INFINITY,
    };

    // Casting an infinite ray, or one that never hits a volume won't infinite-loop (although it may
    // be slightly slower in such degerate cases).
    for _hit in volume.traverse_ray(ray_doesnt_intersect) {
        // No hits will be found here.
    }

    println!("Done!");
}

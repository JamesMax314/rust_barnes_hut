use crate::body::Body;
use crate::quadtree::Quadtree;

pub fn step(bodies: &mut [Body], theta: f64, dt: f64) {
    for body in bodies.iter_mut() {
        body.vel = body.vel + body.acc * (0.5 * dt);
    }

    for body in bodies.iter_mut() {
        body.pos = body.pos + body.vel * dt;
    }

    let tree = Quadtree::build(bodies);
    for i in 0..bodies.len() {
        let force = tree.compute_force(i, bodies, theta);
        bodies[i].acc = force * (1.0 / bodies[i].mass);
    }

    for body in bodies.iter_mut() {
        body.vel = body.vel + body.acc * (0.5 * dt);
    }
}

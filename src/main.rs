mod body;
mod vec2;
mod quadtree;
mod simulation;

use body::Body;
use quadtree::G;
use vec2::Vec2;
use macroquad::prelude::*;
use std::collections::VecDeque;

// A galaxy: one central mass plus `count` disk bodies in near-circular orbit
// around it, all offset by `centre`/`bulk_vel` so the whole galaxy can be
// placed and launched as a single unit (e.g. on a collision course).
fn galaxy(central_mass: f64, count: usize, r_min: f64, r_max: f64, centre: Vec2, bulk_vel: Vec2) -> Vec<Body> {
    let mut bodies = vec![Body::new(centre, bulk_vel, central_mass)];

    bodies.extend((0..count).map(|_| {
        // sample radius so area density is uniform across the disk, rather than
        // bunching bodies up near the centre.
        let u = rand::gen_range(0.0f32, 1.0f32) as f64;
        let radius = r_min + (r_max - r_min) * u.sqrt();
        let angle = rand::gen_range(0.0f32, std::f32::consts::TAU) as f64;

        let local_pos = Vec2::new(radius * angle.cos(), radius * angle.sin());

        // near-circular orbit, perpendicular to the radius vector, with a small
        // random speed wobble so orbits aren't perfectly circular (more interesting
        // long-term dynamics, slight clumping/precession).
        let base_speed = (G * central_mass / radius).sqrt();
        let speed = base_speed * rand::gen_range(0.9f32, 1.1f32) as f64;
        let local_vel = Vec2::new(-angle.sin() * speed, angle.cos() * speed);

        let mass = rand::gen_range(0.2f32, 1.5f32) as f64;
        Body::new(centre + local_pos, bulk_vel + local_vel, mass)
    }));

    bodies
}

// `count` equal-mass bodies at random positions within [-extent, extent] in
// both axes, each with an independent random velocity. No central mass, no
// orbital structure to start with — purely chaotic N-body motion.
fn random_cloud(count: usize, mass: f64, extent: f64, max_speed: f64) -> Vec<Body> {
    (0..count)
        .map(|_| {
            let x = rand::gen_range(-extent as f32, extent as f32) as f64;
            let y = rand::gen_range(-extent as f32, extent as f32) as f64;
            let vx = rand::gen_range(-max_speed as f32, max_speed as f32) as f64;
            let vy = rand::gen_range(-max_speed as f32, max_speed as f32) as f64;
            Body::new(Vec2::new(x, y), Vec2::new(vx, vy), mass)
        })
        .collect()
}

#[macroquad::main("rust_bh")]
async fn main() {
    // -- scenario: two colliding galaxies --
    // let central_mass = 2000.0;
    // // two galaxies on a converging path, offset vertically (impact parameter)
    // // so it's a glancing collision rather than a head-on merge.
    // let separation = 50.0;
    // let impact_parameter = 10.0;
    // let approach_speed = 5.0;
    //
    // let centre_a = Vec2::new(-separation / 2.0, -impact_parameter / 2.0);
    // let centre_b = Vec2::new(separation / 2.0, impact_parameter / 2.0);
    // let vel_a = Vec2::new(approach_speed, 0.0);
    // let vel_b = Vec2::new(-approach_speed, 0.0);
    //
    // let mut bodies = galaxy(central_mass, 120, 2.0, 15.0, centre_a, vel_a);
    // bodies.extend(galaxy(central_mass, 120, 2.0, 15.0, centre_b, vel_b));
    // let pixels_per_unit = 7.0;

    // -- scenario: n equal-mass bodies, random positions and velocities --
    let mut bodies = random_cloud(10000, 1.0, 20.0, 3.0);
    let pixels_per_unit = 4.0;

    let theta = 0.5;
    let dt = 0.01;

    let trail_len = 30;
    let trail_stride = 4; // only record a trail point every N frames
    let mut trails: Vec<VecDeque<Vec2>> = vec![VecDeque::new(); bodies.len()];
    let mut show_trails = true;

    let mut frame_count: u32 = 0;

    loop {
        // let step_start = std::time::Instant::now();
        simulation::step(&mut bodies, theta, dt);
        // let step_time = step_start.elapsed();

        if is_key_pressed(KeyCode::T) {
            show_trails = !show_trails;
        }

        if frame_count % trail_stride == 0 {
            for (i, body) in bodies.iter().enumerate() {
                trails[i].push_back(body.pos);
                if trails[i].len() > trail_len {
                    trails[i].pop_front();
                }
            }
        }

        clear_background(BLACK);

        let origin_x = screen_width() / 2.0;
        let origin_y = screen_height() / 2.0;
        let world_to_screen = |pos: Vec2| -> (f32, f32) {
            let x = origin_x + (pos.x as f32) * pixels_per_unit;
            let y = origin_y - (pos.y as f32) * pixels_per_unit; // flip y: screen y grows downward
            (x, y)
        };

        if show_trails {
            for trail in &trails {
                let len = trail.len();
                if len < 2 {
                    continue;
                }
                for i in 0..len - 1 {
                    let (x0, y0) = world_to_screen(trail[i]);
                    let (x1, y1) = world_to_screen(trail[i + 1]);
                    let alpha = (i + 1) as f32 / len as f32;
                    draw_line(x0, y0, x1, y1, 3.0, Color::new(1.0, 1.0, 1.0, alpha * 0.5));
                }
            }
        }

        for body in &bodies {
            let (x, y) = world_to_screen(body.pos);
            let radius = if body.mass > 10.0 { 6.0 } else { 3.0 };
            draw_circle(x, y, radius, WHITE);
        }

        // let draw_time = step_start.elapsed() - step_time;
        frame_count += 1;
        // if frame_count % 60 == 0 {
        //     println!("step: {:.2}ms  draw: {:.2}ms", step_time.as_secs_f64() * 1000.0, draw_time.as_secs_f64() * 1000.0);
        // }

        next_frame().await;
    }
}

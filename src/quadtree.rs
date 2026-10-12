use crate::vec2::Vec2;
use crate::body::Body;

mod debug;

const MAX_DEPTH: u32 = 30;
pub const G: f64 = 1.0;
const SOFTENING_SQ: f64 = 1.0;

#[derive(Clone, Copy)]
struct BBox {
    centre: Vec2,
    half_size: f64,
}

struct Node {
    bbox: BBox,
    mass: f64,
    centre_of_mass: Vec2,
    kind: NodeKind,
}

enum NodeKind {
    Empty,
    Leaf { bodies: Vec<usize> },
    Internal { children: [usize; 4] },
}

pub struct Quadtree {
    nodes: Vec<Node>,
}

impl Quadtree {
    fn compute_aggregates(&mut self, bodies: &[Body]) {
        for idx in (0..self.nodes.len()).rev() {
            match &self.nodes[idx].kind {
                NodeKind::Empty => {
                    self.nodes[idx].mass = 0.0;
                    self.nodes[idx].centre_of_mass = Vec2::zero();
                }
                NodeKind::Leaf { bodies: body_idxs } => {
                    let body_idxs = body_idxs.clone();
                    let mut mass = 0.0;
                    let mut weighted = Vec2::zero();
                    for &bi in &body_idxs {
                        mass += bodies[bi].mass;
                        weighted = weighted + bodies[bi].pos * bodies[bi].mass;
                    }
                    self.nodes[idx].mass = mass;
                    self.nodes[idx].centre_of_mass = weighted * (1.0 / mass);
                }
                NodeKind::Internal { children } => {
                    let children = *children;
                    let mut mass = 0.0;
                    let mut weighted = Vec2::zero();
                    for child_idx in children {
                        let child = &self.nodes[child_idx];
                        mass += child.mass;
                        weighted = weighted + child.centre_of_mass * child.mass;
                    }
                    self.nodes[idx].mass = mass;
                    self.nodes[idx].centre_of_mass = if mass > 0.0 {
                        weighted * (1.0 / mass)
                    } else {
                        Vec2::zero()
                    };
                }
            }
        }
    }
}

fn gravitational_force(diff: Vec2, mass: f64, source_mass: f64) -> Vec2 {
    let dist_sq = diff.x * diff.x + diff.y * diff.y + SOFTENING_SQ;
    let dist = dist_sq.sqrt();
    let force_mag = G * mass * source_mass / dist_sq;
    diff * (force_mag / dist)
}

impl Quadtree {
    pub fn compute_force(&self, body_idx: usize, bodies: &[Body], theta: f64) -> Vec2 {
        self.compute_force_from(0, body_idx, bodies, theta)
    }

    fn compute_force_from(&self, node_idx: usize, body_idx: usize, bodies: &[Body], theta: f64) -> Vec2 {
        let node = &self.nodes[node_idx];

        match &node.kind {
            NodeKind::Empty => Vec2::zero(),
            NodeKind::Leaf { bodies: leaf_bodies } => {
                let pos = bodies[body_idx].pos;
                let mass = bodies[body_idx].mass;
                let mut total = Vec2::zero();
                for &other_idx in leaf_bodies {
                    if other_idx == body_idx {
                        continue;
                    }
                    let diff = bodies[other_idx].pos - pos;
                    total = total + gravitational_force(diff, mass, bodies[other_idx].mass);
                }
                total
            }
            NodeKind::Internal { children } => {
                let pos = bodies[body_idx].pos;
                let mass = bodies[body_idx].mass;
                let diff = node.centre_of_mass - pos;
                let d_sq = diff.x * diff.x + diff.y * diff.y;
                let s = node.bbox.half_size * 2.0;

                // s/d < theta, compared without a sqrt: s/d < theta <=> s^2 < theta^2 * d^2
                if s * s < theta * theta * d_sq {
                    gravitational_force(diff, mass, node.mass)
                } else {
                    let children = *children;
                    let mut total = Vec2::zero();
                    for child_idx in children {
                        total = total + self.compute_force_from(child_idx, body_idx, bodies, theta);
                    }
                    total
                }
            }
        }
    }
}

impl Quadtree {
    fn compute_quadrant_bbox(bbox: &BBox, quadrant: i32) -> BBox {
        let centre = bbox.centre;
        let half_size = bbox.half_size;

        let n_half_size= half_size / 2.0;
        let sign_x: f64 = (quadrant%2 - (quadrant+1)%2) as f64;
        let sign_y: f64 = ((quadrant/2)%2 - ((quadrant/2)+1)%2) as f64;
        let n_centre = Vec2{x:centre.x+sign_x*n_half_size,
                                            y:centre.y+sign_y*n_half_size};

        BBox {
            centre: n_centre,
            half_size: n_half_size
        }
    }

    fn select_quadrant(pos: Vec2, bbox: &BBox) -> i32 {
        let qx = if pos.x >= bbox.centre.x { 1 } else { 0 };
        let qy = if pos.y >= bbox.centre.y { 1 } else { 0 };
        qy * 2 + qx
    }
}

impl Quadtree {
    fn insert(&mut self, node_idx: usize, body_idx: usize, bodies: &[Body], depth: u32) {
        match &self.nodes[node_idx].kind {
            NodeKind::Empty => {
                self.nodes[node_idx].kind = NodeKind::Leaf { bodies: vec![body_idx] };
            }
            NodeKind::Leaf { bodies: leaf_bodies } if depth >= MAX_DEPTH => {
                // past max depth: don't split further, just pile up in this leaf
                let mut leaf_bodies = leaf_bodies.clone();
                leaf_bodies.push(body_idx);
                self.nodes[node_idx].kind = NodeKind::Leaf { bodies: leaf_bodies };
            }
            NodeKind::Leaf { bodies: leaf_bodies } => {
                let existing_bodies = leaf_bodies.clone();
                let bbox = self.nodes[node_idx].bbox;

                let mut children = [0usize; 4];
                for q in 0..4 {
                    let child_bbox = Quadtree::compute_quadrant_bbox(&bbox, q);
                    children[q as usize] = self.nodes.len();
                    self.nodes.push(Node {
                        bbox: child_bbox,
                        mass: 0.0,
                        centre_of_mass: Vec2::zero(),
                        kind: NodeKind::Empty,
                    });
                }

                self.nodes[node_idx].kind = NodeKind::Internal { children };

                for existing_idx in existing_bodies {
                    self.insert(node_idx, existing_idx, bodies, depth + 1);
                }
                self.insert(node_idx, body_idx, bodies, depth + 1);
            }
            NodeKind::Internal { children } => {
                let bbox = self.nodes[node_idx].bbox;
                let pos = bodies[body_idx].pos;
                let q = Quadtree::select_quadrant(pos, &bbox) as usize;
                let child_idx = children[q];
                self.insert(child_idx, body_idx, bodies, depth + 1);
            }
        }
    }
}

impl Quadtree {
    pub fn build(bodies: &[Body]) -> Self {
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;

        for body in bodies {
            min_x = min_x.min(body.pos.x);
            max_x = max_x.max(body.pos.x);
            min_y = min_y.min(body.pos.y);
            max_y = max_y.max(body.pos.y);
        }

        let bbox = BBox {
            centre: Vec2::new(min_x + (max_x - min_x)/2.0, min_y + (max_y - min_y)/2.0),
            half_size: (max_x - min_x)/2.0,
        };

        let mut tree = Quadtree {
                nodes: vec![Node {
                bbox,
                mass: 0.0,
                centre_of_mass: Vec2::zero(),
                kind: NodeKind::Empty,
            }],
         };

        for i in 0..bodies.len() {
            tree.insert(0, i, bodies, 0);
        }

        tree.compute_aggregates(bodies);

        tree
    }
}
use crate::vec2::Vec2;

pub struct Body {
    pub pos: Vec2,
    pub vel: Vec2,
    pub mass: f64,
    pub acc: Vec2,
}

impl Body {
    pub fn new(pos: Vec2, vel: Vec2, mass: f64) -> Self {
        Body {
            pos,
            vel,
            mass,
            acc: Vec2::zero(),
        }
    }
}

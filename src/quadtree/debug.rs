use super::{NodeKind, Quadtree};
use crate::body::Body;

impl Quadtree {
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn print(&self) {
        self.print_node(0, 0);
    }

    fn print_node(&self, node_idx: usize, depth: usize) {
        let node = &self.nodes[node_idx];
        let indent = "  ".repeat(depth);

        match &node.kind {
            NodeKind::Empty => {
                println!("{indent}Empty  centre={:?} half_size={:.3}", node.bbox.centre, node.bbox.half_size);
            }
            NodeKind::Leaf { bodies } => {
                println!(
                    "{indent}Leaf bodies={:?}  centre={:?} half_size={:.3}  mass={:.3} com={:?}",
                    bodies, node.bbox.centre, node.bbox.half_size, node.mass, node.centre_of_mass
                );
            }
            NodeKind::Internal { children } => {
                println!(
                    "{indent}Internal  centre={:?} half_size={:.3}  mass={:.3} com={:?}",
                    node.bbox.centre, node.bbox.half_size, node.mass, node.centre_of_mass
                );
                for &child_idx in children {
                    self.print_node(child_idx, depth + 1);
                }
            }
        }
    }

    pub fn print_grid(&self, bodies: &[Body], width: usize, height: usize) {
        let root_bbox = self.nodes[0].bbox;
        let min_x = root_bbox.centre.x - root_bbox.half_size;
        let max_x = root_bbox.centre.x + root_bbox.half_size;
        let min_y = root_bbox.centre.y - root_bbox.half_size;
        let max_y = root_bbox.centre.y + root_bbox.half_size;

        let to_col = |x: f64| -> usize {
            let frac = ((x - min_x) / (max_x - min_x)) * (width - 1) as f64;
            frac.round().clamp(0.0, (width - 1) as f64) as usize
        };
        let to_row = |y: f64| -> usize {
            // flip y so larger y ends up nearer the top of the printed grid
            let frac = ((max_y - y) / (max_y - min_y)) * (height - 1) as f64;
            frac.round().clamp(0.0, (height - 1) as f64) as usize
        };

        let mut grid = vec![vec![' '; width]; height];

        self.draw_splits(0, &to_col, &to_row, &mut grid);

        for body in bodies {
            let col = to_col(body.pos.x);
            let row = to_row(body.pos.y);
            grid[row][col] = 'x';
        }

        for row in &grid {
            let line: String = row.iter().collect();
            println!("{line}");
        }
    }

    fn draw_splits(
        &self,
        node_idx: usize,
        to_col: &impl Fn(f64) -> usize,
        to_row: &impl Fn(f64) -> usize,
        grid: &mut Vec<Vec<char>>,
    ) {
        let node = &self.nodes[node_idx];

        if let NodeKind::Internal { children } = &node.kind {
            let bbox = node.bbox;
            let x0 = bbox.centre.x - bbox.half_size;
            let x1 = bbox.centre.x + bbox.half_size;
            let y0 = bbox.centre.y - bbox.half_size;
            let y1 = bbox.centre.y + bbox.half_size;

            let split_row = to_row(bbox.centre.y);
            for col in to_col(x0)..=to_col(x1) {
                grid[split_row][col] = '-';
            }

            let split_col = to_col(bbox.centre.x);
            for row in to_row(y1)..=to_row(y0) {
                grid[row][split_col] = '|';
            }

            for &child_idx in children {
                self.draw_splits(child_idx, to_col, to_row, grid);
            }
        }
    }
}

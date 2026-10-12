use crate::body::Body;
use rustfft::{num_complex::Complex32, FftPlanner};

// Bins bodies into a grid_size x grid_size mass density field covering
// [-extent, extent] in both axes, takes its 2D FFT, and radially averages
// the power |F(k)|^2 by |k| to produce a 1D power-vs-spatial-frequency curve.
// Returns `grid_size / 2` values, index 0 = lowest non-zero spatial frequency.
pub fn compute(bodies: &[Body], grid_size: usize, extent: f64) -> Vec<f32> {
    let mut grid = vec![Complex32::new(0.0, 0.0); grid_size * grid_size];

    for body in bodies {
        let u = (body.pos.x + extent) / (2.0 * extent);
        let v = (body.pos.y + extent) / (2.0 * extent);
        if !(0.0..1.0).contains(&u) || !(0.0..1.0).contains(&v) {
            continue;
        }
        let col = (u * grid_size as f64) as usize;
        let row = (v * grid_size as f64) as usize;
        grid[row * grid_size + col].re += body.mass as f32;
    }

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(grid_size);

    // FFT each row in place.
    for row in 0..grid_size {
        let start = row * grid_size;
        fft.process(&mut grid[start..start + grid_size]);
    }

    // FFT each column in place (gather, transform, scatter back).
    let mut column = vec![Complex32::new(0.0, 0.0); grid_size];
    for col in 0..grid_size {
        for row in 0..grid_size {
            column[row] = grid[row * grid_size + col];
        }
        fft.process(&mut column);
        for row in 0..grid_size {
            grid[row * grid_size + col] = column[row];
        }
    }

    // Radially bin power by |k|, where the signed frequency along each axis
    // is `idx` for idx < grid_size/2, or `idx - grid_size` otherwise.
    let max_k = grid_size / 2;
    let mut power = vec![0.0f32; max_k];
    let mut counts = vec![0u32; max_k];

    for row in 0..grid_size {
        let ky = if row < max_k { row as i32 } else { row as i32 - grid_size as i32 };
        for col in 0..grid_size {
            let kx = if col < max_k { col as i32 } else { col as i32 - grid_size as i32 };
            let k = ((kx * kx + ky * ky) as f32).sqrt().round() as usize;
            if k == 0 || k >= max_k {
                continue; // skip the DC term and anything outside our 1D range
            }
            let c = grid[row * grid_size + col];
            power[k] += c.re * c.re + c.im * c.im;
            counts[k] += 1;
        }
    }

    for k in 0..max_k {
        if counts[k] > 0 {
            power[k] /= counts[k] as f32;
        }
    }

    power
}

#![allow(non_snake_case)]

use failure::Error;
use ndarray::prelude::*;
use ord_subset::OrdSubsetIterExt;

const L: usize = 6;
const STRIDE: usize = 4;

pub trait Point: Sync + std::fmt::Debug {
    fn latitude(&self) -> f64;
    fn longitude(&self) -> f64;
    fn altitude(&self) -> i16;
}

fn haversine_distance<T: Point>(fix1: &T, fix2: &T) -> f64 {
    const R: f64 = 6371.; // kilometres

    let phi1 = fix1.latitude().to_radians();
    let phi2 = fix2.latitude().to_radians();
    let delta_phi = (fix2.latitude() - fix1.latitude()).to_radians();
    let delta_rho = (fix2.longitude() - fix1.longitude()).to_radians();

    let a = (delta_phi / 2.).sin() * (delta_phi / 2.).sin()
        + phi1.cos() * phi2.cos() * (delta_rho / 2.).sin() * (delta_rho / 2.).sin();

    let c = 2. * a.sqrt().atan2((1. - a).sqrt());

    R * c
}

trait CenterLatitude {
    fn center_lat(self: &Self) -> Option<f64>;
}

impl<T: Point> CenterLatitude for [T] {
    fn center_lat(self: &Self) -> Option<f64> {
        let lat_min = self.iter().map(|fix| fix.latitude()).ord_subset_min()?;
        let lat_max = self.iter().map(|fix| fix.latitude()).ord_subset_max()?;

        Some((lat_min + lat_max) / 2.)
    }
}

#[derive(Debug)]
pub struct OptimizationResult {
    pub indices: [usize; L + 1],
    pub distance: f64,
}

pub fn optimize<T: Point>(points: &[T]) -> Result<OptimizationResult, Error> {
    let indices = optimize_waypoints(points);

    let distance = (0..L)
        .map(|i| (indices[i], indices[i + 1]))
        .map(|(i1, i2)| (&points[i1], &points[i2]))
        .map(|(fix1, fix2)| haversine_distance(fix1, fix2))
        .sum();

    Ok(OptimizationResult { indices, distance })
}

fn optimize_waypoints<T: Point>(points: &[T]) -> [usize; L + 1] {
    let mut points_index = Vec::new();

    for i in (0..(points.len() - 1)).step_by(STRIDE) {
        points_index.push(i);
    }
    points_index.push(points.len() - 1);

    let n = points_index.len();
    dbg!(points_index.len());

    let mut distances = Array::from_elem((n, n), 0.0);

    for j in 1..n {
        for i in 0..j {
            distances[[i, j]] = haversine_distance(&points[points_index[i]], &points[points_index[j]]);
        }
    }

    let mut inadmissable_best_distance;
    let mut best = (0, 0, 0.0, vec![1]); //Vec::new());
    let mut dp = Array::from_elem((n, n, L + 1), (0.0, n));
    let mut dp_sub = Array::from_elem((n, n, L + 1), 0.0);

    for di in (1..n).rev() {
        dbg!(di);

        // fill in next dp
        let i0 = n - 1 - di;

        for j in ((i0 + 1)..n).rev() {
            dp[[i0, j, 1]] = (distances[[i0, j]], i0);
        }

        for l in 2..=L {
            for j in (i0 + 1)..n {
                for i in i0..j {
                    if dp[[i0, j, l]].0 < dp[[i0, i, l - 1]].0 + distances[[i, j]] {
                        dp[[i0, j, l]] = (dp[[i0, i, l - 1]].0 + distances[[i, j]], i);
                    }
                }
            }
        }

        for l in 1..=L {
            for j in (i0 + 1)..n {
                for i in i0..j {
                    if dp_sub[[i0, j, l]] < dp_sub[[i0, i, l]] {
                        dp_sub[[i0, j, l]] = dp_sub[[i0, i, l]];
                    }
                    if dp_sub[[i0, j, l]] < dp_sub[[i0, i, l - 1]] + distances[[i, j]] {
                        dp_sub[[i0, j, l]] = dp_sub[[i0, i, l - 1]] + distances[[i, j]];
                    }
                }
            }
        }

        inadmissable_best_distance = 0.0;

        for i0 in 0..(n - di) {
            let iL = i0 + di;

            if inadmissable_best_distance < dp_sub[[i0, iL, L]] {
                inadmissable_best_distance = dp_sub[[i0, iL, L]];
            }

            if points[points_index[i0]].altitude() > points[points_index[iL]].altitude() + 1000 {
                continue;
            }

            if best.2 < dp[[i0, iL, L]].0 {
                let path = {
                    let mut path = vec![iL];

                    for l in (1..=L).rev() {
                        let i = *path.last().unwrap();
                        path.push(dp[[i0, i, l]].1);

                        // We've reached the beginning of the path
                        if dp[[i0, i, l]].0 == 0.0 {
                            break;
                        }
                    }

                    path.reverse();
                    path
                };
                best = (i0, iL, dp[[i0, iL, L]].0, path);

                dbg!(&best);
            }
        }

        dbg!(inadmissable_best_distance);

        if inadmissable_best_distance < best.2 {
            println!("exiting early...");
            break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// Do it again /////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

/*
    let mut i = 0;
    let mut points_index = Vec::new();
    dbg!(&best);
    while i < points.len() {
        for &j in &best.3 {
            if (i as isize - j as isize).abs() as usize <= STRIDE {
                // points_index.push(i);
                break;
            }
        }
        i += 1;
    }
    let n = points_index.len();
    dbg!(points_index.len());
*/
    let old_points_index = points_index;

    let mut points_index = Vec::new();

    for i in 0..points.len() {
        for &j in &best.3 {
            if (i as isize - old_points_index[j] as isize).abs() as usize <= STRIDE {
                points_index.push(i);
                break;
            }
        }
    }

    let n = points_index.len();
    dbg!(points_index.len());

    let mut distances = Array::from_elem((n, n), 0.0);

    for j in 1..n {
        for i in 0..j {
            distances[[i, j]] = haversine_distance(&points[points_index[i]], &points[points_index[j]]);
        }
    }

    let mut inadmissable_best_distance;
    let mut best = (0, 0, 0.0, vec![1]); //Vec::new());
    let mut dp = Array::from_elem((n, n, L + 1), (0.0, n));
    let mut dp_sub = Array::from_elem((n, n, L + 1), 0.0);

    for di in (1..n).rev() {
        dbg!(di);

        // fill in next dp
        let i0 = n - 1 - di;

        for j in ((i0 + 1)..n).rev() {
            dp[[i0, j, 1]] = (distances[[i0, j]], i0);
        }

        for l in 2..=L {
            for j in (i0 + 1)..n {
                for i in i0..j {
                    if dp[[i0, j, l]].0 < dp[[i0, i, l - 1]].0 + distances[[i, j]] {
                        dp[[i0, j, l]] = (dp[[i0, i, l - 1]].0 + distances[[i, j]], i);
                    }
                }
            }
        }

        for l in 1..=L {
            for j in (i0 + 1)..n {
                for i in i0..j {
                    if dp_sub[[i0, j, l]] < dp_sub[[i0, i, l]] {
                        dp_sub[[i0, j, l]] = dp_sub[[i0, i, l]];
                    }
                    if dp_sub[[i0, j, l]] < dp_sub[[i0, i, l - 1]] + distances[[i, j]] {
                        dp_sub[[i0, j, l]] = dp_sub[[i0, i, l - 1]] + distances[[i, j]];
                    }
                }
            }
        }

        inadmissable_best_distance = 0.0;

        for i0 in 0..(n - di) {
            let iL = i0 + di;

            if inadmissable_best_distance < dp_sub[[i0, iL, L]] {
                inadmissable_best_distance = dp_sub[[i0, iL, L]];
            }

            if points[points_index[i0]].altitude() > points[points_index[iL]].altitude() + 1000 {
                continue;
            }

            if best.2 < dp[[i0, iL, L]].0 {
                let path = {
                    let mut path = vec![iL];

                    for l in (1..=L).rev() {
                        let i = *path.last().unwrap();
                        path.push(dp[[i0, i, l]].1);

                        // We've reached the beginning of the path
                        if dp[[i0, i, l]].0 == 0.0 {
                            break;
                        }
                    }

                    path.reverse();
                    path
                };
                best = (i0, iL, dp[[i0, iL, L]].0, path);

                dbg!(&best);
            }
        }

        dbg!(inadmissable_best_distance);

        if inadmissable_best_distance < best.2 {
            println!("exiting early...");
            break;
        }
    }

    // Finally we can return...

    return [
        points_index[best.3[0]],
        points_index[best.3[1]],
        points_index[best.3[2]],
        points_index[best.3[3]],
        points_index[best.3[4]],
        points_index[best.3[5]],
        points_index[best.3[6]]
    ];
}

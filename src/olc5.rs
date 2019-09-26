use failure::Error;
use ord_subset::OrdSubsetIterExt;

const LEGS: usize = 6;
const POINTS: usize = LEGS + 1;

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
    pub points: [usize; POINTS],
    pub distance: f64,
}

pub fn optimize<T: Point>(points: &[T]) -> Result<OptimizationResult, Error> {
    dbg!(points.len());

    let optimal_points = optimal_optimize_and_log_waypoints(points);

    let distance = (0..LEGS)
        .map(|i| (optimal_points[i], optimal_points[i + 1]))
        .map(|(i1, i2)| (&route[i1], &route[i2]))
        .map(|(fix1, fix2)| haversine_distance(fix1, fix2))
        .sum();

    Ok(OptimizationResult { optimal_points, distance })
}

fn optimize_and_log_waypoints<T: Point>(geo_points: &[T]) -> [usize; POINTS] {
    dbg!(num)
    println!("num: {}", num_points);

    //    let flat_points = to_flat_points(geo_points);
    // let distance_matrix = calculate_distance_matrix(&flat_points);


    //    const N: usize = num; // Number of points
    const K: usize = 6; // Maximum number of edges allowed on the path
    let (distance, indices) = optimize_waypoints(num_points, K, &geo_points, &distances);

    let a_index = indices[0];
    let b_index = indices[1];
    let c_index = indices[2];
    let d_index = indices[3];
    let e_index = indices[4];
    let f_index = indices[5];
    let g_index = indices[6];

    let a_geo_point = &geo_points[a_index];
    let b_geo_point = &geo_points[b_index];
    let c_geo_point = &geo_points[c_index];
    let d_geo_point = &geo_points[d_index];
    let e_geo_point = &geo_points[e_index];
    let f_geo_point = &geo_points[f_index];
    let g_geo_point = &geo_points[g_index];

    println!("{:?}", a_geo_point);
    println!("{:?}", b_geo_point);
    println!("{:?}", c_geo_point);
    println!("{:?}", d_geo_point);
    println!("{:?}", e_geo_point);
    println!("{:?}", f_geo_point);
    println!("{:?}", g_geo_point);

    [
        a_index, b_index, c_index, d_index, e_index, f_index, g_index,
    ]
}

fn optimize_waypoints<T: Point>(
    N: usize,
    K: usize,
    points: &[T]
) -> (f64, Vec<usize>) {

        let distances = gen_distances(&points);

    // N = number of points
    // K = number of edges allowed

    // return (0.0, vec![4, 1129, 1666, 4348, 6070, 6681, 7206])
    // dp[k][i] is a tuple containing information about the longest path using `k` edges and ending at point `i`.
    // dp[k][i].0 is the total length of the path.
    // dp[k][i].1 is the index of the previous point before point `i` (i.e. the predecessor) along the path.
    // This variable is often called "dp" for "dynamic programming", *shrug*.
    let mut dp = vec![vec![vec![(0.0, 0); N]; K]; N];

    let mut best_i_first = N;
    let mut best_i_last = N;
    let mut best_distance = 0.0;
    let mut best_path = Vec::new();

    for di in (K..N).rev() {
        /*
        println!("di = {}", di);
        // compute next dp
        {
            let i_first = N-1 - di;

            for k in 1..K {
                for j in (i_first+1)..N { // the point we're going to
                    for i in i_first..j { // the point we're coming from
                        let total_length = dp[i_first][k - 1][i].0 + distances[i][j];
                        if dp[i_first][k][j].0 < total_length {
                            dp[i_first][k][j] = (total_length, i);
                        }
                    }
                }
            }
        }
        */

        for i_first in 0..(N - di) {
            let i_last = i_first + di;

            if points[i_first].altitude() > points[i_last].altitude() + 1000 {
                continue;
            }

            println!("[{}, {}]  ", i_first, dp[i_first][K - 1][i_last].0);
            if best_distance < dp[i_first][K - 1][i_last].0 {
                best_i_first = i_first;
                best_i_last = i_last;
                best_distance = dp[i_first][K - 1][i_last].0;
                best_path = {
                    let mut path = vec![i_last];

                    for k in (1..K).rev() {
                        let i = *path.last().unwrap();
                        path.push(dp[i_first][k][i].1);

                        // We've reached the beginning of the path
                        if dp[i_first][k][i].0 == 0.0 {
                            break;
                        }
                    }

                    path.reverse();
                    path
                };

                println!(
                    "best (i_first, i_last) = ({}, {})",
                    best_i_first, best_i_last
                );
                println!("best distance = {}", best_distance);
                println!("best path = {:?}", best_path);
                println!("-------------------------------------");
            }
        }
    }

    return (best_distance, best_path);
}

fn gen_distances<T: Point>(points: &[T]) -> Vec<Vec<f64>> {
    let n = points.len();
    let mut distances = vec![vec![0.0; n]; n];

    for j in 1..n {
        for i in 0..j {
            let dist = haversine_distance(&points[i], &points[j]);
            distances[i][j] = dist;
        }
    }

    distances
}

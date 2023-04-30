use cimvr_common::{
    glam::{vec3, Vec2, Vec3},
    render::{Mesh, MeshHandle, Primitive, Render, UploadMesh, Vertex},
    Transform,
};
use cimvr_engine_interface::{dbg, make_app_state, pcg::Pcg, pkg_namespace, prelude::*, FrameTime};
use serde::{Deserialize, Serialize};
use std::f32::consts::TAU;

make_app_state!(ClientState, ServerState);

const GLASS_DISPERSION: Dispersion = Dispersion(Quadratic { a: 5.72e-8, b: -1.32e-4, c: 1.57 });
const WATER_DISPERSION: Dispersion = Dispersion(Quadratic { a: 8.64e-8, b: -1.37e-4, c: 1.38 });

struct ClientState;

const BOUNCE_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Bounces"));
const WALLS_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Walls"));

impl UserState for ClientState {
    fn new(io: &mut EngineIo, sched: &mut EngineSchedule<Self>) -> Self {
        io.create_entity()
            .add_component(Transform::default())
            .add_component(Render::new(BOUNCE_RDR).primitive(Primitive::Lines))
            .build();

        io.create_entity()
            .add_component(Transform::default())
            .add_component(Render::new(WALLS_RDR).primitive(Primitive::Lines))
            .build();

        sched
            .add_system(Self::update)
            .subscribe::<FrameTime>()
            .build();

        Self
    }
}

impl ClientState {
    pub fn update(&mut self, io: &mut EngineIo, _query: &mut QueryResult) {
        let Some(FrameTime { time, .. }) = io.inbox_first() else { return };

        let mut rng = Pcg::new();


        let scene = vec![
            (
                Line(Vec2::new(1., 10.), Vec2::new(1., -10.)),
                WallType::Prism(GLASS_DISPERSION),
            ),
            (
                Line(Vec2::new(2., 10.), Vec2::new(2., -10.)),
                WallType::Prism(GLASS_DISPERSION),
            ),
            (
                Line(Vec2::new(-1., -10.), Vec2::new(-1., 10.)),
                WallType::Mirror,
            ),
            //Line(Vec2::new(3., 1.), Vec2::new(2., 2.))
        ];

            let lines: Vec<Line> = scene.iter().map(|(line, _)| *line).collect();

            let mut walls_mesh = Mesh::new();
            lines_mesh(&mut walls_mesh, &lines, [1.; 3]);
            io.send(&UploadMesh {
                id: WALLS_RDR,
                mesh: walls_mesh,
            });

        const N_PATHS: usize = 10;

        let mut paths_mesh = Mesh::new();
        for i in 0..N_PATHS {
            let t = i as f32 / N_PATHS as f32; //rng.gen_f32();
            //let t = (time.sin() + 1.) / 2.;
            let wavelength = t * 400. + (1. - t) * 700.;

            let ray = Ray {
                origin: Vec2::ZERO,
                dir: Vec2::from_angle(TAU * time / 12.),
                wavelength,
            };

            let path = calc_path(ray, &scene, 1000);

            let color = wavelength_to_color(wavelength);

            path_mesh(&mut paths_mesh, &path, color);
        }

        io.send(&UploadMesh {
            id: BOUNCE_RDR,
            mesh: paths_mesh,
        });
    }
}

struct ServerState;

impl UserState for ServerState {
    fn new(_io: &mut EngineIo, _sched: &mut EngineSchedule<Self>) -> Self {
        Self
    }
}

/*
   fn wall_object_from_line(line: Line) -> (Transform, Wall) {
   let Line(p1, p2) = line;
   let length = (p2 - p1).length();
   let wall = Wall(length);

   let origin = (p2 + p1) / 2.;
   let dir = (p2 - origin).angle_between(Vec2::X);
   }

   fn wall_object_to_line(tf: Transform, wall: Wall) -> Line {
   let p1 = Vec3::X * wall.width / 2.;
   let p2 = Vec3::NEG_X * wall.width / 2.;

   let p1 = tf.to_homogeneous().transform_point3(p1);
   let p2 = tf.to_homogeneous().transform_point3(p2);

   Line(p1.xz(), p2.xz())
   }
*/

/*
#[derive(Component, Copy, Serialize, Deserialize, Clone, Debug)]
struct Wall {
    width: f32,
    wall_type: WallType,
}
*/

#[derive(Copy, Serialize, Deserialize, Clone, Debug)]
struct Quadratic {
    a: f32,
    b: f32,
    c: f32,
}

impl Quadratic {
    const fn new(a: f32, b: f32, c: f32) -> Self {
        Self { a, b, c }
    }

    fn get(&self, x: f32) -> f32 {
        self.a * x.powi(2) + self.b * x + self.c
    }
}

#[derive(Copy, Serialize, Deserialize, Clone, Debug)]
struct Dispersion(Quadratic);

#[derive(Copy, Serialize, Deserialize, Clone, Debug)]
enum WallType {
    Mirror,
    Prism(Dispersion),
}

/*
impl Default for Wall {
    fn default() -> Self {
        Wall {
            width: 0.,
            wall_type: WallType::Mirror,
        }
    }
}
*/

#[derive(Copy, Serialize, Deserialize, Clone, Debug)]
struct Line(Vec2, Vec2);

impl Line {
    pub fn position(&self, t: f32) -> Vec2 {
        let Line(p1, p2) = *self;
        p1 * (1. - t) + p2 * t
    }

    pub fn direction(&self) -> Vec2 {
        let Line(p1, p2) = *self;
        (p2 - p1).normalize()
    }

    pub fn normal(&self) -> Vec2 {
        self.direction().perp()
    }
}

#[derive(Copy, Clone, Debug)]
struct Ray {
    origin: Vec2,
    dir: Vec2,
    /// Wavelength in nanometers (nm)
    wavelength: f32,
}

impl Ray {
    pub fn position(&self, t: f32) -> Vec2 {
        self.origin + t * self.dir
    }
}

fn cross2d(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Returns the value `(ray_interp, line_interp)` for an intersection, if any
fn ray_line_intersect(ray: Ray, line: Line) -> Option<(f32, f32)> {
    // Use p1 as the reference in this coordinate system
    let Line(p1, p2) = line;
    let ray_origin = ray.origin - p1;
    let line_dir = p2 - p1;

    // Check for parallel lines
    let discriminant = cross2d(ray.dir, line_dir);
    if discriminant == 0. {
        return None;
    }

    // Calculate indices
    let t = cross2d(line_dir, ray_origin) / discriminant;
    let k = cross2d(ray.dir, ray_origin) / discriminant;

    // Check if we're inside the line segment, and if so return t
    ((0.0..=1.0).contains(&k) && t >= 0.).then(|| (t, k))
}

fn reflect(incident: Vec2, normal: Vec2) -> Vec2 {
    incident - 2. * incident.dot(normal) * normal
}

fn refract(incident: Vec2, normal: Vec2, eta: f32) -> Vec2 {
    // https://registry.khronos.org/OpenGL-Refpages/gl4/html/refract.xhtml
    let dot = incident.dot(normal);
    let k = 1.0 - eta.powi(2) * (1.0 - dot.powi(2));
    if k < 0.0 {
        Vec2::ZERO
    } else {
        eta * incident - (eta * dot + k.sqrt()) * normal
    }
}

fn calc_path(mut ray: Ray, scene: &[(Line, WallType)], max_bounces: usize) -> Vec<Vec2> {
    let mut points = vec![ray.origin];
    let lines: Vec<Line> = scene.iter().map(|(line, _)| *line).collect();

    for _ in 0..max_bounces {
        if let Some((line_idx, ray_interp, line_interp)) = intersect_scene(ray, &lines) {
            let (line, wall_type) = scene[line_idx];
            let end_pt = line.position(line_interp);
            points.push(end_pt);

            let normal = line.normal();
            let new_dir = match wall_type {
                WallType::Mirror => reflect(ray.dir, normal),
                WallType::Prism(Dispersion(quadratic)) => {
                    // Simulate dispersion
                    let mut eta = quadratic.get(ray.wavelength);

                    // Line is assumed to be a boundary of some solid in a vacuum;
                    // If we are a ray approaching the surface, we have dot(N, I) > 0.,
                    // and so we want to use the IOR from the dispersion. Otherwise
                    // we want to invert the IOR, so that we simulate the transition back
                    // to vacuum. TODO: Make this work for solids inside of other solids?
                    if normal.dot(ray.dir) < 0. {
                        eta = 1. / eta;
                    }

                    refract(ray.dir, normal, eta)
                }
            };

            ray = Ray {
                origin: end_pt,
                dir: new_dir,
                wavelength: ray.wavelength,
            };

            // Step ahead just a little bit to get out of the wall...
            ray.origin += ray.dir * 0.0001;
        } else {
            points.push(ray.position(100.));
            return points;
        }
    }

    points
}

/// Returns the index and interpolation of the wall which was hit
fn intersect_scene(ray: Ray, scene: &[Line]) -> Option<(usize, f32, f32)> {
    let mut closest = None;

    for (line_idx, line) in scene.iter().enumerate() {
        if let Some((ray_interp, line_interp)) = ray_line_intersect(ray, *line) {
            if let Some((_, prev_best, _)) = closest {
                if ray_interp > prev_best {
                    // Skip lines which are further away
                    continue;
                }
            }

            if line_interp > 0. {
                closest = Some((line_idx, ray_interp, line_interp));
            }
        }
    }

    closest
}

fn path_mesh(mesh: &mut Mesh, path: &[Vec2], color: [f32; 3]) {
    let base = mesh.vertices.len() as u32;

    mesh.vertices.extend(path
        .iter()
        .map(|v| Vertex::new([v.x, 0., v.y], color)));

    mesh.indices.extend((0..)
        .map(|i| (i + 1) / 2 + base)
        .take((path.len() * 2 - 1) * 2));
}

fn lines_mesh(mesh: &mut Mesh, lines: &[Line], color: [f32; 3]) {
    let base = mesh.vertices.len() as u32;

    mesh.vertices.extend(
        lines
            .iter()
            .map(|Line(p1, p2)| [p1, p2])
            .flatten()
            .map(|v| Vertex::new([v.x, 0., v.y], color)),
    );

    mesh.indices
        .extend((0..lines.len() as u32 * 2).map(|i| i + base));
}

/*
impl Default for Wall {
fn default() -> Self {
Self {
width: 1.,
}
}
}
*/

fn wavelength_to_color(lambda: f32) -> [f32; 3] {
    spectral_zucconi6(lambda).to_array()
}

// By Alan Zucconi
// Based on GPU Gems: https://developer.nvidia.com/sites/all/modules/custom/gpugems/books/GPUGems/gpugems_ch08.html
// But with values optimised to match as close as possible the visible spectrum
// Fits this: https://commons.wikimedia.org/wiki/File:Linear_visible_spectrum.svg
// With weighter MSE (RGB weights: 0.3, 0.59, 0.11)
fn bump3y(x: Vec3, yoffset: Vec3) -> Vec3 {
    let y = Vec3::ONE - x * x;
    (y - yoffset).clamp(Vec3::ZERO, Vec3::ONE)
}

fn spectral_zucconi6(w: f32) -> Vec3 {
    // w: [400, 700]
    // x: [0,   1]
    let x = ((w - 400.0) / 300.0).clamp(0., 1.);

    const C1: Vec3 = vec3(3.54585104, 2.93225262, 2.41593945);
    const X1: Vec3 = vec3(0.69549072, 0.49228336, 0.27699880);
    const Y1: Vec3 = vec3(0.02312639, 0.15225084, 0.52607955);

    const C2: Vec3 = vec3(3.90307140, 3.21182957, 3.96587128);
    const X2: Vec3 = vec3(0.11748627, 0.86755042, 0.66077860);
    const Y2: Vec3 = vec3(0.84897130, 0.88445281, 0.73949448);

    return bump3y(C1 * (x - X1), Y1) + bump3y(C2 * (x - X2), Y2);
}

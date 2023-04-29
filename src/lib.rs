use cimvr_common::{
    glam::{Vec2, Vec3, Vec3Swizzles},
    render::{Mesh, MeshHandle, Render, UploadMesh, Vertex, Primitive},
    Transform,
};
use cimvr_engine_interface::{make_app_state, pkg_namespace, prelude::*, println, dbg};
use serde::{Deserialize, Serialize};
use std::ops::Mul;

make_app_state!(ClientState, ServerState);

struct ClientState;

const BOUNCE_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Bounces"));
const WALLS_RDR: MeshHandle = MeshHandle::new(pkg_namespace!("Walls"));

impl UserState for ClientState {
    fn new(io: &mut EngineIo, _sched: &mut EngineSchedule<Self>) -> Self {
        let scene = vec![
            Line(Vec2::new(1., -1.), Vec2::new(1., 1.)),
            Line(Vec2::new(3., 1.), Vec2::new(2., 2.))
        ];

        io.send(&UploadMesh {
            id: WALLS_RDR,
            mesh: lines_mesh(&scene, [1.; 3]),
        });

        let ray = Ray {
            origin: Vec2::ZERO,
            dir: -Vec2::X,
        };

        let path = calc_path(ray, &scene, 10);
        dbg!(&path);

        io.send(&UploadMesh {
            id: BOUNCE_RDR,
            mesh: path_mesh(&path, [1.; 3]),
        });

        io.create_entity()
            .add_component(Transform::default())
            .add_component(Render::new(BOUNCE_RDR).primitive(Primitive::Lines))
            .build();

        io.create_entity()
            .add_component(Transform::default())
            .add_component(Render::new(WALLS_RDR).primitive(Primitive::Lines))
            .build();

        Self
    }
}

impl ClientState {
    pub fn update(&mut self, io: &mut EngineIo, query: QueryResult) {}
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

#[derive(Component, Copy, Serialize, Deserialize, Clone, Debug)]
struct Wall {
    width: f32,
}
*/

#[derive(Copy, Serialize, Deserialize, Clone, Debug)]
struct Line(Vec2, Vec2);

impl Line {
    pub fn position(&self, t: f32) -> Vec2 {
        let Line(p1, p2) = *self;
        p1 * (1. - t) + p2
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
}

impl Ray {
    pub fn position(&self, t: f32) -> Vec2 {
        self.origin + t * self.dir
    }
}

fn cross2d(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Returns the value `t` corresponding to the position along the ray one should travel to
/// intersect the given line, if any.
fn ray_line_intersect(ray: Ray, line: Line) -> Option<f32> {
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
    let t = cross2d(ray_origin, line_dir) / discriminant;
    let k = cross2d(ray_origin, ray.dir) / discriminant;

    // Check if we're inside the line segment, and if so return t
    ((0.0..=1.0).contains(&k) && t >= 0.).then(|| t)
}

fn reflect(incident: Vec2, normal: Vec2) -> Vec2 {
    incident - 2. * incident.dot(normal) * normal
}

fn calc_path(mut ray: Ray, scene: &[Line], max_bounces: usize) -> Vec<Vec2> {
    let mut points = vec![ray.origin];
    for _ in 0..max_bounces {
        if let Some((idx, interp)) = intersect_scene(ray, scene) {
            let line = scene[idx];
            let end_pt = line.position(interp);
            points.push(end_pt);

            // Mirror reflections
            let new_dir = reflect(ray.dir, line.direction());

            ray = Ray {
                origin: end_pt,
                dir: new_dir,
            };
        } else {
            return points;
        }
    }

    points
}

/// Returns the index and interpolation of the wall which was hit
fn intersect_scene(ray: Ray, scene: &[Line]) -> Option<(usize, f32)> {
    for (idx, line) in scene.iter().enumerate() {
        if let Some(t) = ray_line_intersect(ray, *line) {
            return Some((idx, t));
        }
    }

    None
}

fn path_mesh(path: &[Vec2], color: [f32; 3]) -> Mesh {
    let vertices: Vec<Vertex> = path
        .iter()
        .map(|v| Vertex::new([v.x, 0., v.y], color))
        .collect();
    let indices = (0..)
        .map(|i| (i + 1) / 2)
        .take((vertices.len() - 1) * 2)
        .collect();
    Mesh { vertices, indices }
}

fn lines_mesh(lines: &[Line], color: [f32; 3]) -> Mesh {
    let vertices = lines
        .iter()
        .map(|Line(p1, p2)| [p1, p2])
        .flatten()
        .map(|v| Vertex::new([v.x, 0., v.y], color))
        .collect();

    let indices = (0..lines.len() as u32 * 2).collect();
    Mesh { vertices, indices }
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

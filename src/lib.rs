#![cfg_attr(not(feature = "python"), allow(dead_code))]

use std::{collections::HashMap, fs::File, io::Write, path::PathBuf};

#[cfg(feature = "python")]
use std::{env, fs, process, time::SystemTime};

use csv::StringRecord;
use image::{ImageBuffer, Rgba};
use k::nalgebra::{
    point, vector, Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3,
};
use k::Chain;
use serde::Deserialize;
use thiserror::Error;
use urdf_rs::Robot;

#[cfg(feature = "python")]
use pyo3::{exceptions::PyValueError, prelude::*};

#[derive(Debug, Error)]
pub enum RoboMeshError {
    #[error("URDF load error: {0}")]
    UrdfLoad(String),
    #[error("Joint not found: {0}")]
    MissingJoint(String),
    #[error("Invalid data: {0}")]
    Invalid(String),
    #[error("CSV error: {0}")]
    Csv(String),
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
    #[error("Render error: {0}")]
    Render(String),
    #[error("Mesh load error: {0}")]
    Mesh(String),
}

#[cfg(feature = "python")]
impl From<RoboMeshError> for PyErr {
    fn from(value: RoboMeshError) -> Self {
        PyValueError::new_err(value.to_string())
    }
}

#[derive(Debug, Deserialize)]
pub struct JointFrame {
    pub positions: HashMap<String, f32>,
    #[serde(default)]
    pub time: Option<f32>,
}

/// A triangle mesh with indexed faces. Vertices are specified in URDF space
/// (right-handed, X forward, Y left, Z up).
#[derive(Clone, Debug, PartialEq)]
pub struct MeshData {
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<[u32; 3]>,
}

impl MeshData {
    /// Apply an isometric transform to every vertex and return a new mesh.
    pub fn transformed(&self, iso: &Isometry3<f32>) -> Self {
        let vertices = self
            .vertices
            .iter()
            .map(|v| {
                let p = Point3::new(v[0], v[1], v[2]);
                let t = iso.transform_point(&p);
                [t.x, t.y, t.z]
            })
            .collect();
        Self {
            vertices,
            indices: self.indices.clone(),
        }
    }

    /// Write the mesh as a minimal OBJ file containing only vertices and
    /// triangular faces. Faces are 1-indexed per the OBJ specification.
    pub fn write_obj(&self, path: &str) -> Result<(), RoboMeshError> {
        let mut file = File::create(path)
            .map_err(|e| RoboMeshError::Io(std::io::Error::new(e.kind(), e.to_string())))?;
        for v in &self.vertices {
            writeln!(file, "v {} {} {}", v[0], v[1], v[2])
                .map_err(|e| RoboMeshError::Io(std::io::Error::new(e.kind(), e.to_string())))?;
        }
        for tri in &self.indices {
            writeln!(file, "f {} {} {}", tri[0] + 1, tri[1] + 1, tri[2] + 1)
                .map_err(|e| RoboMeshError::Io(std::io::Error::new(e.kind(), e.to_string())))?;
        }
        Ok(())
    }

    /// Append another mesh, offsetting the face indices to remain valid.
    fn append(&mut self, other: &MeshData) {
        let offset = self.vertices.len() as u32;
        self.vertices.extend(other.vertices.iter().copied());
        self.indices.extend(
            other
                .indices
                .iter()
                .map(|[a, b, c]| [a + offset, b + offset, c + offset]),
        );
    }
}

/// Tessellation settings for primitive mesh generation. Higher segment counts
/// yield smoother geometry at the cost of more triangles.
#[derive(Clone, Debug)]
pub struct MeshTessellation {
    pub sphere_lat_segments: u32,
    pub sphere_lon_segments: u32,
    pub cylinder_radial_segments: u32,
    pub cylinder_height_segments: u32,
}

impl Default for MeshTessellation {
    fn default() -> Self {
        Self {
            sphere_lat_segments: 12,
            sphere_lon_segments: 24,
            cylinder_radial_segments: 32,
            cylinder_height_segments: 1,
        }
    }
}

#[cfg_attr(feature = "python", pyclass)]
pub struct RoboRenderer {
    chain: Chain<f32>,
    joint_names: Vec<String>,
    visuals: HashMap<String, Vec<VisualElement>>, // keyed by link name
}

#[cfg(feature = "python")]
#[pymethods]
impl RoboRenderer {
    #[new]
    pub fn new(urdf_path: &str) -> PyResult<Self> {
        let robot = urdf_rs::read_file(urdf_path)
            .map_err(|e| PyErr::from(RoboMeshError::UrdfLoad(e.to_string())))?;
        let chain =
            Chain::from_urdf_file(urdf_path).map_err(|e| RoboMeshError::UrdfLoad(e.to_string()))?;
        let visuals = load_visuals(&robot, Some(urdf_path))?;
        let joint_names = chain.iter_joints().map(|j| j.name.clone()).collect();
        Ok(Self {
            chain,
            joint_names,
            visuals,
        })
    }

    #[staticmethod]
    pub fn from_urdf_string(urdf: &str) -> PyResult<Self> {
        let robot = urdf_rs::read_from_string(urdf)
            .map_err(|e| PyErr::from(RoboMeshError::UrdfLoad(e.to_string())))?;
        let chain = chain_from_urdf_str(urdf)?;
        let visuals = load_visuals(&robot, None)?;
        let joint_names = chain.iter_joints().map(|j| j.name.clone()).collect();
        Ok(Self {
            chain,
            joint_names,
            visuals,
        })
    }

    /// Render a single frame into a PNG file.
    /// `joint_positions` can be a mapping or JSON string.
    pub fn render_frame(
        &mut self,
        joint_positions: &Bound<'_, PyAny>,
        output_path: &str,
    ) -> PyResult<()> {
        let map = parse_joint_map(joint_positions)?;
        apply_joint_map(&mut self.chain, &self.joint_names, &map)?;
        let skeleton = collect_points(&self.chain)?;
        let visuals = collect_visual_rects(&self.chain, &self.visuals)?;
        draw_scene(&skeleton, &visuals, output_path).map_err(PyErr::from)
    }

    /// Render a list of joint frames (list of dicts) into numbered PNG files.
    pub fn render_trajectory(
        &mut self,
        trajectory: &Bound<'_, PyAny>,
        output_dir: &str,
    ) -> PyResult<()> {
        let frames = parse_trajectory(trajectory)?;
        fs::create_dir_all(output_dir)?;
        for (idx, frame) in frames.iter().enumerate() {
            apply_joint_map(&mut self.chain, &self.joint_names, &frame.positions)?;
            let points = collect_points(&self.chain)?;
            let visuals = collect_visual_rects(&self.chain, &self.visuals)?;
            let out = format!("{}/frame_{:04}.png", output_dir, idx);
            draw_scene(&points, &visuals, &out).map_err(PyErr::from)?;
        }
        Ok(())
    }

    /// Render a trajectory provided as a CSV file with headers matching joint names.
    /// A column named "time" is optional and ignored for rendering.
    pub fn render_trajectory_csv(&mut self, csv_path: &str, output_dir: &str) -> PyResult<()> {
        let frames = load_csv_trajectory(csv_path, &self.joint_names)?;
        fs::create_dir_all(output_dir)?;
        for (idx, frame) in frames.iter().enumerate() {
            apply_joint_map(&mut self.chain, &self.joint_names, &frame.positions)?;
            let points = collect_points(&self.chain)?;
            let visuals = collect_visual_rects(&self.chain, &self.visuals)?;
            let out = format!("{}/frame_{:04}.png", output_dir, idx);
            draw_scene(&points, &visuals, &out).map_err(PyErr::from)?;
        }
        Ok(())
    }

    pub fn joint_order(&self) -> Vec<String> {
        self.joint_names.clone()
    }
}

#[cfg(feature = "python")]
fn parse_joint_map(obj: &Bound<'_, PyAny>) -> PyResult<HashMap<String, f32>> {
    if let Ok(map) = obj.extract::<HashMap<String, f32>>() {
        return Ok(map);
    }
    if let Ok(json) = obj.extract::<String>() {
        let frame: HashMap<String, f32> = serde_json::from_str(&json)
            .map_err(|e| PyValueError::new_err(format!("Failed to parse joint map: {e}")))?;
        return Ok(frame);
    }
    Err(PyValueError::new_err(
        "joint_positions must be a mapping or JSON string",
    ))
}

#[cfg(feature = "python")]
fn parse_trajectory(obj: &Bound<'_, PyAny>) -> PyResult<Vec<JointFrame>> {
    if let Ok(list) = obj.extract::<Vec<HashMap<String, f32>>>() {
        return Ok(list
            .into_iter()
            .map(|positions| JointFrame {
                positions,
                time: None,
            })
            .collect());
    }
    if let Ok(json) = obj.extract::<String>() {
        let frames: Vec<JointFrame> = serde_json::from_str(&json)
            .map_err(|e| PyValueError::new_err(format!("Failed to parse trajectory: {e}")))?;
        return Ok(frames);
    }
    Err(PyValueError::new_err(
        "trajectory must be a list of mappings or JSON string",
    ))
}

#[cfg(feature = "python")]
fn chain_from_urdf_str(urdf: &str) -> PyResult<Chain<f32>> {
    let mut path = env::temp_dir();
    let unique = SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| PyValueError::new_err(e.to_string()))?
        .as_nanos();
    path.push(format!("robomesh_{}_{}.urdf", process::id(), unique));
    fs::write(&path, urdf).map_err(|e| PyErr::from(RoboMeshError::UrdfLoad(e.to_string())))?;
    let chain = Chain::from_urdf_file(&path).map_err(|e| RoboMeshError::UrdfLoad(e.to_string()))?;
    let _ = fs::remove_file(&path);
    Ok(chain)
}

fn load_csv_trajectory(
    path: &str,
    expected_joints: &[String],
) -> Result<Vec<JointFrame>, RoboMeshError> {
    let mut reader = csv::Reader::from_path(path)
        .map_err(|e| RoboMeshError::Csv(format!("Failed to open CSV: {e}")))?;

    let headers = reader
        .headers()
        .map_err(|e| RoboMeshError::Csv(format!("Failed to read CSV headers: {e}")))?
        .clone();

    let missing_headers: Vec<String> = expected_joints
        .iter()
        .filter(|name| headers.iter().all(|h| h != *name))
        .cloned()
        .collect();
    if !missing_headers.is_empty() {
        return Err(RoboMeshError::MissingJoint(missing_headers.join(", ")));
    }

    let time_idx = headers.iter().position(|h| h == "time");
    let mut frames = Vec::new();
    for (idx, record) in reader.records().enumerate() {
        let record =
            record.map_err(|e| RoboMeshError::Csv(format!("Failed to read record {idx}: {e}")))?;

        let mut positions = HashMap::new();
        for name in expected_joints {
            let value = parse_f32_from_record(&record, name, &headers)?;
            positions.insert(name.clone(), value);
        }

        let time = if let Some(t_idx) = time_idx {
            record
                .get(t_idx)
                .and_then(|v| if v.trim().is_empty() { None } else { Some(v) })
                .map(|v| v.parse::<f32>())
                .transpose()
                .map_err(|e| {
                    RoboMeshError::Invalid(format!("Invalid time value in row {idx}: {e}"))
                })?
        } else {
            None
        };

        frames.push(JointFrame { positions, time });
    }

    Ok(frames)
}

fn parse_f32_from_record(
    record: &StringRecord,
    column_name: &str,
    headers: &StringRecord,
) -> Result<f32, RoboMeshError> {
    let idx = headers
        .iter()
        .position(|h| h == column_name)
        .ok_or_else(|| RoboMeshError::MissingJoint(column_name.to_string()))?;
    let val = record
        .get(idx)
        .ok_or_else(|| RoboMeshError::Invalid(format!("Missing value for {column_name}")))?;

    val.parse::<f32>().map_err(|e| {
        RoboMeshError::Invalid(format!(
            "Failed to parse float for column {column_name}: {e}"
        ))
    })
}

fn apply_joint_map(
    chain: &mut Chain<f32>,
    order: &[String],
    map: &HashMap<String, f32>,
) -> Result<(), RoboMeshError> {
    let mut values = Vec::with_capacity(order.len());
    for name in order {
        let val = *map
            .get(name)
            .ok_or_else(|| RoboMeshError::MissingJoint(name.clone()))?;
        values.push(val);
    }
    chain
        .set_joint_positions(&values)
        .map_err(|e| RoboMeshError::Invalid(e.to_string()))?;
    chain.update_transforms();
    Ok(())
}

fn collect_points(
    chain: &Chain<f32>,
) -> Result<Vec<(Point3<f32>, Option<Point3<f32>>)>, RoboMeshError> {
    let mut points = Vec::new();
    for link in chain.iter() {
        let world = link
            .world_transform()
            .ok_or_else(|| RoboMeshError::Render("missing transform".into()))?;
        let origin = world.translation.vector;
        let parent = link
            .parent()
            .and_then(|p| p.world_transform())
            .map(|t| t.translation.vector);
        let p3 = Point3::new(origin.x, origin.y, origin.z);
        let parent_point = parent.map(|v| Point3::new(v.x, v.y, v.z));
        points.push((p3, parent_point));
    }
    Ok(points)
}

fn draw_scene(
    points: &[(Point3<f32>, Option<Point3<f32>>)],
    visuals: &[VisualRect],
    output: &str,
) -> Result<(), RoboMeshError> {
    let (min, max) = bounding_square(points, visuals);
    let (width, height) = (800u32, 800u32);
    let center = Point2::new((max.x + min.x) / 2.0, (max.y + min.y) / 2.0);
    let world_half_range = ((max.x - min.x).abs().max((max.y - min.y).abs()) / 2.0).max(1.0);
    let scale = 0.9 * (width.min(height) as f32) / (2.0 * world_half_range);

    let mut canvas: ImageBuffer<Rgba<u8>, Vec<u8>> =
        ImageBuffer::from_pixel(width, height, Rgba([255, 255, 255, 255]));

    let mut draw_rect = |rect: &VisualRect| -> Result<(), RoboMeshError> {
        let p_min = world_to_pixel(rect.min, center, scale, (width, height));
        let p_max = world_to_pixel(rect.max, center, scale, (width, height));
        fill_rect(&mut canvas, p_min, p_max, Rgba([0, 255, 0, 102]));
        Ok(())
    };

    for rect in visuals {
        draw_rect(rect)?;
    }

    for (child, parent) in points {
        if let Some(parent) = parent {
            let c2 = Point2::new(child.x, child.z);
            let p2 = Point2::new(parent.x, parent.z);
            let c_pix = world_to_pixel(c2, center, scale, (width, height));
            let p_pix = world_to_pixel(p2, center, scale, (width, height));

            draw_line(&mut canvas, p_pix, c_pix, Rgba([0, 0, 255, 255]));
            draw_circle(&mut canvas, c_pix, 3, true, Rgba([255, 0, 0, 255]));
            draw_circle(&mut canvas, c_pix, 5, false, Rgba([255, 0, 0, 255]));
        }
    }

    canvas
        .save(output)
        .map_err(|e| RoboMeshError::Render(e.to_string()))
}

fn world_to_pixel(
    point: Point2<f32>,
    center: Point2<f32>,
    scale: f32,
    dims: (u32, u32),
) -> (i32, i32) {
    let x = (dims.0 as f32 / 2.0 + (point.x - center.x) * scale) as i32;
    let y = (dims.1 as f32 / 2.0 - (point.y - center.y) * scale) as i32;
    (x, y)
}

fn blend_pixel(canvas: &mut ImageBuffer<Rgba<u8>, Vec<u8>>, x: i32, y: i32, color: Rgba<u8>) {
    if x < 0 || y < 0 {
        return;
    }
    let (w, h) = canvas.dimensions();
    let (xu, yu) = (x as u32, y as u32);
    if xu >= w || yu >= h {
        return;
    }

    let dest = canvas.get_pixel_mut(xu, yu);
    let alpha = color[3] as f32 / 255.0;
    for i in 0..3 {
        dest[i] = ((color[i] as f32 * alpha) + (dest[i] as f32 * (1.0 - alpha)))
            .round()
            .clamp(0.0, 255.0) as u8;
    }
    dest[3] = 255;
}

fn fill_rect(
    canvas: &mut ImageBuffer<Rgba<u8>, Vec<u8>>,
    p_min: (i32, i32),
    p_max: (i32, i32),
    color: Rgba<u8>,
) {
    let x_start = p_min.0.min(p_max.0).max(0) as u32;
    let x_end = p_min.0.max(p_max.0).min(canvas.width() as i32 - 1) as u32;
    let y_start = p_min.1.min(p_max.1).max(0) as u32;
    let y_end = p_min.1.max(p_max.1).min(canvas.height() as i32 - 1) as u32;

    for y in y_start..=y_end {
        for x in x_start..=x_end {
            blend_pixel(canvas, x as i32, y as i32, color);
        }
    }
}

fn draw_line(
    canvas: &mut ImageBuffer<Rgba<u8>, Vec<u8>>,
    start: (i32, i32),
    end: (i32, i32),
    color: Rgba<u8>,
) {
    let (mut x0, mut y0) = start;
    let (x1, y1) = end;

    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    loop {
        blend_pixel(canvas, x0, y0, color);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn draw_circle(
    canvas: &mut ImageBuffer<Rgba<u8>, Vec<u8>>,
    center: (i32, i32),
    radius: i32,
    filled: bool,
    color: Rgba<u8>,
) {
    let r_sq = radius * radius;
    let band = radius.max(1);
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let dist_sq = dx * dx + dy * dy;
            let should_draw = if filled {
                dist_sq <= r_sq
            } else {
                dist_sq >= r_sq - band && dist_sq <= r_sq + band
            };
            if should_draw {
                blend_pixel(canvas, center.0 + dx, center.1 + dy, color);
            }
        }
    }
}

fn bounding_square(
    points: &[(Point3<f32>, Option<Point3<f32>>)],
    visuals: &[VisualRect],
) -> (Point2<f32>, Point2<f32>) {
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_z = f32::INFINITY;
    let mut max_z = f32::NEG_INFINITY;

    for (p, parent) in points.iter() {
        for point in [Some(p), parent.as_ref()].iter().flatten() {
            min_x = min_x.min(point.x);
            max_x = max_x.max(point.x);
            min_z = min_z.min(point.z);
            max_z = max_z.max(point.z);
        }
    }

    for rect in visuals {
        min_x = min_x.min(rect.min.x);
        max_x = max_x.max(rect.max.x);
        min_z = min_z.min(rect.min.y);
        max_z = max_z.max(rect.max.y);
    }

    if !min_x.is_finite() || !min_z.is_finite() {
        min_x = -1.0;
        max_x = 1.0;
        min_z = -1.0;
        max_z = 1.0;
    }

    let padding = 0.2 * ((max_x - min_x).abs().max((max_z - min_z).abs()) + 1.0);
    let center_x = (max_x + min_x) / 2.0;
    let center_z = (max_z + min_z) / 2.0;
    let half = padding / 2.0 + ((max_x - min_x).abs().max((max_z - min_z).abs()) / 2.0);

    (
        Point2::new(center_x - half, center_z - half),
        Point2::new(center_x + half, center_z + half),
    )
}

#[derive(Clone, Debug)]
struct VisualElement {
    transform: Isometry3<f32>,
    half_extents: Vector3<f32>,
}

#[derive(Clone, Debug)]
struct VisualRect {
    min: Point2<f32>,
    max: Point2<f32>,
}

/// Generate a triangle mesh for a URDF visual element. Primitive types are
/// tessellated procedurally; existing mesh references are not re-sampled.
/// The mesh is returned in world space if the visual contains an origin pose.
pub fn mesh_from_visual(
    visual: &urdf_rs::Visual,
    tessellation: Option<&MeshTessellation>,
) -> Result<MeshData, RoboMeshError> {
    let tess = tessellation.cloned().unwrap_or_default();
    let mesh = mesh_from_geometry(&visual.geometry, &tess)?;
    let world = pose_to_isometry(&visual.origin);
    Ok(mesh.transformed(&world))
}

/// Generate a triangle mesh from a URDF geometry primitive.
pub fn mesh_from_geometry(
    geometry: &urdf_rs::Geometry,
    tessellation: &MeshTessellation,
) -> Result<MeshData, RoboMeshError> {
    match geometry {
        urdf_rs::Geometry::Box { size } => Ok(generate_box_mesh(size)),
        urdf_rs::Geometry::Cylinder { radius, length } => {
            generate_cylinder_mesh(*radius as f32, *length as f32, tessellation)
        }
        urdf_rs::Geometry::Capsule { radius, length } => {
            generate_capsule_mesh(*radius as f32, *length as f32, tessellation)
        }
        urdf_rs::Geometry::Sphere { radius } => generate_sphere_mesh(*radius as f32, tessellation),
        urdf_rs::Geometry::Mesh { .. } => Err(RoboMeshError::Invalid(
            "Mesh geometry references existing mesh files; nothing to generate".into(),
        )),
    }
}

fn generate_box_mesh(size: &[f64; 3]) -> MeshData {
    let hx = (size[0] as f32) / 2.0;
    let hy = (size[1] as f32) / 2.0;
    let hz = (size[2] as f32) / 2.0;

    let vertices = vec![
        [hx, hy, hz],
        [hx, hy, -hz],
        [hx, -hy, hz],
        [hx, -hy, -hz],
        [-hx, hy, hz],
        [-hx, hy, -hz],
        [-hx, -hy, hz],
        [-hx, -hy, -hz],
    ];

    let indices = vec![
        [0, 2, 1],
        [1, 2, 3], // +X
        [4, 5, 6],
        [5, 7, 6], // -X
        [0, 1, 4],
        [1, 5, 4], // +Y
        [2, 6, 3],
        [3, 6, 7], // -Y
        [0, 4, 2],
        [2, 4, 6], // +Z
        [1, 3, 5],
        [3, 7, 5], // -Z
    ];

    MeshData { vertices, indices }
}

fn generate_cylinder_mesh(
    radius: f32,
    length: f32,
    tessellation: &MeshTessellation,
) -> Result<MeshData, RoboMeshError> {
    let radial = tessellation.cylinder_radial_segments.max(3);
    let height_segments = tessellation.cylinder_height_segments.max(1);
    let half = length / 2.0;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Rings along the Z axis.
    for h in 0..=height_segments {
        let z = -half + (length * h as f32) / (height_segments as f32);
        for i in 0..radial {
            let theta = 2.0 * std::f32::consts::PI * (i as f32) / (radial as f32);
            let (s, c) = theta.sin_cos();
            vertices.push([radius * c, radius * s, z]);
        }
    }

    // Side quads
    for h in 0..height_segments {
        let ring_start = h * radial;
        let next_ring = (h + 1) * radial;
        for i in 0..radial {
            let next = (i + 1) % radial;
            let i0 = ring_start + i;
            let i1 = ring_start + next;
            let i2 = next_ring + i;
            let i3 = next_ring + next;
            indices.push([i0, i1, i3]);
            indices.push([i0, i3, i2]);
        }
    }

    let bottom_center = vertices.len() as u32;
    vertices.push([0.0, 0.0, -half]);
    let top_center = vertices.len() as u32;
    vertices.push([0.0, 0.0, half]);

    // Caps
    for i in 0..radial {
        let next = (i + 1) % radial;
        // Bottom
        indices.push([bottom_center, (next) as u32, i as u32]);
        // Top
        let top_i = (height_segments * radial + i) as u32;
        let top_next = (height_segments * radial + next) as u32;
        indices.push([top_center, top_i, top_next]);
    }

    Ok(MeshData { vertices, indices })
}

fn generate_capsule_mesh(
    radius: f32,
    length: f32,
    tessellation: &MeshTessellation,
) -> Result<MeshData, RoboMeshError> {
    if radius <= 0.0 {
        return Err(RoboMeshError::Invalid(
            "Capsule radius must be positive".into(),
        ));
    }
    if length <= 0.0 {
        return Err(RoboMeshError::Invalid(
            "Capsule length must be positive".into(),
        ));
    }

    let cyl_length = length - 2.0 * radius;
    if cyl_length < 0.0 {
        return Err(RoboMeshError::Invalid(
            "Capsule length must be at least twice the radius".into(),
        ));
    }
    if cyl_length == 0.0 {
        return generate_sphere_mesh(radius, tessellation);
    }

    let mut mesh = generate_cylinder_mesh(radius, cyl_length, tessellation)?;
    let cap_mesh = generate_sphere_mesh(radius, tessellation)?;
    let cap_offset = cyl_length / 2.0 + radius;
    mesh.append(&cap_mesh.transformed(&Isometry3::translation(0.0, 0.0, -cap_offset)));
    mesh.append(&cap_mesh.transformed(&Isometry3::translation(0.0, 0.0, cap_offset)));

    Ok(mesh)
}

fn generate_sphere_mesh(
    radius: f32,
    tessellation: &MeshTessellation,
) -> Result<MeshData, RoboMeshError> {
    generate_spheroid_mesh([radius, radius, radius], tessellation)
}

/// Generate a tessellated ellipsoid (a stretched sphere) with independent X/Y/Z
/// radii.
pub fn generate_ellipsoid_mesh(
    radii: [f32; 3],
    tessellation: &MeshTessellation,
) -> Result<MeshData, RoboMeshError> {
    generate_spheroid_mesh(radii, tessellation)
}

fn generate_spheroid_mesh(
    radii: [f32; 3],
    tessellation: &MeshTessellation,
) -> Result<MeshData, RoboMeshError> {
    if radii.iter().any(|r| *r <= 0.0) {
        return Err(RoboMeshError::Invalid(
            "Ellipsoid radii must be positive".to_string(),
        ));
    }

    let lats = tessellation.sphere_lat_segments.max(3);
    let lons = tessellation.sphere_lon_segments.max(3);
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    let [rx, ry, rz] = radii;

    for lat in 0..=lats {
        let v = lat as f32 / lats as f32;
        let theta = std::f32::consts::PI * v;
        let sin_theta = theta.sin();
        let cos_theta = theta.cos();

        for lon in 0..=lons {
            let u = lon as f32 / lons as f32;
            let phi = 2.0 * std::f32::consts::PI * u;
            let (s, c) = phi.sin_cos();
            let x = rx * sin_theta * c;
            let y = ry * sin_theta * s;
            let z = rz * cos_theta;
            vertices.push([x, y, z]);
        }
    }

    let ring = lons + 1;
    for lat in 0..lats {
        for lon in 0..lons {
            let current = lat * ring + lon;
            let next = current + ring;

            if lat != 0 {
                indices.push([current, next, current + 1]);
            }
            if lat != lats - 1 {
                indices.push([current + 1, next, next + 1]);
            }
        }
    }

    Ok(MeshData { vertices, indices })
}

fn collect_visual_rects(
    chain: &Chain<f32>,
    visuals: &HashMap<String, Vec<VisualElement>>,
) -> Result<Vec<VisualRect>, RoboMeshError> {
    let mut rects = Vec::new();
    for link in chain.iter() {
        let world = link
            .world_transform()
            .ok_or_else(|| RoboMeshError::Render("missing transform".into()))?;
        let joint = link.joint();
        if let Some(link_visuals) = visuals.get(&joint.name) {
            for vis in link_visuals {
                let world_vis = world * vis.transform;
                rects.push(rect_from_box(&world_vis, vis.half_extents));
            }
        }
    }
    Ok(rects)
}

fn rect_from_box(transform: &Isometry3<f32>, half: Vector3<f32>) -> VisualRect {
    let corners = [
        vector![half.x, half.y, half.z],
        vector![half.x, half.y, -half.z],
        vector![half.x, -half.y, half.z],
        vector![half.x, -half.y, -half.z],
        vector![-half.x, half.y, half.z],
        vector![-half.x, half.y, -half.z],
        vector![-half.x, -half.y, half.z],
        vector![-half.x, -half.y, -half.z],
    ];

    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_z = f32::INFINITY;
    let mut max_z = f32::NEG_INFINITY;

    for corner in corners {
        let world = transform.transform_point(&point![corner.x, corner.y, corner.z]);
        min_x = min_x.min(world.x);
        max_x = max_x.max(world.x);
        min_z = min_z.min(world.z);
        max_z = max_z.max(world.z);
    }

    VisualRect {
        min: Point2::new(min_x, min_z),
        max: Point2::new(max_x, max_z),
    }
}

fn load_visuals(
    robot: &Robot,
    urdf_path: Option<&str>,
) -> Result<HashMap<String, Vec<VisualElement>>, RoboMeshError> {
    let base_dir = urdf_path.and_then(|p| PathBuf::from(p).parent().map(|p| p.to_path_buf()));
    let mut map: HashMap<String, Vec<VisualElement>> = HashMap::new();

    for link in &robot.links {
        let mut list = Vec::new();
        for visual in &link.visual {
            if let Some(vis) = visual_to_element(visual, base_dir.as_ref())? {
                list.push(vis);
            }
        }
        if !list.is_empty() {
            map.insert(link.name.clone(), list);
        }
    }

    Ok(map)
}

fn visual_to_element(
    visual: &urdf_rs::Visual,
    base_dir: Option<&PathBuf>,
) -> Result<Option<VisualElement>, RoboMeshError> {
    let pose = &visual.origin;
    let offset = pose_to_isometry(pose);

    let geometry = match &visual.geometry {
        urdf_rs::Geometry::Box { size } => VisualElement {
            transform: offset,
            half_extents: vector![
                size[0] as f32 / 2.0,
                size[1] as f32 / 2.0,
                size[2] as f32 / 2.0
            ],
        },
        urdf_rs::Geometry::Cylinder { radius, length } => VisualElement {
            transform: offset,
            half_extents: vector![*radius as f32, *length as f32 / 2.0, *radius as f32],
        },
        urdf_rs::Geometry::Capsule { radius, length } => VisualElement {
            transform: offset,
            half_extents: vector![
                *radius as f32,
                (*length as f32) / 2.0 + *radius as f32,
                *radius as f32,
            ],
        },
        urdf_rs::Geometry::Sphere { radius } => VisualElement {
            transform: offset,
            half_extents: vector![*radius as f32, *radius as f32, *radius as f32],
        },
        urdf_rs::Geometry::Mesh { filename, scale } => {
            let path = resolve_mesh_path(filename, base_dir);
            let (half_extents, center) = mesh_bounds(&path, scale)?;
            let transform = offset * Translation3::from(center);
            VisualElement {
                transform,
                half_extents,
            }
        }
        _ => return Ok(None),
    };

    Ok(Some(geometry))
}

fn pose_to_isometry(pose: &urdf_rs::Pose) -> Isometry3<f32> {
    let trans = Translation3::new(pose.xyz[0] as f32, pose.xyz[1] as f32, pose.xyz[2] as f32);
    let rot = UnitQuaternion::from_euler_angles(
        pose.rpy[0] as f32,
        pose.rpy[1] as f32,
        pose.rpy[2] as f32,
    );
    Isometry3::from_parts(trans, rot)
}

fn resolve_mesh_path(filename: &str, base_dir: Option<&PathBuf>) -> PathBuf {
    let path = PathBuf::from(filename);
    if path.is_absolute() {
        path
    } else if let Some(base) = base_dir {
        base.join(path)
    } else {
        path
    }
}

fn mesh_bounds(
    path: &PathBuf,
    scale: &Option<[f64; 3]>,
) -> Result<(Vector3<f32>, Vector3<f32>), RoboMeshError> {
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();

    let (min, max) = if ext == "stl" {
        bounds_from_stl(path)?
    } else {
        bounds_from_obj(path)?
    };

    let s = scale.unwrap_or([1.0, 1.0, 1.0]);
    let scale_vec = vector![s[0] as f32, s[1] as f32, s[2] as f32];
    let min_scaled = min.component_mul(&scale_vec);
    let max_scaled = max.component_mul(&scale_vec);
    let center = (min_scaled + max_scaled) / 2.0;
    let half = (max_scaled - min_scaled).map(|v| v.abs() / 2.0);
    Ok((half, center))
}

fn bounds_from_obj(path: &PathBuf) -> Result<(Vector3<f32>, Vector3<f32>), RoboMeshError> {
    let (models, _) = tobj::load_obj(path, &tobj::LoadOptions::default())
        .map_err(|e| RoboMeshError::Mesh(format!("Failed to load OBJ {}: {e}", path.display())))?;

    let mut min = vector![f32::INFINITY, f32::INFINITY, f32::INFINITY];
    let mut max = vector![f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY];

    for m in models {
        let mesh = m.mesh;
        for chunk in mesh.positions.chunks(3) {
            if let [x, y, z] = chunk {
                min.x = min.x.min(*x);
                min.y = min.y.min(*y);
                min.z = min.z.min(*z);
                max.x = max.x.max(*x);
                max.y = max.y.max(*y);
                max.z = max.z.max(*z);
            }
        }
    }

    if !min.x.is_finite() {
        return Err(RoboMeshError::Mesh(format!(
            "No vertices found in OBJ {}",
            path.display()
        )));
    }

    Ok((min, max))
}

fn bounds_from_stl(path: &PathBuf) -> Result<(Vector3<f32>, Vector3<f32>), RoboMeshError> {
    let mut file = File::open(path)
        .map_err(|e| RoboMeshError::Mesh(format!("Failed to load STL {}: {e}", path.display())))?;
    let stl = stl_io::read_stl(&mut file)
        .map_err(|e| RoboMeshError::Mesh(format!("Failed to load STL {}: {e}", path.display())))?;

    let mut min = vector![f32::INFINITY, f32::INFINITY, f32::INFINITY];
    let mut max = vector![f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY];

    for v in stl.vertices {
        min.x = min.x.min(v[0]);
        min.y = min.y.min(v[1]);
        min.z = min.z.min(v[2]);
        max.x = max.x.max(v[0]);
        max.y = max.y.max(v[1]);
        max.z = max.z.max(v[2]);
    }

    if !min.x.is_finite() {
        return Err(RoboMeshError::Mesh(format!(
            "No vertices found in STL {}",
            path.display()
        )));
    }

    Ok((min, max))
}

#[cfg(feature = "python")]
#[pymodule]
fn robomesh(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<RoboRenderer>()?;
    Ok(())
}

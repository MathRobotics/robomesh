#![cfg_attr(not(feature = "python"), allow(dead_code))]

use std::{collections::HashMap, fs::File, path::PathBuf};

#[cfg(feature = "python")]
use std::fs;

use csv::StringRecord;
use k::Chain;
use k::nalgebra::{
    point, vector, Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3,
};
use plotters::{prelude::*, series::LineSeries, series::PointSeries};
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
        let joint_names = chain.joints().iter().map(|j| j.name.clone()).collect();
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
        let chain = Chain::from_urdf_reader(urdf.as_bytes())
            .map_err(|e| RoboMeshError::UrdfLoad(e.to_string()))?;
        let visuals = load_visuals(&robot, None)?;
        let joint_names = chain.joints().iter().map(|j| j.name.clone()).collect();
        Ok(Self {
            chain,
            joint_names,
            visuals,
        })
    }

    /// Render a single frame into a PNG file.
    /// `joint_positions` can be a mapping or JSON string.
    pub fn render_frame(&mut self, joint_positions: &PyAny, output_path: &str) -> PyResult<()> {
        let map = parse_joint_map(joint_positions)?;
        apply_joint_map(&mut self.chain, &self.joint_names, &map)?;
        let skeleton = collect_points(&self.chain)?;
        let visuals = collect_visual_rects(&self.chain, &self.visuals)?;
        draw_scene(&skeleton, &visuals, output_path).map_err(PyErr::from)
    }

    /// Render a list of joint frames (list of dicts) into numbered PNG files.
    pub fn render_trajectory(&mut self, trajectory: &PyAny, output_dir: &str) -> PyResult<()> {
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
fn parse_joint_map(obj: &PyAny) -> PyResult<HashMap<String, f32>> {
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
fn parse_trajectory(obj: &PyAny) -> PyResult<Vec<JointFrame>> {
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
    let root_bounds = bounding_square(points, visuals);
    let backend = BitMapBackend::new(output, (800, 800)).into_drawing_area();
    backend
        .fill(&WHITE)
        .map_err(|e| RoboMeshError::Render(e.to_string()))?;
    let (min, max) = root_bounds;
    let mut chart = ChartBuilder::on(&backend)
        .margin(20)
        .set_left_and_bottom_label_area_size(10)
        .build_cartesian_2d(min.x..max.x, min.y..max.y)
        .map_err(|e| RoboMeshError::Render(e.to_string()))?;
    chart
        .configure_mesh()
        .disable_mesh()
        .draw()
        .map_err(|e| RoboMeshError::Render(e.to_string()))?;

    for rect in visuals {
        chart
            .draw_series(std::iter::once(Rectangle::new(
                [(rect.min.x, rect.min.y), (rect.max.x, rect.max.y)],
                ShapeStyle::from(&GREEN.mix(0.4)).filled(),
            )))
            .map_err(|e| RoboMeshError::Render(e.to_string()))?;
    }

    for (child, parent) in points {
        if let Some(parent) = parent {
            let c2 = Point2::new(child.x, child.z);
            let p2 = Point2::new(parent.x, parent.z);
            chart
                .draw_series(LineSeries::new(vec![(p2.x, p2.y), (c2.x, c2.y)], &BLUE))
                .map_err(|e| RoboMeshError::Render(e.to_string()))?;
            chart
                .draw_series(PointSeries::of_element(
                    vec![(c2.x, c2.y)],
                    3,
                    &RED,
                    &|c, s, st| {
                        return EmptyElement::at(c)
                            + Circle::new((0, 0), s, st.filled())
                            + Circle::new((0, 0), s + 2, st.stroke_width(1));
                    },
                ))
                .map_err(|e| RoboMeshError::Render(e.to_string()))?;
        }
    }

    backend
        .present()
        .map_err(|e| RoboMeshError::Render(e.to_string()))?;
    Ok(())
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
fn robomesh(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<RoboRenderer>()?;
    Ok(())
}

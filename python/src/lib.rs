use std::collections::HashMap;

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::{pyclass, pymethods, pymodule, Bound};

use ::robomesh::{JointFrame, RoboMeshError, RoboRenderer};

#[pyclass(name = "RoboRenderer")]
pub struct PyRoboRenderer {
    inner: RoboRenderer,
}

fn to_py_err(e: RoboMeshError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

#[pymethods]
impl PyRoboRenderer {
    #[new]
    pub fn new(urdf_path: &str) -> PyResult<Self> {
        let inner = RoboRenderer::from_urdf_path(urdf_path).map_err(to_py_err)?;
        Ok(Self { inner })
    }

    #[staticmethod]
    pub fn from_urdf_string(urdf: &str) -> PyResult<Self> {
        Ok(Self {
            inner: RoboRenderer::from_urdf_str(urdf).map_err(to_py_err)?,
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
        self.inner
            .render_frame_to_path(&map, output_path)
            .map_err(to_py_err)
    }

    /// Render a list of joint frames (list of dicts) into numbered PNG files.
    pub fn render_trajectory(
        &mut self,
        trajectory: &Bound<'_, PyAny>,
        output_dir: &str,
    ) -> PyResult<()> {
        let frames = parse_trajectory(trajectory)?;
        self.inner
            .render_frames_to_dir(&frames, output_dir)
            .map_err(to_py_err)
    }

    /// Render a trajectory provided as a CSV file with headers matching joint names.
    /// A column named "time" is optional and ignored for rendering.
    pub fn render_trajectory_csv(&mut self, csv_path: &str, output_dir: &str) -> PyResult<()> {
        self.inner
            .render_trajectory_csv(csv_path, output_dir)
            .map_err(to_py_err)
    }

    pub fn joint_order(&self) -> Vec<String> {
        self.inner.joint_order()
    }
}

fn parse_joint_map(obj: &Bound<'_, PyAny>) -> PyResult<HashMap<String, f32>> {
    if let Ok(map) = obj.extract::<HashMap<String, f32>>() {
        return Ok(map);
    }
    if let Ok(json) = obj.extract::<std::string::String>() {
        let frame: HashMap<String, f32> = serde_json::from_str(&json)
            .map_err(|e| PyValueError::new_err(format!("Failed to parse joint map: {e}")))?;
        return Ok(frame);
    }
    Err(PyValueError::new_err(
        "joint_positions must be a mapping or JSON string",
    ))
}

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
    if let Ok(json) = obj.extract::<std::string::String>() {
        let frames: Vec<JointFrame> = serde_json::from_str(&json)
            .map_err(|e| PyValueError::new_err(format!("Failed to parse trajectory: {e}")))?;
        return Ok(frames);
    }
    Err(PyValueError::new_err(
        "trajectory must be a list of mappings or JSON string",
    ))
}

#[pymodule]
pub fn robomesh(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRoboRenderer>()?;
    Ok(())
}

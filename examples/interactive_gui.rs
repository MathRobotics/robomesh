use std::{collections::HashMap, env, time::Duration};

use kiss3d::{
    camera::ArcBall,
    event::{Action, Key, WindowEvent},
    light::Light,
    nalgebra::{Point3, Translation3},
    window::Window,
};
use robomesh::{load_csv_trajectory, JointFrame, RoboRenderer};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!(
            "Usage: cargo run --example interactive_gui -- <path/to/model.urdf> [trajectory.csv]"
        );
        return Ok(());
    }

    let urdf_path = &args[1];
    let mut renderer = RoboRenderer::from_urdf_path(urdf_path)?;
    let joint_names = renderer.joint_order();

    let mut joint_values: HashMap<String, f32> =
        joint_names.iter().map(|name| (name.clone(), 0.0)).collect();

    let trajectory: Option<Vec<JointFrame>> = if args.len() > 2 {
        let frames = load_csv_trajectory(&args[2], &joint_names)?;
        Some(frames)
    } else {
        None
    };

    let mut window = Window::new("RoboMesh 3D Viewer");
    window.set_light(Light::StickToCamera);
    window.set_background_color(0.95, 0.95, 1.0);

    let eye = Point3::new(1.5, 1.5, 2.0);
    let at = Point3::origin();
    let mut camera = ArcBall::new(eye, at);

    let mut selected_joint = 0usize;
    let mut playing = false;
    let mut traj_index = 0usize;
    let mut last_step = std::time::Instant::now();

    println!("Loaded joints: {:?}", joint_names);
    println!("Mouse to orbit, scroll to zoom. Left/Right: change joint, Up/Down: adjust angle, Space: toggle trajectory playback.");

    while window.render_with_camera(&mut camera) {
        for event in window.events().iter() {
            if let WindowEvent::Key(key, action, _) = event.value {
                if action == Action::Press {
                    match key {
                        Key::Space => {
                            if trajectory.is_some() {
                                playing = !playing;
                                last_step = std::time::Instant::now();
                            }
                        }
                        Key::Left => {
                            selected_joint = selected_joint.saturating_sub(1);
                        }
                        Key::Right => {
                            selected_joint = (selected_joint + 1).min(joint_names.len() - 1);
                        }
                        Key::Up => {
                            let name = &joint_names[selected_joint];
                            *joint_values.entry(name.clone()).or_insert(0.0) += 0.05;
                        }
                        Key::Down => {
                            let name = &joint_names[selected_joint];
                            *joint_values.entry(name.clone()).or_insert(0.0) -= 0.05;
                        }
                        _ => {}
                    }
                }
            }
        }

        if let Some(frames) = trajectory.as_ref() {
            if playing && last_step.elapsed() >= Duration::from_millis(50) {
                joint_values = frames[traj_index].positions.clone();
                traj_index = (traj_index + 1) % frames.len();
                last_step = std::time::Instant::now();
            }
        }

        renderer.set_joint_positions(&joint_values)?;
        let skeleton = renderer.link_positions()?;

        let highlight_color = Point3::new(0.9, 0.2, 0.2);
        for (idx, (child, parent)) in skeleton.iter().enumerate() {
            if let Some(parent) = parent {
                window.draw_line(parent, child, &Point3::new(0.2, 0.4, 0.9));
            }
            let color = if idx == selected_joint {
                highlight_color
            } else {
                Point3::new(0.1, 0.5, 0.1)
            };
            window.draw_sphere(child, 0.02, &color);
        }

        // Small ground hint for spatial context
        for i in -5..=5 {
            let x = i as f32 * 0.1;
            window.draw_line(
                &Point3::new(x, 0.0, -0.5),
                &Point3::new(x, 0.0, 0.5),
                &Point3::new(0.8, 0.8, 0.8),
            );
            window.draw_line(
                &Point3::new(-0.5, 0.0, x),
                &Point3::new(0.5, 0.0, x),
                &Point3::new(0.8, 0.8, 0.8),
            );
        }

        // Show the active joint name in the corner
        let joint_label = format!(
            "Active joint: {} (± keys to adjust)",
            joint_names[selected_joint]
        );
        let translation = Translation3::new(-0.95, -0.9, 0.0);
        window.draw_text(
            &joint_label,
            &Point3::new(0.0, 0.0, 0.0),
            30.0,
            &Translation3::from(translation),
            &Point3::new(0.1, 0.1, 0.1),
        );
    }

    Ok(())
}

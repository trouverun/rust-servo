extern crate std;
use std::vec::Vec;
use std::string::String;
use plotly::{Plot, Scatter, Layout};
use plotly::common::Mode;
use plotly::layout::Axis;
use crate::{DoesFocMath, FocInput, FocInputType, FocResult};
use crate::sim::SimOutput;

pub struct DummyAccelerator;
impl DoesFocMath for DummyAccelerator {
    fn sin_cos(&mut self, angle_rad: f32) -> crate::SinCosResult {
        crate::SinCosResult {
            cos: angle_rad.cos(), sin: angle_rad.sin()
        }
    }

    fn sqrt(&mut self, val: f32) -> f32 {
        val.sqrt()
    }
}

pub struct SimRecord {
    pub input: FocInput,
    pub result: FocResult,
    pub sim: SimOutput,
}

pub fn plot_simulation(path: &str, dt: f32, records: &[SimRecord]) {
    let n = records.len();
    if n == 0 { return; }

    let time: Vec<f64> = (0..n).map(|i| i as f64 * dt as f64).collect();

    let mut plot = Plot::new();
    let xa_id = |r: u32| -> String {
        if r == 1 { "x".into() } else { std::format!("x{r}") }
    };
    let ya_id = |r: u32| -> String {
        if r == 1 { "y".into() } else { std::format!("y{r}") }
    };

    let line_trace = |time: &[f64], data: &[f32], name: &str, row: u32| {
        let xa = xa_id(row);
        let ya = ya_id(row);
        let y: Vec<f64> = data.iter().map(|&v| v as f64).collect();
        Scatter::new(time.to_vec(), y)
            .mode(Mode::Lines)
            .name(name)
            .x_axis(&xa)
            .y_axis(&ya)
    };

    // Collect series
    let theta: Vec<f32> = records.iter().map(|r| r.sim.theta).collect();
    let omega: Vec<f32> = records.iter().map(|r| r.sim.omega).collect();
    let i_u: Vec<f32> = records.iter().map(|r| r.sim.currents.u).collect();
    let i_v: Vec<f32> = records.iter().map(|r| r.sim.currents.v).collect();
    let i_w: Vec<f32> = records.iter().map(|r| r.sim.currents.w).collect();
    let d_u: Vec<f32> = records.iter().map(|r| r.result.duty_cycles.u).collect();
    let d_v: Vec<f32> = records.iter().map(|r| r.result.duty_cycles.v).collect();
    let d_w: Vec<f32> = records.iter().map(|r| r.result.duty_cycles.w).collect();

    let target_torque: Vec<f32> = records.iter().map(|r| {
        match r.input.command {
            FocInputType::TargetTorque(t) => t,
            _ => f32::NAN,
        }
    }).collect();
    let has_torque_target = target_torque.iter().any(|t| !t.is_nan());

    let mut row = 1u32;
    // 1. Rotor angle
    plot.add_trace(line_trace(&time, &theta, "θ", row));
    row += 1;
    // 2. Rotor speed
    plot.add_trace(line_trace(&time, &omega, "ω", row));
    row += 1;
    // 3. Phase currents
    plot.add_trace(line_trace(&time, &i_u, "I_u", row));
    plot.add_trace(line_trace(&time, &i_v, "I_v", row));
    plot.add_trace(line_trace(&time, &i_w, "I_w", row));
    row += 1;
    // 4. Duty cycles
    plot.add_trace(line_trace(&time, &d_u, "D_u", row));
    plot.add_trace(line_trace(&time, &d_v, "D_v", row));
    plot.add_trace(line_trace(&time, &d_w, "D_w", row));
    row += 1;
    // 5. Torque
    let torque: Vec<f32> = records.iter().map(|r| r.sim.torque).collect();
    plot.add_trace(line_trace(&time, &torque, "torque", row));
    if has_torque_target {
        plot.add_trace(line_trace(&time, &target_torque, "Target torque", row));
    }
    row += 1;

    let num_rows = row - 1;
    let gap = 0.05;
    let row_height = (1.0 - gap * (num_rows - 1) as f64) / num_rows as f64;

    // Returns [bottom, top] domain for row r (1-indexed, top-to-bottom)
    let domain_for_row = |r: u32| -> [f64; 2] {
        let top = 1.0 - (r - 1) as f64 * (row_height + gap);
        let bottom = top - row_height;
        [bottom, top]
    };

    let xa = |r: u32, anchor: &str| -> Axis {
        Axis::new().domain(&[0.0, 1.0]).anchor(anchor)
    };
    let ya = |r: u32, title: &str, anchor: &str| -> Axis {
        Axis::new().title(title).domain(&domain_for_row(r)).anchor(anchor)
    };

    let layout = Layout::new()
        .height(300 * num_rows as usize)
        .x_axis(xa(1, "y"))
        .x_axis2(xa(2, "y2"))
        .x_axis3(xa(3, "y3"))
        .x_axis4(xa(4, "y4"))
        .x_axis5(xa(5, "y5"))
        .y_axis(ya(1, "Rotor Angle [rad]", "x"))
        .y_axis2(ya(2, "Rotor Speed [rad/s]", "x2"))
        .y_axis3(ya(3, "Phase Currents [A]", "x3"))
        .y_axis4(ya(4, "Duty Cycles", "x4"))
        .y_axis5(ya(5, "Torque [Nm]", "x5"));

    plot.set_layout(layout);
    plot.write_html(path);
}

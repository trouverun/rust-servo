mod hall_calibration;
mod motor_estimation;
pub(crate) mod utils;
pub use hall_calibration::HallCalibrator;
pub use motor_estimation::{OfflineMotorEstimator, OfflineEstimatorCommand, OfflineEstimatorOutput, OfflineEstimatorConfig};
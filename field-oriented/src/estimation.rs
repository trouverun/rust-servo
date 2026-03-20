#[derive(Clone, Copy)]
pub struct MotorParams {
    pub pm_flux_linkage: f32,
}

pub trait MotorParamEstimator {
    fn update_estimate(&mut self);
    fn get_params(&self) -> MotorParams;
}

pub struct NominalParameters {
    pub params: MotorParams,
}

impl MotorParamEstimator for NominalParameters {
    fn update_estimate(&mut self) {}
    fn get_params(&self) -> MotorParams {
        self.params
    }
}
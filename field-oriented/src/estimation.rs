#[derive(Clone, Copy)]
pub struct MotorParams {
    pub num_pole_pairs: u8,
    pub stator_resistance: f32,
    pub d_inductance: f32,
    pub q_inductance: f32,
    pub pm_flux_linkage: f32,
}

pub struct FocIterationData {

}

pub trait MotorParamEstimator {
    fn initialize_params(&mut self, params: MotorParams);
    fn after_foc_iteration(&mut self, data: FocIterationData);
    fn get_params(&self) -> MotorParams;
}

// --------------------------------------------------------------------------------------------------------------------

pub struct ConstantMotorParameters {
    pub params: MotorParams,
}

impl ConstantMotorParameters {
    pub fn from_other<T>(&mut self, other: &T) where T: MotorParamEstimator {
        self.params = other.get_params();
    }
}

impl MotorParamEstimator for ConstantMotorParameters {
    fn initialize_params(&mut self, params: MotorParams) {
        self.params = params;
    }
    fn after_foc_iteration(&mut self, data: FocIterationData) {}
    fn get_params(&self) -> MotorParams {
        self.params
    }
}
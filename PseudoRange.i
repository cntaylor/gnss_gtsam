#include <PseudoRange.h>
virtual class PseudoRange : gtsam::NonlinearFactor {
  PseudoRangeFactor(size_t key, const double& prange_in,
                    const gtsam::Vector3& sat_loc_in, 
                    const gtsam::noiseModel::Base* model);
};

virtual class sc_PseudoRange : gtsam::NonlinearFactor {
  sc_PseudoRangeFactor(size_t pos_key, size_t sc_key, 
                      const double& prange_meas,
                      const gtsam::Vector3& sat_pos, 
                      const gtsam::noiseModel::Base* model);
};

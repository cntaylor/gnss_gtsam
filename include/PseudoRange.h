/**
 * @file    PseudoRange.h
 * @brief   pseudo range factor for raw GPS measurements (but already corrected and with known satellite in ECEF)
 * @author  Clark Taylor
 * @date    Jul 2024
 **/

#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gnss-gtsam {

/* In the future, should really make this more robust, but for now
just assume the values are a vector5, where the first 3 are the ECEF location
and the fourth is the clock error */

class GTSAM_EXPORT PseudoRange : public NoiseModelFactor1<Vector5> {
  private:
    typedef NoiseModelFactor1<Vector4> Base;
    Vector3 sat_pos_ = Vector3::Zero();
    double prange_meas_ = 0.;

  public:
    PseudoRange(Key key, const double& prange_meas,
                const Vector3& sat_pos, const SharedNoiseModel& model)
        : Base(model, key), sat_pos_(sat_pos), prange_meas_(prange_meas) {}
    
    Vector evaluateError(const Vector4& p, 
                         OptionalMatrixType H ) const override;


};

//Implement PseudoRange, but with a switchable constraint
class GTSAM_EXPORT sc_PseudoRange : public NoiseModelFactor2<Vector4, double> {
  private:
    typedef NoiseModelFactor2<Vector4,double> Base;
    Vector3 sat_pos_ = Vector3::Zero();
    double prange_meas_ = 0.;


  public:
    sc_PseudoRange(Key pos_key, Key sc_key, const double& prange_meas,
                      const Vector3& sat_pos, const SharedNoiseModel& model)
        : Base(model, pos_key, sc_key), sat_pos_(sat_pos), prange_meas_(prange_meas) {}
    
    Vector evaluateError(const Vector4& p, const double& sc, 
                         OptionalMatrixType H_p,
                         OptionalMatrixType H_sc ) const override;

};


}
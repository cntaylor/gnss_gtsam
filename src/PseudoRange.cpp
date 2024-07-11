/**
 * @file    PseudoRange.cpp
 * @brief   Implementation file for PseudoRange
 * @author  Clark Taylor
 * @date    Jul 2024
 **/

#include "PseudoRange.h"

using namespace std;

namespace gnss_gtsam {

    namespace prange{ 
        double time_divider=1E6;
        double c = 2.99792458E8; // m/s, speed of light
        double c_small = c/time_divider; // speed of light in smaller time units (to make solving more numerically stable)
        double sc_epsilon = 1E-5; // Used to make square root derivatives not blow up
    }

Vector PseudoRange::evaluateError(const Vector4& p, 
                                        OptionalMatrixType H) const {
    Vector3 diff_loc = p.head<3>() - sat_pos_;
    double prange_error = diff_loc.norm() + p[3]*prange::c_small - prange_meas_;
    auto normed_diff = diff_loc/diff_loc.norm();
    Vector1 err;
    err[0] = prange_error;

    if (H) {
        Matrix tmp(1, 5);
        for (int i=0; i<3; i++) 
            tmp(0,i) = normed_diff[i];
        tmp(0,3) = prange::c_small;
        tmp(0,4) = 0.0;
        // std::cout << "tmp created successfully" << std::endl;
        (*H) = tmp;
    }
    return err;
}

Vector sc_PseudoRange::evaluateError(const Vector4& p, const double& sc,
                                        OptionalMatrixType H_p,
                                        OptionalMatrixType H_sc) const {
    Vector3 diff_loc = p.head<3>() - sat_pos_;
    double uw_error = diff_loc.norm() + p[3]*prange::c_small - prange_meas_; // unweighted error
    auto normed_diff = diff_loc/diff_loc.norm();

    // I don't know if this is needed or not, but if the switch goes negative, that would
    // throw off the math, so just make sure it doesnt... (while getting the value)
    double orig_switch = std::fmax(0,sc);
    // I do sqrt because I want the switch to scale the squared error, not (possibly negative) error
    double local_switch = std::sqrt(orig_switch);
    double error = uw_error * local_switch;

    if (H_sc) {
        // Maybe I don't need to, but I am worried about the / orig_switch value (the correct value)
        // being numerically stable as the switch value approaches 0.  So, use the sc_epsilon here.
        Matrix tmp(1,1);
        tmp(0,0) = uw_error * 0.5/(local_switch + prange::sc_epsilon);
        (*H_sc) = tmp;
    }

    if (H_p) {
        Matrix tmp(1, 5);
        for (int i=0; i<3; i++) 
            tmp(0,i) = normed_diff[i];
        tmp(0,3) = prange::c_small;
        tmp(0,4) = 0.0;
        // std::cout << "tmp created successfully" << std::endl;
        (*H_p) = tmp * local_switch;
    }
    return (Vector1() << error).finished();
}


} //namespace gnss_gtsam
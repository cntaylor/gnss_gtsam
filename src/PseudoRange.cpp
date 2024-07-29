/**
 * @file    PseudoRange.cpp
 * @brief   Implementation file for PseudoRange
 * @author  Clark Taylor
 * @date    Jul 2024
 **/

#include "PseudoRange.h"

using namespace std;

namespace gnss_gtsam {

    double time_divider=1E6;
    double c = 2.99792458E8; // m/s, speed of light
    double c_small = c/time_divider; // speed of light in smaller time units (to make solving more numerically stable)
    double sc_epsilon = 1E-5; // Used to make square root derivatives not blow up
    double earth_rot_rate = 7.292115E-5; // rad/s, earth rotation rate

    Vector3 compute_diff_loc(Vector3 rec_pos, Vector3 sat_pos) const {
        /* Assume sat_pos was in the ECEF of the time it was transmitted, while it really 
            needs to be in ECEF at the time it was received. Make that correction*/

        double time_transmit = (rec_pos - sat_pos).norm() / prange::c;
        double rot_during_trans = prange::earth_rot_rate * time_transmit;
        
        ////// This is the full code, but angles are small enough that we use the approximation below..
        // // Earth rotates about Z axis.  and earth rotation vector is...
        // double srot = sin(rot_during_trans);
        // double crot = cos(rot_during_trans);
        // // Note the + and - location.  This is a coordinate frame rotation, not object rotation...
        // Vector3 new_sat_loc = Vector3(crot*sat_pos[0] + srot*sat_pos[1], srot*sat_pos[0] - crot*sat_pos[1], sat_pos[2]);
        Vector3 new_sat_loc = Vector3(sat_pos[0] + rot_during_trans*sat_pos[1], sat_pos[1] - rot_during_trans*sat_pos[0], sat_pos[2]);

        return rec_pos - new_sat_loc;
    }


    Vector PseudoRange::evaluateError(const Vector4& p, 
                                            OptionalMatrixType H) const {
        Vector3 diff_loc = compute_diff_loc(p.head<3>(), sat_pos_);

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
        Vector3 diff_loc = compute_diff_loc(p.head<3>(), sat_pos_);
        new_sat_loc = correctRot(diff_loc / c, sat_pos_);
        diff_loc = p.head<3>() - new_sat_loc;

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
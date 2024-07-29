/**
 * @file    PseudoRange.cpp
 * @brief   Implementation file for PseudoRange
 * @author  Clark Taylor
 * @date    Jul 2024
 **/

#include "PseudoRange.h"

using namespace std;

namespace gnss_gtsam {

    namespace prange {
        double time_divider=1E6;
        double c = 2.99792458E8; // m/s, speed of light
        double c_small = c/time_divider; // speed of light in smaller time units (to make solving more numerically stable)
        double sc_epsilon = 1E-5; // Used to make square root derivatives not blow up
        double earth_rot_rate = 7.292115E-5; // rad/s, earth rotation rate
    }

    /**
     * @brief Compute the difference in location between the receiver and the satellite 
     *        at the time it was received.
     * @param rec_pos Receiver position in ECEF coordinates.
     * @param sat_pos Satellite position in ECEF coordinates at the time it was transmitted.
     * @return Difference in location between the receiver and the satellite.
     * @note This function assumes that the satellite position was in the ECEF of the time 
     *       it was transmitted, while it really needs to be in ECEF at the time it was 
     *       received. It makes that correction.
     */
    Vector3 compute_diff_loc(Vector3 rec_pos, Vector3 sat_pos) {
        // Compute the time it took for the signal to travel from the satellite to the receiver
        double time_transmit = (rec_pos - sat_pos).norm() / prange::c;

        // Compute the amount of earth rotation during transmission
        double rot_during_trans = prange::earth_rot_rate * time_transmit;

        // Compute the corrected satellite position at the time it was received
        // Note: The commented code below is a full implementation using trigonometric functions.
        //       Since the angles are small, we use the approximation below to avoid numerical issues.
        // Vector3 new_sat_loc = Vector3(crot*sat_pos[0] + srot*sat_pos[1], srot*sat_pos[0] - crot*sat_pos[1], sat_pos[2]);
        Vector3 new_sat_loc = sat_pos + rot_during_trans * Vector3(sat_pos[1], -sat_pos[0], 0.0);

        // Compute the difference in location between the receiver and the corrected satellite position
        return rec_pos - new_sat_loc;
    }


    Vector PseudoRange::evaluateError(const Vector5& p, 
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

    Vector sc_PseudoRange::evaluateError(const Vector5& p, const double& sc,
                                            OptionalMatrixType H_p,
                                            OptionalMatrixType H_sc) const {
        Vector3 diff_loc = compute_diff_loc(p.head<3>(), sat_pos_);

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
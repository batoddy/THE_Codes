/*
 * yrt_simpleKalman.c
 *
 *  Created on: 9 Aug 2023
 *      Author: Batuhan
 */

#include "yrt_simpleKalman.h"

/*
e_mea: Measurement Uncertainty - How much do we expect to our measurement vary
e_est: Estimation Uncertainty - Can be initilized with the same value as e_mea
since the kalman filter will adjust its value.
q: Process Variance - usually a
small number between 0.001 and 1 - how fast your measurement moves. Recommended
0.01. Should be tunned to your needs.
*/

void yrt_simpleKalman_init(yrt_simpleKalman_t *kalman, float mea_e, float est_e,
                           float q) {
    kalman->err_measure = mea_e;
    kalman->err_estimate = est_e;
    kalman->q = q;

    kalman->current_estimate = 0;
    kalman->last_estimate = 0;
    kalman->kalman_gain = 0;
}
float yrt_simpleKalman_updateEstimate(yrt_simpleKalman_t *kalman, float mea) {
    kalman->kalman_gain =
        kalman->err_estimate / (kalman->err_estimate + kalman->err_measure);
    kalman->current_estimate =
        kalman->last_estimate +
        kalman->kalman_gain * (mea - kalman->last_estimate);
    kalman->err_estimate =
        (1.0 - kalman->kalman_gain) * kalman->err_estimate +
        fabs(kalman->last_estimate - kalman->current_estimate) * kalman->q;
    kalman->last_estimate = kalman->current_estimate;

    return kalman->current_estimate;
}
float yrt_simpleKalman_getEstimate(yrt_simpleKalman_t *kalman) {
    return kalman->current_estimate;
}


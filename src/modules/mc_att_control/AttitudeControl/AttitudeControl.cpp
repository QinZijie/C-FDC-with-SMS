/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}

	//start_time=0;
}

matrix::Vector3f AttitudeControl::update(const Quatf &q, int32_t inp_att_ref, float &rel_time,
		hrt_abstime &start_time) const
{
	Quatf qd = _attitude_setpoint_q;
	Eulerf att_ref(qd);


	//hrt_abstime rel_time;

	inp_att_ref=0;
	att_ref(1)=0.0f;

	if (inp_att_ref == 0)   {
		start_time = hrt_absolute_time();
		rel_time = 0.0f;

	} else if (inp_att_ref == 100 || inp_att_ref == 200 || inp_att_ref == 300 || inp_att_ref == 400) {
		att_ref = Vector3f(0.0f, 0.0f, att_ref(2));
		rel_time = (hrt_absolute_time() - start_time)* 1e-6f;
	}else if (inp_att_ref == 1 || inp_att_ref == 11 || inp_att_ref == 12 || inp_att_ref == 13 || inp_att_ref == 14) {
		att_ref = Vector3f(0.349065850398866f, 0.0f, att_ref(2));
		rel_time = (hrt_absolute_time() - start_time)* 1e-6f;
	} else if (inp_att_ref == 2 || inp_att_ref == 21 || inp_att_ref == 22 || inp_att_ref == 23 || inp_att_ref == 24) {
		rel_time = (hrt_absolute_time() - start_time)* 1e-6f;

		if (rel_time < 1) {
			att_ref = Vector3f(0.0f, 0.0f, att_ref(2));

		} else if (rel_time >= 1 &&  rel_time <= 9.4f) {
			float r_ref = 0.349065850398866f * sinf(1.5f * (rel_time - 1.0f));
			att_ref = Vector3f(r_ref, 0.0f, att_ref(2));

		} else {
			att_ref = Vector3f(0.0f, 0.0f, att_ref(2));
		}

	} else if (inp_att_ref == 3 || inp_att_ref == 31 || inp_att_ref == 32 || inp_att_ref == 33 || inp_att_ref == 34) {
		rel_time = (hrt_absolute_time() - start_time)* 1e-6f;

		if (rel_time  < 1) {
			att_ref = Vector3f(0.0f, 0.0f, att_ref(2));

		} else if (rel_time   >= 1 && rel_time <= 9.4f) {
			float p_ref = 0.349065850398866f * sinf(1.5f * (rel_time - 1.0f));
			att_ref = Vector3f(0.0f, p_ref, att_ref(2));

		} else {
			att_ref = Vector3f(0.0f, 0.0f, att_ref(2));
		}

	} else if (inp_att_ref == 4 || inp_att_ref == 41 || inp_att_ref == 42 || inp_att_ref == 43 || inp_att_ref == 44) {
		rel_time = (hrt_absolute_time() - start_time)* 1e-6f;

		if (rel_time  < 1) {
			att_ref = Vector3f(0.0f, 0.0f, att_ref(2));

		} else if (rel_time >= 1 && rel_time <= 9.4f) {
			float y_ref = 0.349065850398866f * sinf(1.5f * (rel_time - 1.0f));
			att_ref = Vector3f(0.0f, 0.0f, att_ref(2)+y_ref);

		} else {
			att_ref = Vector3f(0.0f, 0.0f, att_ref(2));
		}

	} else {
		start_time = hrt_absolute_time();
		rel_time = 0.0f;
		att_ref = Vector3f(0.0f, 0.0f, att_ref(2));
	}

	Eulerf _attitude_now(q);
	const Vector3f eq = att_ref - _attitude_now;
	matrix::Vector3f rate_setpoint = eq.emult(_proportional_gain);

	if (is_finite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}


	// // calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	// const Vector3f e_z = q.dcm_z();
	// const Vector3f e_z_d = qd.dcm_z();
	// Quatf qd_red(e_z, e_z_d);

	// if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
	// 	// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
	// 	// full attitude control anyways generates no yaw input and directly takes the combination of
	// 	// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
	// 	qd_red = qd;

	// } else {
	// 	// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
	// 	qd_red *= q;
	// }

	// // mix full and reduced desired attitude
	// Quatf q_mix = qd_red.inversed() * qd;
	// q_mix.canonicalize();
	// // catch numerical problems with the domain of acosf and asinf
	// q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	// q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	// qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

	// // quaternion attitude control law, qe is rotation from q to qd
	// const Quatf qe = q.inversed() * qd;

	// // using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// // also taking care of the antipodal unit quaternion ambiguity
	// const Vector3f eq = 2.f * qe.canonical().imag();

	// // calculate angular rates setpoint
	// matrix::Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.


	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}

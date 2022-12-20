/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();

	//const float R1NUM[6]={0.0f, 0.18950f, 0.0490f, -0.2340f,-0.18950f, 0.0f};
	R1 = Matrix<float, 3U, 2U> (R1NUM);
	R2 = Matrix<float, 3U, 2U> (R2NUM);
	R3 = Matrix<float, 3U, 2U> (R3NUM);
	R4 = Matrix<float, 3U, 2U> (R4NUM);

	rel_time=0.0f;
	inp_type=0;

}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		if (_landing_gear_sub.updated()) {
			landing_gear_s landing_gear;

			if (_landing_gear_sub.copy(&landing_gear)) {
				if (landing_gear.landing_gear != landing_gear_s::GEAR_KEEP) {
					if (landing_gear.landing_gear == landing_gear_s::GEAR_UP && (_landed || _maybe_landed)) {
						mavlink_log_critical(&_mavlink_log_pub, "Landed, unable to retract landing gear\t");
						events::send(events::ID("mc_rate_control_not_retract_landing_gear_landed"),
						{events::Log::Error, events::LogInternal::Info},
						"Landed, unable to retract landing gear");

					} else {
						_landing_gear = landing_gear.landing_gear;
					}
				}
			}
		}

		if (_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled) {
			// generate the rate setpoint from sticks
			manual_control_setpoint_s manual_control_setpoint;

			if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
				_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
				_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
				_thrust_sp = -v_rates_sp.thrust_body[2];
				rel_time=v_rates_sp.rel_time;
				inp_type=v_rates_sp.input_att_ref;
			}
		}


		manual_control_setpoint_s manual_control_setpoint;

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			//_tilt_sp = math::constrain(2.489793182146196f * manual_control_setpoint.aux1, -1.0f, 2.09f);
			_tilt_sp = math::constrain(0.5236f * manual_control_setpoint.x, -1.0f, 2.09f);

		}

                if (inp_type==100 || inp_type==11 || inp_type==21 ||inp_type==31 || inp_type==41)
		{
			_tilt_sp=0.0f;
		}else if (inp_type==200 || inp_type==12 || inp_type==22 ||inp_type==32 || inp_type==42)
		{
			_tilt_sp=  0.785398163397448f;
		}else if (inp_type==300 || inp_type==13 || inp_type==23 ||inp_type==33 || inp_type==43)
		{
			_tilt_sp= 1.570796326794897f;
		}else if (inp_type==400 || inp_type==14 || inp_type==24 ||inp_type==34 || inp_type==44)
		{
			if(rel_time<1 || rel_time>8)
			{
				_tilt_sp=0.0f;
			}else
			{
				_tilt_sp=0.785398163397448f*sin(1.5f*(rel_time-2))+0.785398163397448f;
			}
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
				s1_last = _tilt_sp;
				s2_last = _tilt_sp;
				s3_last = _tilt_sp;
				s4_last = _tilt_sp;
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed,_tilt_sp);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			Matrix<float, 4U, 8U> M;
			M.setAll(0.0f);
			M(3, 1) = 1.0f;
			M(3, 3) = 1.0f;
			M(3, 5) = 1.0f;
			M(3, 7) = 1.0f;

			float ctheta = cosf(_tilt_sp);
			float stheta = sinf(_tilt_sp);
			float rfb2num[4] = {ctheta, stheta, -stheta, ctheta};
			float rfbnum[2] = {-stheta, ctheta};
			float rfb3num[9] = {ctheta, 0.0f, stheta, 0.0f, 1.0f, 0.0f, -stheta, 0.0f, ctheta};
			Matrix<float, 2U, 2U> Rfb2(rfb2num);
			Matrix<float, 2U, 2U> Rbf2 = Rfb2.transpose();
			Matrix<float, 1U, 2U> Rfb(rfbnum);
			Matrix<float, 3U, 3U> Rfb3(rfb3num);
			Matrix<float, 3U, 2U> R1t, R2t, R3t, R4t;
			R1t = Rfb3 * R1 * Rbf2;
			R2t = Rfb3 * R2 * Rbf2;
			R3t = Rfb3 * R3 * Rbf2;
			R4t = Rfb3 * R4 * Rbf2;

			for (size_t i = 0; i < 3; i++) {
				for (size_t j = 0; j < 2; j++) {
					M(i, j) = R1t(i, j);
				}
			}

			for (size_t i = 0; i < 3; i++) {
				for (size_t j = 0; j < 2; j++) {
					M(i, j + 2) = R2t(i, j);
				}
			}

			for (size_t i = 0; i < 3; i++) {
				for (size_t j = 0; j < 2; j++) {
					M(i, j + 4) = R3t(i, j);
				}
			}

			for (size_t i = 0; i < 3; i++) {
				for (size_t j = 0; j < 2; j++) {
					M(i, j + 6) = R4t(i, j);
				}
			}

			Matrix<float, 8U, 4U> pinv_M;
			matrix::geninv(M, pinv_M);

			//float fx_sp = 6.0f * _thrust_sp * sinf(_tilt_sp);
			//float fz_sp = -6.0f * _thrust_sp * cosf(_tilt_sp);

			float fz_sp = -6.0f * _thrust_sp ;
			float fx_sp = - fz_sp * tanf(_tilt_sp);


			float rsp = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			rsp = 0.5f * rsp;
			float psp = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			psp = 0.5f * psp;
			float ysp = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			ysp = 0.5f * ysp;

			float LMN3[3] = {rsp, psp, ysp};
			Vector3f vecLMN(LMN3);
			vecLMN = Rfb3 * vecLMN;
			float LMNZ[4] = {vecLMN(0), vecLMN(1), vecLMN(2), -stheta *fx_sp + ctheta * fz_sp};
			Vector<float, 4U> TF(LMNZ);
			float fxz[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
			Vector<float, 8U> vecFxz(fxz);

			vecFxz = pinv_M * TF;

			// float fx1 =                 0.0f * rsp + (0.303022807104339f) * psp + (-1.492919465070017f) * ysp +    0.25f * fx_sp +
			// 			    0.011817889477069f * fz_sp;
			// float fz1 =   1.492919465070018f * rsp + (-1.205907089496858f) * psp +                  0.0f * ysp +
			// 	      (-0.0f) * fx_sp +    0.202969623509623f * fz_sp;
			// float fx2 =                 0.0f * rsp + (-0.303022807104339f) * psp +    1.307781694995372f * ysp +    0.25f * fx_sp
			// 			    + (-0.011817889477069f) * fz_sp;
			// float fz2 = (-1.307781694995372f) * rsp + (1.205907089496858f) * psp +                  0.0f * ysp +
			// 	    (-0.0f) * fx_sp +    0.297030376490378f * fz_sp;
			// float fx3 =                 0.0f * rsp + (0.303022807104339f) * psp +    1.492919465070017f * ysp +    0.25f * fx_sp +
			// 			    0.011817889477069f * fz_sp;
			// float fz3 = (-1.492919465070018f) * rsp + (-1.205907089496859f) * psp +                  0.0f * ysp +
			// 	    (-0.0f) * fx_sp +    0.202969623509622f * fz_sp;
			// float fx4 =                 0.0f * rsp + (-0.303022807104339f) * psp + (-1.307781694995371f) * ysp +    0.25f * fx_sp
			// 			    + (-0.011817889477069f) * fz_sp;
			// float fz4 =   1.307781694995372f * rsp + (1.205907089496858f) * psp + (-0.0f) * ysp +
			// 	      (-0.0f) * fx_sp +    0.297030376490378f * fz_sp;

			// for (size_t i = 0; i < 8; i++) {
			// 	for (size_t j = 0; j < 4; j++) {
			// 		fxz[i] += pinv_M(i, j) * LMNZ[j];
			// 	}
			// }

			// float fx1 =  0.0f* rsp + (0.303022807104339f) * psp + (-1.492919465070017f) * ysp +    0.25f * fx_sp +
			// 			    0.011817889477069f * fz_sp;
			// float fz1 =   1.492919465070018f * rsp + (-1.205907089496858f) * psp +                  0.0f * ysp +
			// 	      (-0.0f) * fx_sp +    0.202969623509623f * fz_sp;
			// float fx2 =                 0.0f * rsp + (-0.303022807104339f) * psp +    1.307781694995372f * ysp +    0.25f * fx_sp
			// 			    + (-0.011817889477069f) * fz_sp;
			// float fz2 = (-1.307781694995372f) * rsp + (1.205907089496858f) * psp +                  0.0f * ysp +
			// 	    (-0.0f) * fx_sp +    0.297030376490378f * fz_sp;
			// float fx3 =                 0.0f * rsp + (0.303022807104339f) * psp +    1.492919465070017f * ysp +    0.25f * fx_sp +
			// 			    0.011817889477069f * fz_sp;
			// float fz3 = (-1.492919465070018f) * rsp + (-1.205907089496859f) * psp +                  0.0f * ysp +
			// 	    (-0.0f) * fx_sp +    0.202969623509622f * fz_sp;
			// float fx4 =                 0.0f * rsp + (-0.303022807104339f) * psp + (-1.307781694995371f) * ysp +    0.25f * fx_sp
			// 			    + (-0.011817889477069f) * fz_sp;
			// float fz4 =   1.307781694995372f * rsp + (1.205907089496858f) * psp + (-0.0f) * ysp +
			// 	      (-0.0f) * fx_sp +    0.297030376490378f * fz_sp;


			//Vector<float,8U> vecFxz={fx1,fz1,fx2,fz2,fx3,fz3,fx4,fz4};

			float Fmax[4] = {1.2f, 1.8f, 1.2f, 1.8f};
			float ang_max = -0.5236f - _tilt_sp;
			float ang_min = 1.5708f - _tilt_sp;

			float dthetaMin = tan(ang_max > -0.5326f ? ang_max : -0.5326f);
			float dthetaMax = tan(ang_min < 0.5326f ? ang_min : 0.5326f);

			int itr = 0;
			int nCount = 0;
			bool sat = false;
			Matrix<float, 8U, 8U> W;
			W.setIdentity();

			Vector<float, 4> dTF;
			dTF.setZero();

			while (nCount < 8) {
			for (size_t i = 0; i < 8; i = i + 2) {
					int n = int(i / 2.0f);
					float absFz = fabs(fxz[i + 1]);

					if ((absFz) > Fmax[n]) {
						float lamda = Fmax[n] / sqrtf(fxz[i] * fxz[i] + fxz[i + 1] * fxz[i + 1]);
						vecFxz(i) = lamda * vecFxz(i);
						vecFxz(i+1)  = lamda * vecFxz(i+1);
						itr += 2;
						W(i, i) = 0;
						W(i + 1, i + 1) = 0;
						sat=true;

					} else {
						float Fxmin = absFz * dthetaMin;
						float Fxmax = absFz * dthetaMax;

						if (fxz[i] > Fxmax) {
							vecFxz(i) = Fxmax;
							itr = itr + 1;
							W(i, i) = 0;
							sat=true;

						} else if (fxz[i] < Fxmin) {
							vecFxz(i) = Fxmin;
							itr = itr + 1;
							W(i + 1, i + 1) = 0;
							sat=true;

						} else {
							break;
						}
					}
				}

				if(!sat)
				{
					break;
				}

				dTF = TF - M * vecFxz;

				if (fabs(dTF(0)) < 0.0001f &&  fabs(dTF(1)) < 0.0001f  && fabs(dTF(2)) < 0.0001f ) {
					break;
				}

				M = M * W;
				geninv(M, pinv_M);
				Vector<float, 8U> dF = pinv_M * dTF;
				vecFxz += dF;
				nCount++;
			}

			float fx1 = vecFxz(0); float fz1 = vecFxz(1); float fx2 = vecFxz(2); float fz2 = vecFxz(3);
			float fx3 = vecFxz(4); float fz3 = vecFxz(5); float fx4 = vecFxz(6); float fz4 = vecFxz(7);

			float f1 = (sqrtf(fx1 * fx1 + fz1 * fz1)) / 2.0f;
			float f2 = (sqrtf(fx2 * fx2 + fz2 * fz2)) / 3.0f;
			float f3 = (sqrtf(fx3 * fx3 + fz3 * fz3)) / 2.0f;
			float f4 = (sqrtf(fx4 * fx4 + fz4 * fz4)) / 3.0f;
			float s1;
			float s2;
			float s3;
			float s4;

			int armf = _v_control_mode.flag_armed ? 1 : 0;

			if (f1 < 0.05f) {
			s1 = _tilt_sp/2.489793182146196f;

		} else {
			s1 = atan2f(fx1, -fz1) + _tilt_sp;

				if (s1 > 1.5707f) {
					s1 = 1.5707f;
				}

				s1 = s1 / 2.489793182146196f;
			}


			if (f2 < 0.05f) {
			s2 = _tilt_sp/2.489793182146196f;

		} else {
			s2 = atan2f(fx2, -fz2) + _tilt_sp;

				if (s2 > 1.5707f) {
					s2 = 1.5707f;
				}

				s2 = s2 / 2.489793182146196f;
			}

			if (f3 < 0.05f) {
			s3 = _tilt_sp/2.489793182146196f;

		} else {
			s3 = atan2f(fx3, -fz3) + _tilt_sp;

				if (s3 > 1.5707f) {
					s3 = 1.5707f;
				}

				s3 = s3 / 2.489793182146196f;
			}

			if (f4 < 0.05f) {
			s4 = _tilt_sp/2.489793182146196f;

		} else {
			s4 = atan2f(fx4, -fz4) + _tilt_sp;

				if (s4 > 1.5707f) {
					s4 = 1.5707f;
				}

				s4 = s4 / 2.489793182146196f;
			}

			// publish actuator controls
			actuator_controls_s actuators{};
			actuator_controls_s actuators_servo{};

			actuators.control[4] = PX4_ISFINITE(f1) ? f1 : 0.0f;
			actuators.control[5] = PX4_ISFINITE(f2) ? f2 : 0.0f;
			actuators.control[6] = PX4_ISFINITE(f3) ? f3 : 0.0f;
			actuators.control[7] = PX4_ISFINITE(f4) ? f4 : 0.0f;

			actuators_servo.control[0] = PX4_ISFINITE(s1) ? s1 : 0.0f;
			actuators_servo.control[1] = PX4_ISFINITE(s2) ? s2 : 0.0f;
			actuators_servo.control[2] = PX4_ISFINITE(s3) ? s3 : 0.0f;
			actuators_servo.control[3] = PX4_ISFINITE(s4) ? s4 : 0.0f;
			actuators_servo.control[4] = armf;

			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			//actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;
			actuators_servo.timestamp_sample = angular_velocity.timestamp_sample;

			if (!_vehicle_status.is_vtol) {
			publishTorqueSetpoint(att_control, angular_velocity.timestamp_sample);
				publishThrustSetpoint(angular_velocity.timestamp_sample);
			}

			// scale effort by battery status if enabled
			// if (_param_mc_bat_scale_en.get()) {
			// 	if (_battery_status_sub.updated()) {
			// 		battery_status_s battery_status;

			// 		if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
			// 			_battery_status_scale = battery_status.scale;
			// 		}
			// 	}

			// 	if (_battery_status_scale > 0.0f) {
			// 		for (int i = 0; i < 4; i++) {
			// 			actuators.control[i] *= _battery_status_scale;
			// 		}
			// 	}
			// }

			actuators.timestamp = hrt_absolute_time();
			actuators_servo.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);
			_actuators_1_pub.publish(actuators_servo);

			s1_last = s1;
			s2_last = s2;
			s3_last = s3;
			s4_last = s4;

			updateActuatorControlsStatus(actuators, dt);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

void MulticopterRateControl::publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void MulticopterRateControl::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;
	v_thrust_sp.xyz[0] = 0.0f;
	v_thrust_sp.xyz[1] = 0.0f;
	v_thrust_sp.xyz[2] = PX4_ISFINITE(_thrust_sp) ? -_thrust_sp : 0.0f; // Z is Down

	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void MulticopterRateControl::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}

/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

#include <math.h>

#include "Arduino.h"

class SimpleKalmanFilter {
private:
	float _err_measure;
	float _err_estimate;
	float _q;
	float _current_estimate = 0;
	float _last_estimate = 0;
	float _kalman_gain = 0;
	bool firstMeasure = true;

public:
	SimpleKalmanFilter(float mea_e, float q) : _err_measure(mea_e), _err_estimate(mea_e), _q(q){};
	void setMeasurementError(float mea_e) { _err_measure = mea_e; };
	void setProcessNoise(float q) { _q = q; };
	void setEstimateError(float est_e) { _err_estimate = est_e; };
	float getMeasurementError() { return _err_measure; };
	float getProcessNoise() { return _q; };
	float getEstimateError() { return _err_estimate; };
	float getKalmanGain() { return _kalman_gain; };
	
	void reset() {
		firstMeasure = true;
		_err_estimate = _err_measure;
	}

	float updateEstimate(float mea) {
		if (firstMeasure) {
			firstMeasure = false;
			_last_estimate = mea;
		}
		_kalman_gain = _err_estimate / (_err_estimate + _err_measure);
		_current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
		_err_estimate = (1.0f - _kalman_gain) * _err_estimate + fabsf(_last_estimate - _current_estimate) * _q;
		_last_estimate = _current_estimate;

		return _current_estimate;
	};
};

#endif

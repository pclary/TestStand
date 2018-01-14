/*
 * StateEstimator.h
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#ifndef STATEESTIMATOR_H_
#define STATEESTIMATOR_H_

#include "StateEstimatorStructs.h"
#include "BodyStateEstimator.h"
#include "FloatingBaseEstimator.h"
#include "ContactStateEstimator.h"
#include "DynamicModel.h"
#include "DynamicState.h"

class StateEstimator {
public:
	StateEstimator();
	virtual ~StateEstimator();

	void Init(DynamicModel* dyn, cassie_out_t sensors);

	void Update(DynamicModel* dyn, DynamicState* dyn_state, cassie_out_t sensors);

	void GetStateEstimate(state_t* s) {*s = m_State; };

private:

	state_t m_State;
	measurement_t m_Measurement;

	BodyStateEstimator m_BodyEstimator;
	FloatingBaseEstimator m_FloatingBaseEstimator;
	ContactStateEstimator m_ContactStateEstimator;

	void CassieOutToState(DynamicModel* dyn, cassie_out_t sensors, state_t* m_State);

	void CassieOutToMeasurement(cassie_out_t sensors, measurement_t* m);

	//should really just be used when initializing the state
	void MeasurementToState(DynamicModel* dyn, measurement_t m, state_t* s);

};

#endif /* STATEESTIMATOR_H_ */

/*
 * ContactStateEstimator.h
 *
 *  Created on: Jan 10, 2018
 *      Author: tapgar
 */

#ifndef CONTACTSTATEESTIMATOR_H_
#define CONTACTSTATEESTIMATOR_H_

class ContactStateEstimator {
public:
	ContactStateEstimator();
	virtual ~ContactStateEstimator();

	void Update(state_t* s, measurement_t z);
};

#endif /* CONTACTSTATEESTIMATOR_H_ */

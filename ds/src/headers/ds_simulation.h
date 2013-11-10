/*
 * ds_simulation.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef DS_SIMULATION_H_
#define DS_SIMULATION_H_

#include<set>
//#include "ds_lg.h"

namespace ds_simulation {

//using ds_lg::lg_v64;
//using ds_lg::int64;

enum Value {
	BIT_0 = 0,
	BIT_1,
	BIT_X,
	BIT_UD
};

//class ErrorObserver : public ds_lg::Monitor {
//protected:
//	lg_v64 spec;
//	bool *error;
//public:
//	ErrorObserver(bool *e):error(e){}
//	virtual void observe(const lg_v64& v) {
//		int64 = (~spec.x & ~v.x & ~spev.v & ~v.v) | (~spec.x & ~v.x & spev.v & v.v);
//	}
//	void set_spec(const lg_v64& s){
//		spec = s;
//	}
//};
}


#endif /* DS_SIMULATION_H_ */

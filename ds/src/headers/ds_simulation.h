/*
 * ds_simulation.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef DS_SIMULATION_H_
#define DS_SIMULATION_H_

#include <set>
#include "ds_lg.h"
#include "ds_faults.h"
#include <boost/log/trivial.hpp>

namespace ds_simulation {

class ErrorObserver : public ds_lg::Monitor {
protected:
	ds_lg::lg_v64 spec;
	ds_common::int64 *ds;
	ds_common::int64 *np;
public:
	ErrorObserver(ds_common::int64* d, ds_common::int64* n):ds(d), np(n){}
	virtual void observe(const ds_lg::lg_v64& v) {
		ds_common::int64 detected = (~spec.x & ~v.x & spec.v & ~v.v) | (~spec.x & ~v.x & ~spec.v & v.v);
		*ds |= detected;

		ds_common::int64 possibly_detected = ~spec.x & v.x;
		*np |= (possibly_detected & ~detected);
	}

	void set_spec(const ds_lg::lg_v64& s){
		spec.v = s.v;
		spec.x = s.x;
	}
	virtual ~ErrorObserver(){}
};

void run_combinational_fault_coverage(ds_lg::LeveledGraph* lg, ds_faults::FaultList* fl, ds_pattern::CombinationalPatternProvider* provider);

}


#endif /* DS_SIMULATION_H_ */

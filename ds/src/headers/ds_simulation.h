/*
 * ds_simulation.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef DS_SIMULATION_H_
#define DS_SIMULATION_H_

#include<set>
#include "ds_lg.h"
#include "ds_faults.h"
#include <boost/log/trivial.hpp>

namespace ds_simulation {

void run_combinational_fault_coverage(ds_lg::LeveledGraph* lg, ds_faults::FaultList* fl,
		ds_pattern::CombinationalPatternProvider* provider);

void run_transition_fault_coverage(ds_lg::TLeveledGraph* lg, ds_faults::FaultList* fl,
		ds_pattern::SequentialPatternProvider* provider);

void run_combinational_fault_coverage(ds_lg::LeveledGraph* lg, ds_faults::MFaultList* mfl,
		ds_pattern::CombinationalPatternList& provider, ds_common::int64 *detected);

}


#endif /* DS_SIMULATION_H_ */

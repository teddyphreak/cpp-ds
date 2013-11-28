/*
 * ds_simulation.cpp
 *
 *  Created on: 12.11.2013
 *      Author: cookao
 */
#include "ds_simulation.h"
#include "ds_lg.h"
#include "ds_faults.h"
#include <boost/log/trivial.hpp>

using ds_faults::SAFaultDescriptor;
using ds_lg::LGNode;

void ds_simulation::run_combinational_fault_coverage(ds_lg::LeveledGraph* lg, ds_faults::FaultList* fl, ds_pattern::CombinationalPatternProvider* provider){
	lg->adapt(provider);
	std::set<SAFaultDescriptor*> fault_set;
	fl->get_undetected_faults(fault_set);

	std::map<LGNode*, std::set<ds_faults::StuckAt*>* > check_points;
	std::map<ds_faults::StuckAt*, SAFaultDescriptor*> fault_map;
	int total = 0;

	for (SAFaultDescriptor* d:fault_set){

		LGNode *node = lg->get_node(d->gate_name);
		if (node==0)
			node = lg->get_node(d->port_name);

		LGNode *cp = lg->get_check_point(node);

		auto it = check_points.find(cp);
		std::set<ds_faults::StuckAt*>* faults = 0;
		if (it==check_points.end()){
			faults = new std::set<ds_faults::StuckAt*>();
			check_points[cp] = faults;
		} else {
			faults = check_points[cp];
		}
		ds_faults::StuckAt *sa = new ds_faults::StuckAt(lg->get_netlist(), d->gate_name, d->port_name, d->value);
		fault_map[sa]=d;
		faults->insert(sa);
		total++;
	}

	std::vector<LGNode*> ordered_check_points;
	for (auto it=check_points.begin();it!=check_points.end();it++){
		ordered_check_points.push_back(it->first);
	}
	std::sort(ordered_check_points.begin(), ordered_check_points.end(), [] (const LGNode* n1, const LGNode* n2) { return (n1->level > n2->level); });

	ds_common::int64 detected;
	ds_common::int64 possibly_detected;

	std::map<LGNode*, ds_simulation::ErrorObserver*> output_map;
	for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
		LGNode *o = *it;
		ds_simulation::ErrorObserver *observer = new ds_simulation::ErrorObserver(&detected, &possibly_detected);
		output_map[o] = observer;
		o->add_monitor(observer);
	}
	int c = 0;
	while (provider->has_next()){

		ds_pattern::SimPatternBlock *block = provider->next();
		lg->clear_hooks();
		lg->sim(block);
		for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
			LGNode *output = *it;
			ds_simulation::ErrorObserver *observer = output_map[output];
			observer->set_spec(output->peek());
		}

		ds_common::int64 mask = block->mask;

		BOOST_LOG_TRIVIAL(trace) << "Pattern block " << (c++);

		for (auto it=ordered_check_points.begin();it!=ordered_check_points.end();it++){

			LGNode *cp = *it;
			detected=0;
			possibly_detected=0;

			std::set<ds_faults::StuckAt*>* cp_faults = check_points[cp];

			if (cp_faults->size()==0)
				continue;

			cp->flip_and_observe();
			for (LGNode* o:cp->outputs){
				lg->push_node(o);
			}
			lg->clear_hooks();
			lg->sim_intermediate();
			cp->rollback();

			ds_common::int64 a = detected & mask;
			ds_common::int64 b = possibly_detected & mask;

			if (a !=0 || b!=0){
				ds_common::int64 a = detected & mask;
				ds_common::int64 b = possibly_detected & mask;

				for (auto f_it=cp_faults->begin();f_it!=cp_faults->end();){

					ds_faults::StuckAt *f = *f_it;
					ds_faults::SAFaultDescriptor* d = fault_map[f];
					ds_common::int64 obs = lg->propagate_to_check_point(f);
					if ((obs & a) !=0){
						cp_faults->erase(f_it++);
						fl->set_fault_category(d, ds_faults::DS);
					} else if((obs & b) !=0){
						fl->set_fault_category(d, ds_faults::NP);
						++f_it;
					}else {
						++f_it;
					}
				}
			}
		}
	}

	for (auto it=output_map.begin();it!=output_map.end();it++){

		LGNode *output = it->first;
		ds_simulation::ErrorObserver* observer = it->second;
		output->remove_monitor(observer);
		delete observer;
	}

	for (auto it=check_points.begin();it!=check_points.end();it++){
		std::set<ds_faults::StuckAt*>* faults = it->second;
		for (auto f_it=faults->begin();f_it!=faults->end();f_it++){
			ds_faults::StuckAt* f = *f_it;
			delete(f);
		}
		delete faults;
	}
	check_points.clear();
}




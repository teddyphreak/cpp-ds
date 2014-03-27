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
using ds_lg::LogicNode;
using ds_lg::TNode;
using ds_lg::TState;

void ds_simulation::run_combinational_fault_coverage(ds_lg::LeveledGraph* lg, ds_faults::FaultList* fl, ds_pattern::CombinationalPatternProvider* provider){
	//adapt pattern provider to leveled graph
	lg->adapt(provider);

	//get undetected faults
	std::set<SAFaultDescriptor*> fault_set;
	fl->get_undetected_faults(fault_set);

	//the check points are inputs, outputs or fanount nodes. The values of the map are the faults dominated by
	//the corresponding check point
	std::map<LogicNode*, std::set<ds_faults::StuckAt*>* > check_points;
	std::map<ds_faults::StuckAt*, SAFaultDescriptor*> fault_map;
	int total = 0;

	for (SAFaultDescriptor* d:fault_set){

		LogicNode *node = lg->get_node(d->gate_name);
		if (node==0)
			node = lg->get_node(d->port_name);

		LogicNode *cp = lg->get_check_point(node);

		auto it = check_points.find(cp);
		std::set<ds_faults::StuckAt*>* faults = 0;
		if (it==check_points.end()){
			faults = new std::set<ds_faults::StuckAt*>();	// no checkpoint for this fault yet
			check_points[cp] = faults;
		} else {
			faults = check_points[cp];						// insert into the available check point entry
		}
		ds_faults::StuckAt *sa = new ds_faults::StuckAt(lg->get_netlist(), d->gate_name, d->port_name, d->value);
		fault_map[sa]=d;
		faults->insert(sa);
		total++;
	}

	// order check points: for now irrelevant
	std::vector<LogicNode*> ordered_check_points;
	for (auto it=check_points.begin();it!=check_points.end();it++){
		ordered_check_points.push_back(it->first);
	}
	std::sort(ordered_check_points.begin(), ordered_check_points.end(), [] (const LogicNode* n1, const LogicNode* n2) { return (n1->level > n2->level); });

	//error info
	ds_common::int64 detected;
	ds_common::int64 possibly_detected;

	//attach observers to the outputs so an error can be identified
	std::map<LogicNode*, ds_simulation::ErrorObserver*> output_map;
	for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
		LogicNode *o = *it;
		ds_simulation::ErrorObserver *observer = new ds_simulation::ErrorObserver(&detected, &possibly_detected);
		output_map[o] = observer;
		o->add_monitor(observer);
	}

	//repeat for all pattern blocks
	int c = 0;
	while (provider->has_next()){

		ds_pattern::SimPatternBlock *block = provider->next();
		lg->clear_hooks();
		//fault-free simulation
		lg->sim(block);
		//set the expected value in the observers
		for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
			LogicNode *output = *it;
			ds_simulation::ErrorObserver *observer = output_map[output];
			observer->set_spec(output->peek());
		}

		//get the pattern block mask (not all blocks are 64 patterns deep)
		ds_common::int64 mask = block->mask;

		BOOST_LOG_TRIVIAL(trace) << "Pattern block " << (c++);

		//evaluate each check point
		for (auto it=ordered_check_points.begin();it!=ordered_check_points.end();it++){

			LogicNode *cp = *it;

			//reset the error info
			detected=0;
			possibly_detected=0;

			//get all dominated faults
			std::set<ds_faults::StuckAt*>* cp_faults = check_points[cp];

			//check if all faults in this check point have been detected
			if (cp_faults->size()==0)
				continue;

			//flip the fault-free value and apply any monitors (required for fanout nodes)
			cp->flip_and_observe();
			//queue all check point outputs for intermediate simulation
			for (LogicNode* o:cp->outputs){
				lg->push_node(o);
			}

			//intermediate simulation
			lg->sim_intermediate();
			//reset check point to fault-free state
			cp->rollback();
			// save the error information (it could be indirectly modified later)
			ds_common::int64 a = detected & mask;
			ds_common::int64 b = possibly_detected & mask;

			if (a !=0 || b!=0){

				for (auto f_it=cp_faults->begin();f_it!=cp_faults->end();){

					ds_faults::StuckAt *f = *f_it;
					ds_faults::SAFaultDescriptor* d = fault_map[f];
					//propagate fault to check point
					ds_common::int64 obs = lg->propagate_to_check_point(f);
					//check if the fault is propagated AND an error is detected
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
	// delete allocated objects
	for (auto it=output_map.begin();it!=output_map.end();it++){

		LogicNode *output = it->first;
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

void ds_simulation::run_transition_fault_coverage(ds_lg::TLeveledGraph* lg, ds_faults::FaultList* fl, ds_pattern::SequentialPatternProvider* provider){

	//adapt pattern provider to leveled graph
	lg->adapt(provider);

	//get undetected faults
	std::set<SAFaultDescriptor*> fault_set;
	fl->get_undetected_faults(fault_set);

	//the check points are inputs, outputs or fanount nodes. The values of the map are the faults dominated by
	//the corresponding check point
	std::map<TState*, std::set<ds_faults::TransitionFault*>* > in_check_points;
	std::map<TNode*, std::set<ds_faults::TransitionFault*>* > out_check_points;
	std::map<ds_faults::TransitionFault*, SAFaultDescriptor*> fault_map;
	int total = 0;

	int in = 0;
			int ex = 0;

	for (SAFaultDescriptor* d:fault_set){

		TNode *node = lg->get_node(d->gate_name);

		if (node==0){
			node = lg->get_node(d->port_name);
		}

		TNode *cp = lg->get_check_point(node);

		ds_faults::TransitionFault *sa = new ds_faults::TransitionFault(lg->get_netlist(), d->gate_name, d->port_name, d->value);

		if ((cp->has_state() && d->gate_name == cp->get_name() && sa->is_input()) ||
				(cp->has_state() && d->gate_name != cp->get_name())){
			in++;
			TState *state = lg->get_register(cp->get_name());
			std::set<ds_faults::TransitionFault*>* faults = 0;
			auto it = in_check_points.find(state);
			if (it==in_check_points.end()){
				faults = new std::set<ds_faults::TransitionFault*>();	// no checkpoint for this fault yet
				in_check_points[state] = faults;
			} else {
				faults = in_check_points[state];						// insert into the available check point entry
			}
			fault_map[sa]=d;
			faults->insert(sa);
			total++;
		} else {
			ex++;
			std::set<ds_faults::TransitionFault*>* faults = 0;
			auto it = out_check_points.find(cp);
			if (it==out_check_points.end()){
				faults = new std::set<ds_faults::TransitionFault*>();	// no checkpoint for this fault yet
				out_check_points[cp] = faults;
			} else {
				faults = out_check_points[cp];						// insert into the available check point entry
			}
			fault_map[sa]=d;
			faults->insert(sa);
			total++;

		}
	}

	std::cout << "IN: " << in << " EX: " << ex << std::endl;

	//error info
	ds_common::int64 detected;
	ds_common::int64 possibly_detected;

	//attach observers to the outputs and errors so an error can be identified
	std::map<TNode*, ds_simulation::TErrorObserver*> output_map;
	for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
		TNode *o = *it;
		ds_simulation::TErrorObserver *observer = new ds_simulation::TErrorObserver(&detected, &possibly_detected);
		output_map[o] = observer;
		o->add_monitor(observer);
	}
	for (auto it=lg->registers.begin(); it!=lg->registers.end();it++){
		ds_lg::TState *r = *it;
		TNode *d = r->get_sink();
		ds_simulation::TErrorObserver *observer = new ds_simulation::TErrorObserver(&detected, &possibly_detected);
		observer->tag = d->get_name();
		output_map[d] = observer;
		d->add_monitor(observer);
	}

	//repeat for all pattern blocks
	std::set<TNode*> total_cp;
	int c = 0;

	int in_det = 0;
	int out_det = 0;

	while (provider->has_next()){
		ds_pattern::SimPatternBlock *block = provider->next();

		lg->clear_hooks();
		//fault-free simulation

		lg->sim(block);

		//set the expected value in the observers
		for (auto it=lg->outputs.begin(); it!=lg->outputs.end();it++){
			TNode *output = *it;
			ds_simulation::TErrorObserver *observer = output_map[output];
			ds_lg::driver_v64 s = output->peek();
			observer->set_spec(s.value);
		}

		for (auto it=lg->registers.begin(); it!=lg->registers.end();it++){
			ds_lg::TState *r = *it;
			TNode *d = r->get_sink();
			ds_simulation::TErrorObserver *observer = output_map[d];
			ds_lg::driver_v64 s = r->peek_sink();
			observer->set_spec(s.value);
		}

		//get the pattern block mask (not all blocks are 64 patterns deep)
		ds_common::int64 mask = block->mask;

		BOOST_LOG_TRIVIAL(info) << "Pattern block " << std::dec << (c++);



		//evaluate each check point
		for (auto it=out_check_points.begin();it!=out_check_points.end();it++){

			TNode *cp = it->first;

			//reset the error info
			detected=0;
			possibly_detected=0;

			//get all dominated faults
			std::set<ds_faults::TransitionFault*>* out_faults = it->second;

			//check if all faults in this check point have been detected
			if (out_faults == 0)
				continue;

			//flip the fault-free value and apply any monitors (required for fanout nodes)
			cp->flip_and_observe();
			//queue all check point outputs for intermediate simulation
			for (TNode* o:cp->outputs){
				lg->push_node(o);
			}

			//intermediate simulation
			lg->sim_intermediate();

			//reset check point to fault-free state
			cp->rollback();

			ds_common::int64 a = detected & mask;
			ds_common::int64 b = possibly_detected & mask;



			if (a !=0 || b!=0){

				for (auto f_it=out_faults->begin();f_it!=out_faults->end();){

					ds_faults::TransitionFault *f = *f_it;
					ds_faults::SAFaultDescriptor* d = fault_map[f];

					//propagate fault to check point
					ds_common::int64 obs = lg->propagate_to_check_point(f);

					//check if the fault is propagated AND an error is detected
					if ((obs & a) !=0){
						out_faults->erase(f_it++);
						fl->set_fault_category(d, ds_faults::DS);
						out_det++;
					} else if((obs & b) !=0){
						fl->set_fault_category(d, ds_faults::NP);
						++f_it;
					}else {
						++f_it;
					}
				}
			}
		}

		for (auto it=in_check_points.begin();it!=in_check_points.end();it++){

			TState *state = it->first;

			std::set<ds_faults::TransitionFault*>* in_faults = it->second;

			for (auto f_it=in_faults->begin();f_it!=in_faults->end();){

				ds_faults::TransitionFault *f = *f_it;
				ds_faults::SAFaultDescriptor* d = fault_map[f];

				ds_common::int64 obs = lg->propagate_to_check_point(f);

				if (obs !=0){
					in_faults->erase(f_it++);
					fl->set_fault_category(d, ds_faults::DS);
					in_det++;
				} else {
					++f_it;
				}
			}
		}
	}

	std::cout << "IN " << std::dec << in_det << "/" << in << std::endl;
	std::cout << "EX " << std::dec << out_det << "/" << ex << std::endl;

//	// delete allocated objects
//	for (auto it=output_map.begin();it!=output_map.end();it++){
//
//		TNode *output = it->first;
//		ds_simulation::TErrorObserver* observer = it->second;
//		output->remove_monitor(observer);
//		delete observer;
//	}
//
//	for (auto it=in_check_points.begin();it!=in_check_points.end();it++){
//		std::set<ds_faults::TransitionFault*>* faults = it->second;
//		for (auto f_it=faults->begin();f_it!=faults->end();f_it++){
//			ds_faults::TransitionFault* f = *f_it;
//			delete(f);
//		}
//		delete faults;
//	}
//	ordered_check_points.clear();
}

void ds_simulation::TErrorObserver::observe(const ds_lg::driver_v64& v) {
	ds_common::int64 detected = (~spec.x & ~v.value.x & spec.v & ~v.value.v) | (~spec.x & ~v.value.x & ~spec.v & v.value.v);
	*ds |= detected;

	ds_common::int64 possibly_detected = ~spec.x & v.value.x;
	*np |= (possibly_detected & ~detected);
}


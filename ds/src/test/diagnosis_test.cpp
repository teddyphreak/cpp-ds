/*
 * diagnosis_test.cpp
 *
 *  Created on: Apr 4, 2014
 *      Author: cookao
 */

#include <boost/test/unit_test.hpp>
#include "diagnosis_test.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"
#include "ds_pattern.h"
#include "ds_lg.h"
#include "ds_faults.h"
#include "ds_diagnosis.h"
#include "ds_timing.h"
#include <boost/log/trivial.hpp>
#include <boost/random.hpp>

void diagnosis_test::t_diagnosis_test(){
	BOOST_LOG_TRIVIAL(info) << "T Diagnosis Test...";
	diagnose("p45k_nan_sff.v", "p45k_nan_patterns.wgl", "top", "p45k_nan_faults", 10, 10);
};

void diagnosis_test::diagnose(const std::string& design, const std::string& wgl_file, const std::string& top, const std::string& faults,
		const unsigned int& circuits, const unsigned int& iterations){

	ds_library::Library *lib = ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	BOOST_LOG_TRIVIAL(info) << "Diagnosing circuit " << design;;
	std::string path = d?d:"";
	std::string pattern_file = path + "/files/" + wgl_file;
	std::string design_file = path + "/files/" + design;
	std::string fault_file = path + "/files/" + faults;

	BOOST_LOG_TRIVIAL(info) << "Importing fault list: " << fault_file;
	std::vector<ds_faults::fastscan_descriptor> descriptors;
	ds_faults::read_fastscan_descriptors(fault_file, descriptors);
	BOOST_LOG_TRIVIAL(info) << descriptors.size() << " faults found";
	ds_faults::FaultList fl(descriptors.begin(), descriptors.end());
	BOOST_LOG_TRIVIAL(info) << "Importing pattern file: " << pattern_file;
	ds_pattern::SequentialPatternProvider* provider = ds_pattern::load_loc_blocks(pattern_file);
	BOOST_LOG_TRIVIAL(info) << "Creating netlist";
	ds_structural::NetList* nl = ds_workspace::load_netlist(top, design_file);
	nl->define_clock("clk");

	BOOST_LOG_TRIVIAL(info) << "Generating leveled graph";
	std::vector<ds_lg::TLeveledGraph*> duds;
	std::vector< std::vector<ds_diagnosis::TCandidate*>* >candidates;

	for (std::size_t i=0;i<circuits;i++){

		ds_lg::TLeveledGraph* lg = nl->get_loc_graph(lib);
		lg->adapt(provider);
		candidates.push_back(new std::vector<ds_diagnosis::TCandidate*>());
		duds.push_back(lg);
	}

	std::vector<ds_faults::SAFaultDescriptor*> fl_descriptors;
	fl.get_undetected_faults(fl_descriptors);

	boost::random::mt19937 rng;
	boost::random::uniform_int_distribution<> range(0,fl_descriptors.size());

	std::vector<ds_faults::SAFaultDescriptor*> injected;

	for (std::size_t i=0;i<circuits;i++){

		ds_lg::TLeveledGraph* lg = duds[i];
		lg->clear_hooks();

		int index = range(rng);

		std::cout << "Injecting " << index << ":" << fl_descriptors.size() << std::endl;

		ds_faults::SAFaultDescriptor* d = fl_descriptors[index];

		ds_faults::TransitionFault *t = new ds_faults::TransitionFault(lg->get_netlist(), d->gate_name, d->port_name, d->value);
		lg->add_hook(t);
		injected.push_back(d);

	}

	ds_lg::TLeveledGraph* lg = nl->get_loc_graph(lib);
	lg->adapt(provider);
	ds_diagnosis::TDiagnoser* diagnoser = new ds_diagnosis::TDiagnoser(lg, circuits);

	ds_diagnosis::diagnose_loc(diagnoser, provider, duds, candidates);

	for (std::size_t index=0;index<circuits;index++){

		ds_faults::SAFaultDescriptor* d = injected[index];
		std::vector<ds_diagnosis::TCandidate*>* exp = candidates[index];

		ds_faults::SAFaultDescriptor* rep = diagnoser->get_representative(d);

		std::cout << "Injected: " << d->get_string() << " -> " << rep->get_string() << "   (" << index << "/" << fl_descriptors.size() << ")" << std::endl;

		for (std::size_t rank = 0;rank < 5; rank++){

			ds_diagnosis::TCandidate* c = exp->at(rank);
			ds_diagnosis::Evidence* e = c->get_evidence(index);

			std::cout << "\t" << rank << " : " << c->get_qualified_name() << ":(" << e->get_sigma() << "," << e->get_iota() << "," << e->get_gamma() << ")" << std::endl;

		}
	}
}


/*
 * sim_test.cpp
 *
 *  Created on: 23.04.2013
 *      Author: cookao
 */
#include <boost/test/unit_test.hpp>
#include "sim_test.h"
#include "ds_library.h"
#include "ds_structural.h"
#include "ds_workspace.h"
#include "ds_pattern.h"
#include "ds_lg.h"
#include "ds_faults.h"
#include "ds_simulation.h"
#include <boost/log/trivial.hpp>

void sim_test::test_sim(){
	lg_sim_test("p100k.v", "p100k.wgl", "top");
};

void sim_test::test_fc(){
	fc_test("p45k.v", "p45k.wgl", "top");
};

void sim_test::fc_test(const std::string& design, const std::string& wgl_file, const std::string& top){
	ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	std::string path = d?d:"";
	std::string pattern_file = path + "/files/" + wgl_file;
	std::string design_file = path + "/files/" + design;
	ds_pattern::CombinationalPatternProvider* provider = ds_pattern::load_pattern_blocks(pattern_file, true);
	ds_structural::NetList* nl = ds_workspace::load_netlist(top,design_file);
	ds_lg::LeveledGraph* lg = nl->get_sim_graph();
	BOOST_LOG_TRIVIAL(debug) << "Calculating fault set...";
	ds_faults::FaultList fl(nl);
	int total_blocks = provider->num_blocks();
	BOOST_LOG_TRIVIAL(debug) << "Beginning fault coverage calculation with " << total_blocks << " ...";

	ds_simulation::run_combinational_fault_coverage(lg, &fl, provider);
	double fc = fl.get_fc();
	BOOST_LOG_TRIVIAL(info) << "Fault coverage: " << fc << std::endl;
	std::set<ds_faults::SAFaultDescriptor*> detected;
	fl.get_detected_faults(detected);
	BOOST_LOG_TRIVIAL(info) << "Detected faults: " << detected.size() << std::endl;

}

void sim_test::lg_sim_test(const std::string& design, const std::string& wgl_file, const std::string& top){
	ds_library::load_default_lib();
	const char* d = getenv("DS");
	std::string path = d?d:"";
	std::string pattern_file = path + "/files/" + wgl_file;
	std::string design_file = path + "/files/" + design;
	ds_pattern::CombinationalPatternProvider* provider = ds_pattern::load_pattern_blocks(pattern_file, false);
	ds_structural::NetList* nl = ds_workspace::load_netlist(top,design_file);
	ds_lg::LeveledGraph* lg = nl->get_sim_graph();
	lg->adapt(provider);

	while (provider->has_next()){

		ds_pattern::SimPatternBlock *block = provider->next();
		ds_pattern::SimPatternBlock spec(*block);

		for (std::size_t idx=0;idx<provider->get_num_outputs();idx++){
			int pos = provider->get_output_offset() + idx;
			block->values[pos].v = 0L;
			block->values[pos].x = 0L;
		}

		lg->sim(block);

		for (std::size_t i=0;i<provider->num_ports();i++){
			BOOST_ASSERT((block->values[i].v & ~spec.values[i].x) == (spec.values[i].v & ~spec.values[i].x));
		}

	}
}





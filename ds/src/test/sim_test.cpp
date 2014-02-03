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
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

void sim_test::test_sim(){
	BOOST_LOG_TRIVIAL(info) << "Simulation Test...";
	lg_sim_test("p45k.v", "p45k.wgl", "top");
	lg_sim_test("p100k.v", "p100k.wgl", "top");
	lg_sim_test("p141k.v", "p141k.wgl", "top");
	lg_sim_test("p239k.v", "p239k.wgl", "top");
	lg_sim_test("p259k.v", "p259k.wgl", "top");
	lg_sim_test("p267k.v", "p267k.wgl", "top");
	lg_sim_test("p269k.v", "p269k.wgl", "top");
	lg_sim_test("p279k.v", "p279k.wgl", "top");
	lg_sim_test("p286k.v", "p286k.wgl", "top");
	lg_sim_test("p295k.v", "p295k.wgl", "top");
	lg_sim_test("p330k.v", "p330k.wgl", "top");
};

void sim_test::test_fc(){
	BOOST_LOG_TRIVIAL(info) << "FC Test";
	fc_test("p45k.v", "p45k.dsp", "top");
	fc_test("p100k.v", "p100k.dsp", "top");
	fc_test("p141k.v", "p141k.dsp", "top");
	fc_test("p239k.v", "p239k.dsp", "top");
	fc_test("p259k.v", "p259k.dsp", "top");
	fc_test("p267k.v", "p267k.dsp", "top");
	fc_test("p269k.v", "p269k.dsp", "top");
	fc_test("p279k.v", "p279k.dsp", "top");
	fc_test("p286k.v", "p286k.dsp", "top");
	fc_test("p295k.v", "p295k.dsp", "top");
	fc_test("p330k.v", "p330k.dsp", "top");
};

void sim_test::fc_test(const std::string& design, const std::string& wgl_file, const std::string& top){
	ds_library::Library *lib = ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	BOOST_LOG_TRIVIAL(info) << "Circuit " << design;;
	std::string path = d?d:"";
	std::string pattern_file = path + "/files/" + wgl_file;
	std::string design_file = path + "/files/" + design;

	ds_pattern::CombinationalPatternProvider* provider = 0;
	{
		std::ifstream ifs(pattern_file);
		boost::archive::binary_iarchive ia(ifs);
		ia >> provider;
	}

	ds_structural::NetList* nl = ds_workspace::load_netlist(top,design_file);
	ds_lg::LeveledGraph* lg = nl->get_sim_graph(lib);
	BOOST_LOG_TRIVIAL(info) << "Calculating fault set...";
	ds_faults::FaultList fl(nl);
	int total_blocks = provider->num_blocks();
	BOOST_LOG_TRIVIAL(info) << "Beginning fault coverage calculation with " << total_blocks << " ...";

	ds_simulation::run_combinational_fault_coverage(lg, &fl, provider);
	double fc = fl.get_fc();
	BOOST_LOG_TRIVIAL(info) << "Fault coverage: " << fc;
	std::set<ds_faults::SAFaultDescriptor*> detected;
	fl.get_detected_faults(detected);
	BOOST_LOG_TRIVIAL(info) << "Detected faults: " << detected.size();

}

void sim_test::lg_sim_test(const std::string& design, const std::string& wgl_file, const std::string& top){
	ds_library::Library *lib = ds_library::load_default_lib();
	const char* d = getenv("DS");
	std::string path = d?d:"";
	std::string pattern_file = path + "/files/" + wgl_file;
	std::string design_file = path + "/files/" + design;
	ds_pattern::CombinationalPatternProvider* provider = ds_pattern::load_pattern_blocks(pattern_file, false);
	ds_structural::NetList* nl = ds_workspace::load_netlist(top,design_file);
	ds_lg::LeveledGraph* lg = nl->get_sim_graph(lib);
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





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
	lg_sim_test("p45k_nan.v", "p45k.wgl", "top");
	lg_sim_test("p100k_nan.v", "p100k.wgl", "top");
	lg_sim_test("p141k_nan.v", "p141k.wgl", "top");
	lg_sim_test("p239k_nan.v", "p239k.wgl", "top");
	lg_sim_test("p259k_nan.v", "p259k.wgl", "top");
	lg_sim_test("p267k_nan.v", "p267k.wgl", "top");
	lg_sim_test("p269k_nan.v", "p269k.wgl", "top");
	lg_sim_test("p279k_nan.v", "p279k.wgl", "top");
	lg_sim_test("p286k_nan.v", "p286k.wgl", "top");
	lg_sim_test("p295k_nan.v", "p295k.wgl", "top");
	lg_sim_test("p330k_nan.v", "p330k.wgl", "top");
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

void sim_test::test_tdf(){
	BOOST_LOG_TRIVIAL(info) << "TDF Test...";
	fc_tdf_test("p45k_nan_sff.v", "p45k_nan_patterns66.wgl", "top", "p45k_nan_faults66");
//	fc_tdf_test("p100k_nan_sff.v", "p100k_nan_patterns.wgl", "top", "p100k_nan_faults");
//	fc_tdf_test("p141k_nan_sff.v", "p141k_nan_patterns.wgl", "top", "p141k_nan_faults");
//	fc_tdf_test("p267k_nan_sff.v", "p267k_nan_patterns.wgl", "top", "p267k_nan_faults");
//	fc_tdf_test("p269k_nan_sff.v", "p269k_nan_patterns.wgl", "top", "p269k_nan_faults");
//	fc_tdf_test("p279k_nan_sff.v", "p279k_nan_patterns.wgl", "top", "p279k_nan_faults");
//	fc_tdf_test("p286k_nan_sff.v", "p286k_nan_patterns.wgl", "top", "p286k_nan_faults");
//	fc_tdf_test("p295k_nan_sff.v", "p295k_nan_patterns.wgl", "top", "p295k_nan_faults");
//	fc_tdf_test("p330k_nan_sff.v", "p330k_nan_patterns.wgl", "top", "p330k_nan_faults");
};

void sim_test::test_loc(){
	BOOST_LOG_TRIVIAL(info) << "LOC Test";
	lg_loc_test("p45k_nan_sff.v", "p45k_nan_patterns.wgl", "top");
//	lg_loc_test("p100k_nan_sff.v", "p100k_nan_patterns.wgl", "top");
//	lg_loc_test("p141k_nan_sff.v", "p141k_nan_patterns.wgl", "top");
//	lg_loc_test("p267k_nan_sff.v", "p267k_nan_patterns.wgl", "top");
//	lg_loc_test("p269k_nan_sff.v", "p269k_nan_patterns.wgl", "top");
//	lg_loc_test("p279k_nan_sff.v", "p279k_nan_patterns.wgl", "top");
//	lg_loc_test("p286k_nan_sff.v", "p286k_nan_patterns.wgl", "top");
//	lg_loc_test("p295k_nan_sff.v", "p295k_nan_patterns.wgl", "top");
//	lg_loc_test("p330k_nan_sff.v", "p330k_nan_patterns.wgl", "top");
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
	BOOST_LOG_TRIVIAL(info) << "Beginning fault coverage calculation";

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
	BOOST_LOG_TRIVIAL(info) << "Circuit: " << design;
	ds_pattern::CombinationalPatternProvider* provider = ds_pattern::load_combinational_blocks(pattern_file, false);
	BOOST_LOG_TRIVIAL(info) << "Pattern file loaded: " << pattern_file;
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

void sim_test::lg_loc_test(const std::string& design, const std::string& wgl_file, const std::string& top){
	ds_library::Library *lib = ds_library::load_default_lib();
	const char* d = getenv("DS");
	std::string path = d?d:"";
	std::string pattern_file = path + "/files/" + wgl_file;
	std::string design_file = path + "/files/" + design;
	BOOST_LOG_TRIVIAL(info) << "Circuit: " << design;
	ds_pattern::SequentialPatternProvider* provider = ds_pattern::load_loc_blocks(pattern_file);
	BOOST_LOG_TRIVIAL(info) << "Pattern file loaded: " << pattern_file;
	ds_structural::NetList* nl = ds_workspace::load_netlist(top, design_file);
	BOOST_LOG_TRIVIAL(info) << "Netlist created";
	nl->define_clock("clk");
	ds_lg::TLeveledGraph* lg = nl->get_loc_graph(lib);
	BOOST_LOG_TRIVIAL(info) << "Leveled graph generated";
	lg->adapt(provider);

	std::size_t output_offset = provider->get_output_offset();
	std::size_t scan_offset = provider->get_scan_offset();

	int a = 0;

	while (provider->has_next()){

		ds_pattern::SimPatternBlock *block = provider->next();
		ds_pattern::SimPatternBlock spec(*block);

		std::cout << "P: " << (a++) <<std::endl;

		lg->sim(block);

		for (std::size_t i=0;i<provider->get_num_outputs();i++){
			int pos = output_offset + i;
			std::string name = provider->get_name(pos);
			//std::cout << "output name " << name << "calculated: " << std::hex << block->values[pos].v << " predicted " << spec.values[pos].v << std::endl;
			BOOST_ASSERT((block->values[pos].v & ~spec.values[pos].x) == (spec.values[pos].v & ~spec.values[pos].x));
		}
		for (std::size_t i=0;i<provider->get_num_scan_cells();i++){
			int pos = scan_offset + i;
			std::string name = provider->get_name(pos);
			ds_lg::TState *reg = lg->get_register(name);
			ds_lg::lg_v64 val = reg->peek_input().value;
			if ((val.v & ~spec.values[provider->get_num_scan_cells() + pos].x) != (spec.values[provider->get_num_scan_cells() + pos].v & ~spec.values[provider->get_num_scan_cells() + pos].x)){
				std::cout << "FF name " << name << "calculated: " << std::hex << val.v << " predicted " << spec.values[provider->get_num_scan_cells() + pos].x << std::endl;
			}
			//BOOST_ASSERT((val.v & ~spec.values[provider->get_num_scan_cells() + pos].x) == (spec.values[provider->get_num_scan_cells() + pos].v & ~spec.values[provider->get_num_scan_cells() + pos].x));
		}
	}
}

void sim_test::fc_tdf_test(const std::string& design, const std::string& wgl_file, const std::string& top, const std::string& faults){
	ds_library::Library *lib = ds_library::load_default_lib();
	const char* d = getenv("DS");
	if (!d){
		BOOST_LOG_TRIVIAL(warning) << "Environmental variable DS not set";
	}
	BOOST_LOG_TRIVIAL(info) << "Circuit " << design;;
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
	BOOST_LOG_TRIVIAL(info) << "Generating leveled graph";
	nl->define_clock("clk");
	ds_lg::TLeveledGraph* lg = nl->get_loc_graph(lib);

	BOOST_LOG_TRIVIAL(info) << "Beginning fault coverage calculation";

	ds_simulation::run_transition_fault_coverage(lg, &fl, provider);
	double fc = fl.get_fc();
	BOOST_LOG_TRIVIAL(info) << "Fault coverage: " << fc;
	std::set<ds_faults::SAFaultDescriptor*> detected;
	fl.get_detected_faults(detected);

	std::set<ds_faults::SAFaultDescriptor*> undetected;
	fl.get_undetected_faults(undetected);

	int fast_scan_detected = 0;
	for (ds_faults::fastscan_descriptor f:descriptors){
		if ((f.code == "DS") || (f.code == "DI")){
			fast_scan_detected++;
		}
	}


	BOOST_LOG_TRIVIAL(info) << "Detected faults: " << detected.size() << " FS: " << fast_scan_detected;

	for (ds_faults::SAFaultDescriptor* d:undetected){
		std::string path = d->gate_name + "/" + d->port_name;
		if (d->gate_name.size() == 0)
			path = d->port_name;
		char v = d->value == ds_common::BIT_1 ? '1' : '0';

		auto descriptor = std::find_if(descriptors.begin(), descriptors.end(), [&](ds_faults::fastscan_descriptor f){
			if ((f.path_name == path) && (f.type == v))
				return true;
			return false;
		});

		ds_faults::FaultCategory cat = fl.get_fault_category(d->get_string());
		if (descriptor->code == "DS" || descriptor->code == "DI"){
			std::cout << "ERROR " << d->get_string() << ":" << descriptor->code << ":" << cat << std::endl;
		}
	}

	for (ds_faults::SAFaultDescriptor* d:detected){
		std::string path = d->gate_name + "/" + d->port_name;
		if (d->gate_name.size() == 0)
			path = d->port_name;
		char v = d->value == ds_common::BIT_1 ? '1' : '0';

		auto descriptor = std::find_if(descriptors.begin(), descriptors.end(), [&](ds_faults::fastscan_descriptor f){
			if ((f.path_name == path) && (f.type == v))
				return true;
			return false;
		});

		ds_faults::FaultCategory cat = fl.get_fault_category(d->get_string());
		if (descriptor->code != "DS" && descriptor->code != "DI"){
			std::cout << "ERROR #" << d->get_string() << ":" << descriptor->code << ":" << cat << std::endl;
		}
	}

}

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

void sim_test::test_tricore_sim(){
	lg_sim_test("p100k.v", "p100k.det.wgl", "top");
};


void sim_test::lg_sim_test(const std::string& design, const std::string& wgl_file, const std::string& top){
	ds_library::LibraryFactory *factory = ds_library::LibraryFactory::getInstance();
	ds_library::Library *defaultLib = factory->load_library();
	const char* d = getenv("DS");
	std::string path = d?d:"";
	std::string pattern_file = path + "/files/" + wgl_file;
	std::string design_file = path + "/files/" + design;
	ds_pattern::CombinationalPatternProvider* provider = ds_pattern::load_pattern_blocks(pattern_file, false);
	ds_workspace::Workspace* wp = ds_workspace::Workspace::get_workspace();
	wp->add_library(defaultLib);
	ds_structural::NetList* nl = ds_library::import(design_file, top, wp);
	nl->remove_floating_signals();
	bool r = nl->remove_unused_gates();
	while (r)
		r = nl->remove_unused_gates();

	ds_lg::LeveledGraph* lg = nl->build_leveled_graph();
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





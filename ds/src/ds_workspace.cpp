/*
 * ds_workspace.cpp
 *
 *  Created on: Sep 27, 2012
 *      Author: cookao
 */
#include <ds_workspace.h>

ds_workspace::Workspace* ds_workspace::Workspace::instance = 0;

bool ds_workspace::Workspace::is_defined(const std::string& name, std::size_t ports){
	bool found = false;
	typedef std::vector<ds_library::Library*>::iterator LIB_IT;
	for (LIB_IT lib_it = libraries.begin();lib_it!=libraries.end();lib_it++){
		ds_library::Library *l = *lib_it;
		if (l->has_gate(name, ports)){
			found = true;
			break;
		}
	}
	if (!found){
		auto net_it = std::find_if(netlists.begin(), netlists.end(),
				[&](ds_structural::NetList* n){
			return n->get_instance_name() == name;
		});
		if (net_it != netlists.end()){
			ds_structural::NetList *nl = *net_it;
			if (nl->get_num_ports() == ports){
				found = true;
			}
		}
	}
	if (!found){
		auto par_it =	std::find_if(parsed_netlists.begin(), parsed_netlists.end(),
			boost::bind(&ds_library::parse_netlist::nl_name, _1)==name);
		if (par_it != parsed_netlists.end()){
			std::size_t p = par_it->ports.size();
			if (p == ports){
				found = true;
			}
		}
	}
	return found;
}


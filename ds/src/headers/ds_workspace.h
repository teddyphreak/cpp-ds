/*
 * ds_workspace.h
 *
 *  Created on: Sep 27, 2012
 *      Author: cookao
 */

#ifndef DS_WORKSPACE_H_
#define DS_WORKSPACE_H_

#include <vector>
#include "ds_library.h"
#include "ds_structural.h"
#include "boost/lambda/bind.hpp"


namespace ds_workspace {

	class Workspace {

	public:

		std::vector<ds_library::Library*> libraries;
		std::vector<ds_structural::NetList*> netlists;
		std::vector<ds_library::parse_netlist> parsed_netlists;

		static Workspace *get_workspace(){
			if (instance == 0)
				instance = new Workspace();
			return instance;
		}

		void add_library(ds_library::Library *lib){
			libraries.push_back(lib);
		}

		void add_netList(ds_structural::NetList *nl){
			netlists.push_back(nl);
		}

		void add_box(const ds_library::parse_netlist& pnl){
			parsed_netlists.push_back(pnl);
		}

		bool is_defined(const std::string& name, std::size_t ports);

		ds_structural::Gate* get_gate(const std::string& name, std::size_t ports){
			ds_structural::Gate* g = 0;
			for (auto lib_it = libraries.begin();lib_it!=libraries.end();lib_it++){
				ds_library::Library *l = *lib_it;
				g = l->getGate(name, ports);
				if (g!=0)
					break;
			}

			return g;
		}

		ds_structural::Gate* get_gate(const std::string& name, const std::vector<std::string>& ports){
			ds_structural::Gate* g = 0;
			typedef std::vector<ds_library::Library*>::iterator LIB_IT;
			for (LIB_IT lib_it = libraries.begin();lib_it!=libraries.end();lib_it++){
				ds_library::Library *l = *lib_it;
				g = l->getGate(name, ports);
				if (g!=0)
					break;
			}
			return g;
		}

		bool elaborate_netlist(const std::string& name, std::size_t ports){

			ds_structural::NetList *netlist = 0;
			typedef std::vector<ds_library::parse_netlist>::iterator PAR_IT;
			PAR_IT par_it =	std::find_if(parsed_netlists.begin(), parsed_netlists.end(),
			boost::bind(&ds_library::parse_netlist::nl_name, _1)==name);
			if (par_it != parsed_netlists.end()){
				std::size_t p = par_it->ports.size();
				if (p == ports){
					netlist = ds_library::convert(*par_it, this);
					if (netlist!=0)
						netlists.push_back(netlist);
				}
			}

			return (netlist!=0);
		}

		ds_structural::NetList* get_netlist(const std::string& name, std::size_t ports){

			ds_structural::NetList *netlist = 0;
			auto net_it = std::find_if(netlists.begin(), netlists.end(),
					[&] (ds_structural::NetList* n){
					return n->get_instance_name() == name;
			});
			if (net_it != netlists.end()){
				ds_structural::NetList *nl = *net_it;
				if (nl->get_num_ports() == ports){
					netlist = nl->clone();
				}
			}
			return netlist;
		}

		ds_structural::NetList* get_design(const std::string& name){
			ds_structural::NetList* nl = 0;
			auto gate_it = std::find_if(netlists.begin(), netlists.end(),
					[&] (ds_structural::NetList* n){
				return n->get_instance_name() == name;
			});
			if (gate_it != netlists.end())
				nl = *gate_it;
			return nl;
		}

	private:
		static Workspace* instance;
		~Workspace(){instance = 0;}
		Workspace(){};

	};
}

#endif /* DS_WORKSPACE_H_ */

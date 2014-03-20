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


	/*!
	 * the workspace is a container for diverse design units: libraries, netlists, and intermediate netlists representations.
	 * It implements the Singleton design pattern.
	 */
	class Workspace {

	public:

		std::vector<ds_library::Library*> libraries; 			//!< container for available libraries
		std::vector<ds_structural::NetList*> netlists;			//!< container for available netlists
		std::vector<ds_library::parse_netlist> parsed_netlists;	//!< container for netlists in intermediate representation

		/*!
		 * entry point
		 * @return active workspace pointer
		 */
		static Workspace* get_workspace(){
			if (instance == 0)
				instance = new Workspace();
			return instance;
		}

		/*!
		 * adds a new library to the workspace
		 * @param lib
		 */
		void add_library(ds_library::Library *lib){
			libraries.push_back(lib);
		}

		/*!
		 * adds a new netlist to the workspace
		 * @param nl
		 */
		void add_netList(ds_structural::NetList *nl){
			netlists.push_back(nl);
		}

		/*!
		 * adds a new intermediate netlist representation to the workspace
		 * @param pnl
		 */
		void add_box(const ds_library::parse_netlist& pnl){
			parsed_netlists.push_back(pnl);
		}

		/*!
		 * checks whether there is a gate with the provided name and number of ports
		 * @param name name of desired gate description
		 * @param ports number of ports of the desired gate description
		 * @return true if the parameters identify a known gate description
		 */
		bool is_defined(const std::string& name, std::size_t ports);

		ds_structural::Gate* get_gate(const std::string& name, std::size_t ports){
			ds_structural::Gate* g = 0;
			for (auto lib_it = libraries.begin();lib_it!=libraries.end();lib_it++){
				ds_library::Library *l = *lib_it;
				g = l->get_gate(name, ports);
				if (g!=0)
					break;
			}

			return g;
		}

		ds_lg::LogicNode* get_primitive(const std::string& name, std::size_t ports){
			ds_lg::LogicNode* n = 0;
			for (auto lib_it = libraries.begin();lib_it!=libraries.end();lib_it++){
				ds_library::Library *l = *lib_it;
				n = l->get_primitive(name, ports);
				if (n!=0)
					break;
				}

			return n;
		}

		ds_library::LogicFunction get_function(const std::string& gate_type){
			ds_library::LogicFunction v = ds_library::UNKNOWN;
			for (auto lib_it = libraries.begin();lib_it!=libraries.end();lib_it++){
				ds_library::Library *l = *lib_it;
				v = l->get_function(gate_type);
				if (v != ds_library::UNKNOWN)
					break;
			}
			return v;
		}


		/*!
		 * searches for a gate in all available libraries. If a gate with the desired name and number of ports is found
		 * a copy of the library gate is returned according to the prototype design pattern
		 * @param name name of the desired gate
		 * @param ports port names of the desired gate
		 * @return pointer to a newly allocated gate. Null is gate does not exist
		 */
		ds_structural::Gate* get_gate(const std::string& name, const std::vector<std::string>& ports){
			ds_structural::Gate* g = 0;
			for (auto lib_it = libraries.begin();lib_it!=libraries.end();lib_it++){
				ds_library::Library *l = *lib_it;
				g = l->get_gate(name, ports);
				if (g!=0)
					break;
			}
			return g;
		}

		/*!
		 * transforms an intermediate netlist description (from a parser) into a netlist.
		 * Netlists are stored internally inside the workspace and can be retrieved by name. The internal netlists are referred to as "prototype" netlists
		 * @param name name of the netlist to elaborate
		 * @param ports number of ports of the desired netlist
		 * @return true if netlist is successfully elaborated
		 */
		bool elaborate_netlist(const std::string& name, std::size_t ports){

			ds_structural::NetList *netlist = 0;
			auto par_it =	std::find_if(parsed_netlists.begin(), parsed_netlists.end(),
				[&] (ds_library::parse_netlist& pn){
				return pn.nl_name == name;
			});
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

		/*!
		 * retrieves a deep copy of the desired netlist according to the prototype design pattern
		 * @param name name of the desired netlist
		 * @param ports number of ports of the desired netlist
		 * @return newly allocated netlist
		 */
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

		/*!
		 * retrieves the prototype of the desired netlist @sa elaborate_netlist
		 * @param name name of the desired netlist
		 * @return source netlist (prototype) in the library
		 */
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
		static Workspace* instance;		//!< singleton instance
		~Workspace(){instance = 0;}		//!< clears singleton instance
		Workspace(){};					//!< hide default constructor

	};

	ds_structural::NetList* load_netlist(const std::string& toplevel, const std::string& path);
}

#endif /* DS_WORKSPACE_H_ */

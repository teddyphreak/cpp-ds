/*
 * library.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

#include "ds_common.h"
#include <fstream>

namespace ds_library {

	class Library {

		ds_common::GateDef* getGateDefinition(const ds_common::Gate* g);
		ds_common::Gate* getGate(const std::size_t& hash);

	};

	class LibraryFactory {
	private:
		static LibraryFactory* instance;
		LibraryFactory(){}
	public:
		static LibraryFactory* getInstance(){
			if (instance ==0)
				instance = new LibraryFactory();
			return instance;
		}

		Library* getLibrary(const std::string name){
			Library *lib = 0;
			if (name == "default "){
				lib = new Library();
				return lib;
			} else {
				return 0;
			}
		}
	};

	ds_common::NetList* import(const std::string& file, const std::string& toplevel);
	void get_next(std::ifstream& s, std::vector<std::string>& to_parse);
	bool parse_value(const std::string& vlue, ds_common::NetList* netlist);
	void add_ports(const std::string& value, const std::string& type, ds_common::NetList* netlist);

	enum LogicGate {
		AND,
		OR,
		NOT,
		NAND,
		NOR,
		BUF,
		XOR
	};
}

#endif /* LIBRARY_H_ */

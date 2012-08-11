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


	};

	class LibraryFactory {
	private:
		static LibraryFactory* instance = 0;
		LibraryFactory(){}
	public:
		static LibraryFactory* getInstance(){
			if (instance ==0)
				instance = new LibraryFactory();
			return instance;
		}

		Library* getLibrary(const std::string name){
			Library *lib = 0;
			switch (name){
			case "default":
				lib = new Library();
				break;
			}
			return 0;
		}
	};

	ds_common::NetList* import(std::string file, std::string toplevel);
	void get_next(const std::ifstream& s, std::vector<std::string>& to_parse);
	void parse_value(std::string, ds_common::NetList* netlist);
}

#endif /* LIBRARY_H_ */

/*
 * library.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

#include "ds_common.h"
#include "ds_lg.h"
#include <fstream>
#include <map>
#include <boost/spirit/include/qi.hpp>

namespace ds_library {

	extern ds_lg::LGNode1I not1;
	extern ds_lg::LGNode1I buf1;

	extern ds_lg::LGNode2I and2;
	extern ds_lg::LGNode2I or2;
	extern ds_lg::LGNode2I nand2;
	extern ds_lg::LGNode2I nor2;
	extern ds_lg::LGNode2I xor2;
	extern ds_lg::LGNode2I xnor2;

	extern ds_lg::LGNode3I and3;
	extern ds_lg::LGNode3I or3;
	extern ds_lg::LGNode3I nand3;
	extern ds_lg::LGNode3I nor3;
	extern ds_lg::LGNode3I xor3;
	extern ds_lg::LGNode3I xnor3;

	extern ds_lg::LGNode4I and4;
	extern ds_lg::LGNode4I or4;
	extern ds_lg::LGNode4I nand4;
	extern ds_lg::LGNode4I nor4;
	extern ds_lg::LGNode4I xor4;
	extern ds_lg::LGNode4I xnor4;

	extern ds_lg::LGNode5I and5;
	extern ds_lg::LGNode5I or5;
	extern ds_lg::LGNode5I nand5;
	extern ds_lg::LGNode5I nor5;
	extern ds_lg::LGNode5I xor5;
	extern ds_lg::LGNode5I xnor5;

	extern ds_lg::LGNode6I and6;
	extern ds_lg::LGNode6I or6;
	extern ds_lg::LGNode6I nand6;
	extern ds_lg::LGNode6I nor6;
	extern ds_lg::LGNode6I xor6;
	extern ds_lg::LGNode6I xnor6;

	extern ds_lg::LGNode7I and7;
	extern ds_lg::LGNode7I or7;
	extern ds_lg::LGNode7I nand7;
	extern ds_lg::LGNode7I nor7;
	extern ds_lg::LGNode7I xor7;
	extern ds_lg::LGNode7I xnor7;

	extern ds_lg::LGNode8I and8;
	extern ds_lg::LGNode8I or8;
	extern ds_lg::LGNode8I nand8;
	extern ds_lg::LGNode8I nor8;
	extern ds_lg::LGNode8I xor8;
	extern ds_lg::LGNode8I xnor8;

	namespace qi = boost::spirit::qi;

	class Library {
	public:
		Library(){load();}
		ds_common::Gate* getGate(const std::string& name, std::vector<std::string>& ports);
	protected:
		std::map<std::string,ds_common::Gate*> gate_map;
		virtual void load();
		static const std::string lib_name;
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

		Library* loadLibrary(const std::string name="default"){
			Library *lib = 0;
			if (name == "default"){
				lib = new Library();
				return lib;
			} else {
				return 0;
			}
		}
	};

	ds_common::NetList* import(const std::string& file, const std::string& toplevel);

	template<typename Iterator>
	ds_common::NetList* import_verilog(Iterator begin, Iterator end, const std::string& toplevel);

	template <typename Iterator>
	struct verilog_parser : qi::grammar<Iterator, ds_common::NetList(), qi::space_type>
	{



	};

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

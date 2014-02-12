/*
 * ds_pattern.h
 *
 *  Created on: 24.03.2013
 *      Author: cookao
 */

#ifndef DS_PATTERN_H_
#define DS_PATTERN_H_

#include "ds_common.h"
#include "stdio.h"
#include <fstream>
#include <map>
#include <unordered_map>
#include <iostream>
#include <boost/dynamic_bitset.hpp>
#include <boost/spirit/repository/include/qi_distinct.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/qi_no_skip.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/variant.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>


namespace ds_pattern {

using ds_common::int64;
using ds_common::lg_v64;

/*
 * Data structures for pattern parsing
 */

/*!
 * intermediate signal representation for pattern parsing
 */
struct signal
{
	bool bus;
	std::string name;
	int right;
	int left;
};

/*!
 * intermediate port representation for pattern parsing
 */
struct ports
{
	std::vector<ds_pattern::signal> inputs;
	std::vector<ds_pattern::signal> outputs;
};

/*!
 * intermediate timeplate representation for pattern parsing
 */
struct timeplate
{
	std::vector<std::string> inputs;
	std::vector<std::string> outputs;
};

struct scan_cells
{
	std::string group;
	std::vector<std::string> cells;
};

struct pattern
{
	std::string name;
	std::vector<std::string> ports;
};

struct vector
{
	std::string timeplate;
	std::string port_data;
};

struct scan_chain
{
	std::string name;
	std::vector<std::string> elements;
};

struct scan_state
{
	std::string id;
	std::string group;
	std::string data;
};
struct scan_directive {
	std::string direction;
	std::string chain_name;
	std::string chain_data;
};
struct scan {
	std::string timeplate;
	std::string port_data;
	std::vector<ds_pattern::scan_directive> directives;
};
typedef boost::variant<ds_pattern::vector, ds_pattern::scan> tester_cycle;
/*!
 * intermediate scan representation for pattern parsing
 */
struct scan_data
{
	std::vector<ds_pattern::timeplate> timeplates;
	ds_pattern::ports ports;
	std::vector<std::string> order;
	std::vector<ds_pattern::tester_cycle> cycles;
	std::vector<ds_pattern::scan_cells> cell_groups;
	std::vector<ds_pattern::scan_chain> chains;
	std::vector<ds_pattern::scan_state> states;
};

}


/*
 * Wrapper auxiliary structures necessary for boost::qi
 */
BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::timeplate,
    (std::vector<std::string>, inputs)
    (std::vector<std::string>, outputs)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::scan_cells,
    (std::string, group)
    (std::vector<std::string>, cells)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::vector,
    (std::string, timeplate)
    (std::string, port_data)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::scan_directive,
    (std::string, direction)
    (std::string, chain_name)
    (std::string, chain_data)
)
BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::scan,
    (std::string, timeplate)
    (std::string, port_data)
    (std::vector<ds_pattern::scan_directive>, directives)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::pattern,
    (std::string, name)
    (std::vector<std::string>, ports)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::scan_chain,
    (std::string, name)
    (std::vector<std::string>, elements)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::ports,
    (std::vector<ds_pattern::signal>, inputs)
    (std::vector<ds_pattern::signal>, outputs)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::scan_state,
    (std::string, id)
    (std::string, group)
    (std::string, data)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::scan_data,
    (std::vector<ds_pattern::timeplate>, timeplates)
    (ds_pattern::ports, ports)
    (std::vector<std::string>, order)
    (std::vector<ds_pattern::tester_cycle>, cycles)
    (std::vector<ds_pattern::scan_cells>, cell_groups)
    (std::vector<ds_pattern::scan_chain>, chains)
    (std::vector<ds_pattern::scan_state>, states)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::signal,
    (bool, bus)
    (std::string, name)
    (int, left)
    (int, right)
)


namespace ds_pattern {

	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;
	namespace phoenix = boost::phoenix;

	/*!
	 * wgl qi grammar
	*/
	template <typename Iterator>
	struct wgl_parser : qi::grammar<Iterator,scan_data(), ascii::space_type>
	{

		wgl_parser() : wgl_parser::base_type(start)
		{

			using qi::int_;
			using qi::char_;
			using qi::blank;
			using qi::lit;
			using boost::phoenix::at_c;
			using boost::spirit::_val;
			using qi::eps;
			using boost::phoenix::at_c;
			using qi::_1;
			using boost::spirit::omit;
			using phoenix::push_back;
			using boost::spirit::no_skip;
			start =
					+comment
					>> waveform  [at_c<1>(_val)=_1]
					>> -(scancells[at_c<4>(_val)=_1] >> scanchains[at_c<5>(_val)=_1])
					>> +timeplate [push_back(at_c<0>(_val)),_1]
					>> -(scanstates[at_c<6>(_val)=_1])
					>> (port_order [at_c<2>(_val)=_1] | comment)
					>> +((vector | scan) [push_back(at_c<3>(_val), _1)] | comment);


			name = char_("a-zA-Z_") >> *char_("a-zA-Z_0-9");
			magnitud = +char_("0-9") >> +char_("a-zA-Z");
			comment = (lit("{") >> +(char_ - '}') >> lit("}")) | (lit("#") >> no_skip[*(char_ - '\n')]);
			waveform =
						lit("waveform") >> name >> lit("signal")
						>> +( input [push_back(at_c<0>(_val), _1)] | output [push_back(at_c<1>(_val), _1)] )
						>> lit("end");
			input =
					eps[at_c<0>(_val)=false] >> lit("\"") >> name [at_c<1>(_val)=_1] >> lit("\"")
					>> -(lit("[") >> int_ [at_c<2>(_val)=_1] >> lit("..") >> int_ [at_c<3>(_val)=_1] >> lit("]")) [at_c<0>(_val)=true]
					>> lit(":") >> lit("input") >> omit [ -(name >> lit("[") >> name >> lit("]")) ]>> lit(";");
			output =
					eps[at_c<0>(_val)=false] >> lit("\"") >> name [at_c<1>(_val)=_1] >> lit("\"")
					>> -(lit("[") >> int_ [at_c<2>(_val)=_1] >> lit("..") >> int_ [at_c<3>(_val)=_1] >> lit("]")) [at_c<0>(_val)=true]
					>> lit(":") >> lit("output") >> omit [ -(name >> lit("[") >> name >> lit("]")) ]>> lit(";");
			timeplate =
					lit("timeplate") >> -lit("\"") >> name >> -lit("\"") >> name >> magnitud
					>> +( input_timeplate [push_back(at_c<0>(_val), _1)] | output_timeplate [push_back(at_c<1>(_val), _1)] )
					>> lit("end");
			input_timeplate =
					lit("\"") >> name >> lit("\"") >> -(char_("[") >> +char_("0-9") >>  char_("]")) >> lit(":=") >>
					lit("input") >> lit("[") >> (magnitud >> lit(":") >> name) % ',' >> lit("]") >> lit(";");
			output_timeplate =
					lit("\"") >> name >> lit("\"") >> -(char_("[") >> +char_("0-9") >>  char_("]")) >> lit(":=") >>
					lit("output") >> lit("[") >> (magnitud >> lit(":") >> -(name >> lit("'")) >> name) % ','>> lit("]") >> lit(";");
			qname = lit("\"") >> name >> lit("\"") >> -(char_('[') >> +char_("0-9") >> char_("]") );
			port_order  =
					lit("pattern") >> omit[name] >> lit("(")
					>> qname % ','
					>> lit(")");
			data = +(char_ - ']');
			vector =
					lit("vector") >> lit("(") >> char_ >> lit(",") >> name[at_c<0>(_val)=_1] >> lit(")")
					>> lit(":=")
					>> lit("[")
					>> data [at_c<1>(_val)=_1]
					>> lit("]")
					>> lit(";");
			cells =
					*comment >> +(qname >> lit(';'))
					>> name [at_c<0>(_val)=_1]
					>> lit('[') >> qname [push_back(at_c<1>(_val),_1)] % ',' >> lit("];");
			scancells =
					lit("scancell")
					>> +cells[push_back(_val,_1)]
					>> lit("end");
			chain =
					*comment
					>> name [at_c<0>(_val)=_1]
					>> lit('[') >> qname [push_back(at_c<1>(_val),_1)] % ',' >> lit("];");
			scanchains =
					lit("scanchain")
					>> +chain[push_back(_val,_1)]
					>> lit("end");
			value =
					+char_("01X-");
			state =
					name >>
					lit(":=")
					>> name  >> lit('(') >> value >> lit(");");
			scanstates =
					lit("scanstate")
					>> +state[push_back(_val,_1)]
					>> lit("end");
			pattern =
					lit("pattern") >> name >> lit('(') >> qname % ',' >> lit(')');
			scan_directive =
					name >> lit('[') >> name >> lit(':') >> name >> lit(']');
			scan =
					lit("scan") >> lit("(") >> char_ >> lit(",") >> name[at_c<0>(_val)=_1] >> lit(")")
					>> lit(":=")
					>> lit("[")
					>> data [at_c<1>(_val)=_1]
					>> lit("]")
					>> lit(",")
					>> *scan_directive[push_back(at_c<2>(_val),_1)] % ','
					>> lit(';');

			//start.name("start");
			//scancells.name("scancells");
			//scanchains.name("scanchains");
			//cells.name("cells");
			//scanstates.name("scanstates");
			//state.name("state");
			//timeplate.name("timeplate");
			//name.name("name");
			//debug(scancells);
			//debug(scanchains);
			//debug(cells);
			//debug(scanstates);
			//debug(start);
			//debug(timeplate);
			//debug(name);
			//debug(state);
		}

		qi::rule<Iterator, std::string()> name, magnitud, qname;
		qi::rule<Iterator, std::string(), ascii::space_type> value;
		qi::rule<Iterator, std::string(), ascii::space_type> comment, input_timeplate, output_timeplate, data;
		qi::rule<Iterator, std::vector<std::string>(), ascii::space_type> port_order;
		qi::rule<Iterator, ds_pattern::ports(), ascii::space_type> waveform;
		qi::rule<Iterator, ds_pattern::signal(), ascii::space_type> input, output;
		qi::rule<Iterator, ds_pattern::timeplate(), ascii::space_type> timeplate;
		qi::rule<Iterator, ds_pattern::scan_cells(), ascii::space_type> cells;
		qi::rule<Iterator, std::vector<ds_pattern::scan_cells>(), ascii::space_type> scancells;
		qi::rule<Iterator, ds_pattern::scan_chain(), ascii::space_type> chain;
		qi::rule<Iterator, std::vector<ds_pattern::scan_chain>(), ascii::space_type> scanchains;
		qi::rule<Iterator, ds_pattern::scan_state(), ascii::space_type> state;
		qi::rule<Iterator, std::vector<ds_pattern::scan_state>(), ascii::space_type> scanstates;
		qi::rule<Iterator, ds_pattern::pattern(), ascii::space_type> pattern;
		qi::rule<Iterator, ds_pattern::scan(), ascii::space_type> scan;
		qi::rule<Iterator, ds_pattern::vector(), ascii::space_type> vector;
		qi::rule<Iterator, ds_pattern::scan_directive(), ascii::space_type> scan_directive;
		qi::rule<Iterator, scan_data(), ascii::space_type> start;

	};

	struct combinational_port_data_extractor : boost::static_visitor<void>{

		std::vector<std::string> *container;

		combinational_port_data_extractor (std::vector<std::string> *c){
			container = c;
		}

		void operator()(const ds_pattern::vector& v) {
			container->push_back(v.port_data);
		}


		void operator()(const ds_pattern::scan& s){}

	};
	/*!
	 * Values in a test pattern
	 */
	class PatternValue {
	private:
		boost::dynamic_bitset<> v;		//!< logic values
		boost::dynamic_bitset<> x;		//!< x mask
	public:
		PatternValue():v(0),x(0){}
		/*!
		 * Empty patter values
		 */
		PatternValue(int length):v(length),x(length){};
		/*!
		 * Copies the values of this pattern
		 */
		PatternValue(const PatternValue& pv):v(pv.v), x(pv.x){};

		/*!
		 * sets a value in the specified position
		 * @param pos offset position in the pattern
		 * @param val value to write in the specified position
		 */
		void set(std::size_t pos, ds_common::Value val) {
			if (val == ds_common::BIT_X){
				x[pos] = true;
				v[pos] = false;
			} else {
				x[pos] = false;
				v[pos] = val == ds_common::BIT_1;
			}
		}

		/*!
		 * returns the pattern value in a specified ofset position
		 */
		ds_common::Value get(int pos) const {
			if(x[pos]==1){
				return ds_common::BIT_X;
			}
			if (v[pos] == 1)
				return ds_common::BIT_1;
			if (v[pos] == 0)
				return ds_common::BIT_0;
			return ds_common::BIT_X;
		}

		bool is_compatible(const PatternValue& pv);

		std::size_t get_specified_bits(){
			boost::dynamic_bitset<> t_x(~x);
			return t_x.count();
		}

		std::size_t get_size()const{
			if (x.size()!=v.size())
				return -1;
			return v.size();
		}
	};

	struct ate_cycle{
		PatternValue ports;
		PatternValue scan;
		std::string timeplate_name;
		ate_cycle(const PatternValue& p, const PatternValue& s, const std::string& tp): ports(p),scan(s), timeplate_name(tp){};
		ate_cycle(const PatternValue& p, const std::string& tp): ports(p), timeplate_name(tp){};
	};

	typedef std::vector<ate_cycle> cycle_values;
	typedef std::vector<std::string> port_definition;

	/*!
	 * intermediate representation of a list of patterns. All information in the wgl file is captured in this data structure
	 */
	class CombinationalPatternList {
	protected:
		unsigned int inputs;				//!< number of inputs
		unsigned int outputs;				//!< number of outputs
		cycle_values cycles;					//!< port data
		port_definition port_order;			//!< order of ports
		void setup_ports(const scan_data& data);
		CombinationalPatternList(){}

	public:

		/*!
		 * Creates a pattern list from an intermediate wgl representation.
		 * @param sc wgl representation of test patterns
		 * @param compact true if compatible patterns are discarded
		 */
		CombinationalPatternList(const scan_data& sc, bool compact = true);
		/*
		 * several convenience methods
		 */
		unsigned int get_vector_count() const {return cycles.size();}
		unsigned int get_port_count() const {return inputs + outputs;}
		unsigned int get_num_inputs() const {return inputs;}
		unsigned int get_num_outputs() const {return outputs;}
		unsigned int get_output_offset() const {return inputs;}
		PatternValue get_port_values(std::size_t patternIdx) const {return cycles[patternIdx].ports;}
		std::string get_port_name(const std::size_t offset) const {return port_order[offset];}
		port_definition::const_iterator begin_port_order() const {return port_order.begin();}
		port_definition::const_iterator end_port_order() const {return port_order.end();}
		virtual ~CombinationalPatternList(){}
	};

	typedef std::vector<lg_v64> PatternBlock;

	/*!
	 * contains a block of 64 patterns encoded with a sequence of lg_v64 values
	 */
	class SimPatternBlock {
	public:
		int num_patterns;		//!< number of patterns in the block (<64)
		ds_common::int64 mask;
		PatternBlock values;	//!< sequence of pattern values
		SimPatternBlock(const int& n):mask(0){
			num_patterns = n;
			for (int i=0;i<num_patterns;i++){
				mask |= 1L << i;
			}
		}
		/*!
		 * provided pattern values are copied
		 */
		SimPatternBlock(const SimPatternBlock& spb):num_patterns(spb.num_patterns),mask(spb.mask){
			values.insert(values.begin(), spb.values.begin(), spb.values.end());
		}
	private:
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version){
			ar & num_patterns;
			ar & mask;
			ar & values;
		}

		SimPatternBlock(){}
	};

	/*!
	 * this interface is intended to ease the coupling between a simulation graph and a pattern block.
	 * Derived classes provide a mapping between port names and an offset in the pattern block
	 */
	class CombinationalPatternAdapter {
	public:
		/*!
		 * returns the offset position of a port in the pattern block
		 * @param name name of the port for which an offset is requested
		 */
		virtual std::size_t get_offset(const std::string& name) const = 0;
		/*!
		 * returns the name of the port at the provided offset position
		 * @param idx offset position for which a port number is requested
		 */
		virtual std::string get_name(const int& idx) const = 0;
	};

	class SequentialPatternAdapter {
	public:
		virtual int get_port_offset(const std::size_t& vector, const std::string& name) const = 0;
		virtual int get_scan_offset(const std::string& name) const = 0;
		virtual std::string get_name(const int& idx) const = 0;
	};

	/*!
	 * this interface provides a generalized way to obtain a sequence of patterns.
	 * Derived classes adapt a test pattern set for efficient access during simulation
	 */
	class PatternProvider {
		virtual bool has_next() const = 0;
		virtual SimPatternBlock* next() = 0;
		virtual void reset() = 0;
	};

	/*!
	 * this interface provides a generalized way to random access to a pattern set.
	 * Derived classes adapt a test pattern set for efficient access during simulation.
	 */
	class RandomAccessPatternProvider {
		virtual unsigned int num_blocks() const = 0;
		virtual SimPatternBlock& get_block(unsigned int) = 0;
	};

	/*!
	 * this class realizes all required functionality for the arbitrary access to combinational patterns
	 */
	class CombinationalPatternProvider : public PatternProvider , public RandomAccessPatternProvider, public CombinationalPatternAdapter{

	public:
		/*!
		 * Creates a combinational pattern provider out of an intermediate pattern set description
		 */
		CombinationalPatternProvider(const CombinationalPatternList& pl);
		/*!
		 * Creates a combinational pattern provider out of parsed pattern data
		 * @param compact true if compactible patterns are discarded
		 */
		CombinationalPatternProvider(const scan_data& sc, bool compact=true);

		/*
		 * convenience functions
		 */
		unsigned int num_ports() const {return total_ports;}
		unsigned int get_output_offset() const {return output_offset;}
		unsigned int get_num_inputs() const {return num_inputs;}
		unsigned int get_num_outputs() const {return num_outputs;}

		/*
		 * implementation of PatternProvider. Pattern blocks are provided in order
		 */
		virtual bool has_next() const {return blockIndex != values.size();}
		virtual void reset() {blockIndex = 0;}
		virtual SimPatternBlock* next() { return &values[blockIndex++]; };

		/*
		 * implementation of RandomAccessPatternProvider. Pattern block array is indexed. All pattern blocks are available in memory
		 */
		virtual unsigned int num_blocks() const {return values.size();};
		virtual SimPatternBlock& get_block(unsigned int idx) { return values[idx]; };

		/*
		 * implementation of CombinationalPatternAdapter. Port order in the wgl file is maintained
		 */
		virtual std::size_t get_offset(const std::string& name) const;
		virtual std::string get_name(const int& idx) const;


		virtual ~CombinationalPatternProvider(){}

	private:

		unsigned int total_ports;			//!< total number of ports
		unsigned int blockIndex;			//!< current block index
		unsigned int output_offset;			//!< offset position of the first output port
		unsigned int num_inputs;			//!< number of inputs
		unsigned int num_outputs;			//!< number of outputs
		std::vector<SimPatternBlock> values;	//!< internal pattern block representation
		std::vector<std::string> ports;			//!< port order

		CombinationalPatternProvider(){}

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version){
			ar & total_ports;
			ar & blockIndex;
			ar & output_offset;
			ar & num_inputs;
			ar & num_outputs;
			ar & values;
			ar & ports;
		}

	};

	/*
	 * convenience functions to load a pattern set
	 */

	/*!
	 * loads an intermediate representation of the pattern set
	 * @param file path to the wgl file
	 * @param discard_x true if compatible patterns are discarded
	 *
	 */
	ds_pattern::CombinationalPatternList* parse_combinational_wgl(const std::string& file, bool compact=true);

	/*!
	 * loads a combinational pattern provider ready for simulation
	 * @param file path to the wgl file
	 * @param discard_x true if compatible patterns are discarded
	 *
	 */
	ds_pattern::CombinationalPatternProvider* load_pattern_blocks(const std::string& file, bool compact=true);

	bool parse_transition_wgl(const std::string& file, ds_pattern::scan_data& rawPatterns);

	class SequentialPatternList : public CombinationalPatternList{
	protected:
		std::vector<std::vector<std::string> > cell_order;
		std::unordered_map<std::string, std::size_t> cell_map;
		std::vector<std::string> cell_names;
		std::size_t num_scan_cells;
	public:
		SequentialPatternList(const scan_data& sc);
		std::string get_cell_name(const std::size_t idx) const {return cell_names[idx];}
		PatternValue get_scan_values(std::size_t cycleIdx) const {return cycles[cycleIdx].scan;}
		std::size_t get_num_scan_cells() const {return num_scan_cells;}
		virtual ~SequentialPatternList(){}

	};

	PatternValue transform(const std::string& data, const std::size_t& length, const int& offset);
	void transform(const std::string& s, ds_pattern::PatternValue& val, const int& offset);

	struct sequential_extractor : public boost::static_visitor<void>{

		cycle_values *c;
		const std::unordered_map<std::string, int>& om;
		const std::unordered_map<std::string, std::string>& vm;
		std::size_t tc;

		sequential_extractor(cycle_values *container, const std::unordered_map<std::string, int>& offset_map,
				const std::unordered_map<std::string, std::string>& value_map, const std::size_t& total_cells):
					c(container),om(offset_map),vm(value_map), tc(total_cells){}

		void operator()(const ds_pattern::vector& v){
			PatternValue ports = transform(v.port_data, v.port_data.size(), 0);
			ds_pattern::ate_cycle cycle(ports, v.timeplate);
			c->push_back(cycle);
		}
		void operator()(const ds_pattern::scan& s);
	};

	class SequentialPatternProvider : public PatternProvider , public RandomAccessPatternProvider, public SequentialPatternAdapter{
public:
		SequentialPatternProvider (const SequentialPatternList& pl);

		virtual int get_port_offset(const std::size_t& vector, const std::string& name) const;
		virtual int get_scan_offset(const std::string& name) const;
		virtual std::string get_name(const int& idx) const;

		virtual bool has_next() const {return blockIndex != values.size();}
		virtual void reset() {blockIndex = 0;}
		virtual SimPatternBlock* next() { return &values[blockIndex++]; };

		virtual unsigned int num_blocks() const {return values.size();};
		virtual SimPatternBlock& get_block(unsigned int idx) { return values[idx]; };

	private:

		const int VECTOR_NUMBER = 2;

		unsigned int blockIndex;
		unsigned int output_offset;
		unsigned int num_inputs;
		unsigned int num_outputs;
		std::vector<SimPatternBlock> values;
		std::vector<std::string> ports;
		std::vector<std::string> cells;
		void set_values(const std::size_t& slot, const std::size_t& offset, const PatternValue pv, SimPatternBlock& block);

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version){
			ar & blockIndex;
			ar & output_offset;
			ar & num_inputs;
			ar & num_outputs;
			ar & values;
			ar & ports;
			ar & cells;
		}
	};
}



#endif /* DS_PATTERN_H_ */

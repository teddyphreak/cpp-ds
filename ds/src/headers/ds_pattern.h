/*
 * ds_pattern.h
 *
 *  Created on: 24.03.2013
 *      Author: cookao
 */

#ifndef DS_PATTERN_H_
#define DS_PATTERN_H_

//#define BOOST_SPIRIT_DEBUG

#include "ds_common.h"
#include "ds_simulation.h"
#include "stdio.h"
#include <fstream>
#include <map>
#include <boost/dynamic_bitset.hpp>
#include <boost/spirit/repository/include/qi_distinct.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/fusion/include/std_pair.hpp>
#include <boost/config/warning_disable.hpp>
#include <boost/variant.hpp>
#include <boost/spirit/include/phoenix_object.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/fusion/include/io.hpp>
#include <iostream>

namespace ds_lg {
	struct lg_v64;
}

namespace ds_pattern {

using ds_common::int64;

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

/*!
 * intermediate scan representation for pattern parsing
 */
struct scan_data
{
	ds_pattern::timeplate timeplate;
	ds_pattern::ports ports;
	std::vector<std::string> order;
	std::vector<std::string> scan;	};
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
    ds_pattern::ports,
    (std::vector<ds_pattern::signal>, inputs)
    (std::vector<ds_pattern::signal>, outputs)
)

BOOST_FUSION_ADAPT_STRUCT(
    ds_pattern::scan_data,
    (ds_pattern::timeplate, timeplate)
    (ds_pattern::ports, ports)
    (std::vector<std::string>, order)
    (std::vector<std::string>, scan)
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
			using qi::lit;
			using boost::phoenix::at_c;
			using boost::spirit::_val;
			using qi::eps;
			using boost::phoenix::at_c;
			using qi::_1;
			using boost::spirit::omit;
			using phoenix::push_back;
			start =
					+comment
					>> waveform  [at_c<1>(_val)=_1]
					>> timeplate [at_c<0>(_val)=_1]
					>> (order [at_c<2>(_val)=_1] | comment)
					>> +(scan [push_back(at_c<3>(_val), _1)] | comment);


			name = char_("a-zA-Z_") >> *char_("a-zA-Z_0-9");
			magnitud = +char_("0-9") >> +char_("a-zA-Z");
			comment = lit("{") >> +(char_ - '}') >> lit("}");
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
					lit("timeplate") >> name >> name >> magnitud
					>> +( input_timeplate [push_back(at_c<0>(_val), _1)] | output_timeplate [push_back(at_c<1>(_val), _1)] )
					>> lit("end");
			input_timeplate =
					lit("\"") >> name >> lit("\"") >> -(char_("[") >> +char_("0-9") >>  char_("]")) >> lit(":=") >>
					lit("input") >> lit("[") >> (magnitud >> lit(":") >> name) % ',' >> lit("]") >> lit(";");
			output_timeplate =
					lit("\"") >> name >> lit("\"") >> -(char_("[") >> +char_("0-9") >>  char_("]")) >> lit(":=") >>
					lit("output") >> lit("[") >> (magnitud >> lit(":") >> name) % ','>> lit("]") >> lit(";");
			port_name = lit("\"") >> name >> lit("\"") >> -(char_('[') >> +char_("0-9") >> char_("]") );
			order  =
					lit("pattern") >> omit[name] >> lit("(")
					>> port_name % ','
					>> lit(")");
			data = +(char_ - ']');
			scan =
					lit("vector") >> lit("(") >> char_ >> lit(",") >> name >> lit(")")
					>> lit(":=")
					>> lit("[")
					>> data [_val = _1]
					>> lit("]")
					>> lit(";");


			BOOST_SPIRIT_DEBUG_NODE(order);
			BOOST_SPIRIT_DEBUG_NODE(scan);
			BOOST_SPIRIT_DEBUG_NODE(comment);
			BOOST_SPIRIT_DEBUG_NODE(waveform);
			BOOST_SPIRIT_DEBUG_NODE(timeplate);
			BOOST_SPIRIT_DEBUG_NODE(data);
			BOOST_SPIRIT_DEBUG_NODE(name);
			BOOST_SPIRIT_DEBUG_NODE(magnitud);
			BOOST_SPIRIT_DEBUG_NODE(start);
			BOOST_SPIRIT_DEBUG_NODE(input_timeplate);
			BOOST_SPIRIT_DEBUG_NODE(output_timeplate);
		}

		qi::rule<Iterator, std::string()> name, magnitud;
		qi::rule<Iterator, std::string(), ascii::space_type> comment, input_timeplate, output_timeplate, port_name, scan, data;
		qi::rule<Iterator, std::vector<std::string>(), ascii::space_type> order;
		qi::rule<Iterator, ds_pattern::ports(), ascii::space_type> waveform;
		qi::rule<Iterator, ds_pattern::signal(), ascii::space_type> input, output;
		qi::rule<Iterator, ds_pattern::timeplate(), ascii::space_type> timeplate;
		qi::rule<Iterator, scan_data(), ascii::space_type> start;
	};


	/*!
	 * Values in a test pattern
	 */
	class PatternValue {
	public:
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
		void set(std::size_t pos, ds_simulation::Value val) {
			if (val == ds_simulation::BIT_X){
				x[pos] = true;
				v[pos] = false;
			} else {
				x[pos] = false;
				v[pos] = val == ds_simulation::BIT_1;
			}
		}

		/*!
		 * returns the pattern value in a specified ofset position
		 */
		ds_simulation::Value get(int pos) const {
			if(x[pos]==1){
				return ds_simulation::BIT_X;
			}
			if (v[pos] == 1)
				return ds_simulation::BIT_1;
			if (v[pos] == 0)
				return ds_simulation::BIT_0;
			return ds_simulation::BIT_X;
		}

	private:
		boost::dynamic_bitset<> v;		//!< logic values
		boost::dynamic_bitset<> x;		//!< x mask
	};

	typedef std::vector<std::vector<PatternValue>> scan_values;
	typedef std::vector<PatternValue> port_values;
	typedef std::vector<std::string> port_definition;
	typedef std::vector<std::vector<std::string>> chain_configuration;

	/*!
	 * intermediate representation of a list of patterns. All information in the wgl file is captured in this data structure
	 */
	class PatternList {
	private:
		unsigned int inputs;				//!< number of inputs
		unsigned int outputs;				//!< number of outputs
		port_values ports;					//!< port data
		scan_values chains;					//!< scan chain data
		port_definition port_order;			//!< order of ports
		chain_configuration scan_order;		//!< order of scan elements

	public:

		/*!
		 * Creates a pattern list from an intermediate wgl representation.
		 * @param sc wgl representation of test patterns
		 * @param discard_x true if any pattern containing 'X' values is discarded
		 */
		PatternList(const scan_data& sc, bool discard_x = true);
		/*
		 * several convenience methods
		 */
		unsigned int get_pattern_count() const {return ports.size();}
		unsigned int get_port_count() const {return inputs + outputs;}
		unsigned int get_num_inputs() const {return inputs;}
		unsigned int get_num_outputs() const {return outputs;}
		unsigned int get_output_offset() const {return inputs;}
		PatternValue get_port_values(std::size_t patternIdx) const {return ports[patternIdx];}
		PatternValue get_chain_values(std::size_t patternIdx, std::size_t chainIdx) const {return chains[chainIdx][patternIdx];}
		unsigned int get_num_chains() const {return chains.size();}
		std::string get_port_name(const std::size_t offset) const {return port_order[offset];}
		port_definition::const_iterator begin_port_order() const {return port_order.begin();}
		port_definition::const_iterator end_port_order() const {return port_order.end();}
	};

	typedef std::vector<ds_lg::lg_v64> PatternBlock;

	/*!
	 * contains a block of 64 patterns encoded with a sequence of lg_v64 values
	 */
	struct SimPatternBlock {
		int num_patterns;		//!< number of patterns in the block (<64)
		PatternBlock values;	//!< sequence of pattern values
		SimPatternBlock(){}
		/*!
		 * provided pattern values are copied
		 */
		SimPatternBlock(const SimPatternBlock& spb):num_patterns(spb.num_patterns){
			values.insert(values.begin(), spb.values.begin(), spb.values.end());
		}
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
		CombinationalPatternProvider(const PatternList& pl);
		/*!
		 * Creates a combinational pattern provider out of parsed pattern data
		 * @param discard_x true if any pattern containing 'X' values is discarded
		 */
		CombinationalPatternProvider(const scan_data& sc, bool discard_x=true);

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


	private:

		unsigned int total_ports;			//!< total number of ports
		unsigned int blockIndex;			//!< current block index
		unsigned int output_offset;			//!< offset position of the first output port
		unsigned int num_inputs;			//!< number of inputs
		unsigned int num_outputs;			//!< number of outputs
		std::vector<SimPatternBlock> values;	//!< internal pattern block representation
		std::vector<std::string> ports;			//!< port order

	};

	/*
	 * convenience functions to load a pattern set
	 */

	/*!
	 * loads an intermediate representation of the pattern set
	 * @param file path to the wgl file
	 * @param discard_x true if any pattern containing 'X' values is discarded
	 *
	 */
	ds_pattern::PatternList* parse_wgl(const std::string& file, bool discard_x=true);

	/*!
	 * loads a combinational pattern provider ready for simulation
	 * @param file path to the wgl file
	 * @param discard_x true if any pattern containing 'X' values is discarded
	 *
	 */
	ds_pattern::CombinationalPatternProvider* load_pattern_blocks(const std::string& file, bool discard_x=true);

}



#endif /* DS_PATTERN_H_ */

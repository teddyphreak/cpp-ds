/*
 * ds_trans.h
 *
 *  Created on: 23.01.2014
 *      Author: cookao
 */

#ifndef DS_TRANS_H_
#define DS_TRANS_H_

#include "ds_structural.h"

namespace ds_trans {

	struct ScanFFInfo {
		std::string name;
		std::string d_port;
		std::string se_port;
		std::string si_port;
		std::string clk;
		std::string q_port;
		bool has_qn_port;
		std::string qn_port;
		bool has_reset;
		std::string reset_port;
		bool has_preset;
		std::string preset_port;
	};

	struct ScanProperties {
		bool has_reset;
		std::string reset_name;
		bool has_preset;
		std::string preset_name;
		std::string clk_name;
		std::string se_name;
		std::string ff_name;
		std::string si_name;
		std::string so_name;

	};

	struct PulseDesc{
		std::string signal_name;
		unsigned int offset;
		unsigned int width;
	};

	struct TimeplateDesc{
		std::string tp_name;
		unsigned int force_pi;
		unsigned int measure_po;
		std::vector<PulseDesc> pulses;
		unsigned int period;

		template <class T>
		void print(T& output) const {
			output << "timeplate " << tp_name << " = \n";
			output << "\tforce_pi " << force_pi << ";\n";
			output << "\tmeasure_po " << measure_po << ";\n";
			for (PulseDesc pd:pulses){
				output << "\tpulse " << pd.signal_name << " " << pd.offset << " " << pd.width << ";\n";
			}
			output << "\tperiod " << period << ";\n";
			output << "end;\n";
		}
	};

	ds_structural::NetList* insert_scan_chains(ds_structural::NetList *nl, const ds_structural::CombinationalScanMap *map, const ScanProperties& scan, const ScanFFInfo& sff );

	ds_structural::Gate* get_scan_ff(const ScanFFInfo& info);

	void insert_top_level_scan(std::string top_level, std::string scan_port, ds_structural::NetList *nl, std::vector<std::vector<ds_structural::Gate*> >& ffs);

	void write_test_procedures(const std::string& dofile_name, const std::string& tp_name, const std::string& sg_name, const ds_structural::CombinationalScanMap *map, const TimeplateDesc& tp_slow, const TimeplateDesc& tp_fast, const ds_trans::ScanProperties& properties, const std::string& patttern_file, const std::string& fault_file);

	template<class T>
	void print_load_unload(T& out, const TimeplateDesc& tp, const std::string& scan_group,
			const std::string& clk_name, const bool& clk_p,
			const std::string& se_name, const bool& se_p, const std::string& rst_name, const bool& rst_p, const unsigned int& shifts){
		out << "procedure load_unload = \n";
		out << "\tscan_group " << scan_group << ";\n";
		out << "\ttimeplate " << tp.tp_name << ";\n";
		out << "\tcycle = \n";
		char p = clk_p ? '1':'0';
		out << "\t\tforce " << clk_name << " " << p << ";\n";
		p =  se_p ? '1':'0';
		out << "\t\tforce " << se_name << " " << p << ";\n";
		p =  rst_p ? '1':'0';
		out << "\t\tforce " << rst_name << " " << p << ";\n";
		out << "\tend;\n";
		out << "apply shift " << shifts << ";\n";
		out << "end;\n";
	}

	template <class T>
	void print_shift(T& out, const TimeplateDesc& tp, const std::string& clk_name, const std::string& sg_name){
		out << "procedure shift = \n";
		out << "\ttimeplate " << tp.tp_name << ";\n";
		out << "\tcycle = \n";
		out << "\t\tforce_sci;\n";
		out << "\t\tmeasure_sco;\n";
		out << "\t\tpulse " << clk_name << ";\n";
		out << "\tend;\n";
		out << "end;\n";
	}

	template <class T>
	void print_capture(T& out, const TimeplateDesc& tp){
		out << "procedure capture = \n";
		out << "\ttimeplate " << tp.tp_name << ";\n";
		out << "\tcycle = \n";
		out << "\t\tforce_pi;\n";
		out << "\t\tmeasure_po;\n";
		out << "\t\tpulse_capture_clock;\n";
		out << "\tend;\n";
		out << "end;\n";
	}

}
#endif /* DS_TRANS_H_ */

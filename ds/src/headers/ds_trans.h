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

	ds_structural::NetList* insert_scan_chains(ds_structural::NetList *nl, const ds_structural::CombinationalScanMap *map, const ScanProperties& scan, const ScanFFInfo& sff );

	ds_structural::Gate* get_scan_ff(const ScanFFInfo& info);

	void insert_top_level_scan(std::string top_level, std::string scan_port, ds_structural::NetList *nl, std::vector<std::vector<ds_structural::Gate*> >& ffs);
}
#endif /* DS_TRANS_H_ */

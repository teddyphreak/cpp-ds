/*
 * timing.h
 *
 *  Created on: 04.01.2014
 *      Author: cookao
 */

#ifndef TIMING_H_
#define TIMING_H_

#include <list>
#include "ds_common.h"

struct Wave{
	float timestamp;
	ds_common::lg_v64 value;
};

class WNode : public ds_lg::LGNode{
	std::list<Wave> waves;
};


#endif /* TIMING_H_ */

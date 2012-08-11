/*
 * ds_simulation.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cookao
 */

#ifndef DS_SIMULATION_H_
#define DS_SIMULATION_H_

#include<set>

namespace ds_simulation {

	class LGNode {
	public:
		virtual void fire()=0;
		virtual bool propagate(){return true;};
		virtual ~LGNode(){}
	private:
		std::set<LGNode&> outputs;
		ds_common::int64 val;
		ds_common::int64 temp;
	};

	class LGNode2I : public LGNode {
		ds_common::int64  &in0;
		ds_common::int64  &in1;
		virtual ds_common::int64 fire();
	};

}


#endif /* DS_SIMULATION_H_ */

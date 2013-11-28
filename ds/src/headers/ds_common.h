#ifndef DS_COMMON_H_
#define DS_COMMON_H_
#include <boost/exception/all.hpp>

namespace ds_common {

	const int WIDTH = 64; 
	typedef unsigned long int64;
	typedef boost::error_info<struct tag_errmsg, std::string> errmsg_info;
	struct file_read_error: virtual boost::exception, virtual std::exception { };
	struct parse_error: virtual boost::exception, virtual std::exception { };
	
	enum Value {
		BIT_0 = 0,
		BIT_1,
		BIT_X,
		BIT_UD
	};
}


#endif /* DS_COMMON_H_ */

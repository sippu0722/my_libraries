#pragma once

#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace bs
{
	inline std::string
	createDirWithDate(const std::string& dir_root, const std::string& facet_str = "%m%d_%H%M")
	{
		return setDateDirName(dir_root, facet_str);
	}

	inline std::string
	setDateDirName(const std::string& dir_root, const std::string& facet_str = "%m%d_%H%M")
	{
		// Set output directory
		auto facet = new boost::posix_time::time_facet(facet_str.c_str());
		std::stringstream ss;
		ss.imbue(std::locale(std::cout.getloc(), facet));
		auto now_time = boost::posix_time::second_clock::local_time();
		ss << now_time;

		const std::string dir_tmp = dir_root + ss.str();
		std::string dir_out = dir_tmp;
		size_t idx = 0;

		while (boost::filesystem::exists(dir_out))
			dir_out = dir_tmp + "_" + std::to_string(++idx);
		boost::filesystem::create_directories(dir_out);
		return (dir_out + "/");
	}
}

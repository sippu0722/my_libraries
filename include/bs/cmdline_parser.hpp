#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cassert>


/*
---------- Sample code ----------
#include <iostream>
#include <opencv2/opencv.hpp>

#include <bs/cmdline_parser.hpp>


int main(int argc, char **argv)
{
	std::vector<bs::CmdlineParserKey> keys =
	{
		bs::CmdlineParserKey("h help usage ?", "3.141592"	   , "pi"),
		bs::CmdlineParserKey("some", "<none>", "positional arg", true),
		bs::CmdlineParserKey("l lgg"		 , "stringName"	   , "name"),
		bs::CmdlineParserKey("i int integer" , "10"			   , "integer value"),
		bs::CmdlineParserKey("n num"		 , ""			   , "# of something")
	};

	std::string str = bs::createCommandLineParserKeys(keys);

	cv::CommandLineParser p(argc, argv, str);
	p.printMessage();

	return 0;
}
*/


namespace bs
{
	struct CmdlineParserKey
	{
		std::string option;
		std::string default_value_str;
		std::string description;
		bool is_positional;

		CmdlineParserKey(){}

		CmdlineParserKey(const std::string& _option,
			const std::string& _defalt_val,
			const std::string& _description,
			const bool _is_positional = false) :
			default_value_str(_defalt_val), description(_description), is_positional(_is_positional)
		{
			option = (is_positional ? "@" : "") + _option;
		}

		CmdlineParserKey(const std::vector<std::string>& _options,
			const std::string& _defalt_val,
			const std::string& _description,
			const bool _is_positional = false) :
			default_value_str(_defalt_val),
			description(_description), is_positional(_is_positional)
		{
			option = (is_positional ? "@" : "") +
				std::accumulate(_options.cbegin(), _options.cend(), std::string(),
				[](const std::string& a, const std::string& b)
			{ return (a + " " + b); });
		}
	};
}


std::string bs__createOneCommandLineParserKey(const bs::CmdlineParserKey& key, const size_t mop, const size_t mdv, const size_t mdc)
{
	size_t count, spaces;
	std::string dst;
	dst.resize(mop + mdv + mdc + 8);
	auto it = dst.begin();

	// Option
	*it = '{';	++it;
	count = key.option.size();
	spaces = mop - count;
	dst.replace(it, it + count, key.option);	it += count;

	for (size_t i = 0; i < spaces + 1; ++i, ++it)
		*it = ' ';

	// Default value
	*it = '|';	++it;
	count = key.default_value_str.size();
	spaces = mdv - count;

	for (size_t i = 0; i < spaces + 1; ++i, ++it)
		*it = ' ';
	dst.replace(it, it + count, key.default_value_str);	it += count;

	// Description
	*it = '|';	++it;
	*it = ' ';	++it;
	count = key.description.size();
	spaces = mdc - count;
	dst.replace(it, it + count, key.description);	it += count;

	for (size_t i = 0; i < spaces + 1; ++i, ++it)
		*it = ' ';
	*it = '}';	++it;

	assert(it == dst.end());
	return dst;
}


namespace bs
{
	std::string createCommandLineParserKeys(const std::vector<CmdlineParserKey>& keys)
	{
		const size_t max_option_size = std::max_element(keys.cbegin(), keys.cend(),
									   [](const CmdlineParserKey& a, const CmdlineParserKey& b)
									   { return (a.option.size() < b.option.size()); })
			->option.size();

		const size_t max_dval_size = std::max_element(keys.cbegin(), keys.cend(),
									 [](const CmdlineParserKey& a, const CmdlineParserKey& b)
									 { return (a.default_value_str.size() < b.default_value_str.size()); })
			->default_value_str.size();

		const size_t max_descrip_size = std::max_element(keys.cbegin(), keys.cend(),
										[](const CmdlineParserKey& a, const CmdlineParserKey& b)
										{ return (a.description.size() < b.description.size()); })
			->description.size();

		std::string dst;

		for (const auto& key : keys)
			dst += bs__createOneCommandLineParserKey(key, max_option_size, max_dval_size, max_descrip_size);
		return dst;
	}
}



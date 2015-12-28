#pragma once

#pragma warning(disable:4819)
#include <opencv2/core/utility.hpp>

namespace bs
{
	class CmdlineParser : public cv::CommandLineParser
	{
		std::vector<cv::String> keys_;
		size_t numof_maxchars_op_, numof_maxchars_defval_, numfof_maxchars_mess_;

	public:
		CmdlineParser() : numof_maxchars_op_(0), numof_maxchars_defval_(0), numfof_maxchars_mess_(0) {}

		void add(const cv::String& options,
				 const cv::String& default_value = "",
				 const cv::String& message = "",
				 const bool is_positional = false
				 );

		template<typename T>
		void add(const cv::String& options,
				 const T& default_value,
				 const cv::String& message = "",
				 const bool is_positional = false
				 );
	};


	void CmdlineParser::add(const cv::String& options, const cv::String& default_value = "", const cv::String& message = "", const bool is_positional = false)
	{
		cv::CommandLineParser p;
	}


}

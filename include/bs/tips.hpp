#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


namespace bs
{
	class IndexReader
	{
		static std::vector<std::string> split(const std::string& str, const char delim = ',');
		static void replace(std::string& str, const std::string& s1, const std::string& s2);
		static std::vector<int> str2indices(const std::string& str);

	public:
		static std::vector<int> read(const std::string& filename);
	};


	template<typename T = double>
	std::vector<T> linspace(const T a, const T b, size_t n)
	{
		const T h = (b - a) / static_cast<T>(n - 1);
		
		std::vector<T> res(n);
		res[0] = a;

		for (auto& it = res.begin() + 1; it != res.end(); ++it)
			*it = *(it - 1) + h;
		return res;
	}

///////

	inline std::vector<int> IndexReader::read(const std::string& filename)
	{
		std::ifstream ifs(filename);
		if (!ifs.is_open())
		{
			std::cerr << "File cannot open. " << filename << std::endl;
			std::system("pause");
			return std::vector<int>();
		}

		std::string tmp, str;

		while (std::getline(ifs, tmp))
		{
			replace(tmp, " ", "");
			str += tmp + (*tmp.rbegin() == ',' ? "" : ",");
		}
		const auto str_vec = split(str);

		if (str_vec.size() == 1 && str_vec[0] == "-1")
			return std::vector<int>({ -1 });

		std::vector<int> output;
		for (size_t i = 0; i < str_vec.size(); ++i)
		{
			const auto tmp = str2indices(str_vec[i]);
			output.insert(output.end(), tmp.begin(), tmp.end());
		}
		return output;
	}


	inline std::vector<std::string> IndexReader::split(const std::string& str, const char delim)
	{
		std::istringstream iss(str);
		std::string field;
		std::vector<std::string> result_vec;

		while (std::getline(iss, field, delim))
			result_vec.push_back(field);
		return result_vec;
	}


	inline void IndexReader::replace(std::string& str, const std::string& s1, const std::string& s2)
	{
		std::string::size_type pos(str.find(s1));

		while (pos != std::string::npos)
		{
			str.replace(pos, s1.length(), s2);
			pos = str.find(s1, pos + s2.length());
		}
	}


	inline std::vector<int> IndexReader::str2indices(const std::string& str)
	{
		if (str.find('-') == std::string::npos)
			return{ std::stoi(str) };

		std::istringstream iss(str);
		std::string s1, s2;

		std::getline(iss, s1, '-');
		std::getline(iss, s2, '-');

		const int i1 = std::stoi(s1), i2 = std::stoi(s2);
		return linspace<int>(i1, i2, i2 - i1 + 1);
	}
}

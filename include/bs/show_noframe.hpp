#pragma once

#pragma warning(disable:4819)
#include <opencv2/highgui.hpp>
#pragma warning(default:4819)

// CPUのアーキテクチャ設定。
//  'winnt.h'('windef.h'が内部でincludeしている)で必要。
//  普段は'Windows.h'が設定している
#ifdef _M_IX86          // コンパイラが定義しているはず
#define _X86_
#else
#define _AMD64_
#endif

#define NOMINMAX		// マクロ min, max を定義されたくない場合アクティブにする
#include <Windows.h>


namespace bs
{
	inline void showWindowNoframe(cv::InputArray image, const cv::Point2i& window_pos, const std::string& win_name = "projection")
	{
		cv::imshow(win_name, image);
		unsigned int win_flags = (SWP_SHOWWINDOW | SWP_NOSIZE);
		cv::setWindowProperty(win_name, cv::WINDOW_FULLSCREEN, cv::WINDOW_FULLSCREEN);
		HWND win_handle = FindWindowA(0, win_name.c_str());
		SetWindowPos(win_handle, HWND_NOTOPMOST, window_pos.x, window_pos.y, image.getMat().cols, image.getMat().rows, win_flags);
		SetWindowLong(win_handle, GWL_STYLE, GetWindowLong(win_handle, GWL_EXSTYLE) | WS_EX_TOPMOST);
		ShowWindow(win_handle, SW_SHOW);
		return;
	}
}

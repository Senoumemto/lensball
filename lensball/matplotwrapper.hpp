#ifndef MATPLOTLIB_HPP
#define MATPLOTLIB_HPP

#include "general.hpp"

#include <cstdio>

#include <pybind11/pybind11.h>

#include <string>
#include <cstdio>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <memory>
namespace detail
{
    /* C++ 17版 */
    /* std::string型をconst char*に変換し、それ以外はそのまま出力 */
    template<typename T>
    auto Convert(T&& value)
    {
        /* std::string型をconst char*に変換 */
        if constexpr (std::is_same<std::remove_cv_t<std::remove_reference_t<T>>, std::string>::value)
        {
            return std::forward<T>(value).c_str();
        }
        /* std::string型以外は、そのまま出力 */
        else
        {
            return std::forward<T>(value);
        }
    }
    /* 文字列のフォーマッティング(内部処理) */
    template<typename ... Args>
    std::string StringFormatInternal(const std::string& format, Args&& ... args)
    {
        /* フォーマット後の文字数を算出 */
        int str_len = std::snprintf(nullptr, 0, format.c_str(), std::forward<Args>(args) ...);
        /* フォーマット失敗 */
        if (str_len < 0)
        {
            throw std::runtime_error("String Formatting Error");
        }
        else
        {
            /* Nothing to do */
        }
        /* バッファサイズを算出(文字列長 + null文字サイズ) */
        size_t buffer_size = str_len + sizeof(char);
        /* バッファサイズ分メモリ確保 */
        std::unique_ptr<char[]> buffer(new char[buffer_size]);
        /* 文字列のフォーマット */
        std::snprintf(buffer.get(), buffer_size, format.c_str(), args ...);
        /* 文字列をstd::string型に変換して出力 */
        return std::string(buffer.get(), buffer.get() + str_len);
    }
}
/* 文字列のフォーマッティング */
template<typename ... Args>
std::string StringFormat(const std::string& format, Args&& ... args)
{
    /* 各パラメータの型を変換して、文字列のフォーマッティング */
    return detail::StringFormatInternal(format, detail::Convert(std::forward<Args>(args)) ...);
}

class matplotlib {

public:

	bool open(const std::string& command="fig, ax = plt.subplots()\nplt.show(block=False)\n") {
		Py_Initialize();
        send_command("import matplotlib.pyplot as plt");
        send_command("import matplotlib.patches as patches");

		send_command(command.c_str());
		return true;
	}

	bool close() {
		//send_command("plt.close()");
		//send_command("quit()");
        send_command("plt.clf()");
        send_command("plt.close()");
        Py_Finalize();
		return true;
	}

    void line(const uvec2& b,const uvec2& e,const std::string&color="black") {
        send_command(StringFormat("plt.plot([%f,%f],[%f,%f],color=\'%s\')", b.x(),e.x(),b.y(),e.y(),color).c_str());
    }
    void lineUp2d(const up2d& b, const up2d& e, const std::string& color = "black") {
        line(uvec2(b.x(), b.y()), uvec2(e.x(), e.y()), color);
    }

	void save(const char* filename) const {
		send_command(StringFormat("\n", filename).c_str());
	}

	void send_command(const char* s) const {
		PyRun_SimpleString(s);
	}
    void send_import(const char* s) const {
        PyImport_AddModule(s);
    }

	void clear() const {
		send_command("plt.clf()");

	}
    void show() {
        send_command("plt.show()");
    }
    void pause(ureal interval = 1. / 100.) {
        send_command(StringFormat("plt.pause(%f)", interval).c_str());
    }
};

#endif // MATPLOTLIB_HPP
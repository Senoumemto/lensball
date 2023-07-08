#ifndef __INCLUDEGURAD_GENERAL_HPP
#define __INCLUDEGURAD_GENERAL_HPP

#include <array>
#include <iostream>
#include <string>
#include <deque>
#include <vector>
#include <algorithm>
#include <stack>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <chrono>
#include <sstream>
#include <unordered_map>
#include <sstream>
#include <list>

#include <Eigen/Core>
//#include <Eigen/LU>
//#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <numbers>

#define COPY_BAN(objname) objname(const objname&)=delete; objname& operator=(const objname&)=delete

using ureal = double;
using index = int;

using uvec6 = Eigen::Vector<ureal, 6>;

template<size_t SIZ> using uvec = Eigen::Vector<ureal, SIZ>;
using uvec1 = Eigen::Vector<ureal, 1>;
using uvec2 = Eigen::Vector<ureal, 2>;
using uvec3 = Eigen::Vector<ureal, 3>;
using uvec4 = Eigen::Vector<ureal, 4>;


using umat2 = Eigen::Matrix2<ureal>;
using umat3 = Eigen::Matrix3<ureal>;
using umat4 = Eigen::Matrix4<ureal>;

using uvecx = Eigen::VectorX<ureal>;
using umatx = Eigen::MatrixX<ureal>;

using uaffine3 = Eigen::Transform<ureal, 3, 2>;

template<typename T>using sptr = std::shared_ptr<T>;
template<typename T>using uptr = std::unique_ptr<T>;

using ivec2 = Eigen::Vector2i;

//二次元一般化座標 x座標,y座標 theta姿勢 1
using up2d = uvec4;
//片足の関節角度列
using footjoints = uvec3;

constexpr ureal upi = std::numbers::pi;

//変換とその逆変換をペアにしたもの
template<typename T>class bitrans :public std::pair<T, T> {
	using super = std::pair<T, T>;

public:
	bitrans(const T& prograte, const T& untiprog) :super(prograte, untiprog) {}
	bitrans(const T& prograte) :super(prograte, prograte.inverse()) {}

	bitrans() = default;
	bitrans(const bitrans&) = default;

	virtual ~bitrans() {}

	bitrans<T>& operator=(const bitrans<T>&) = default;
	//違う方をメンバにするもの同士のコピー
	template<typename Tright> bitrans<T>& operator=(const bitrans<Tright>& right) {
		this->prograte() = right.prograte();
		this->untiprograte() = right.untiprograte();

		return *this;
	}


	T& prograte() { return this->first; }
	T& untiprograte() { return this->second; }
	const T& prograte()const { return this->first; }
	const T& untiprograte()const { return this->second; }

	bitrans& Set(const T& prog, const T& untip) {
		this->prograte() = prog;
		this->untiprograte() = untip;

		return *this;
	}

};


//範囲比較文 両端を含む
template<typename T>bool isIn(const T& val, const T& min, const T& max) {
	return min >= val && val <= max;
}

//ベクトルを変形する
uvec3 Reshape(const up2d& gen);

constexpr ureal ToRad(const ureal deg) {
	return deg / 180. * upi;
}

template<typename T>uvec2 ReshapeXY(const T& gen) {
	return uvec2(gen.x(), gen.y());
}

//線形補間する
template<typename T>T uleap(const std::pair<T,T>& range,const T& t) {
	return range.first + (range.second - range.first) * t;
}



namespace std {
	template <typename Scalar, int Rows, int Cols>
	struct hash<Eigen::Matrix<Scalar, Rows, Cols>> {
		// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
		size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
			size_t seed = 0;
			for (size_t i = 0; i < static_cast<size_t>(matrix.size()); ++i) {
				Scalar elem = *(matrix.data() + i);
				seed ^=
					std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			}
			return seed;
		}
	};
} // namespace std

#ifdef _DEBUG
//vc上のウォッチリストでeigenオブジェクトを見るための関数
std::string TestWatch(const uvecx& hello);
std::string TestWatch(const umatx& hello);
#endif

#ifndef _MSC_VER 
namespace std {

	template<class _Period>std::string writt() {
		if constexpr (std::is_same_v<_Period, std::milli>) {
			return "ms";
		}
		else if constexpr (std::is_same_v<_Period, std::micro>) {
			return "us";
		}

		std::stringstream ss;
		ss << " [" << _Period::num << "/" << _Period::den << "s]";
		return ss.str();
	}

	template <class _CharT, class _Traits, class _Rep, class _Period>
	basic_ostream<_CharT, _Traits>& operator<<(
		basic_ostream<_CharT, _Traits>& _Os, const std::chrono::duration<_Rep, _Period>& _Dur) {
		basic_ostringstream<_CharT, _Traits> _Sstr;
		_Sstr.flags(_Os.flags());
		_Sstr.imbue(_Os.getloc());
		_Sstr.precision(_Os.precision());
		_Sstr << _Dur.count();

		_Period hei;
		_Sstr << writt<_Period>();
		//_Write_unit_suffix<_Period>(_Sstr);

		return _Os << _Sstr.str();
	}

};
#endif

//なんかセクションから抜けるときに使う例外　とくにエラーではない
class escapeException :public std::exception {

};


template < typename T > constexpr T sqrt_constexpr(T s) {
	T x = s / 2.0;
	T prev = 0.0;

	while (x != prev)
	{
		prev = x;
		x = (x + s / x) / 2.0;
	}
	return x;
}

//この変数を使うときに必ずmutexをロックするようになる 所有権もこいつが持ってるよ
template<typename VAL>class mutexedVariant :public std::mutex {
private:
	using super = std::mutex;
	std::unique_ptr<VAL> val;
public:
	mutexedVariant() :super(), val(new VAL){}
	mutexedVariant(const VAL&& v) :super() {
		val = make_unique(v);
	}

	std::unique_ptr<VAL>& GetAndLock() {
		this->lock();

		return val;
	}
};

#endif
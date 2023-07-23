#pragma once

#include <iostream>
#include "matplotwrapper.hpp"
#include "general.hpp"


#include <fstream>

#define CEREAL_THREAD_SAFE 1
#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/list.hpp>
#include <cereal/types/unordered_map.hpp>

//plotterの準備関数
uptr<matplotlib> SetupPythonRuntime();
void DefinePythonFunctions(uptr<matplotlib>& plt);


//描画系
void DrawSphere(uptr<matplotlib>& plt, uvec3 center, ureal r, int splitnum = 10, const std::string& color = "\"red\"", ureal alpha = 1.);
void DrawLine(uptr<matplotlib>& plt, const uvec3& a, const uvec3& b, const std::string& color = R"("red")");


//レンズボールの計算
using sphereParam = std::pair<uvec3, ureal>;
uptr<std::list<sphereParam>> CalcSmallLensPosAndRadius();

//レイトレーシング系
//始点と方向を持つ半直線
template<size_t SIZ>class arrow :public std::pair<uvec<SIZ>, uvec<SIZ>> {
private:
	using super = std::pair<uvec<SIZ>, uvec<SIZ>>;

public:
	//始点
	decltype(arrow::first)& org() { return this->first; }
	const decltype(arrow::first)& org()const { return this->first; }
	//終点
	decltype(arrow::second)& dir() { return this->second; }
	const decltype(arrow::second)& dir() const { return this->second; }

	//org dirの順で初期化
	arrow() = default;
	arrow(const decltype(arrow::first)& orign, const decltype(arrow::second)& direction) :super(orign, direction) {}
	arrow(const arrow& val) = default;

	arrow& operator=(const arrow& val) {
		this->org() = val.org();
		this->dir() = val.dir();

		return *this;
	}


	template<size_t CASTSIZ> operator arrow<CASTSIZ>() const {

		arrow<CASTSIZ> ret;
		ret.org() = (decltype(ret.org()))(this->org());
		ret.dir() = (decltype(ret.org()))(this->dir());

		if constexpr (SIZ < CASTSIZ) {
			//zero埋めする
			for (index i = SIZ; i < CASTSIZ; i++) {
				ret.org()(i) = 0.;
				ret.dir()(i) = 0.;
			}
		}

		return ret;
	}

	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) const{
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->org()[d]);
		}
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->dir()[d]);
		}

		//archive(this->org().x(), this->org().y(), this->org().z(),
		//this->dir().x(), this->dir().y(), this->dir().z());
	}
	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) {
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->org()[d]);
		}
		for (size_t d = 0; d < SIZ; d++) {
			archive(this->dir()[d]);
		}
		//archive(this->org().x(), this->org().y(), this->org().z(),
		//this->dir().x(), this->dir().y(), this->dir().z());
	}
};
using arrow2 = arrow<2>;
using arrow3 = arrow<3>;
using arrow4 = arrow<4>;

constexpr inline ureal pi = std::numbers::pi;//円周率のエイリアス

//半直線をつなげたらレイになる、一連の折れ線を計算
template<size_t SIZ>class ray:public std::list<arrow<SIZ>> {
	using super = std::list<arrow<SIZ>>;

public:
	ray() = default;
	ray(const arrow<SIZ>& first) {
		this->push_back(first);
	}
	ray(const ray<SIZ>& left) = default;

};
using ray3 = ray<3>;

//自由飛行させる t時間だけ
template<size_t SIZ>ray<SIZ> FreeFlightRay(ray<SIZ>& target, ureal time = 10.) {
	//新たな点
	arrow<SIZ> newone = target.back();
	newone.org() += newone.dir() * time;

	target.push_back(newone);
	return target;
}

//直線を描画
void DrawRay(uptr<matplotlib>& plt, const ray3& target, const std::string& color = R"("red")");
void DrawRaySkipFirstArrow(uptr<matplotlib>& plt, const ray3& target, const std::string& color = R"("red")");

//球とarrowの交差判定
struct resultIntersecteSphere {
	bool isHit;

	uvec3 pos;
	uvec3 norm;
	ureal t;//判定を行ったarrowでのt

	resultIntersecteSphere();
	resultIntersecteSphere(const uvec3& pos, const uvec3& norm,const ureal t);
	ray3& ApplyToRay(ray3& target)const;
};
resultIntersecteSphere IntersectSphere(const arrow3& rayback, const uvec3& c, const ureal r);
resultIntersecteSphere __IntersectSphere_GetFarOne_TEMPORARYONE(const arrow3& rayback, const uvec3& c, const ureal r);

//鏡面反射する
ray3& ReflectMirror(ray3& target, const uvec3& norm);
//屈折する
bool RefractSnell(ray3& target, const uvec3& norm, const ureal eta);


//連番gifを作る
void MakeGifAnim(const std::string& palletfile, const std::string& outputfile, const std::string& inputfile, const size_t fps);


std::array<ureal, 3> RgbToHsv(const std::array<ureal, 3>& rgb);

std::array<ureal, 3> HsvToRgb(const std::array<ureal, 3>& hsv);

template<typename T> std::pair<T,T> PairMinusPlus(const T& t) {
	return std::make_pair(-t, +t);
}

//座標系関係の関数
//球面極座標を直交座標に変換 //
uvec3 PolarToXyz(const uvec2& spolar);
uvec2 XyzToPolar(const uvec3& xyz);
uvec3 Polar3DToXyz(const uvec3& phiThetaRadius);
uvec2 MapToLocalPolar(const uvec2& xy);
uvec2 PolarToMap(const uvec2& xy);


//pythonでベクトル系列を管理する

//系列を表す 次元と名前
template<size_t DIM>class pyVecSeries :public std::string {
	using super = std::string;

public:

};

//系列リスト
template<size_t D>using prefixlist = std::optional<const std::reference_wrapper<const std::array<const std::string, D>>>;

//系列名を作成する
template<size_t D> std::string GetSeriesPrefix(const unsigned char& id, const prefixlist<D> prefixList) {
	if (prefixList) {
		const std::array<const std::string, D>& ref = prefixList.value();
		if (id < ref.size())return ref.at(id);
	}
	else if (id < 3)return { (char)('x' + id) };

	throw std::runtime_error("This function(GetSeriesPrefix) arrows id be 0~2.");
}
//pythonでベクトル系列を作成する
template<size_t D>void ResetPyVecSeries(const pyVecSeries<D>& vecname, const prefixlist<D> prefix = std::nullopt) {
	std::string ret;
	for (size_t i = 0; i < D; i++)
		ret += StringFormat("%s%s=[]\n", vecname.c_str(), GetSeriesPrefix<D>(i, prefix));

	pythonRuntime::s(ret);
}
//pythonにベクトルをappendする
template<typename VEC, size_t D = VEC::RowsAtCompileTime>void AppendPyVecSeries(const pyVecSeries<D>& vecname, const VEC& vec, const prefixlist<D> prefix = std::nullopt) {
	std::string ret;
	for (size_t i = 0; i < D; i++)
		ret += StringFormat("%s%s.append(%f)\n", vecname.c_str(), GetSeriesPrefix<D>(i, prefix), vec[i]);

	pythonRuntime::s(ret);
}
//pythonでベクトル系列を,で列挙したものを得る 要素が0のベクトルだとバグる
template<size_t D>std::string GetPySeriesForPlot(const pyVecSeries<D>& vecname, const prefixlist<D> prefix = std::nullopt) {
	std::string ret = StringFormat("%s%s", vecname.c_str(), GetSeriesPrefix<D>(0, prefix));
	for (size_t i = 1; i < D; i++)
		ret += StringFormat(",%s%s", vecname.c_str(), GetSeriesPrefix<D>(i, prefix));

	return ret;
}

//二編幅から六角形を作る
std::list<uvec2> MakeHexagon(const ureal& edgeWidth);


//projectorRefractionDicのヘッダ構造
class projRefraDicHeader {
public:
	size_t horizontalRes;//水平分解能　ピクセル数
	size_t verticalRes;//垂直分解能　ピクセル数
	size_t rotationRes;//回転分解能t つまり一周に何回投影するか　つまり分散数
	

	projRefraDicHeader(const size_t& hRes, const size_t& vRes, const size_t& rotRes) :
		rotationRes(rotRes),verticalRes(vRes),horizontalRes(hRes){}
	projRefraDicHeader() :horizontalRes(0), verticalRes(0), rotationRes(0){}

	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) const{
		archive(this->horizontalRes, this->verticalRes, this->rotationRes);
	}
	template<class Archive> void serialize(Archive& archive) {
		archive(this->horizontalRes, this->verticalRes, this->rotationRes);
	}

	//ヘッダファイルを保存
	void SaveHeader(const std::string& path) const{
		std::ofstream ofs(path + ".head");
		cereal::BinaryOutputArchive o_archive(ofs);

		o_archive((projRefraDicHeader&)(*this));

		return;
	}
};

//std::pairをハッシュする
struct HashPair {

	static size_t m_hash_pair_random;

	template<class T1, class T2>
	size_t operator()(const std::pair<T1, T2>& p) const {

		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);

		size_t seed = 0;
		seed ^= hash1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hash2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= 0x9e3779b9 + (seed << 6) + (seed >> 2);
		return seed;
	}
};

//環境によって設定を変える
struct appContext{
	int version;//コンテキストファイルのバージョン　違うと怒られる
	size_t threadNumMax;//マルチスレッドするときに使えるスレッド数の最大
	std::string rezpath;//結果パス　ブランチパスの親になる
	std::list<std::string> defaultArg;//デフォルト引数

	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) const {
		archive(CEREAL_NVP(version), CEREAL_NVP(threadNumMax), CEREAL_NVP(rezpath), CEREAL_NVP(defaultArg));
	}
	template<class Archive> void serialize(Archive& archive) {
		archive(CEREAL_NVP(version), CEREAL_NVP(threadNumMax), CEREAL_NVP(rezpath), CEREAL_NVP(defaultArg));
	}

	//テスト用に適当な内容のコンテキストを出力する
	static void WriteDammyContext(const std::string appContextPath);

	appContext():version(0), threadNumMax(0){}
};


//ある半直線(ray)と線分(line)の当たり判定
bool IntersectLineAndWay(const std::pair<uvec2, uvec2>& line, const arrow2& ray);
bool NaihouHanteiX(const uvec2& p, const std::list<uvec2>& vs);


//2dベクトルに要素を加える
uvec3 ExtendUvec2(const uvec2& v, const ureal& z);
//arrowをvec6に変える
uvec6 ArrowToUVec6(const arrow<3>& v);

//角度を正規化する
ureal NormalizeAngle(ureal Angle);

//いろいろなものを正規化する　整数番
template<typename intger>intger NormalizeIntger(const intger& i, const intger& siz) {
	//正ならmodすればよし
	if (i > 0)return i % siz;
	if (i % siz == 0)return 0;//sizの整数倍なら絶対0
	else return siz + (i % siz);//負なら全体から引けば良い
}

//0~始まるインデックスを、ある中心から両側に検索するような形に変換する
size_t GetBisideIndex(size_t lini, size_t center, int way, const size_t indSiz);

//レイを棒で描画する
void PlotRayInMlab(const arrow3& ray, const std::string& prefix);

std::optional<ureal> IntersectArrowAndElipsoid(const arrow3& ray, const uvec3& radius);


namespace cereal
{
	template <class Archive, class Derived> inline
		typename std::enable_if<traits::is_output_serializable<BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
		save(Archive& ar, Eigen::PlainObjectBase<Derived> const& m) {
		typedef Eigen::PlainObjectBase<Derived> ArrT;
		if (ArrT::RowsAtCompileTime == Eigen::Dynamic) ar(m.rows());
		if (ArrT::ColsAtCompileTime == Eigen::Dynamic) ar(m.cols());
		ar(binary_data(m.data(), m.size() * sizeof(typename Derived::Scalar)));
	}

	template <class Archive, class Derived> inline
		typename std::enable_if<traits::is_input_serializable<BinaryData<typename Derived::Scalar>, Archive>::value, void>::type
		load(Archive& ar, Eigen::PlainObjectBase<Derived>& m) {
		typedef Eigen::PlainObjectBase<Derived> ArrT;
		Eigen::Index rows = ArrT::RowsAtCompileTime, cols = ArrT::ColsAtCompileTime;
		if (rows == Eigen::Dynamic) ar(rows);
		if (cols == Eigen::Dynamic) ar(cols);
		m.resize(rows, cols);
		ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(typename Derived::Scalar))));
	}
}

class developResult {

public:
	std::unordered_map<ivec2, uvec3> colorList;//カメラの受光素子ごとの色の合計
	std::unordered_map<ivec2, ureal> colorSiz;//受光素子ごとの入射量


	//シリアライズできるように
	template<class Archive> void serialize(Archive& archive) const {
		archive(colorList);
		archive(colorSiz);
	}
	template<class Archive> void serialize(Archive& archive) {
		archive(colorList);
		archive(colorSiz);
	}
};
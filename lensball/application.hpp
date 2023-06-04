#pragma once

#include <iostream>
#include "matplotwrapper.hpp"
#include "general.hpp"

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
};
using arrow2 = arrow<2>;
using arrow3 = arrow<3>;
using arrow4 = arrow<4>;

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

//鏡面反射する
ray3& ReflectMirror(ray3& target, const uvec3& norm);
//屈折する
bool RefractSnell(ray3& target, const uvec3& norm, const ureal eta);


//連番gifを作る
void MakeGifAnim(const std::string& palletfile, const std::string& outputfile, const std::string& inputfile, const size_t fps);

//マイナス　プラスのペアを作る
template<typename R>std::pair<R, R> PairMinusPlus(const R& val) { return std::make_pair(-val, val); }

std::array<ureal, 3> RgbToHsv(const std::array<ureal, 3>& rgb);

std::array<ureal, 3> HsvToRgb(const std::array<ureal, 3>& hsv);
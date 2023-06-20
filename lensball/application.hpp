#pragma once

#include <iostream>
#include "matplotwrapper.hpp"
#include "general.hpp"

//plotter�̏����֐�
uptr<matplotlib> SetupPythonRuntime();
void DefinePythonFunctions(uptr<matplotlib>& plt);


//�`��n
void DrawSphere(uptr<matplotlib>& plt, uvec3 center, ureal r, int splitnum = 10, const std::string& color = "\"red\"", ureal alpha = 1.);
void DrawLine(uptr<matplotlib>& plt, const uvec3& a, const uvec3& b, const std::string& color = R"("red")");


//�����Y�{�[���̌v�Z
using sphereParam = std::pair<uvec3, ureal>;
uptr<std::list<sphereParam>> CalcSmallLensPosAndRadius();

//���C�g���[�V���O�n
//�n�_�ƕ�������������
template<size_t SIZ>class arrow :public std::pair<uvec<SIZ>, uvec<SIZ>> {
private:
	using super = std::pair<uvec<SIZ>, uvec<SIZ>>;

public:
	//�n�_
	decltype(arrow::first)& org() { return this->first; }
	const decltype(arrow::first)& org()const { return this->first; }
	//�I�_
	decltype(arrow::second)& dir() { return this->second; }
	const decltype(arrow::second)& dir() const { return this->second; }

	//org dir�̏��ŏ�����
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
			//zero���߂���
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

constexpr inline ureal pi = std::numbers::pi;//�~�����̃G�C���A�X

//���������Ȃ����烌�C�ɂȂ�A��A�̐܂�����v�Z
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

//���R��s������ t���Ԃ���
template<size_t SIZ>ray<SIZ> FreeFlightRay(ray<SIZ>& target, ureal time = 10.) {
	//�V���ȓ_
	arrow<SIZ> newone = target.back();
	newone.org() += newone.dir() * time;

	target.push_back(newone);
	return target;
}

//������`��
void DrawRay(uptr<matplotlib>& plt, const ray3& target, const std::string& color = R"("red")");
void DrawRaySkipFirstArrow(uptr<matplotlib>& plt, const ray3& target, const std::string& color = R"("red")");

//����arrow�̌�������
struct resultIntersecteSphere {
	bool isHit;

	uvec3 pos;
	uvec3 norm;
	ureal t;//������s����arrow�ł�t

	resultIntersecteSphere();
	resultIntersecteSphere(const uvec3& pos, const uvec3& norm,const ureal t);
	ray3& ApplyToRay(ray3& target)const;
};
resultIntersecteSphere IntersectSphere(const arrow3& rayback, const uvec3& c, const ureal r);

//���ʔ��˂���
ray3& ReflectMirror(ray3& target, const uvec3& norm);
//���܂���
bool RefractSnell(ray3& target, const uvec3& norm, const ureal eta);


//�A��gif�����
void MakeGifAnim(const std::string& palletfile, const std::string& outputfile, const std::string& inputfile, const size_t fps);


std::array<ureal, 3> RgbToHsv(const std::array<ureal, 3>& rgb);

std::array<ureal, 3> HsvToRgb(const std::array<ureal, 3>& hsv);

template<typename T> std::pair<T,T> PairMinusPlus(const T& t) {
	return std::make_pair(-t, +t);
}

//���W�n�֌W�̊֐�
//���ʋɍ��W�𒼌����W�ɕϊ� //
uvec3 PolarToXyz(const uvec2& spolar);
uvec3 Polar3DToXyz(const uvec3& phiThetaRadius);
uvec2 MapToLocalPolar(const uvec2& xy);
uvec2 PolarToMap(const uvec2& xy);


//python�Ńx�N�g���n����Ǘ�����

//�n���\�� �����Ɩ��O
template<size_t DIM>class pyVecSeries :public std::string {
	using super = std::string;

public:

};

//�n�񃊃X�g
template<size_t D>using prefixlist = std::optional<const std::reference_wrapper<const std::array<const std::string, D>>>;

//�n�񖼂��쐬����
template<size_t D> std::string GetSeriesPrefix(const unsigned char& id, const prefixlist<D> prefixList) {
	if (prefixList) {
		const std::array<const std::string, D>& ref = prefixList.value();
		if (id < ref.size())return ref.at(id);
	}
	else if (id < 3)return { (char)('x' + id) };

	throw std::runtime_error("This function(GetSeriesPrefix) arrows id be 0~2.");
}
//python�Ńx�N�g���n����쐬����
template<size_t D>void ResetPyVecSeries(const pyVecSeries<D>& vecname, const prefixlist<D> prefix = std::nullopt) {
	std::string ret;
	for (size_t i = 0; i < D; i++)
		ret += StringFormat("%s%s=[]\n", vecname.c_str(), GetSeriesPrefix<D>(i, prefix));

	pythonRuntime::s(ret);
}
//python�Ƀx�N�g����append����
template<typename VEC, size_t D = VEC::RowsAtCompileTime>void AppendPyVecSeries(const pyVecSeries<D>& vecname, const VEC& vec, const prefixlist<D> prefix = std::nullopt) {
	std::string ret;
	for (size_t i = 0; i < D; i++)
		ret += StringFormat("%s%s.append(%f)\n", vecname.c_str(), GetSeriesPrefix<D>(i, prefix), vec[i]);

	pythonRuntime::s(ret);
}
//python�Ńx�N�g���n���,�ŗ񋓂������̂𓾂� �v�f��0�̃x�N�g�����ƃo�O��
template<size_t D>std::string GetPySeriesForPlot(const pyVecSeries<D>& vecname, const prefixlist<D> prefix = std::nullopt) {
	std::string ret = StringFormat("%s%s", vecname.c_str(), GetSeriesPrefix<D>(0, prefix));
	for (size_t i = 1; i < D; i++)
		ret += StringFormat(",%s%s", vecname.c_str(), GetSeriesPrefix<D>(i, prefix));

	return ret;
}

//��ҕ�����Z�p�`�����
std::list<uvec2> MakeHexagon(const ureal& edgeWidth);








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

	//�V���A���C�Y�ł���悤��
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
	//�V���A���C�Y�ł���悤��
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
resultIntersecteSphere __IntersectSphere_GetFarOne_TEMPORARYONE(const arrow3& rayback, const uvec3& c, const ureal r);

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
uvec2 XyzToPolar(const uvec3& xyz);
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


//projectorRefractionDic�̃w�b�_�\��
class projRefraDicHeader {
public:
	size_t horizontalRes;//��������\�@�s�N�Z����
	size_t verticalRes;//��������\�@�s�N�Z����
	size_t rotationRes;//��]����\t �܂����ɉ��񓊉e���邩�@�܂蕪�U��
	

	projRefraDicHeader(const size_t& hRes, const size_t& vRes, const size_t& rotRes) :
		rotationRes(rotRes),verticalRes(vRes),horizontalRes(hRes){}
	projRefraDicHeader() :horizontalRes(0), verticalRes(0), rotationRes(0){}

	//�V���A���C�Y�ł���悤��
	template<class Archive> void serialize(Archive& archive) const{
		archive(this->horizontalRes, this->verticalRes, this->rotationRes);
	}
	template<class Archive> void serialize(Archive& archive) {
		archive(this->horizontalRes, this->verticalRes, this->rotationRes);
	}

	//�w�b�_�t�@�C����ۑ�
	void SaveHeader(const std::string& path) const{
		std::ofstream ofs(path + ".head");
		cereal::BinaryOutputArchive o_archive(ofs);

		o_archive((projRefraDicHeader&)(*this));

		return;
	}
};

//std::pair���n�b�V������
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

//���ɂ���Đݒ��ς���
struct appContext{
	int version;//�R���e�L�X�g�t�@�C���̃o�[�W�����@�Ⴄ�Ɠ{����
	size_t threadNumMax;//�}���`�X���b�h����Ƃ��Ɏg����X���b�h���̍ő�
	std::string rezpath;//���ʃp�X�@�u�����`�p�X�̐e�ɂȂ�
	std::list<std::string> defaultArg;//�f�t�H���g����

	//�V���A���C�Y�ł���悤��
	template<class Archive> void serialize(Archive& archive) const {
		archive(CEREAL_NVP(version), CEREAL_NVP(threadNumMax), CEREAL_NVP(rezpath), CEREAL_NVP(defaultArg));
	}
	template<class Archive> void serialize(Archive& archive) {
		archive(CEREAL_NVP(version), CEREAL_NVP(threadNumMax), CEREAL_NVP(rezpath), CEREAL_NVP(defaultArg));
	}

	//�e�X�g�p�ɓK���ȓ��e�̃R���e�L�X�g���o�͂���
	static void WriteDammyContext(const std::string appContextPath);

	appContext():version(0), threadNumMax(0){}
};


//���锼����(ray)�Ɛ���(line)�̓����蔻��
bool IntersectLineAndWay(const std::pair<uvec2, uvec2>& line, const arrow2& ray);
bool NaihouHanteiX(const uvec2& p, const std::list<uvec2>& vs);


//2d�x�N�g���ɗv�f��������
uvec3 ExtendUvec2(const uvec2& v, const ureal& z);
//arrow��vec6�ɕς���
uvec6 ArrowToUVec6(const arrow<3>& v);

//�p�x�𐳋K������
ureal NormalizeAngle(ureal Angle);

//���낢��Ȃ��̂𐳋K������@������
template<typename intger>intger NormalizeIntger(const intger& i, const intger& siz) {
	//���Ȃ�mod����΂悵
	if (i > 0)return i % siz;
	if (i % siz == 0)return 0;//siz�̐����{�Ȃ���0
	else return siz + (i % siz);//���Ȃ�S�̂�������Ηǂ�
}

//0~�n�܂�C���f�b�N�X���A���钆�S���痼���Ɍ�������悤�Ȍ`�ɕϊ�����
size_t GetBisideIndex(size_t lini, size_t center, int way, const size_t indSiz);

//���C��_�ŕ`�悷��
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
	std::unordered_map<ivec2, uvec3> colorList;//�J�����̎���f�q���Ƃ̐F�̍��v
	std::unordered_map<ivec2, ureal> colorSiz;//����f�q���Ƃ̓��˗�


	//�V���A���C�Y�ł���悤��
	template<class Archive> void serialize(Archive& archive) const {
		archive(colorList);
		archive(colorSiz);
	}
	template<class Archive> void serialize(Archive& archive) {
		archive(colorList);
		archive(colorSiz);
	}
};
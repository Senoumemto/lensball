#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "SphereCoord/";//����branch�̌��ʂ��i�[����t�H���_

using py = pythonRuntime;

constexpr inline ureal pi = std::numbers::pi;//�~�����̃G�C���A�X

//�ɍ��W�ɂ�...�ꎟ���̃v���W�F�N�^������o���ꂽ���C(phi=0��)��phi�ŉ�]���鋅�̌����O��
uvec2 RayHitPath(const ureal rayTheta, const ureal& t) {
	return uvec2(rayTheta, t);
}

//�v�f�����Y�̍L�����lsize(�P�� theta)�Ƃ����Ƃ��̃����Y�̃A���C�������g
uvec2 LensAlignment(const ureal& t, const ureal& lsize, const ureal rayTheta) {
	const ureal theta = -(lsize) / (2. * std::numbers::pi) * t + rayTheta;
	return uvec2(theta, t);
}

//���ʋɍ��W�𒼌����W�ɕϊ� //
uvec3 PolarToXyz(const uvec2& spolar) {
	return uvec3(cos(spolar.y()) * cos(spolar.x()),
		cos(spolar.y()) * sin(spolar.x()),
		sin(spolar.y()));
}
uvec3 Polar3DToXyz(const uvec3& phiThetaRadius) {
	return uvec3(phiThetaRadius.z() * cos(phiThetaRadius.y()) * cos(phiThetaRadius.x()),
		phiThetaRadius.z() * cos(phiThetaRadius.y()) * sin(phiThetaRadius.x()),
		phiThetaRadius.z() * sin(phiThetaRadius.y()));
}

uvec2 MapToPolar(const uvec2& xy) {
	return uvec2(xy.x(), 2. * atan(-pow(std::numbers::e, -xy.y())) + pi / 2.);
}
uvec2 PolarToMap(const uvec2& xy) {
	return uvec2(xy.x(), -log(abs(tan(xy.y() / 2. - pi / 4.))));
}

//�n���\�� �����Ɩ��O
template<size_t DIM>class pyVecSeries :public std::string {
	using super = std::string;
	
public:

};

//�n�񖼂��쐬����
template<size_t D> std::string GetSeriesPrefix(const unsigned char& id, std::optional<const std::reference_wrapper<std::array<const std::string, D>>>& prefixList) {
	if (prefixList) {
		std::array<const std::string, D>& ref = prefixList.value();
		if (id < ref.size())return ref.at(id);
	}
	else if (id < 3)return { (char)('x' + id) };
	
	throw runtime_error("This function(GetSeriesPrefix) arrows id be 0~2.");
}
//python�Ńx�N�g���n����쐬����
template<size_t D>void ResetPyVecSeries(const pyVecSeries<D>& vecname, std::optional<const std::reference_wrapper<std::array<const std::string, D>>> prefix = std::nullopt) {
	std::string ret;
	for (size_t i = 0; i < D; i++)
		ret += StringFormat("%s%s=[]\n", vecname.c_str(), GetSeriesPrefix<D>(i, prefix));
	
	py::s(ret);
}
//python�Ƀx�N�g����append����
template<typename VEC,size_t D=VEC::RowsAtCompileTime>void AppendPyVecSeries(const pyVecSeries<D>& vecname,const VEC& vec,std::optional<const std::reference_wrapper<std::array<const std::string, D>>> prefix = std::nullopt) {
	std::string ret;
	for (size_t i = 0; i < D; i++)
		ret += StringFormat("%s%s.append(%f)\n", vecname.c_str(), GetSeriesPrefix<D>(i, prefix), vec[i]);

	py::s(ret);
}
//python�Ńx�N�g���n���,�ŗ񋓂������̂𓾂� �v�f��0�̃x�N�g�����ƃo�O��
template<size_t D>std::string GetPySeriesForPlot(const pyVecSeries<D>& vecname, std::optional<const std::reference_wrapper<std::array<const std::string, D>>> prefix = std::nullopt) {
	std::string ret=StringFormat("%s%s", vecname.c_str(), GetSeriesPrefix<D>(0, prefix));
	for (size_t i = 1; i < D; i++)
		ret += StringFormat(",%s%s", vecname.c_str(), GetSeriesPrefix<D>(i, prefix));

	return ret;
}
//��ҕ�����Z�p�`�����
std::list<uvec2> MakeHexagon(const ureal& edgeWidth) {
	//�O�ڋ��̔��a���o������
	const ureal radius = 2.*edgeWidth / sqrt(3.);
	std::list<uvec2> ret;

	//�_��������Əo���Ă���
	for (size_t i = 0; i < 6; i++) {
		const auto t = uleap(PairMinusPlus(pi), i / 6.);

		const uvec2 pos = radius * uvec2(sin(t), cos(t));
		ret.push_back(pos);
	}

	return ret;
}

int main() {

	try {

		//python�����^�C�����������Ă��낢�돉������
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

		//mayavi�̐ݒ�
		const std::pair<size_t, size_t> figResolution(800, 600);
		py::sf("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

		//matplotlib�̐ݒ�
		py::s("fig, ax = plt.subplots()\nax.set_aspect(\"equal\")");

		//����`�悷��
		constexpr ureal sphereRadius = 1.;
		constexpr size_t sphereResolution = 20;
		py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
x = np.cos(sphphi)*np.sin(sphtheta)
y = np.sin(sphphi)*np.sin(sphtheta)
z = np.cos(sphtheta)
mlab.mesh(%f*x, %f*y, %f*z ,color=(1.,1.,1.) )  
)", sphereResolution, sphereResolution, sphereRadius, sphereRadius, sphereRadius);


		//python plt�p�̃x�N�g���n��
		const pyVecSeries<3> mlabSeries("mlabv");//mlab�v���b�g�p�Ɏg��3D�x�N�g���z��
		const pyVecSeries<2> pypltSeries("pypltv");//�񎟌��v���b�g�p

		//�����Y�A���C���쐬
		//�Z�p�`�Ń^�C�����O����@�����s�������Ă����s���������Ċ���

		constexpr size_t lensNumInCollum = 20;

		constexpr ureal rowAngle = 0.04331481 * 2.;//�s�̊p�x
		const ureal rowLength = 2. * pi * cos(rowAngle);//�s�̒���
		const ureal lensEdgeWidth = rowLength / (ureal)lensNumInCollum / 2.;
		const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInCollum;//�Z�p�`�̈�ς����V�t�g����
		const Eigen::Rotation2D<ureal> localPlaneToGrobal(rowAngle);

		const ureal nodeLensRadius = 2. * lensEdgeWidth / sqrt(3.);//�v�f�����Y�`����쐬�@���̒��a
		constexpr size_t nodeLensResolution = 10;//�v�f�����Y�̕�����
		constexpr size_t rowNum = 15;//��ɂ��Ă�
		for (std::decay<decltype(rowNum)>::type rd = 0; rd < rowNum; rd++) {
			const ureal tlati = eachRowsDistance * rd-(eachRowsDistance*(ureal)(rowNum-1)/2.);//lati�����̌��݈ʒu
			const bool eachFlag = rd % 2;//���݂ɐ؂�ւ��t���O
			for (std::decay<decltype(lensNumInCollum)>::type ld = 0; ld < lensNumInCollum; ld++) {
				//�Z�p�`�����߂�o�b�t�@���N���A
				ResetPyVecSeries(pypltSeries);
				ResetPyVecSeries(mlabSeries);

				const ureal tlonn = uleap(PairMinusPlus(rowLength/2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);//lonn�����̌��݈ʒu
				auto hexvertices = MakeHexagon(lensEdgeWidth);//�Z�p�`�̒��_

				//���_��]�����ĕ`��
				hexvertices.push_back(hexvertices.front());//������邽�߂ɍŏ��̓_�𖖔��ɑ}��
				for (auto v : hexvertices) {
					const auto localHeight=sqrt(1.-pow(v.norm()/nodeLensRadius,2.))*nodeLensRadius;//���[�J�����W�ō��������߂�
					v = localPlaneToGrobal * (v + uvec2(tlonn, tlati));
					AppendPyVecSeries(pypltSeries, v);
					const auto polarpos = MapToPolar(v);//���ɋɍ��W�𓾂�
					AppendPyVecSeries(mlabSeries, Polar3DToXyz(uvec3(polarpos.x(), polarpos.y(), sphereRadius + localHeight)));
				}

				auto color = HsvToRgb({ uleap({0.,1.},ld / (ureal)lensNumInCollum),1.,0.5 });
				py::sf("plt.plot(%s,color=(0,0,0))", GetPySeriesForPlot(pypltSeries));
				py::sf("mlab.plot3d(%s,color=(%f,%f,%f),tube_radius=0.01)", GetPySeriesForPlot(mlabSeries), color[0], color[1], color[2]);
			}
		}


		//�X�L����������
		constexpr size_t projectorResInTheta = 40;
		constexpr ureal projectorHalfAngle = 60. / 180. * pi;
		constexpr size_t scanLineResolutionPhi = 180;
		for (std::decay<decltype(projectorResInTheta)>::type sd = 0; sd < projectorResInTheta; sd++) {
			//�X�L�������C���̍���
			const ureal scanTheta = uleap(PairMinusPlus(projectorHalfAngle), sd / (ureal)(projectorResInTheta - 1));

			//���C����`�悷��
			ResetPyVecSeries(mlabSeries);//x y z
			ResetPyVecSeries(pypltSeries);
			for (std::decay<decltype(scanLineResolutionPhi)>::type rd = 0; rd < scanLineResolutionPhi; rd++) {
				const ureal time = uleap(PairMinusPlus(pi), rd / (ureal)(scanLineResolutionPhi - 1));

				uvec2 mapped = PolarToMap(uvec2(time, scanTheta));
				AppendPyVecSeries(pypltSeries, mapped);

				//������ǂ��}�b�v���邩�@�ɍ��W�n�œn���΂�������
				const auto polarpos = PolarToXyz(mapped);
				AppendPyVecSeries(mlabSeries, polarpos);
			}

			//plt
			auto color = HsvToRgb({ uleap({0.,1.},sd / (ureal)projectorResInTheta),1.,0.5 });
			py::sf("plt.plot(%s,color=(%f,%f,%f))", GetPySeriesForPlot(pypltSeries), color[0], color[1], color[2]);
			//py::sf("mlab.plot3d(%s,color=(%f,%f,%f),tube_radius=0.01)", GetPySeriesForPlot(mlabSeries), color[0], color[1], color[2]);
		}

		//�\������ 3d 2d�̏�
		py::s("plt.show()");
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		py::Terminate();
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		py::Terminate();
		return -2;
	}


	py::Terminate();
	return 0;
}
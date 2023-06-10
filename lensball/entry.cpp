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
	return uvec3(cos(spolar.x()) * cos(spolar.y()),
		cos(spolar.x()) * sin(spolar.y()),
		sin(spolar.x()));
}
uvec3 PolarToXyz(const ureal phi,const ureal theta) {
	return PolarToXyz(uvec2(theta, phi));
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

		//�����Y�A���C���쐬
		//�c����ɍl����@�񂪍����Ă��炵�Ȃ���`�悵�Ă���
		constexpr size_t lensNumInCollum = 10;
		constexpr size_t collumNum = lensNumInCollum * 2;//�͈͓I�ɔ{
		constexpr ureal lensRadiusInMap = pi / (ureal)lensNumInCollum/2.;//�n�}��ł̃����Y�̌o(�p�x)
		constexpr ureal shiftSizeEachCollum = lensRadiusInMap * 2.;//�񂲂Ƃɂǂꂾ���ʒu���V�t�g�����邩

		constexpr size_t lensResolution = 20;//�����Y�̊O�`�~�̉𑜓x

		for (std::decay<decltype(collumNum)>::type cd = 0; cd < collumNum; cd++) {//�񂲂Ƃ�
			//��̈ʒu���v�Z����(eq)
			const ureal colPosEquater = uleap(PairMinusPlus(pi), cd / (ureal)collumNum) + lensRadiusInMap;
			//���̗�̃I�t�Z�b�g���v�Z����
			const ureal nowOffsetLatitude = shiftSizeEachCollum * uleap(PairMinusPlus(0.5), cd / (ureal)collumNum);

			for (std::decay<decltype(lensNumInCollum)>::type lid = 0; lid < lensNumInCollum; lid++) {//����̗v�f�����Y�ɂ���
				
				//�����Y���S�ʒu�����@equater latitude
				const auto nowp = uvec2(colPosEquater, uleap(PairMinusPlus(pi / 2.), lid / (ureal)lensNumInCollum) + lensRadiusInMap + nowOffsetLatitude);


				//�ɍ��W�ɕϊ�����
				py::s("x=[]\ny=[]\nz=[]\nlenx=[]\nleny=[]");
				for (std::decay<decltype(lensResolution)>::type ld = 0; ld < lensResolution; ld++) {
					const ureal lenCycle = uleap(PairMinusPlus(pi), ld / (ureal)(lensResolution - 1));
					//���ʍ��W�Ƀ}�b�s���O���W����� ���[�J��
					const ureal locallon = lensRadiusInMap * cos(lenCycle) + nowp.x();//�o�x���o���@������͋��ʏ�ł�cos(theta)�{�����
					const ureal locallati = lensRadiusInMap * sin(lenCycle) + nowp.y();//�܂����̈ܓx���o�� ��������cos(theta�{����΂������)

					//uvec2 mapped = uvec2(locallon,locallati);//2D�}�b�v���W ���̂܂�
					uvec2 mapped = uvec2(locallon, 2. * atan(-pow(std::numbers::e, -locallati)) + pi / 2.);//2D�}�b�v���W �����J�g��
					py::sf("lenx.append(%f)\nleny.append(%f)", mapped.x(), mapped.y());

					//������ǂ��}�b�v���邩�@�ɍ��W�n�œn���΂�������
					const auto polarpos = PolarToXyz(mapped.x(), mapped.y());

					py::sf("x.append(%f)\ny.append(%f)\nz.append(%f)", polarpos.x(), polarpos.y(), polarpos.z());
				}

				//�v���b�g
				auto color = HsvToRgb({ uleap({0.,1.},cd / (ureal)collumNum),1.,1. });
				py::sf("plt.plot(lenx,leny,color=(%f,%f,%f))", color[0], color[1], color[2]);
				py::sf("mlab.plot3d(x,y,z,color=(%f,%f,%f))", color[0], color[1], color[2]);

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
			py::s("x=[]\ny=[]\nz=[]\nslx=[]\nsly=[]");
			for (std::decay<decltype(scanLineResolutionPhi)>::type rd = 0; rd < scanLineResolutionPhi; rd++) {
				const ureal time = uleap(PairMinusPlus(pi), rd / (ureal)(scanLineResolutionPhi - 1));

				uvec2 mapped = uvec2(time,scanTheta);
				py::sf("slx.append(%f)\nsly.append(%f)", mapped.x(), mapped.y());

				//������ǂ��}�b�v���邩�@�ɍ��W�n�œn���΂�������
				const auto polarpos = PolarToXyz(mapped.x(), mapped.y());
				py::sf("x.append(%f)\ny.append(%f)\nz.append(%f)", polarpos.x(), polarpos.y(), polarpos.z());
			}

			//plt
			auto color = HsvToRgb({ uleap({0.,1.},sd / (ureal)projectorResInTheta),1.,0.5 });
			py::sf("plt.plot(slx,sly,color=(%f,%f,%f))", color[0], color[1], color[2]);
			py::sf("mlab.plot3d(x,y,z,color=(%f,%f,%f),tube_radius=0.01)", color[0], color[1], color[2]);
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
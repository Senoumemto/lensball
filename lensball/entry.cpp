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

		//����`�悷��
		constexpr ureal sphereRadius = 1.;
		constexpr size_t sphereResolution = 20;
		py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
x = np.cos(sphphi)*np.sin(sphtheta)
y = np.sin(sphphi)*np.sin(sphtheta)
z = np.cos(sphtheta)
mlab.mesh(%f*x, %f*y, %f*z )  
)", sphereResolution, sphereResolution, sphereRadius, sphereRadius, sphereRadius);

		//polar���W�n�ɂ����Ă���Ȃ��傫���������ĕ���ł���Ƃ�
		constexpr ureal lensnumTheta = 10;
		constexpr ureal lensnumPhi = lensnumTheta * 2;//phi�͓�{�̍L����������Ă邩��
		constexpr ureal lensSize = (pi) / (lensnumTheta * 2.);//�����Y�̔��a�����߂� �����𐔂̃o�C�Ŋ���΂���

		constexpr size_t lensResolution = 20;//�����Y�̊O�`�~�̉𑜓x
		py::sf("lentheta = np.linspace(-np.pi,np.pi,%d)",lensResolution);//python���ō���Ƃ�

		for(std::decay<decltype(lensnumTheta)>::type td=0;td<lensnumTheta;td++)
			for (std::decay<decltype(lensnumPhi)>::type pd = 0; pd < lensnumPhi; pd++) {
				//���S�ʒu�����
				const auto nowp = uvec2(uleap(PairMinusPlus(pi), pd / (ureal)lensnumPhi) + (2.*pi / (ureal)(2*lensnumPhi)),
					uleap(PairMinusPlus(pi/2.), td / (ureal)lensnumTheta) + ((pi/2.*2.) / (ureal)(2 * lensnumTheta)));


				//�ɍ��W�ɕϊ�����
				py::s("x=[]\ny=[]\nz=[]\nlenx=[]\nleny=[]");
				for (std::decay<decltype(lensResolution)>::type ld = 0; ld < lensResolution; ld++) {
					const ureal lenCycle = uleap(PairMinusPlus(pi), ld / (ureal)(lensResolution-1));
					//���ʍ��W�Ƀ}�b�s���O���W����� ���[�J��
					const ureal localphi = lensSize * cos(lenCycle)+ nowp.x();//�o�x���o���@������͋��ʏ�ł�cos(theta)�{�����
					const ureal localtheta = lensSize * sin(lenCycle)+ nowp.y();//�܂����̈ܓx���o�� ��������cos(theta�{����΂������)
					uvec2 phiV = uvec2(localphi, 2.*atan(pow(std::numbers::e,localtheta))-pi/2.);//2D�}�b�v���W
					py::sf("lenx.append(%f)\nleny.append(%f)", phiV.x(), phiV.y());

					//������ǂ��}�b�v���邩�@�ɍ��W�n�œn���΂�������
					const auto polarpos = PolarToXyz(phiV.x(), phiV.y());

					py::sf("x.append(%f)\ny.append(%f)\nz.append(%f)", polarpos.x(), polarpos.y(), polarpos.z());
				}

				//�v���b�g
				py::s("plt.plot(lenx,leny)");
				py::s("mlab.plot3d(x,y,z)");

			}


		//�\������ 3d 2d�̏�
		py::s("plt.show()");
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		return -2;
	}
}
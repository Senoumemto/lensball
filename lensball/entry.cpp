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

//���ʋɍ��W�𒼌����W�ɕϊ�
uvec3 PolarToXyz(const uvec2& spolar) {
	return uvec3(cos(spolar.x()) * cos(spolar.y()),
		cos(spolar.x()) * sin(spolar.y()),
		sin(spolar.x()));
}

int main() {

	try {
		//python�����^�C�����������Ă��낢�돉������
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\n");

		//mayavi�̐ݒ�
		const std::pair<size_t, size_t> figResolution(800, 600);
		py::sf("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

		//�v���W�F�N�^������o���郌�C��theta�����ɍL����
		constexpr size_t rayWayResolution = 5;
		constexpr ureal projectorHalfAngle = 30. / 180. * pi;
		//���̉񂵕�
		constexpr size_t rotationResolution = 100;

		constexpr ureal lensSizeOnTheta = (180. / 10.) / 180. * pi;//�����Y��theta�����̑傫��

		for (std::decay<decltype(rayWayResolution)>::type rayWayD = 0; rayWayD < rayWayResolution; rayWayD++) {
			const ureal rayWay = uleap(PairMinusPlus(projectorHalfAngle), rayWayD / (ureal)(rayWayResolution - 1));//���C�̕���(theta)

			py::s("s=[[],[],[]]\nl=[[],[],[]]");//�X�L�����p�X�ƃ����Y

			//�������邮��񂷁@phi����
			for (std::decay<decltype(rotationResolution)>::type tD = 0; tD < rotationResolution; tD++) {
				const ureal t = uleap(PairMinusPlus(pi), tD / (ureal)(rotationResolution - 1));//��]�p�x(phi)

				const auto scanPath = PolarToXyz(RayHitPath(rayWay, t));//���̃��C�����������_(���[�J��)
				const auto bestLensPos = PolarToXyz(LensAlignment(t, lensSizeOnTheta, rayWay));//���C�̌�_�̒����̃����Y�̏ꏊ(���[�J��)

				//�`���]������
				py::sf("s[0].append(%f)\ns[1].append(%f)\ns[2].append(%f)\nl[0].append(%f)\nl[1].append(%f)\nl[2].append(%f)\n", scanPath.x(), scanPath.y(), scanPath.z(), bestLensPos.x(), bestLensPos.y(), bestLensPos.z());
			}

			//py::s("mlab.plot3d(s[0],s[1],s[2])");
			py::s("mlab.plot3d(l[0],l[1],l[2])");
		}


		py::s("mlab.show()");
		
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
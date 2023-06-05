#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "SphereCoord/";//����branch�̌��ʂ��i�[����t�H���_

using py = pythonRuntime;

//�ɍ��W�ɂ�...�ꎟ���̃v���W�F�N�^������o���ꂽ���C(phi=0��)��phi�ŉ�]���鋅�̌����O��
uvec2 RayHitPath(const ureal rayTheta, const ureal& t) {
	return uvec2(rayTheta, t);
}

//�v�f�����Y�̍L�����lsize(�P�� theta)�Ƃ����Ƃ��̃����Y�̃A���C�������g
uvec2 LensAlignment(const ureal& t, const ureal& lsize, const ureal rayTheta) {
	const ureal theta = -(lsize) / (2.*std::numbers::pi)+rayTheta;
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
		py::s(StringFormat("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second));

		//�܂��e�X�g�Ƃ��ċ��������Ă݂�
		const size_t thetaRes = 100;
		const size_t phiRes = 100;

		for (size_t t = 0; t < thetaRes; t++) {

			py::s("x=[]\ny=[]\nz=[]\n");//�O��plt�p�ϐ���������
			for (size_t p = 0; p < phiRes; p++){
				const uvec2 nowp(uleap(PairMinusPlus(std::numbers::pi / 2.), t / (ureal)(thetaRes-1)), uleap(PairMinusPlus(std::numbers::pi), p / (ureal)(phiRes-1)));

				const uvec3 xyz = PolarToXyz(nowp);
				cout << xyz << endl;
				py::s(StringFormat("x.append(%f)\ny.append(%f)\nz.append(%f)\n", xyz.x(), xyz.y(), xyz.z()));
			}


			py::s("mlab.plot3d(x,y,z)\n");
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
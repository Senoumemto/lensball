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
		py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

		//mayavi�̐ݒ�
		const std::pair<size_t, size_t> figResolution(800, 600);
		py::sf("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

		//�v���W�F�N�^������o���郌�C��theta�����ɍL����
		constexpr size_t rayWayResolution = 1;
		constexpr ureal projectorHalfAngle = 30. / 180. * pi;
		//���̉񂵕�
		constexpr size_t rotationResolution = 60;

		constexpr ureal lensSizeOnTheta = (180. / 10.) / 180. * pi;//�����Y��theta�����̑傫��

		for(int yy=0;yy<2;yy++)
		for (std::decay<decltype(rayWayResolution)>::type rayWayD = 0; rayWayD < rayWayResolution; rayWayD++) {
			//const ureal rayWay = uleap(PairMinusPlus(projectorHalfAngle), rayWayD / (ureal)(rayWayResolution - 1));//���C�̕���(theta)
			const ureal rayWay = 60. / 180. * pi;

			py::s("s=[[],[],[]]\nl=[[],[],[]]");//�X�L�����p�X�ƃ����Y
			py::s("lsp=[[],[]]");//�����Y�̋ɍ��W

			//�������邮��񂷁@phi����
			vector<uvec3> ss, ls;
			for (std::decay<decltype(rotationResolution)>::type tD = 0; tD < rotationResolution; tD++) {

				py::s("mlab.clf()");
				py::s("s=[[],[],[]]\nl=[[],[],[]]");//�X�L�����p�X�ƃ����Y

				const ureal t = uleap(PairMinusPlus(pi), tD / (ureal)(rotationResolution - 1));//��]�p�x(phi)
				const auto transform = Eigen::AngleAxis<ureal>(-t, uvec3::UnitZ());//�ϊ�


				py::sf(R"(
[phi, theta] = np.mgrid[0+%f:2 * np.pi+%f:12j, 0:np.pi:12j]
x = np.cos(phi) * np.sin(theta)
y = np.sin(phi) * np.sin(theta)
z = np.cos(theta)

mlab.mesh(x,y, z,representation="wireframe",color=(1,1,1)))", -t, -t);

				ss.push_back(PolarToXyz(RayHitPath(rayWay, t)));//���̃��C�����������_(���[�J��)
				ls.push_back(PolarToXyz(LensAlignment(t, lensSizeOnTheta, rayWay)));//���C�̌�_�̒����̃����Y�̏ꏊ(���[�J��)


				uvec3 ssx, lsx;
				for (int i = 0; i < ss.size(); i++) {
					ssx = (transform * ss.at(i));
					lsx = (transform * ls.at(i));

					//�`���]������
					py::sf("s[0].append(%f)\ns[1].append(%f)\ns[2].append(%f)\nl[0].append(%f)\nl[1].append(%f)\nl[2].append(%f)\n", ssx.x(), ssx.y(), ssx.z(), lsx.x(), lsx.y(), lsx.z());
				}

				//���C�m�X�L�����p�X�ƃ����Y�̔z�u��`��
				py::s("mlab.plot3d(s[0],s[1],s[2],color=(1,0,0))");

				const auto rayTerm = PolarToXyz(uvec2(rayWay, 0));//���C��`�悷��@���_����...�����܂�
				py::sf("mlab.plot3d([0,%f],[0,%f],[0,%f],color=(%f,0,0))", rayTerm.x(), rayTerm.y(), rayTerm.z(), 1.);


				py::sf("mlab.savefig(\"%s\")", rezpath + branchpath + string("rez") + to_string(tD) + ".png");
			}
			//py::s("mlab.plot3d(l[0],l[1],l[2])");

			//�񎟌��ł�plt
			py::s("plt.plot(lsp[0],lsp[1],\"r--\")");

		}
		MakeGifAnim(rezpath + branchpath + "pallet.png", rezpath + branchpath + "output.gif", rezpath + branchpath + "rez%d.png", rotationResolution);

		//�\������ 3d 2d�̏�
		py::s("mlab.show()");
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
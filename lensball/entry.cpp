#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "Magsim/";//����branch�̌��ʂ��i�[����t�H���_
using py = pythonRuntime;

void DrawCoil(const ureal radius, const ureal length, const arrow3& posdir, const ureal& pitch, const size_t& split) {
	//��������̂� pitch�������Ԃɐi�ދ���
	const ureal cycle = length / pitch;

	//�R�C�����[�J������O���[�o���Ɏ���ϊ������
	const uvec3 rotAxis = uvec3(0, 0, 1.).cross(posdir.dir());
	const ureal rotAngle = acos(uvec3(0, 0, 1.).dot(posdir.dir()));
	const uvec3 rotVec = rotAngle * rotAxis;
	py::SendCommandFormat("springrot=Rotation.from_rotvec([%f,%f,%f])",rotVec.x(), rotVec.y(), rotVec.z());

	//�R�C��������
	py::SendCommandFormat("springt=np.linspace(0.,2.*math.pi*%f,%d)", cycle, split);//�܂��}��ϐ���錾 ���񂾂����@pitch/2pi��������ƒ���
	py::SendCommandFormat("springvertices=springrot.apply(np.transpose(np.array([%f*np.sin(springt),%f*np.cos(springt),(%f*(springt-math.pi*%f))])))", radius,radius,pitch/(2.*std::numbers::pi), cycle);
	py::SendCommandFormat("mlab.plot3d(springvertices[:,0]+%f,springvertices[:,1]+%f,springvertices[:,2]+%f)",posdir.org().x(), posdir.org().y(), posdir.org().z());

	return;
}

ureal BioSaba(const ureal I,const ureal radius,const ureal theta,const ureal dl) {
	const auto dH = (I * dl * sin(theta)) / (4. * std::numbers::pi * pow(radius, 2));

	return dH;
}

//�ւ�����̓d������鎥��
using circleExpression = std::function<arrow3(const ureal&)>;//-pi ~ pi�܂ł���͂���Ƃ��̈ʒu�Ɛڐ���Ԃ��֐�

int countax = 0;
uvec3 MagFieldFromCircuitCurrent(const uvec3& obsp,const ureal radius,const circleExpression& func,const ureal I,const size_t split) {
	//�r�I�E�T�o�[���Ōv�Z�ł���Ǝv��
	uvec3 H = uvec3::Zero();

	//�~����1/split���čl����
	const ureal dl = 2. * std::numbers::pi * radius / (ureal)split;//��������
	for (std::decay <decltype(split) > ::type i = 0; i < split; i++) {
		const ureal t = uleap(PairMinusPlus(std::numbers::pi), i / (ureal)split);//���[�v���Ă��邩���
		const arrow3 nowpd = func(t);//�����ڂ��Ă���_�Ɛڐ�����ɓ����
		const uvec3 far = (nowpd.org() - obsp);//�ϑ��_���猩�����ړ_
		const ureal theta = acos(nowpd.dir().dot(far.normalized()));//�ڐ�������far�̊p�x

		const ureal dHnorm = BioSaba(I, far.norm(), theta, dl);//�������E�̑傫�����킩����

		//�����͑����@dir cross far ���Ǝv��
		const uvec3 dH = dHnorm * (nowpd.dir().cross(far.normalized()));

		H += dH;
	}

	//cout << H.norm() << endl;
	return H;
}

int main() {

	try {
		//python�����^�C�����������Ă��낢�돉������
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\nimport math\nfrom scipy.spatial.transform import Rotation\n");

		//����R�C����z�u����
		const ureal coillen = 1.;
		const ureal coilr = 1.;
		const arrow3 coilposdir = arrow3(uvec3(0, 0, 0), uvec3(0, 0, 1).normalized());
		const ureal coilpitch = coillen/10.;//1��������ɐi�ޒ���
		const ureal coilsplit = 16. * (coillen / coilpitch);//������
		const std::pair<size_t, size_t> coilnum = make_pair<size_t, size_t>(4,4);//�R�C���̐��@x������y
		const std::pair<ureal,ureal> coildist = make_pair<ureal,ureal>(4.,4.);//�R�C�����m�̋���
		for(std::decay<decltype(coilnum)::first_type>::type cx=0;cx<coilnum.first;cx++)
			for (std::decay<decltype(coilnum)::second_type>::type cy = 0; cy < coilnum.second; cy++) {
				//0,0�̂Ƃ��̃I�t�Z�b�g���v�Z����
				const uvec2 offset(-(ureal)(coilnum.first - 1) * coildist.first/2., -(ureal)(coilnum.second - 1) * coildist.second/2.);
				DrawCoil(coilr, coillen, arrow3(uvec3(offset.x() + coildist.first * cx, offset.y() + coildist.second * cy, 0), uvec3(0, 0, 1).normalized()), coilpitch, coilsplit);
			}


		//�R�C�������n����v�Z����
		std::list<arrow3> vfield;
		constexpr std::array<ureal, 3> halfCubeEdgeLength = { 12. / 2. ,12. / 2. ,4. / 2. };//������L���[�u�̈�ӂ̒����̔���
		constexpr std::array<size_t, 3> cubeResolution = { 12,12,4 };
		for (std::decay<decltype(cubeResolution)::value_type>::type z = 0; z < cubeResolution.at(2); z++)
			for (std::decay<decltype(cubeResolution)::value_type>::type y = 0; y < cubeResolution.at(1); y++)
				for (std::decay<decltype(cubeResolution)::value_type>::type x = 0; x < cubeResolution.at(0); x++) {
					const uvec3 nowp(uleap(PairMinusPlus(halfCubeEdgeLength[0]), x / (ureal)(cubeResolution.at(0) - 1)),
						uleap(PairMinusPlus(halfCubeEdgeLength[1]), y / (ureal)(cubeResolution.at(1) - 1)),
						uleap(PairMinusPlus(halfCubeEdgeLength[2]), z / (ureal)(cubeResolution.at(2) - 1)));//���̍��W

					uvec3 dir = uvec3::Zero();
					//���ׂẴR�[���̉e�����v�Z����
					for (std::decay<decltype(coilnum)::first_type>::type cx = 0; cx < coilnum.first; cx++)
						for (std::decay<decltype(coilnum)::second_type>::type cy = 0; cy < coilnum.second; cy++) {
							//0,0�̂Ƃ��̃I�t�Z�b�g���v�Z����
							const uvec2 offset(-(ureal)(coilnum.first - 1) * coildist.first / 2., -(ureal)(coilnum.second - 1) * coildist.second / 2.);
							const auto coilcenter = uvec3(offset.x() + coildist.first * cx, offset.y() + coildist.second * cy, 0);


							dir += MagFieldFromCircuitCurrent(nowp, 1, [&](const ureal& t) {
								return arrow3(uvec3(cos(t), sin(t), 0.)+ coilcenter, uvec3(-sin(t), cos(t), 0)); }, 0.25, 1000);
						}

					vfield.push_back(arrow3(nowp, dir));
				}
		//plt
		for (const auto& vp : vfield) {
			const ureal len = vp.dir().norm();
			const std::array<ureal, 3> color = { clamp(fabs(vp.dir().x()*30.),0.,1.),clamp(fabs(vp.dir().y()*30.),0.,1.) ,clamp(fabs(vp.dir().z()*30.),0.,1.) };
			py::SendCommandFormat("vfields = mlab.quiver3d(%f,%f,%f,%f,%f,%f,color=(%f,%f,%f))\n", vp.org().x(), vp.org().y(), vp.org().z(), vp.dir().x(), vp.dir().y(), vp.dir().z(), color.at(0), color.at(1), color.at(2));
			py::SendCommand("vfields.glyph.glyph_source.glyph_position=\"center\"");
		}

		py::s("mlab.show()\n");
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
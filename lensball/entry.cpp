#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int MMain();

int main() {
	try {
		return MMain();
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

int MMain() {
	/*
	�����Y�𓮂����āA���̎U�����m�肽��
	�����Y�̓���x�����ɂƂ���dir��x,y���v���b�g
	**********y
	���ˌ��̕�����@�����炸�炵�čs��
	
	
	*/

	printf("hello v1235\n");

	auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
	DefinePythonFunctions(plotter);//�~�{�I��{�֐����`


	//UV���̂��߂̋��p�����[�^��錾
	const sphereParam centerUVSphereParam = make_pair(uvec3(0., 0., 0.), 2.);

	//UV���̒��_���Z�o
	std::list<uvec3>uvPoses;
	constexpr size_t numOfVerticalResolution = 10;//UV���̏c�����̕���\
	constexpr size_t numOfRadicalResolution = 12;//UV���̕��˕����̕���\
	for(std::decay<decltype(numOfVerticalResolution)>::type v=0;v<numOfVerticalResolution;v++)
		for (std::decay<decltype(numOfRadicalResolution)>::type r = 0; r < numOfRadicalResolution; r++) {
			//�܂���v�𐳋K������@���W�ł�
			const auto vpos = uleap(make_pair(-centerUVSphereParam.second, +centerUVSphereParam.second), v / (ureal)(numOfVerticalResolution - 1));
			const auto rpos = uleap(make_pair(-std::numbers::pi, +std::numbers::pi), r / (ureal)(numOfRadicalResolution));//r���� ��[�Ղ�����(�ŏ�=�Ōゾ����)1�Ђ��Ȃ�

			//vpos�ł�uv�f�ʂ̔��a�����߂� x^2+y^2=r^2 y^2=r^2-x^2
			const ureal radiusOfCross = sqrt(pow(centerUVSphereParam.second, 2) - pow(vpos, 2));

			//���s���W�ɂ��Ēǉ�
			uvPoses.push_back(uvec3(radiusOfCross * cos(rpos), radiusOfCross * sin(rpos), vpos) + centerUVSphereParam.first);
		}
		
	//���ɋɂ��X����
	const auto tiltOfPoll = Eigen::AngleAxis<ureal>(15. / 180. * std::numbers::pi, uvec3(0., 1., 0.));
	for (auto& p : uvPoses)p = tiltOfPoll * p;

	//���Ƀ��C�𓖂Ăă}�b�s���O�������̂œ��Ă郌�C���`����
	const arrow3 mapRay = arrow3(uvec3(0., -5., 0.), uvec3(0., 1., 0.));
	std::list<uvec3> mappingList;

	//�܂킷
	constexpr size_t rotationResolution = 60;
	for (int xx = 0; xx < 1; xx++) {
		for (std::decay<decltype(rotationResolution)>::type rotIndex = 0; rotIndex < rotationResolution; rotIndex++) {
			const ureal rotAngle = uleap(make_pair(-std::numbers::pi, +std::numbers::pi), rotIndex / (ureal)rotationResolution);//���K���@���[�v�Ȃ��Ƃɒ���
			const Eigen::AngleAxis<ureal> rotMat = Eigen::AngleAxis<ureal>(rotAngle, uvec3(0., 0., 1.));

			//�}�b�s���O���� ������񍡂̃��[�J���ϊ����܂߂Ă�
			const auto rez = IntersectSphere(mapRay, centerUVSphereParam.first, centerUVSphereParam.second);
			if (!rez.isHit)throw logic_error("dameda");
			cout << rez.pos << endl;
			mappingList.push_back(rotMat * rez.pos);



			//�t���[���N���A
			plotter->send_command(R"(
plt.cla()
#ax = fig.add_subplot(111, projection = '3d')
ax.set_box_aspect((1, 1, 1))
# �����x���̐ݒ�
ax.set_xlabel("x", fontsize = 16)
ax.set_ylabel("y", fontsize = 16)
ax.set_zlabel("z", fontsize = 16)
# ���͈͂̐ݒ�
ax.set_xlim(-3.,3.)
ax.set_ylim(-3.,3.)
ax.set_zlim(-3.,3.))");
			//Start drawing
			plotter->send_command("ax.view_init(elev=0\n)");//���_��ݒ�

			//�}�b�s���O��������
			if (false) {
				plotter->send_command("x=[]\ny=[]\nz=[]\n");
				for (const auto& p : mappingList) {
					plotter->send_command(StringFormat(""
						"x.append(%f)\n"
						"y.append(%f)\n"
						"z.append(%f)\n"
						"\n", p.x(), p.y(), p.z()));
				}
				plotter->send_command("ax.plot(x,y,z,color=\"red\")\n");
			}

			//Draw uv vertices
			for (auto p : uvPoses) {
				p = rotMat * p;
				plotter->send_command(StringFormat("ax.scatter(%f,%f,%f)\n", p.x(), p.y(), p.z()));
			}

			//���ʂ�ۑ�
			const std::string resultsPathPrefix = R"(C:/local/user/lensball/lensball/results2/)";
			plotter->save(resultsPathPrefix + StringFormat("rez%d.png", rotIndex));
			plotter->pause();
		}
	}
	plotter->show();
	plotter->send_command("plt.cla()\nplt.clf()\n");//pyplot�I���|���V�[
	plotter->close();
	return 0;
}
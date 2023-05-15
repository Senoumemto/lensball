#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {

	printf("hello v1234\n");

	auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
	DefinePythonFunctions(plotter);//�~�{�I��{�֐����`

	//�v�f�����Y���`�@�������Y��
	const ureal nodeLensEta = 1.5;
	const sphereParam nodeLensParam = make_pair(uvec3(0., 0., 0.), 1.);

	//���C�𐶐�����
	list<ray3> rays;
	rays.push_back(ray3(arrow3(uvec3(0.25,0.,10.),uvec3(0.,0.,-1.))));//�����ʒu�����
	rays.push_back(ray3(arrow3(uvec3(-0.25, 0., 10.), uvec3(0., 0., -1.))));//�����ʒu�����
	for (ray3& target : rays) {
		//���C�g���p�C�v���C��
		[&] {
			const auto rez0 = IntersectSphere(target.back(), nodeLensParam.first, nodeLensParam.second);//�v�f�����Y�ƌ���

			//������Ȃ������炱��ȏ���K�v�͂Ȃ�
			if (!rez0.isHit) {
				FreeFlightRay(target);
				return;
			}

			//�������烌���Y���̏���
			rez0.ApplyToRay(target);
			if (!RefractSnell(target, rez0.norm, nodeLensEta))throw runtime_error("�S���˂��N����");//���܌v�Z

			const auto rez1 = IntersectSphere(target.back(), nodeLensParam.first, nodeLensParam.second);//�v�f�����Y������ʉ�
			if (!rez1.isHit)throw logic_error("logic err0");//�����Y�����Ȃ̂Ő�Γ�����
			rez1.ApplyToRay(target);//�i�߂�

			if (!RefractSnell(target, -rez1.norm, 1. / nodeLensEta))throw runtime_error("�S���˂��N����");//���܌v�Z

			FreeFlightRay(target);

		}();
	}

	//�v�f�����Y��`��
	DrawSphere(plotter, nodeLensParam.first, nodeLensParam.second, 10, R"("red")");
	//���ׂẴ��C��`��
	for(const auto&r:rays)
		DrawRaySkipFirstArrow(plotter, r, R"("green")");//���C��`��

	plotter->show();//�\���@�Ȃ񂩏I����
	plotter->close();//�I���@����܂�Ӗ����Ȃ�
	return 0;
}
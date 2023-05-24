#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {
	/*
	�����Y�𓮂����āA���̎U�����m�肽��
	�����Y�̓���x�����ɂƂ���dir��x,y���v���b�g
	**********y
	���ˌ��̕�����@�����炸�炵�čs��
	
	
	*/

	printf("hello v1235\n");

	auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
	DefinePythonFunctions(plotter);//�~�{�I��{�֐����`

	//�v�f�����Y���`�@�������Y��
	const ureal nodeLensEta = 1.5;
	sphereParam nodeLensParam = make_pair(uvec3(100.,0., 0.), 1.);

	//���C�𐶐�����
	list<ray3> rays;
	constexpr unsigned int raynum = 18;
	constexpr ureal projectionLimit = 30. / 180. * std::numbers::pi;//�Б��p�x
	const uvec3 projectionOrign(0., 0., 0.);
	for (std::decay<decltype(raynum)>::type i = 0; i < raynum; i++) {
		//�l�𐳋K������-1~1
		const ureal reg = 2. * (i / (ureal)(raynum - 1)) - 1.;
		//������p�x�ɂ���
		const ureal angle = reg * projectionLimit;//��������̊p�x

		//�܂���������܂� x,y���ʂ�y��������zero��angle�����p�x�����߂�
		const uvec3 direction(sin(angle), cos(angle), 0.);
		rays.push_back(ray3(arrow3(projectionOrign, direction)));//���C���쐬
	}
	for (ray3& target : rays) {
		//���C�g���p�C�v���C��
		[&] {
			try {
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
			}
			catch (std::exception& ex) {
				cout << ex.what() << endl;
				system("pause");
			}
			catch (...) {
				cout << "Unknown err" << endl;
				abort();
			}
			
		}();
	}

	//�v�f�����Y��`��
	//DrawSphere(plotter, nodeLensParam.first, nodeLensParam.second, 10, R"("red")");
	//���ׂẴ��C��`��
	const array<string,5> cols = { "\"red\"","\"orange\"","\"yellow\"","\"green\"","\"blue\"" };
	for (const auto& r : rays) {
		DrawRaySkipFirstArrow(plotter, r, "\"red\"");//���C��`��
		//���C�̕�����\��
		cout <<"dir:\n" << r.back().dir() <<"\n\n" << endl;
		//�p�x���v�Z����
		const ureal angleax = atan(r.back().dir().x() / r.back().dir().z()) / std::numbers::pi * 180.;
		cout << "angle: " << angleax << endl;
	}

	plotter->show();//�\���@�Ȃ񂩏I����
	plotter->close();//�I���@����܂�Ӗ����Ȃ�
	return 0;
}
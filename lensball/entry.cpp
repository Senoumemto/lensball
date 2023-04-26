#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"

using namespace std;

int main() {

	auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
	DefinePythonFunctions(plotter);//�~�{�I��{�֐����`

	//�����Y�{�[���̗v�f�����Y�̃p�����[�^���v�Z����
	auto lensBallsParams = CalcSmallLensPosAndRadius();
	//�����Y�{�[���̂��ׂĂ̋ʂ�`��
	for (const auto& param : *lensBallsParams)
		DrawSphere(plotter, param.first, param.second, 5, R"("red")");



	list<ray3> rays;//���C�𐶐�����
	for (int i = 0; i < 4; i++) {
		rays.push_back(ray3(arrow3(uvec3(-5., (ureal)(i-2)/4., 0.), uvec3(1., 0., 0.).normalized())));
		auto result = IntersectSphere(rays.back().back(), uvec3(0, 0, 0), 1.);//�����Ȍ��ʂ��v�Z
		if (result.isHit) {
			result.ApplyToRay(rays.back());//���̒���܂Ői�܂���
			ReflectMirror(rays.back(), result.norm);//���ʔ���
			FreeFlightRay(rays.back());
		}
		else FreeFlightRay(rays.back());//������Ȃ���Ύ��R��s
	}
	//���ׂẴ��C��`��
	for(const auto&r:rays)
		DrawRay(plotter, r, R"("green")");//���C��`��



	plotter->show();//�\���@�Ȃ񂩏I����
	plotter->close();//�I���@����܂�Ӗ����Ȃ�
	return 0;
}
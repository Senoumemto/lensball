#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {

	printf("hello\n");

	auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
	DefinePythonFunctions(plotter);//�~�{�I��{�֐����`

	//�v�f�����Y���`�@�������Y��
	const sphereParam nodeLensParam = make_pair(uvec3(0., 0., 0.), 1.);

	//���C�𐶐�����
	list<ray3> rays;
	rays.push_back(ray3(arrow3(uvec3(0.,0.,10.),uvec3(0.,0.,-1.))));//�����ʒu�����
	ray3& target = rays.back();

	const auto rez=IntersectSphere(target.back(), nodeLensParam.first, nodeLensParam.second);//�v�f�����Y�ƌ���
	if (rez.isHit)//���ɓ���������
		rez.ApplyToRay(target);
	else//������Ȃ�������
		FreeFlightRay(target);

	//�v�f�����Y��`��
	DrawSphere(plotter, nodeLensParam.first, nodeLensParam.second, 10, R"("red")");
	//���ׂẴ��C��`��
	for(const auto&r:rays)
		DrawRaySkipFirstArrow(plotter, r, R"("green")");//���C��`��

	plotter->show();//�\���@�Ȃ񂩏I����
	plotter->close();//�I���@����܂�Ӗ����Ȃ�
	return 0;
}
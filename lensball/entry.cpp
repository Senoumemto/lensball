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
	;


	//�����Y���`���Ă���
	constexpr ureal nodeLensEta = 1.5;
	constexpr ureal lensBallRadius = 5. / 2.;
	constexpr unsigned int nodeLensNum = 10;
	const ureal nodeLensRadius = lensBallRadius * std::sin((std::numbers::pi /(nodeLensNum-1.))/2.);//���ڂ���2(n-1)�p�`�̈�ӂ̒��������a������...
	std::list<sphereParam> nodeLensesParams;
	for (std::decay<decltype(nodeLensNum)>::type i = 0; i < nodeLensNum; i++) {
		//�l�𐳋K������-1~1
		const ureal reg = 2. * (i / (ureal)(nodeLensNum - 1)) - 1.;
		//������p�x�ɂ���
		const ureal angle = reg * std::numbers::pi/2.;//��������̊p�x

		//�܂�ʒu�����܂� x,y���ʂ�y��������zero��angle�����p�x�����߂�
		const uvec3 pos(lensBallRadius * sin(angle), lensBallRadius * cos(angle), 0.);
		nodeLensesParams.push_back(sphereParam(pos, nodeLensRadius));
	}
	//���C�𐶐�����
	list<ray3> rays;
	constexpr unsigned int targetBallsNum = 6;//���𓖂Ă�{�[���̐�v ������nodelensNum������������������A����������ɂ��邱��
	constexpr unsigned int targetBallsOffset = ((nodeLensNum % 2) != (targetBallsNum % 2)) ? 0 : (nodeLensNum - targetBallsNum) / 2;
	const uvec3 projectionOrign(0., .0, 0.);
	constexpr ureal layAngleLimitFromTarget = 5. / 180. * std::numbers::pi;
	auto ite=nodeLensesParams.begin();
	for (int i = 0; i < targetBallsOffset; i++)ite++;
	for (std::decay<decltype(targetBallsNum)>::type i = 0; i < targetBallsNum; i++) {
		const uvec3 targetDirection = ((ite->first) - projectionOrign).normalized();//���ꂪ�����Y�̒��S��ʂ郌�C�̕���
		//��������+-theta����z�����S�ɑł���������
		const uvec3 direction0 = Eigen::AngleAxis<ureal>(layAngleLimitFromTarget, uvec3(0, 0, 1)) * targetDirection;
		const uvec3 direction1 = Eigen::AngleAxis<ureal>(-layAngleLimitFromTarget, uvec3(0, 0, 1))*targetDirection;


		rays.push_back(ray3(arrow3(projectionOrign, targetDirection)));//���C���쐬
		rays.push_back(ray3(arrow3(projectionOrign, direction0)));//���C���쐬
		rays.push_back(ray3(arrow3(projectionOrign, direction1)));//���C���쐬

		ite++;
	}


	//���C�g���p�C�v���C��
	std::list<decltype(rays)::value_type::iterator> auxTarget;//�⏕�����������߂�arrow�������ɓ˂�����
	int rayTracingCount = -1;//���܏�������Ă���͉̂��{�ڂ̃��C��
	for (ray3& target : rays) {
		[&] {
			try {
				rayTracingCount++;

				//���ׂĂ̗v�f�����Y�ɑ΂��ē����蔻����s�� ����Ō������郌���Y��T��
				auto hittedNode = nodeLensesParams.end();
				resultIntersecteSphere rezIns;//���Ў��̌�������
				for (auto i = nodeLensesParams.begin(); i != nodeLensesParams.end();i++) {
					const auto rez0 = IntersectSphere(target.back(), i->first, i->second);//�v�f�����Y�ƌ���
					if (rez0.isHit) {//����������I���
						hittedNode = i;
						rezIns = rez0;
						break;
					}
				}

				//������Ȃ������炱��ȏ���K�v�͂Ȃ�
				if (hittedNode==nodeLensesParams.end()) {
					FreeFlightRay(target);
					return;
				}

				//�������烌���Y���̏���
				rezIns.ApplyToRay(target);
				if (!RefractSnell(target, rezIns.norm, nodeLensEta))throw runtime_error("�S���˂��N����");//���܌v�Z

				const auto rezExp = IntersectSphere(target.back(), hittedNode->first, hittedNode->second);//�v�f�����Y������ʉ�
				if (!rezExp.isHit)throw logic_error("logic err0");//�����Y�����Ȃ̂Ő�Γ�����
				rezExp.ApplyToRay(target);//�i�߂�

				if (!RefractSnell(target, -rezExp.norm, 1. / nodeLensEta))throw runtime_error("�S���˂��N����");//���܌v�Z

				//�����ŕ⏕�����������ǂ������߂�
				if (true/*rayTracingCount % 3 != 1*/) auxTarget.push_back(--target.end());

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

	//���ɍ�}����
	//�v�f�����Y��`��
	for (const auto& i : nodeLensesParams)
		DrawSphere(plotter, i.first, i.second, 20, R"("green")");
	//�v���W�F�N�^�̍ő哊�f�p��
	constexpr ureal projectorLimitAngle = 60. / 180. * std::numbers::pi;//�v���W�F�N�^�̓��f�͈́@�Б�
	constexpr ureal auxProjectorLimitLineLength = 10.;//�⏕������
	DrawLine(plotter, projectionOrign, projectionOrign + uvec3(sin(projectorLimitAngle), cos(projectorLimitAngle), 0.) * auxProjectorLimitLineLength, R"("blue")");
	DrawLine(plotter, projectionOrign, projectionOrign + uvec3(sin(-projectorLimitAngle), cos(-projectorLimitAngle), 0.) * auxProjectorLimitLineLength, R"("blue")");


	//�\���̈�̂��߂̕⏕��
	constexpr ureal auxLineLength = 10.;
	for (const auto& t : auxTarget) {
		//arrow���}�C�i�X�����։��������������
		DrawLine(plotter, t->org(), t->org() - (auxLineLength * t->dir()), R"("green")");

	}

	//���ʂ�ۑ�
	const std::string resultsPathPrefix = R"(C:/local/user/lensball/lensball/results/)";
	plotter->save(resultsPathPrefix + "rez0.png");
	plotter->pause(1.);//�\���@�Ȃ񂩏I����
	plotter->send_command("plt.cla()\nplt.clf()\n");//pyplot�I���|���V�[
	plotter->close();//�I���@����܂�Ӗ����Ȃ�
	return 0;
}
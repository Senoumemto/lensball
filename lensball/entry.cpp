#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {

	auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
	DefinePythonFunctions(plotter);//�~�{�I��{�֐����`

	//�����Y�{�[���̗v�f�����Y�̃p�����[�^���v�Z����
	auto lensBallsParams = CalcSmallLensPosAndRadius();
	//�����Y�{�[���̂��ׂĂ̋ʂ�`��
	for (const auto& param : *lensBallsParams)
		DrawSphere(plotter, param.first, param.second, 5, R"("red")");
	//DrawSphere(plotter, uvec3::Zero(), 1.);

	list<ray3> rays;//���C�𐶐�����
	constexpr size_t timingNum = 1500/60;//�������Ԃɂǂꂾ���I������ 1.5kHz���Ƃ����
	const std::pair<size_t,size_t> res = make_pair(4,4);//�v���W�F�N�^�̉𑜓x

	for (int g = 0; g < timingNum; g++) {
		//�O���[�o��->�����Y�{�[���ϊ�
		uaffine3 translenses(Eigen::AngleAxisd(2. * std::numbers::pi * ((ureal)g / (ureal)timingNum), uvec3::UnitZ()));
		for (size_t h = 0; h < res.first; h++) {
			for (size_t i = 0; i < res.second; i++) {
				rays.push_back(ray3(arrow3(translenses*uvec3(-5., (ureal)(i - res.first/2.) / res.first, (ureal)(h - res.second/2.) / res.second), translenses * uvec3(1., 0., 0.).normalized())));//�����ʒu����� �����Y�{�[�����W�n��

				//���ׂĂ̋��ɑ΂��ē����蔻����s��
				resultIntersecteSphere closestRez = resultIntersecteSphere();
				closestRez.t = std::numeric_limits<ureal>::infinity();
				for (const auto& param : *lensBallsParams) {
					auto result = IntersectSphere(rays.back().back(), param.first, param.second);//�����ƌ���

					if (!result.isHit)continue;
					//��������������
					if (result.t < closestRez.t)closestRez = result;//���߂���΍X�V
				}

				//����Ƀu���b�J�[�Ƃ̓����蔻�������
				const auto blocker = IntersectSphere(rays.back().back(), uvec3(0, 0, 0), 1.);
				//�����u���b�J�[�ɂ��������炻���ŋz��
				if (blocker.t <= closestRez.t) {
					blocker.ApplyToRay(rays.back());
				}

				//�����ɂ���������
				else if (closestRez.isHit) {
					closestRez.ApplyToRay(rays.back());
					ReflectMirror(rays.back(), closestRez.norm);//���ʔ���
					FreeFlightRay(rays.back());
				}
				else FreeFlightRay(rays.back());//������Ȃ���Ύ��R��s*/

				//�Ō�Ƀ��C���t�ϊ����Ė߂�
				const uaffine3 backtrans = translenses.inverse();
				for (auto& rn : rays.back()) {
					rn.dir() = backtrans * rn.dir();
					rn.org() = backtrans * rn.org();
				}
			}
		}
	}

	//���ׂẴ��C��`��
	for(const auto&r:rays)
		DrawRaySkipFirstArrow(plotter, r, R"("green")");//���C��`��



	plotter->show();//�\���@�Ȃ񂩏I����
	plotter->close();//�I���@����܂�Ӗ����Ȃ�
	return 0;
}
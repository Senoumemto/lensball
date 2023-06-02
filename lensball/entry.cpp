#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

constexpr ureal theta = 30. / 180. * std::numbers::pi;//���񂾂��X����
const std::pair<ureal,ureal> scanHeightRange = make_pair(-0.8,0.8);

//���[�J�����猩���X�L�����ʒu�@t ��]�p�x,gh �X�L�������C���̍���(�O���[�o��)�@theta ���[�J���̌X��
uvec2 ScanPointWithGh(ureal t,ureal gh,ureal theta) {
	const ureal radius = sqrt(1. - pow(gh, 2));//�X�L���������ł̋O�Ղ̔��a
	//r,v
	return uvec2(atan((radius * sin(t)) / (radius * cos(t) * cos(theta) - gh * sin(theta))), radius * cos(t) * sin(theta) + gh * cos(theta));
}
//�����Y�z�u�p�X�֐��@���[�J���@����ɉ����ăX�L�������C���ߕӂ̃����Y������ł���
ureal NodeLensesPath(ureal t,ureal gh,ureal theta) {
	return sqrt(3) / 2. * gh;
}
//�����Y�z�u�p�X�֐��@���[�J���@����ɉ����ă����Y����ׂ�Ɨ��z�ʂ�̃X�L�����O�Ղ̍����ƂȂ�
ureal NodeLensesPathCross(ureal t, ureal gh, ureal theta) {
	return sqrt(1 - pow(gh,2)) * cos(t) * sin(theta) - (2 * sqrt(1 - pow(gh,2)) * t * sin(theta)) / std::numbers::pi + gh * cos(theta);
}
//�����Y�p�X����̃X�L�����O�Ղ̍��� �����Y�p�X���w�肵�Ă�
ureal ScanHeightFromLensesPath(ureal t, ureal gh, ureal theta, const std::function<ureal(ureal, ureal, ureal)>& lensespath) {
	return ScanPointWithGh(t, gh, theta)[1] - lensespath(t, gh, theta);
}

int main() {

	try {
		//plot�̏���
		auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
		DefinePythonFunctions(plotter);//�~�{�I��{�֐����`

		//������ς���plt
		constexpr size_t scanheightResolution = 40;
		while(1)for (std::decay<decltype(scanheightResolution)>::type h = 0; h < scanheightResolution; h++) {
			plotter->send_command("plt.clf()\n");
			plotter->send_command("t=[]\nv=[]\nr=[]\n");

			//���܂�scan����
			const auto scanHeight = uleap(scanHeightRange, h / (ureal)(scanheightResolution - 1));
			cout << scanHeight << endl;

			constexpr size_t circleRes = 360;
			for (std::decay<decltype(circleRes)>::type phi = 0; phi < circleRes; phi++) {
				const ureal t = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), phi / (ureal)circleRes);//t���v�Z

				const auto scanUV = ScanPointWithGh(t, scanHeight, theta);//UV���W�Q�b�g UV�����[�J��
				const auto scanHFromLensesPath = ScanHeightFromLensesPath(t, scanHeight, theta, NodeLensesPathCross);//�����Y�p�X����̍��� UV�����[�J��
				//...����𒼐��ɂ��Ȃ��Ⴂ���Ȃ��@

				plotter->send_command(StringFormat("t.append(%f)\nv.append(%f)\nr.append(%f)\n", t, scanHFromLensesPath, scanUV.x()));
			}

			plotter->send_command(StringFormat("plt.text(0,-1.5,\"theta = 30[deg], scan height = %f\")",scanHeight));
			plotter->send_command("plt.plot(t,r,label=\"r [rad]\",color=\"magenta\")\n");
			plotter->send_command("plt.plot(t,v,label=\"v\",color=\"cyan\")\n");
			plotter->send_command("plt.xlabel(\"Rotation angle [rad]\")\nplt.ylabel(\"UV coordition\")\n");
			plotter->send_command("plt.legend()\n");
			plotter->pause(.1);
			plotter->save(StringFormat("C:/local/user/lensball/lensball/results3/rez%d.png",h));
		}
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
#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "CrossPathShape/";//����branch�̌��ʂ��i�[����t�H���_
constexpr ureal theta = 5. / 180. * std::numbers::pi;//���񂾂��X����
const std::pair<ureal,ureal> scanHeightRange = make_pair(-0.95,0.95);

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
	return sqrt(1 - pow(gh,2)) * cos(t) * sin(theta) - (2 * sqrt(1 - pow(gh,2)) * t * sin(theta)) / (std::numbers::pi/2.) + gh * cos(theta);
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
		plotter->send_command(R"(
# ���͈͂̐ݒ�
ax.set_xlim(-1.,1.)
ax.set_ylim(-1.,1.)
ax.set_zlim(-1.,1.))");
		plotter->send_command("cm = plt.get_cmap(\"Spectral\")\n");//�J���[�Z�b�g�����

		//������ς���plt
		constexpr size_t scanheightResolution = 100;
		for (std::decay<decltype(scanheightResolution)>::type h = 0; h < scanheightResolution; h++) {
			plotter->send_command("#ax.clear()\n");
			plotter->send_command("x=[]\ny=[]\nv=[]\n");

			//���܂�scan����(�O���[�o��)
			const auto scanHeight = uleap(scanHeightRange, h / (ureal)(scanheightResolution - 1));

			constexpr size_t circleRes = 360;
			for (std::decay<decltype(circleRes)>::type phi = 0; phi < circleRes; phi++) {
				const ureal t = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), phi / (ureal)circleRes);//t���v�Z

				const auto scanUV = ScanPointWithGh(t, scanHeight, theta);//UV���W�Q�b�g UV�����[�J��
				const auto scanHFromLensesPath = ScanHeightFromLensesPath(t, scanHeight, theta, NodeLensesPathCross);//�����Y�p�X����̍��� UV�����[�J��
				//...����𒼐��ɂ��Ȃ��Ⴂ���Ȃ��@

				//plotter->send_command(StringFormat("t.append(%f)\nv.append(%f)\nr.append(%f)\n", t, scanHFromLensesPath, scanUV.x()));
			}

			//�O���t��plt
			//plotter->send_command(StringFormat("plt.text(0,-1.5,\"theta = 30[deg], scan height = %f\")",scanHeight));
			//plotter->send_command("plt.plot(t,r,label=\"r [rad]\",color=\"magenta\")\n");
			//plotter->send_command("plt.plot(t,v,label=\"v\",color=\"cyan\")\n");
			//plotter->send_command("plt.xlabel(\"Rotation angle [rad]\")\nplt.ylabel(\"UV coordition\")\n");
			//plotter->send_command("plt.legend()\n");

			//NodeLensesPathCross��plt������
			constexpr size_t lensespathRes = 360;
			for (std::decay<decltype(lensespathRes)>::type pindex = 0; pindex <= lensespathRes; pindex++) {
				const ureal pireg = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), pindex / (ureal)lensespathRes);

				const auto pathv = NodeLensesPathCross(pireg, scanHeight, theta);//���[�J���ł̃����Y�p�X
				//���̎��̔��a(���[�J�����a)
				const auto radiusNowH = sqrt(1. - clamp(pow(pathv, 2),-1.,1.));
				plotter->send_command(StringFormat("x.append(%f)\ny.append(%f)\nv.append(%f)\n", radiusNowH*cos(pireg), radiusNowH*sin(pireg), pathv));
			}
			plotter->send_command(StringFormat("plt.plot(x,y,v,label=\"v\",color=cm(%f))\n", h / (ureal)(scanheightResolution - 1)));


			plotter->pause(.1);
			plotter->save(StringFormat(rezpath + branchpath + "rez%d.png", h));
		}

		MakeGifAnim(rezpath + branchpath + "pallet.png", rezpath + branchpath + "anim.gif", rezpath + branchpath + "rez%d.png", scanheightResolution);
		plotter->show();
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
#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "Junk/";//����branch�̌��ʂ��i�[����t�H���_
constexpr ureal theta = 5. / 180. * std::numbers::pi;//���񂾂��X����
const std::pair<ureal,ureal> scanHeightRange = make_pair(-0.95,0.95);

using py = pythonRuntime;

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

	//���[�J�����W�n�ŕ\�����@�O���[�o������=���(�܂�����͏�ɓ�������)�ŋO�Ղ������āA��������[�J��v�ł݂��Ƃ��ɒ���(�����Y��evenly�ɃX�L�������邽�߂�)�ɃX�L�������邽�߂̃����Y�z�u��`��

	try {
		//python�����^�C�����������Ă��낢�돉������
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\n");

		//������ς���plt
		constexpr size_t scanheightResolution = 100;
		for (std::decay<decltype(scanheightResolution)>::type h = 0; h < scanheightResolution; h++) {
			py::s("x=[]\ny=[]\nv=[]\n");//�O��plt�p�ϐ���������

			//���܂�scan����(�O���[�o��)
			const auto scanHeight = uleap(scanHeightRange, h / (ureal)(scanheightResolution - 1));

			constexpr size_t circleRes = 360;
			for (std::decay<decltype(circleRes)>::type phi = 0; phi < circleRes; phi++) {
				const ureal t = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), phi / (ureal)circleRes);//t���v�Z

				const auto scanUV = ScanPointWithGh(t, scanHeight, theta);//UV���W�Q�b�g UV�����[�J��
				const auto scanHFromLensesPath = ScanHeightFromLensesPath(t, scanHeight, theta, NodeLensesPathCross);//�����Y�p�X����̍��� UV�����[�J��
			}

			//NodeLensesPathCross��plt������
			constexpr size_t lensespathRes = 360;
			for (std::decay<decltype(lensespathRes)>::type pindex = 0; pindex <= lensespathRes; pindex++) {
				const ureal pireg = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), pindex / (ureal)lensespathRes);

				const auto pathv = NodeLensesPathCross(pireg, scanHeight, theta);//���[�J���ł̃����Y�p�X
				//���̎��̔��a(���[�J�����a)
				const auto radiusNowH = sqrt(1. - clamp(pow(pathv, 2),-1.,1.));
				py::s(StringFormat("x.append(%f)\ny.append(%f)\nv.append(%f)\n", radiusNowH*cos(pireg), radiusNowH*sin(pireg), pathv));
			}
			//plt���ăt�@�C���ɕۑ�
			auto col = HsvToRgb({ h/100.,1.,1. });
			py::s(StringFormat(StringFormat("mlab.plot3d(x,y,v,color=(%f,%f,%f))\n", col.at(0), col.at(1), col.at(2))));
			py::s(StringFormat("mlab.savefig(filename=\'%s\')", StringFormat(rezpath + branchpath + "rez%d.png", h)));

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		//�I�������ӂ��ɕ\������gif�A�j�������
		MakeGifAnim(rezpath + branchpath + "pallet.png", rezpath + branchpath + "anim.gif", rezpath + branchpath + "rez%d.png", scanheightResolution);
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
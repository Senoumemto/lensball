#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "Magsim/";//����branch�̌��ʂ��i�[����t�H���_
using py = pythonRuntime;

int main() {

	try {
		//python�����^�C�����������Ă��낢�돉������
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\n");

		//�܂��̓x�N�g����̗��K�@�����̏�Ɍ���=�ʒu�̏�����
		std::list<arrow3> vfield;
		constexpr ureal halfCubeEdgeLength = 2./2.;//�L���[�u�̈�ӂ̒����̔���
		constexpr std::array<size_t, 3> cubeResolution = { 5,5,5 };
		for(std::decay<decltype(cubeResolution)::value_type>::type z=0;z<cubeResolution.at(2);z++)
			for (std::decay<decltype(cubeResolution)::value_type>::type y = 0; y < cubeResolution.at(1); y++)
				for (std::decay<decltype(cubeResolution)::value_type>::type x = 0; x < cubeResolution.at(0); x++) {
					const uvec3 nowp(uleap(PairMinusPlus(halfCubeEdgeLength),x/(ureal)(cubeResolution.at(0)-1)),
						uleap(PairMinusPlus(halfCubeEdgeLength), y / (ureal)(cubeResolution.at(1) - 1)), 
						uleap(PairMinusPlus(halfCubeEdgeLength), z / (ureal)(cubeResolution.at(2) - 1)));//���̍��W

					vfield.push_back(arrow3(nowp, nowp.normalized()));
				}
		
		for (const auto& vp : vfield)
			py::SendCommandFormat("mlab.quiver3d(%f,%f,%f,%f,%f,%f,color=(%f,%f,%f))\n",vp.org().x(), vp.org().y(), vp.org().z(), vp.dir().x(), vp.dir().y(), vp.dir().z(), fabs(vp.dir().x()), fabs(vp.dir().y()), fabs(vp.dir().z()));

		//�I�������ӂ��ɕ\������gif�A�j�������
		//MakeGifAnim(rezpath + branchpath + "pallet.png", rezpath + branchpath + "anim.gif", rezpath + branchpath + "rez%d.png", scanheightResolution);
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
#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

constexpr ureal theta = 30. / 180. * std::numbers::pi;//���񂾂��X����
constexpr ureal scanHeight = 0.0;

uvec3 ScanPointGlobal(const ureal t) {
	return uvec3((cos(t + theta) + cos(t - theta)) / 2., sin(t), (sin(theta + t) + sin(theta - t))/2.);
}
//�����ȃX�L�����|�C���g
uvec2 ScanPointWithGh(ureal t,ureal gh,ureal theta) {
	const ureal radius = sqrt(1. - pow(gh, 2));//�X�L���������ł̋O�Ղ̔��a
	//r,v
	return uvec2(atan((radius * sin(t)) / (radius * cos(t) * cos(theta) - gh * sin(theta))), radius * cos(t) * sin(theta) + gh * cos(theta));
}

int main() {

	try {
		//plot�̏���
		auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
		DefinePythonFunctions(plotter);//�~�{�I��{�֐����`
		plotter->send_command("t=[]\nv=[]\nr=[]\n");

		constexpr size_t circleRes = 360;
		for (std::decay<decltype(circleRes)>::type phi = 0; phi < circleRes; phi++) {
			const ureal t = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), phi / (ureal)circleRes);//t���v�Z

			const auto scanP = ScanPointWithGh(t, scanHeight, theta);//UV���W�Q�b�g
			
			//�����uv���W��
			const ureal v = scanP.y();
			const ureal r = scanP.x();

			plotter->send_command(StringFormat("t.append(%f)\nv.append(%f)\nr.append(%f)\n", t, v, r));
		}

		plotter->send_command("plt.clf()\nplt.plot(t,r,label=\"r [rad]\",color=\"magenta\")\n");
		plotter->send_command("plt.plot(t,v,label=\"v\",color=\"cyan\")\n");
		plotter->send_command("plt.xlabel(\"Rotation angle [rad]\")\nplt.ylabel(\"UV coordition\")\n");
		plotter->send_command("plt.legend()\n");
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
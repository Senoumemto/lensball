#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

constexpr ureal theta = 30. / 180. * std::numbers::pi;//こんだけ傾ける
const std::pair<ureal,ureal> scanHeightRange = make_pair(-0.8,0.8);

//ローカルから見たスキャン位置　t 回転角度,gh スキャンラインの高さ(グローバル)　theta ローカルの傾き
uvec2 ScanPointWithGh(ureal t,ureal gh,ureal theta) {
	const ureal radius = sqrt(1. - pow(gh, 2));//スキャン高さでの軌跡の半径
	//r,v
	return uvec2(atan((radius * sin(t)) / (radius * cos(t) * cos(theta) - gh * sin(theta))), radius * cos(t) * sin(theta) + gh * cos(theta));
}
//レンズ配置パス関数　ローカル　これに沿ってスキャンライン近辺のレンズが並んでいる
ureal NodeLensesPath(ureal t,ureal gh,ureal theta) {
	return sqrt(3) / 2. * gh;
}
//レンズ配置パス関数　ローカル　これに沿ってレンズを並べると理想通りのスキャン軌跡の高さとなる
ureal NodeLensesPathCross(ureal t, ureal gh, ureal theta) {
	return sqrt(1 - pow(gh,2)) * cos(t) * sin(theta) - (2 * sqrt(1 - pow(gh,2)) * t * sin(theta)) / std::numbers::pi + gh * cos(theta);
}
//レンズパスからのスキャン軌跡の高さ レンズパスを指定してね
ureal ScanHeightFromLensesPath(ureal t, ureal gh, ureal theta, const std::function<ureal(ureal, ureal, ureal)>& lensespath) {
	return ScanPointWithGh(t, gh, theta)[1] - lensespath(t, gh, theta);
}

int main() {

	try {
		//plotの準備
		auto plotter = SetupPythonRuntime();//pythonをセットアップする
		DefinePythonFunctions(plotter);//梅本的基本関数を定義

		//高さを変えてplt
		constexpr size_t scanheightResolution = 40;
		while(1)for (std::decay<decltype(scanheightResolution)>::type h = 0; h < scanheightResolution; h++) {
			plotter->send_command("plt.clf()\n");
			plotter->send_command("t=[]\nv=[]\nr=[]\n");

			//いまのscan高さ
			const auto scanHeight = uleap(scanHeightRange, h / (ureal)(scanheightResolution - 1));
			cout << scanHeight << endl;

			constexpr size_t circleRes = 360;
			for (std::decay<decltype(circleRes)>::type phi = 0; phi < circleRes; phi++) {
				const ureal t = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), phi / (ureal)circleRes);//tを計算

				const auto scanUV = ScanPointWithGh(t, scanHeight, theta);//UV座標ゲット UV球ローカル
				const auto scanHFromLensesPath = ScanHeightFromLensesPath(t, scanHeight, theta, NodeLensesPathCross);//レンズパスからの高さ UV球ローカル
				//...これを直線にしなきゃいけない　

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
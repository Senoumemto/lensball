#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "Junk/";//このbranchの結果を格納するフォルダ
constexpr ureal theta = 5. / 180. * std::numbers::pi;//こんだけ傾ける
const std::pair<ureal,ureal> scanHeightRange = make_pair(-0.95,0.95);

using py = pythonRuntime;

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
	return sqrt(1 - pow(gh,2)) * cos(t) * sin(theta) - (2 * sqrt(1 - pow(gh,2)) * t * sin(theta)) / (std::numbers::pi/2.) + gh * cos(theta);
}
//レンズパスからのスキャン軌跡の高さ レンズパスを指定してね
ureal ScanHeightFromLensesPath(ureal t, ureal gh, ureal theta, const std::function<ureal(ureal, ureal, ureal)>& lensespath) {
	return ScanPointWithGh(t, gh, theta)[1] - lensespath(t, gh, theta);
}

int main() {

	//ローカル座標系で表示中　グローバル高さ=一定(つまり光線は常に同じ方向)で軌跡を書いて、それをローカルvでみたときに直線(レンズをevenlyにスキャンするために)にスキャンするためのレンズ配置を描画

	try {
		//pythonランタイムを準備していろいろ初期処理
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\n");

		//高さを変えてplt
		constexpr size_t scanheightResolution = 100;
		for (std::decay<decltype(scanheightResolution)>::type h = 0; h < scanheightResolution; h++) {
			py::s("x=[]\ny=[]\nv=[]\n");//軌跡plt用変数を初期化

			//いまのscan高さ(グローバル)
			const auto scanHeight = uleap(scanHeightRange, h / (ureal)(scanheightResolution - 1));

			constexpr size_t circleRes = 360;
			for (std::decay<decltype(circleRes)>::type phi = 0; phi < circleRes; phi++) {
				const ureal t = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), phi / (ureal)circleRes);//tを計算

				const auto scanUV = ScanPointWithGh(t, scanHeight, theta);//UV座標ゲット UV球ローカル
				const auto scanHFromLensesPath = ScanHeightFromLensesPath(t, scanHeight, theta, NodeLensesPathCross);//レンズパスからの高さ UV球ローカル
			}

			//NodeLensesPathCrossをpltしたい
			constexpr size_t lensespathRes = 360;
			for (std::decay<decltype(lensespathRes)>::type pindex = 0; pindex <= lensespathRes; pindex++) {
				const ureal pireg = uleap(std::make_pair(-std::numbers::pi, +std::numbers::pi), pindex / (ureal)lensespathRes);

				const auto pathv = NodeLensesPathCross(pireg, scanHeight, theta);//ローカルでのレンズパス
				//その時の半径(ローカル半径)
				const auto radiusNowH = sqrt(1. - clamp(pow(pathv, 2),-1.,1.));
				py::s(StringFormat("x.append(%f)\ny.append(%f)\nv.append(%f)\n", radiusNowH*cos(pireg), radiusNowH*sin(pireg), pathv));
			}
			//pltしてファイルに保存
			auto col = HsvToRgb({ h/100.,1.,1. });
			py::s(StringFormat(StringFormat("mlab.plot3d(x,y,v,color=(%f,%f,%f))\n", col.at(0), col.at(1), col.at(2))));
			py::s(StringFormat("mlab.savefig(filename=\'%s\')", StringFormat(rezpath + branchpath + "rez%d.png", h)));

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		//終わったらふつうに表示してgifアニメを作る
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
#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "Junk/";//このbranchの結果を格納するフォルダ
constexpr ureal theta = 5. / 180. * std::numbers::pi;//こんだけ傾ける
const std::pair<ureal,ureal> scanHeightRange = make_pair(-0.95,0.95);

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

	try {
		//plotの準備
		auto plotter = SetupPythonRuntime();//pythonをセットアップする
		//DefinePythonFunctions(plotter);//梅本的基本関数を定義
//		plotter->send_command(R"(
//# 軸範囲の設定
//ax.set_xlim(-1.,1.)
//ax.set_ylim(-1.,1.)
//ax.set_zlim(-1.,1.))");
//		plotter->send_command("cm = plt.get_cmap(\"Spectral\")\n");//カラーセットを作る

		//高さを変えてplt
		constexpr size_t scanheightResolution = 100;
		for (std::decay<decltype(scanheightResolution)>::type h = 0; h < scanheightResolution; h++) {
			//plotter->send_command("#ax.clear()\n");
			plotter->send_command("x=[]\ny=[]\nv=[]\n");

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
				plotter->send_command(StringFormat("x.append(%f)\ny.append(%f)\nv.append(%f)\n", radiusNowH*cos(pireg), radiusNowH*sin(pireg), pathv));
			}
			plotter->send_command(StringFormat("mlab.plot3d(x,y,v)\n"));

			plotter->send_command(StringFormat("mlab.savefig(filename=\'%s\')", StringFormat(rezpath + branchpath + "rez%d.png", h)));
			//plotter->pause(.1);
			//plotter->save(StringFormat(rezpath + branchpath + "rez%d.png", h));

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		plotter->send_command("mlab.show()\n");
		MakeGifAnim(rezpath + branchpath + "pallet.png", rezpath + branchpath + "anim.gif", rezpath + branchpath + "rez%d.png", scanheightResolution);
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
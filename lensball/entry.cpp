#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int MMain();

int main() {
	try {
		return MMain();
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

int MMain() {
	/*
	レンズを動かして、光の散り方を知りたい
	レンズの動きxを軸にとってdirのx,yをプロット
	**********y
	入射光の方向を法線からずらして行う
	
	
	*/

	printf("hello v1235\n");

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義


	//UV球のための球パラメータを宣言
	const sphereParam centerUVSphereParam = make_pair(uvec3(0., 0., 0.), 2.);

	//UV球の頂点を算出
	std::list<uvec3>uvPoses;
	constexpr size_t numOfVerticalResolution = 10;//UV球の縦方向の分解能
	constexpr size_t numOfRadicalResolution = 12;//UV球の放射方向の分解能
	for(std::decay<decltype(numOfVerticalResolution)>::type v=0;v<numOfVerticalResolution;v++)
		for (std::decay<decltype(numOfRadicalResolution)>::type r = 0; r < numOfRadicalResolution; r++) {
			//まずはvを正規化する　座標でね
			const auto vpos = uleap(make_pair(-centerUVSphereParam.second, +centerUVSphereParam.second), v / (ureal)(numOfVerticalResolution - 1));
			const auto rpos = uleap(make_pair(-std::numbers::pi, +std::numbers::pi), r / (ureal)(numOfRadicalResolution));//rもね るーぷだから(最初=最後だから)1ひかない

			//vposでのuv断面の半径を求める x^2+y^2=r^2 y^2=r^2-x^2
			const ureal radiusOfCross = sqrt(pow(centerUVSphereParam.second, 2) - pow(vpos, 2));

			//直行座標にして追加
			uvPoses.push_back(uvec3(radiusOfCross * cos(rpos), radiusOfCross * sin(rpos), vpos) + centerUVSphereParam.first);
		}
		
	//次に極を傾ける
	const auto tiltOfPoll = Eigen::AngleAxis<ureal>(15. / 180. * std::numbers::pi, uvec3(0., 1., 0.));
	for (auto& p : uvPoses)p = tiltOfPoll * p;

	//つぎにレイを当ててマッピングしたいので当てるレイを定義する
	const arrow3 mapRay = arrow3(uvec3(0., -5., 0.), uvec3(0., 1., 0.));
	std::list<uvec3> mappingList;

	//まわす
	constexpr size_t rotationResolution = 60;
	for (int xx = 0; xx < 1; xx++) {
		for (std::decay<decltype(rotationResolution)>::type rotIndex = 0; rotIndex < rotationResolution; rotIndex++) {
			const ureal rotAngle = uleap(make_pair(-std::numbers::pi, +std::numbers::pi), rotIndex / (ureal)rotationResolution);//正規化　ループなことに注意
			const Eigen::AngleAxis<ureal> rotMat = Eigen::AngleAxis<ureal>(rotAngle, uvec3(0., 0., 1.));

			//マッピングする もちろん今のローカル変換も含めてね
			const auto rez = IntersectSphere(mapRay, centerUVSphereParam.first, centerUVSphereParam.second);
			if (!rez.isHit)throw logic_error("dameda");
			cout << rez.pos << endl;
			mappingList.push_back(rotMat * rez.pos);



			//フレームクリア
			plotter->send_command(R"(
plt.cla()
#ax = fig.add_subplot(111, projection = '3d')
ax.set_box_aspect((1, 1, 1))
# 軸ラベルの設定
ax.set_xlabel("x", fontsize = 16)
ax.set_ylabel("y", fontsize = 16)
ax.set_zlabel("z", fontsize = 16)
# 軸範囲の設定
ax.set_xlim(-3.,3.)
ax.set_ylim(-3.,3.)
ax.set_zlim(-3.,3.))");
			//Start drawing
			plotter->send_command("ax.view_init(elev=0\n)");//視点を設定

			//マッピング線を書く
			if (false) {
				plotter->send_command("x=[]\ny=[]\nz=[]\n");
				for (const auto& p : mappingList) {
					plotter->send_command(StringFormat(""
						"x.append(%f)\n"
						"y.append(%f)\n"
						"z.append(%f)\n"
						"\n", p.x(), p.y(), p.z()));
				}
				plotter->send_command("ax.plot(x,y,z,color=\"red\")\n");
			}

			//Draw uv vertices
			for (auto p : uvPoses) {
				p = rotMat * p;
				plotter->send_command(StringFormat("ax.scatter(%f,%f,%f)\n", p.x(), p.y(), p.z()));
			}

			//結果を保存
			const std::string resultsPathPrefix = R"(C:/local/user/lensball/lensball/results2/)";
			plotter->save(resultsPathPrefix + StringFormat("rez%d.png", rotIndex));
			plotter->pause();
		}
	}
	plotter->show();
	plotter->send_command("plt.cla()\nplt.clf()\n");//pyplot終了ポリシー
	plotter->close();
	return 0;
}
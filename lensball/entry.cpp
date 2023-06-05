#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "SphereCoord/";//このbranchの結果を格納するフォルダ

using py = pythonRuntime;

constexpr inline ureal pi = std::numbers::pi;//円周率のエイリアス

//極座標にて...一次元のプロジェクタから放出されたレイ(phi=0面)とphiで回転する球の交差軌跡
uvec2 RayHitPath(const ureal rayTheta, const ureal& t) {
	return uvec2(rayTheta, t);
}

//要素レンズの広がりをlsize(単位 theta)としたときのレンズのアラインメント
uvec2 LensAlignment(const ureal& t, const ureal& lsize, const ureal rayTheta) {
	const ureal theta = -(lsize) / (2. * std::numbers::pi) * t + rayTheta;
	return uvec2(theta, t);
}

//球面極座標を直交座標に変換
uvec3 PolarToXyz(const uvec2& spolar) {
	return uvec3(cos(spolar.x()) * cos(spolar.y()),
		cos(spolar.x()) * sin(spolar.y()),
		sin(spolar.x()));
}

int main() {

	try {
		//pythonランタイムを準備していろいろ初期処理
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

		//mayaviの設定
		const std::pair<size_t, size_t> figResolution(800, 600);
		py::sf("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

		//プロジェクタから放出するレイはtheta方向に広がる
		constexpr size_t rayWayResolution = 5;
		constexpr ureal projectorHalfAngle = 30. / 180. * pi;
		//球の回し方
		constexpr size_t rotationResolution = 100;

		constexpr ureal lensSizeOnTheta = (180. / 10.) / 180. * pi;//レンズのtheta方向の大きさ

		for (std::decay<decltype(rayWayResolution)>::type rayWayD = 0; rayWayD < rayWayResolution; rayWayD++) {
			const ureal rayWay = uleap(PairMinusPlus(projectorHalfAngle), rayWayD / (ureal)(rayWayResolution - 1));//レイの方向(theta)

			py::s("s=[[],[],[]]\nl=[[],[],[]]");//スキャンパスとレンズ
			py::s("lsp=[[],[]]");//レンズの極座標

			//球をぐるぐる回す　phi方向
			for (std::decay<decltype(rotationResolution)>::type tD = 0; tD < rotationResolution; tD++) {
				const ureal t = uleap(PairMinusPlus(pi), tD / (ureal)(rotationResolution - 1));//回転角度(phi)

				const auto scanPath = PolarToXyz(RayHitPath(rayWay, t));//このレイが当たった点(ローカル)
				const auto bestLensPos = PolarToXyz(LensAlignment(t, lensSizeOnTheta, rayWay));//レイの交点の直下のレンズの場所(ローカル)

				//描画を転送する
				py::sf("s[0].append(%f)\ns[1].append(%f)\ns[2].append(%f)\nl[0].append(%f)\nl[1].append(%f)\nl[2].append(%f)\n", scanPath.x(), scanPath.y(), scanPath.z(), bestLensPos.x(), bestLensPos.y(), bestLensPos.z());
				py::sf("lsp[0].append(%f)\nlsp[1].append(%f)", LensAlignment(t, lensSizeOnTheta, rayWay).x(), LensAlignment(t, lensSizeOnTheta, rayWay).y());
			}
			//レイノスキャンパスとレンズの配置を描画
			py::s("mlab.plot3d(s[0],s[1],s[2])");
			py::s("mlab.plot3d(l[0],l[1],l[2])");

			//二次元でもplt
			py::s("plt.plot(lsp[0],lsp[1],\"r--\")");

			const auto rayTerm = PolarToXyz(uvec2(rayWay, 0.));//レイを描画する　原点から...ここまで
			py::sf("mlab.plot3d([0,%f],[0,%f],[0,%f],color=(%f,0.5,0.5))", rayTerm.x(), rayTerm.y(), rayTerm.z(), (rayWay + projectorHalfAngle) / (2. * projectorHalfAngle));
		}

		//表示する 3d 2dの順
		py::s("mlab.show()");
		py::s("plt.show()");
		
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
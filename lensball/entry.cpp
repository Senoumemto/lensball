#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "SphereCoord/";//このbranchの結果を格納するフォルダ

using py = pythonRuntime;

//極座標にて...一次元のプロジェクタから放出されたレイ(phi=0面)とphiで回転する球の交差軌跡
uvec2 RayHitPath(const ureal rayTheta, const ureal& t) {
	return uvec2(rayTheta, t);
}

//要素レンズの広がりをlsize(単位 theta)としたときのレンズのアラインメント
uvec2 LensAlignment(const ureal& t, const ureal& lsize, const ureal rayTheta) {
	const ureal theta = -(lsize) / (2.*std::numbers::pi)+rayTheta;
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
		py::s("import numpy as np\nfrom mayavi import mlab\n");

		//mayaviの設定
		const std::pair<size_t, size_t> figResolution(800, 600);
		py::s(StringFormat("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second));

		//まずテストとして球を書いてみる
		const size_t thetaRes = 100;
		const size_t phiRes = 100;

		for (size_t t = 0; t < thetaRes; t++) {

			py::s("x=[]\ny=[]\nz=[]\n");//軌跡plt用変数を初期化
			for (size_t p = 0; p < phiRes; p++){
				const uvec2 nowp(uleap(PairMinusPlus(std::numbers::pi / 2.), t / (ureal)(thetaRes-1)), uleap(PairMinusPlus(std::numbers::pi), p / (ureal)(phiRes-1)));

				const uvec3 xyz = PolarToXyz(nowp);
				cout << xyz << endl;
				py::s(StringFormat("x.append(%f)\ny.append(%f)\nz.append(%f)\n", xyz.x(), xyz.y(), xyz.z()));
			}


			py::s("mlab.plot3d(x,y,z)\n");
		}

		py::s("mlab.show()");
		
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
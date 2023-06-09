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

//球面極座標を直交座標に変換 //
uvec3 PolarToXyz(const uvec2& spolar) {
	return uvec3(cos(spolar.x()) * cos(spolar.y()),
		cos(spolar.x()) * sin(spolar.y()),
		sin(spolar.x()));
}
uvec3 PolarToXyz(const ureal phi,const ureal theta) {
	return PolarToXyz(uvec2(theta, phi));
}

int main() {

	try {
		//pythonランタイムを準備していろいろ初期処理
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

		//mayaviの設定
		const std::pair<size_t, size_t> figResolution(800, 600);
		py::sf("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

		//球を描画する
		constexpr ureal sphereRadius = 1.;
		constexpr size_t sphereResolution = 20;
		py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
x = np.cos(sphphi)*np.sin(sphtheta)
y = np.sin(sphphi)*np.sin(sphtheta)
z = np.cos(sphtheta)
mlab.mesh(%f*x, %f*y, %f*z )  
)", sphereResolution, sphereResolution, sphereRadius, sphereRadius, sphereRadius);

		//polar座標系においておんなじ大きさを持って並んでいるとき
		constexpr ureal lensnumTheta = 10;
		constexpr ureal lensnumPhi = lensnumTheta * 2;//phiは二倍の広がりを持ってるから
		constexpr ureal lensSize = (pi) / (lensnumTheta * 2.);//レンズの半径を決める 半周を数のバイで割ればいい

		constexpr size_t lensResolution = 20;//レンズの外形円の解像度
		py::sf("lentheta = np.linspace(-np.pi,np.pi,%d)",lensResolution);//python側で作っとく

		for(std::decay<decltype(lensnumTheta)>::type td=0;td<lensnumTheta;td++)
			for (std::decay<decltype(lensnumPhi)>::type pd = 0; pd < lensnumPhi; pd++) {
				//中心位置を作る
				const auto nowp = uvec2(uleap(PairMinusPlus(pi), pd / (ureal)lensnumPhi) + (2.*pi / (ureal)(2*lensnumPhi)),
					uleap(PairMinusPlus(pi/2.), td / (ureal)lensnumTheta) + ((pi/2.*2.) / (ureal)(2 * lensnumTheta)));


				//極座標に変換する
				py::s("x=[]\ny=[]\nz=[]\nlenx=[]\nleny=[]");
				for (std::decay<decltype(lensResolution)>::type ld = 0; ld < lensResolution; ld++) {
					const ureal lenCycle = uleap(PairMinusPlus(pi), ld / (ureal)(lensResolution-1));
					//球面座標にマッピング座標を作る ローカル
					const ureal localphi = lensSize * cos(lenCycle)+ nowp.x();//経度を出す　こちらは球面上ではcos(theta)倍される
					const ureal localtheta = lensSize * sin(lenCycle)+ nowp.y();//まず今の緯度を出す こっちもcos(theta倍すればいいやん)
					uvec2 phiV = uvec2(localphi, 2.*atan(pow(std::numbers::e,localtheta))-pi/2.);//2Dマップ座標
					py::sf("lenx.append(%f)\nleny.append(%f)", phiV.x(), phiV.y());

					//これをどうマップするか　極座標系で渡せばいいから
					const auto polarpos = PolarToXyz(phiV.x(), phiV.y());

					py::sf("x.append(%f)\ny.append(%f)\nz.append(%f)", polarpos.x(), polarpos.y(), polarpos.z());
				}

				//プロット
				py::s("plt.plot(lenx,leny)");
				py::s("mlab.plot3d(x,y,z)");

			}


		//表示する 3d 2dの順
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
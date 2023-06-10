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

		//matplotlibの設定
		py::s("fig, ax = plt.subplots()\nax.set_aspect(\"equal\")");

		//球を描画する
		constexpr ureal sphereRadius = 1.;
		constexpr size_t sphereResolution = 20;
		py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
x = np.cos(sphphi)*np.sin(sphtheta)
y = np.sin(sphphi)*np.sin(sphtheta)
z = np.cos(sphtheta)
mlab.mesh(%f*x, %f*y, %f*z ,color=(1.,1.,1.) )  
)", sphereResolution, sphereResolution, sphereRadius, sphereRadius, sphereRadius);

		//レンズアレイを作成
		//縦を基準に考える　列が合ってずらしながら描画していく
		constexpr size_t lensNumInCollum = 10;
		constexpr size_t collumNum = lensNumInCollum * 2;//範囲的に倍
		constexpr ureal lensRadiusInMap = pi / (ureal)lensNumInCollum/2.;//地図上でのレンズの経(角度)
		constexpr ureal shiftSizeEachCollum = lensRadiusInMap * 2.;//列ごとにどれだけ位置をシフトさせるか

		constexpr size_t lensResolution = 20;//レンズの外形円の解像度

		for (std::decay<decltype(collumNum)>::type cd = 0; cd < collumNum; cd++) {//列ごとに
			//列の位置を計算する(eq)
			const ureal colPosEquater = uleap(PairMinusPlus(pi), cd / (ureal)collumNum) + lensRadiusInMap;
			//今の列のオフセットを計算する
			const ureal nowOffsetLatitude = shiftSizeEachCollum * uleap(PairMinusPlus(0.5), cd / (ureal)collumNum);

			for (std::decay<decltype(lensNumInCollum)>::type lid = 0; lid < lensNumInCollum; lid++) {//列内の要素レンズについて
				
				//レンズ中心位置を作る　equater latitude
				const auto nowp = uvec2(colPosEquater, uleap(PairMinusPlus(pi / 2.), lid / (ureal)lensNumInCollum) + lensRadiusInMap + nowOffsetLatitude);


				//極座標に変換する
				py::s("x=[]\ny=[]\nz=[]\nlenx=[]\nleny=[]");
				for (std::decay<decltype(lensResolution)>::type ld = 0; ld < lensResolution; ld++) {
					const ureal lenCycle = uleap(PairMinusPlus(pi), ld / (ureal)(lensResolution - 1));
					//球面座標にマッピング座標を作る ローカル
					const ureal locallon = lensRadiusInMap * cos(lenCycle) + nowp.x();//経度を出す　こちらは球面上ではcos(theta)倍される
					const ureal locallati = lensRadiusInMap * sin(lenCycle) + nowp.y();//まず今の緯度を出す こっちもcos(theta倍すればいいやん)

					//uvec2 mapped = uvec2(locallon,locallati);//2Dマップ座標 そのまま
					uvec2 mapped = uvec2(locallon, 2. * atan(-pow(std::numbers::e, -locallati)) + pi / 2.);//2Dマップ座標 メルカトル
					py::sf("lenx.append(%f)\nleny.append(%f)", mapped.x(), mapped.y());

					//これをどうマップするか　極座標系で渡せばいいから
					const auto polarpos = PolarToXyz(mapped.x(), mapped.y());

					py::sf("x.append(%f)\ny.append(%f)\nz.append(%f)", polarpos.x(), polarpos.y(), polarpos.z());
				}

				//プロット
				auto color = HsvToRgb({ uleap({0.,1.},cd / (ureal)collumNum),1.,1. });
				py::sf("plt.plot(lenx,leny,color=(%f,%f,%f))", color[0], color[1], color[2]);
				py::sf("mlab.plot3d(x,y,z,color=(%f,%f,%f))", color[0], color[1], color[2]);

			}
		}

		//スキャンをする
		constexpr size_t projectorResInTheta = 40;
		constexpr ureal projectorHalfAngle = 60. / 180. * pi;
		constexpr size_t scanLineResolutionPhi = 180;
		for (std::decay<decltype(projectorResInTheta)>::type sd = 0; sd < projectorResInTheta; sd++) {
			//スキャンラインの高さ
			const ureal scanTheta = uleap(PairMinusPlus(projectorHalfAngle), sd / (ureal)(projectorResInTheta - 1));

			//ラインを描画する
			py::s("x=[]\ny=[]\nz=[]\nslx=[]\nsly=[]");
			for (std::decay<decltype(scanLineResolutionPhi)>::type rd = 0; rd < scanLineResolutionPhi; rd++) {
				const ureal time = uleap(PairMinusPlus(pi), rd / (ureal)(scanLineResolutionPhi - 1));

				uvec2 mapped = uvec2(time,scanTheta);
				py::sf("slx.append(%f)\nsly.append(%f)", mapped.x(), mapped.y());

				//これをどうマップするか　極座標系で渡せばいいから
				const auto polarpos = PolarToXyz(mapped.x(), mapped.y());
				py::sf("x.append(%f)\ny.append(%f)\nz.append(%f)", polarpos.x(), polarpos.y(), polarpos.z());
			}

			//plt
			auto color = HsvToRgb({ uleap({0.,1.},sd / (ureal)projectorResInTheta),1.,0.5 });
			py::sf("plt.plot(slx,sly,color=(%f,%f,%f))", color[0], color[1], color[2]);
			py::sf("mlab.plot3d(x,y,z,color=(%f,%f,%f),tube_radius=0.01)", color[0], color[1], color[2]);
		}

		//表示する 3d 2dの順
		py::s("plt.show()");
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		py::Terminate();
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		py::Terminate();
		return -2;
	}


	py::Terminate();
	return 0;
}
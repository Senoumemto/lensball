#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "Magsim/";//このbranchの結果を格納するフォルダ
using py = pythonRuntime;

int main() {

	try {
		//pythonランタイムを準備していろいろ初期処理
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\n");

		//まずはベクトル場の練習　立方体上に向き=位置の場を作る
		std::list<arrow3> vfield;
		constexpr ureal halfCubeEdgeLength = 2./2.;//キューブの一辺の長さの半分
		constexpr std::array<size_t, 3> cubeResolution = { 5,5,5 };
		for(std::decay<decltype(cubeResolution)::value_type>::type z=0;z<cubeResolution.at(2);z++)
			for (std::decay<decltype(cubeResolution)::value_type>::type y = 0; y < cubeResolution.at(1); y++)
				for (std::decay<decltype(cubeResolution)::value_type>::type x = 0; x < cubeResolution.at(0); x++) {
					const uvec3 nowp(uleap(PairMinusPlus(halfCubeEdgeLength),x/(ureal)(cubeResolution.at(0)-1)),
						uleap(PairMinusPlus(halfCubeEdgeLength), y / (ureal)(cubeResolution.at(1) - 1)), 
						uleap(PairMinusPlus(halfCubeEdgeLength), z / (ureal)(cubeResolution.at(2) - 1)));//今の座標

					vfield.push_back(arrow3(nowp, nowp.normalized()));
				}
		
		for (const auto& vp : vfield)
			py::SendCommandFormat("mlab.quiver3d(%f,%f,%f,%f,%f,%f,color=(%f,%f,%f))\n",vp.org().x(), vp.org().y(), vp.org().z(), vp.dir().x(), vp.dir().y(), vp.dir().z(), fabs(vp.dir().x()), fabs(vp.dir().y()), fabs(vp.dir().z()));

		//終わったらふつうに表示してgifアニメを作る
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
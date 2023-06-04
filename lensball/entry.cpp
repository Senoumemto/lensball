#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "Magsim/";//このbranchの結果を格納するフォルダ
using py = pythonRuntime;

void DrawCoil(const ureal radius, const ureal length, const arrow3& posdir, const ureal& pitch, const size_t& split) {
	//何周するのか pitch一周する間に進む距離
	const ureal cycle = length / pitch;

	//コイルローカルからグローバルに至る変換を作る
	const uvec3 rotAxis = uvec3(0, 0, 1.).cross(posdir.dir());
	const ureal rotAngle = acos(uvec3(0, 0, 1.).dot(posdir.dir()));
	const uvec3 rotVec = rotAngle * rotAxis;
	py::SendCommandFormat("springrot=Rotation.from_rotvec([%f,%f,%f])",rotVec.x(), rotVec.y(), rotVec.z());

	//コイルを書く
	py::SendCommandFormat("springt=np.linspace(0.,2.*math.pi*%f,%d)", cycle, split);//まず媒介変数を宣言 角度
	py::SendCommandFormat("springvertices=springrot.apply(np.transpose(np.array([np.sin(springt),np.cos(springt),(%f*(springt-math.pi*%f))])))", pitch, cycle);
	py::SendCommandFormat("mlab.plot3d(springvertices[:,0]+%f,springvertices[:,1]+%f,springvertices[:,2]+%f)",posdir.org().x(), posdir.org().y(), posdir.org().z());

	return;
}

int main() {

	try {
		//pythonランタイムを準備していろいろ初期処理
		py::Init();
		py::s("import numpy as np\nfrom mayavi import mlab\nimport math\nfrom scipy.spatial.transform import Rotation\n");

		DrawCoil(1, 0.5, arrow3(uvec3(0, 0, 0), uvec3(0, 0, 1)), 0.025, 8 / 0.025);
		DrawCoil(1, 0.5, arrow3(uvec3(0, 0, 0), uvec3(0, 1, 0)), 0.025, 8 / 0.025);

		/*//まずはベクトル場の練習　立方体上に向き=位置の場を作る
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
			*/
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
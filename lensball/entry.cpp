#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {

	printf("hello v1234\n");

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義

	//要素レンズを定義　旧レンズに
	const ureal nodeLensEta = 1.5;
	const sphereParam nodeLensParam = make_pair(uvec3(0., 0., 0.), 1.);

	//レイを生成する
	list<ray3> rays;
	rays.push_back(ray3(arrow3(uvec3(0.25,0.,10.),uvec3(0.,0.,-1.))));//初期位置を作る
	rays.push_back(ray3(arrow3(uvec3(-0.25, 0., 10.), uvec3(0., 0., -1.))));//初期位置を作る
	for (ray3& target : rays) {
		//レイトレパイプライン
		[&] {
			const auto rez0 = IntersectSphere(target.back(), nodeLensParam.first, nodeLensParam.second);//要素レンズと交差

			//当たらなかったらこれ以上やる必要はない
			if (!rez0.isHit) {
				FreeFlightRay(target);
				return;
			}

			//ここからレンズ内の処理
			rez0.ApplyToRay(target);
			if (!RefractSnell(target, rez0.norm, nodeLensEta))throw runtime_error("全反射が起きた");//屈折計算

			const auto rez1 = IntersectSphere(target.back(), nodeLensParam.first, nodeLensParam.second);//要素レンズ内部を通過
			if (!rez1.isHit)throw logic_error("logic err0");//レンズ内部なので絶対当たる
			rez1.ApplyToRay(target);//進める

			if (!RefractSnell(target, -rez1.norm, 1. / nodeLensEta))throw runtime_error("全反射が起きた");//屈折計算

			FreeFlightRay(target);

		}();
	}

	//要素レンズを描画
	DrawSphere(plotter, nodeLensParam.first, nodeLensParam.second, 10, R"("red")");
	//すべてのレイを描画
	for(const auto&r:rays)
		DrawRaySkipFirstArrow(plotter, r, R"("green")");//レイを描画

	plotter->show();//表示　なんか終わらん
	plotter->close();//終了　あんまり意味がない
	return 0;
}
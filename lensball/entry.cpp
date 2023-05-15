#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {

	printf("hello\n");

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義

	//要素レンズを定義　旧レンズに
	const sphereParam nodeLensParam = make_pair(uvec3(0., 0., 0.), 1.);

	//レイを生成する
	list<ray3> rays;
	rays.push_back(ray3(arrow3(uvec3(0.,0.,10.),uvec3(0.,0.,-1.))));//初期位置を作る
	ray3& target = rays.back();

	const auto rez=IntersectSphere(target.back(), nodeLensParam.first, nodeLensParam.second);//要素レンズと交差
	if (rez.isHit)//球に当たったら
		rez.ApplyToRay(target);
	else//当たらなかったら
		FreeFlightRay(target);

	//要素レンズを描画
	DrawSphere(plotter, nodeLensParam.first, nodeLensParam.second, 10, R"("red")");
	//すべてのレイを描画
	for(const auto&r:rays)
		DrawRaySkipFirstArrow(plotter, r, R"("green")");//レイを描画

	plotter->show();//表示　なんか終わらん
	plotter->close();//終了　あんまり意味がない
	return 0;
}
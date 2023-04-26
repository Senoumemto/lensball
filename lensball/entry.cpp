#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"

using namespace std;

int main() {

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義

	//レンズボールの要素レンズのパラメータを計算する
	auto lensBallsParams = CalcSmallLensPosAndRadius();
	//レンズボールのすべての玉を描画
	for (const auto& param : *lensBallsParams)
		DrawSphere(plotter, param.first, param.second, 5, R"("red")");



	list<ray3> rays;//レイを生成する
	for (int i = 0; i < 4; i++) {
		rays.push_back(ray3(arrow3(uvec3(-5., (ureal)(i-2)/4., 0.), uvec3(1., 0., 0.).normalized())));
		auto result = IntersectSphere(rays.back().back(), uvec3(0, 0, 0), 1.);//いろんな結果を計算
		if (result.isHit) {
			result.ApplyToRay(rays.back());//球の直後まで進ませる
			ReflectMirror(rays.back(), result.norm);//鏡面反射
			FreeFlightRay(rays.back());
		}
		else FreeFlightRay(rays.back());//当たらなければ自由飛行
	}
	//すべてのレイを描画
	for(const auto&r:rays)
		DrawRay(plotter, r, R"("green")");//レイを描画



	plotter->show();//表示　なんか終わらん
	plotter->close();//終了　あんまり意味がない
	return 0;
}
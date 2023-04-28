#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義

	//レンズボールの要素レンズのパラメータを計算する
	auto lensBallsParams = CalcSmallLensPosAndRadius();
	//レンズボールのすべての玉を描画
	for (const auto& param : *lensBallsParams)
		DrawSphere(plotter, param.first, param.second, 5, R"("red")");
	//DrawSphere(plotter, uvec3::Zero(), 1.);

	list<ray3> rays;//レイを生成する
	constexpr size_t timingNum = 1500/60;//一周する間にどれだけ露光する 1.5kHzだとすると
	const std::pair<size_t,size_t> res = make_pair(4,4);//プロジェクタの解像度

	for (int g = 0; g < timingNum; g++) {
		//グローバル->レンズボール変換
		uaffine3 translenses(Eigen::AngleAxisd(2. * std::numbers::pi * ((ureal)g / (ureal)timingNum), uvec3::UnitZ()));
		for (size_t h = 0; h < res.first; h++) {
			for (size_t i = 0; i < res.second; i++) {
				rays.push_back(ray3(arrow3(translenses*uvec3(-5., (ureal)(i - res.first/2.) / res.first, (ureal)(h - res.second/2.) / res.second), translenses * uvec3(1., 0., 0.).normalized())));//初期位置を作る レンズボール座標系で

				//すべての球に対して当たり判定を行う
				resultIntersecteSphere closestRez = resultIntersecteSphere();
				closestRez.t = std::numeric_limits<ureal>::infinity();
				for (const auto& param : *lensBallsParams) {
					auto result = IntersectSphere(rays.back().back(), param.first, param.second);//小球と交差

					if (!result.isHit)continue;
					//もし交差したら
					if (result.t < closestRez.t)closestRez = result;//より近ければ更新
				}

				//さらにブロッカーとの当たり判定をする
				const auto blocker = IntersectSphere(rays.back().back(), uvec3(0, 0, 0), 1.);
				//もしブロッカーにあたったらそこで吸収
				if (blocker.t <= closestRez.t) {
					blocker.ApplyToRay(rays.back());
				}

				//昇給にあたったら
				else if (closestRez.isHit) {
					closestRez.ApplyToRay(rays.back());
					ReflectMirror(rays.back(), closestRez.norm);//鏡面反射
					FreeFlightRay(rays.back());
				}
				else FreeFlightRay(rays.back());//当たらなければ自由飛行*/

				//最後にレイを逆変換して戻す
				const uaffine3 backtrans = translenses.inverse();
				for (auto& rn : rays.back()) {
					rn.dir() = backtrans * rn.dir();
					rn.org() = backtrans * rn.org();
				}
			}
		}
	}

	//すべてのレイを描画
	for(const auto&r:rays)
		DrawRaySkipFirstArrow(plotter, r, R"("green")");//レイを描画



	plotter->show();//表示　なんか終わらん
	plotter->close();//終了　あんまり意味がない
	return 0;
}
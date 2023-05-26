#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int main() {
	/*
	レンズを動かして、光の散り方を知りたい
	レンズの動きxを軸にとってdirのx,yをプロット
	**********y
	入射光の方向を法線からずらして行う
	
	
	*/

	printf("hello v1235\n");

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義
	;


	//レンズを定義していく
	constexpr ureal nodeLensEta = 1.5;
	constexpr ureal lensBallRadius = 5. / 2.;
	constexpr unsigned int nodeLensNum = 10;
	const ureal nodeLensRadius = lensBallRadius * std::sin((std::numbers::pi /(nodeLensNum-1.))/2.);//内接する2(n-1)角形の一辺の長さが直径だから...
	std::list<sphereParam> nodeLensesParams;
	for (std::decay<decltype(nodeLensNum)>::type i = 0; i < nodeLensNum; i++) {
		//値を正規化する-1~1
		const ureal reg = 2. * (i / (ureal)(nodeLensNum - 1)) - 1.;
		//それを角度にする
		const ureal angle = reg * std::numbers::pi/2.;//鉛直からの角度

		//つまり位置が決まる x,y平面でy軸方向をzeroにangleだけ角度を決める
		const uvec3 pos(lensBallRadius * sin(angle), lensBallRadius * cos(angle), 0.);
		nodeLensesParams.push_back(sphereParam(pos, nodeLensRadius));
	}
	//レイを生成する
	list<ray3> rays;
	constexpr unsigned int targetBallsNum = 4;//光を当てるボールの数v ただしnodelensNumが偶数だったら偶数、奇数だったら奇数にすること
	constexpr unsigned int targetBallsOffset = ((nodeLensNum % 2) != (targetBallsNum % 2)) ? 0 : (nodeLensNum - targetBallsNum) / 2;
	const uvec3 projectionOrign(0., 0., 0.);
	constexpr ureal layAngleLimitFromTarget = 5. / 180. * std::numbers::pi;
	auto ite=nodeLensesParams.begin();
	for (int i = 0; i < targetBallsOffset; i++)ite++;
	for (std::decay<decltype(targetBallsNum)>::type i = 0; i < targetBallsNum; i++) {
		const uvec3 targetDirection = ((ite->first) - projectionOrign).normalized();//これがレンズの中心を通るレイの方向
		//ここから+-thetaだけz軸中心に打ち分けたい
		const uvec3 direction0 = Eigen::AngleAxis<ureal>(layAngleLimitFromTarget, uvec3(0, 0, 1)) * targetDirection;
		const uvec3 direction1 = Eigen::AngleAxis<ureal>(-layAngleLimitFromTarget, uvec3(0, 0, 1))*targetDirection;


		rays.push_back(ray3(arrow3(projectionOrign, targetDirection)));//レイを作成
		rays.push_back(ray3(arrow3(projectionOrign, direction0)));//レイを作成
		rays.push_back(ray3(arrow3(projectionOrign, direction1)));//レイを作成

		ite++;
	}
	for (ray3& target : rays) {
		//レイトレパイプライン
		[&] {
			try {
				const auto rez0 = IntersectSphere(target.back(), nodeLensesParams.front().first, nodeLensesParams.front().second);//要素レンズと交差

				//当たらなかったらこれ以上やる必要はない
				if (!rez0.isHit) {
					FreeFlightRay(target);
					return;
				}

				//ここからレンズ内の処理
				rez0.ApplyToRay(target);
				if (!RefractSnell(target, rez0.norm, nodeLensEta))throw runtime_error("全反射が起きた");//屈折計算

				const auto rez1 = IntersectSphere(target.back(), nodeLensesParams.front().first, nodeLensesParams.front().second);//要素レンズ内部を通過
				if (!rez1.isHit)throw logic_error("logic err0");//レンズ内部なので絶対当たる
				rez1.ApplyToRay(target);//進める

				if (!RefractSnell(target, -rez1.norm, 1. / nodeLensEta))throw runtime_error("全反射が起きた");//屈折計算

				FreeFlightRay(target);
			}
			catch (std::exception& ex) {
				cout << ex.what() << endl;
				system("pause");
			}
			catch (...) {
				cout << "Unknown err" << endl;
				abort();
			}
			
		}();
	}

	//要素レンズを描画
	for (const auto& i : nodeLensesParams)
		DrawSphere(plotter, i.first, i.second, 20, R"("green")");
	//すべてのレイを描画
	const array<string,5> cols = { "\"red\"","\"orange\"","\"yellow\"","\"green\"","\"blue\"" };
	for (const auto& r : rays) {
		DrawRaySkipFirstArrow(plotter, r, "\"red\"");//レイを描画
		//レイの方向を表示
		cout <<"dir:\n" << r.back().dir() <<"\n\n" << endl;
		//角度を計算する
		const ureal angleax = atan(r.back().dir().x() / r.back().dir().z()) / std::numbers::pi * 180.;
		cout << "angle: " << angleax << endl;
	}

	plotter->show();//表示　なんか終わらん
	plotter->close();//終了　あんまり意味がない
	return 0;
}
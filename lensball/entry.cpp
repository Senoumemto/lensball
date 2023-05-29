#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

int MMain();

int main() {
	try {
		return MMain();
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

int MMain() {
	/*
	レンズを動かして、光の散り方を知りたい
	レンズの動きxを軸にとってdirのx,yをプロット
	**********y
	入射光の方向を法線からずらして行う
	
	
	*/

	printf("hello v1235\n");

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義


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

	//orgのyをちょっとずつ換えてく
	constexpr size_t numProjectorOrignResolution = 10;
	const std::pair<ureal, ureal> projectorOrgYRange = make_pair<ureal, ureal>(-0.7, 0.3);
	for (std::decay<decltype(numProjectorOrignResolution)>::type indexOrgPos = 0; indexOrgPos < numProjectorOrignResolution; indexOrgPos++) {

		//レイを生成する
		list<ray3> rays;
		constexpr unsigned int targetBallsNum = 6;//光を当てるボールの数v ただしnodelensNumが偶数だったら偶数、奇数だったら奇数にすること
		constexpr unsigned int targetBallsOffset = ((nodeLensNum % 2) != (targetBallsNum % 2)) ? 0 : (nodeLensNum - targetBallsNum) / 2;
		const uvec3 projectionOrign(0., projectorOrgYRange.first + (projectorOrgYRange.second - projectorOrgYRange.first) * (indexOrgPos / (ureal)(numProjectorOrignResolution - 1)), 0.);//こんな感じで定義

		//const uvec3 projectionOrign(0., .0, 0.);
		constexpr ureal layAngleLimitFromTarget = 5. / 180. * std::numbers::pi;
		auto ite = nodeLensesParams.begin();
		for (int i = 0; i < targetBallsOffset; i++)ite++;
		for (std::decay<decltype(targetBallsNum)>::type i = 0; i < targetBallsNum; i++) {
			const uvec3 targetDirection = ((ite->first) - projectionOrign).normalized();//これがレンズの中心を通るレイの方向
			//ここから+-thetaだけz軸中心に打ち分けたい
			const uvec3 direction0 = Eigen::AngleAxis<ureal>(layAngleLimitFromTarget, uvec3(0, 0, 1)) * targetDirection;
			const uvec3 direction1 = Eigen::AngleAxis<ureal>(-layAngleLimitFromTarget, uvec3(0, 0, 1)) * targetDirection;


			rays.push_back(ray3(arrow3(projectionOrign, targetDirection)));//レイを作成
			rays.push_back(ray3(arrow3(projectionOrign, direction0)));//レイを作成
			rays.push_back(ray3(arrow3(projectionOrign, direction1)));//レイを作成

			ite++;
		}


		//レイトレパイプライン
		std::list<decltype(rays)::value_type::iterator> auxTarget;//補助線を書くためのarrowをここに突っ込む
		int rayTracingCount = -1;//いま処理されているのは何本目のレイか
		for (ray3& target : rays) {
			[&] {
				try {
					rayTracingCount++;

					//すべての要素レンズに対して当たり判定を行う これで交差するレンズを探す
					auto hittedNode = nodeLensesParams.end();
					resultIntersecteSphere rezIns;//入社時の交差結果
					for (auto i = nodeLensesParams.begin(); i != nodeLensesParams.end(); i++) {
						const auto rez0 = IntersectSphere(target.back(), i->first, i->second);//要素レンズと交差
						if (rez0.isHit) {//当たったら終わり
							hittedNode = i;
							rezIns = rez0;
							break;
						}
					}

					//当たらなかったらこれ以上やる必要はない
					if (hittedNode == nodeLensesParams.end()) {
						FreeFlightRay(target);
						return;
					}

					//ここからレンズ内の処理
					rezIns.ApplyToRay(target);
					if (!RefractSnell(target, rezIns.norm, nodeLensEta))throw runtime_error("全反射が起きた");//屈折計算

					const auto rezExp = IntersectSphere(target.back(), hittedNode->first, hittedNode->second);//要素レンズ内部を通過
					if (!rezExp.isHit)throw logic_error("logic err0");//レンズ内部なので絶対当たる
					rezExp.ApplyToRay(target);//進める

					if (!RefractSnell(target, -rezExp.norm, 1. / nodeLensEta))throw logic_error("全反射が起きた");//屈折計算

					//ここで補助線を書くかどうか決める
					if (rayTracingCount % 3 != 0) auxTarget.push_back(--target.end());

					FreeFlightRay(target);
				}
				catch (std::runtime_error& ex) {
					cout << ex.what() << endl;
					//続行できる 図的にはちょっと変になるけど

				}
				catch (std::exception& ex) {
					cout << ex.what() << endl;
					system("pause");
					//続行できるかどうかはユーザー次第
				}
				catch (...) {
					cout << "Unknown err" << endl;
					abort();
				}

			}();
		}

		//こっから描画
		plotter->send_command("ax.view_init(elev=90\n)");//視点を設定

		//すべてのレイを描画
		const array<string, 5> cols = { "\"red\"","\"orange\"","\"yellow\"","\"green\"","\"blue\"" };
		for (const auto& r : rays) {
			DrawRaySkipFirstArrow(plotter, r, "\"red\"");//レイを描画
			//レイの方向を表示
			cout << "dir:\n" << r.back().dir() << "\n\n" << endl;
			//角度を計算する
			const ureal angleax = atan(r.back().dir().x() / r.back().dir().z()) / std::numbers::pi * 180.;
			cout << "angle: " << angleax << endl;
		}

		//つぎに作図する
		//要素レンズを描画
		for (const auto& i : nodeLensesParams)
			DrawSphere(plotter, i.first, i.second, 20, R"("green")");
		//プロジェクタの最大投映角を
		constexpr ureal projectorLimitAngle = 60. / 180. * std::numbers::pi;//プロジェクタの投映範囲　片側
		constexpr ureal auxProjectorLimitLineLength = 10.;//補助線長さ
		DrawLine(plotter, projectionOrign, projectionOrign + uvec3(sin(projectorLimitAngle), cos(projectorLimitAngle), 0.) * auxProjectorLimitLineLength, R"("blue")");
		DrawLine(plotter, projectionOrign, projectionOrign + uvec3(sin(-projectorLimitAngle), cos(-projectorLimitAngle), 0.) * auxProjectorLimitLineLength, R"("blue")");


		//表示領域のための補助線
		constexpr ureal auxLineLength = 10.;
		for (const auto& t : auxTarget) {
			//arrowをマイナス方向へ延長した線を作る
			DrawLine(plotter, t->org(), t->org() - (auxLineLength * t->dir()), R"("green")");

		}

		//結果を保存
		const std::string resultsPathPrefix = R"(C:/local/user/lensball/lensball/results/)";
		plotter->save(resultsPathPrefix + StringFormat("rez%d.png",indexOrgPos));
		plotter->pause(0.5);//表示
		plotter->send_command(R"(
plt.cla()
#ax = fig.add_subplot(111, projection = '3d')
ax.set_box_aspect((1, 1, 1))
# 軸ラベルの設定
ax.set_xlabel("x", fontsize = 16)
ax.set_ylabel("y", fontsize = 16)
ax.set_zlabel("z", fontsize = 16)
# 軸範囲の設定
ax.set_xlim(-3.,3.)
ax.set_ylim(-3.,3.)
ax.set_zlim(-3.,3.))");
	}
	plotter->send_command("plt.cla()\nplt.clf()\n");//pyplot終了ポリシー
	plotter->close();//終了　あんまり意味がない
	return 0;
}
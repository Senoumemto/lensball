#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "HexBall/";//このbranchの結果を格納するフォルダ

using py = pythonRuntime;


//要素レンズにある点が内包されているかどうかをチェックする
bool NaihouHantei(const uvec2& p ,const std::list<uvec2>& vs) {
	constexpr ureal naihouDist = 0.001;

	auto ite =vs.cbegin();//末尾から一個前
	ureal sumAngle = 0.;
	for (size_t i = 0; i < 6;i++) {
		//一つ次の頂点をチェック
		auto nite = std::next(ite);

		//ite pとp niteの角度を求める
		const uvec2 e0 = (*ite - p).normalized(),
			e1 = (*nite - p).normalized();
		
		sumAngle += acos(e0.dot(e1));
		ite++;
	}

	//合計が2piか否か
	if (fabs(2. * pi - fabs(sumAngle)) < naihouDist)return true;
	return false;
}

//2dベクトルに要素を加える
uvec3 ExtendUvec2(const uvec2& v, const ureal& z) {
	return uvec3(v.x(), v.y(), z);
}
//arrowをvec6に変える
uvec6 ArrowToUVec6(const arrow<3>& v) {
	return uvec6(v.org().x(), v.org().y(), v.org().z(), v.dir().x(), v.dir().y(), v.dir().z());
}

int main() {

	try {
		//python plt用のベクトル系列名
		const pyVecSeries<3> mlabSeries("mlabv");//mlabプロット用に使う3Dベクトル配列
		const pyVecSeries<3> nlensSeries("nl");//要素レンズのグリッド
		const pyVecSeries<2> pypltSeries("pypltv");//二次元プロット用
		const pyVecSeries<6> quiverSeries("mlabquiver");//ベクトル場用
		const std::array<const string, 6> quiverPrefix = { "x","y","z","a","b","c" };

		constexpr bool drawSphere = true;//レンズボール概形を描画する
		//Pythonをセットアップしてからレンズボールの概形を書く
		constexpr ureal sphereRadius = 1.;
		{

			//pythonランタイムを準備していろいろ初期処理
			py::Init();

			py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

			//mayaviの設定
			const std::pair<size_t, size_t> figResolution(800, 600);
			py::sf("fig = mlab.figure(\'Refract dir In Global\', size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

			//matplotlibの設定
			py::s("fig, ax = plt.subplots()\nax.set_aspect(\"equal\")\nplt.title('Lens dist In MapD')");

			//球を描画する
			constexpr size_t sphereResolution = 20;
			if (drawSphere)py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
spx = np.cos(sphphi)*np.sin(sphtheta)
spy = np.sin(sphphi)*np.sin(sphtheta)
spz = np.cos(sphtheta)
mlab.mesh(%f*spx, %f*spy, %f*spz ,color=(1.,1.,1.) )  
)", sphereResolution, sphereResolution, sphereRadius, sphereRadius, sphereRadius);
		};



		//レンズアレイを作成、描画する
		//六角形でタイリングする　偶数行を書いてから奇数行を書くって感じ
		constexpr size_t lensNumInCollum = 20;

		constexpr ureal rowAngle = 0.04331481 * 2.;//行の角度
		const ureal rowLength = 2. * pi * cos(rowAngle);//行の長さ
		const ureal lensEdgeWidth = rowLength / (ureal)lensNumInCollum / 2.;
		const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInCollum;//六角形の一変だけシフトする
		const auto DesignedMapToMap = bitrans<Eigen::Rotation2D<ureal>>(Eigen::Rotation2D<ureal>(rowAngle));//レンズアレイを傾斜させる前から傾斜させたあとにする

		const ureal nodeLensRadius = 2. * lensEdgeWidth / sqrt(3.);//要素レンズ形状を作成　球の半径
		const ureal nodeLensRadiusSq = pow(nodeLensRadius, 2);//要素レンズ半径の二条
		const std::pair<size_t,size_t> nodeLensResolution = make_pair(5*2,5);//要素レンズの分割数
		constexpr size_t rowNum = 15;//奇数にしてね 行の数

		//要素レンズの概形は六角形になるはず
		auto hexverticesNodelensOuter = [&] {
			auto hexvjunk = MakeHexagon(lensEdgeWidth);//六角形の頂点
			hexvjunk.push_back(hexvjunk.front());//一周するために最初の点を末尾に挿入

			return hexvjunk;
		}();

		constexpr bool calcNodelenses = false;//ノードレンズの位置を計算してレンズボールを形成する
		constexpr bool drawNodelenses = calcNodelenses & true;//要素レンズを描画する
		constexpr bool drawNodelensEdges = calcNodelenses & true;//ノードレンズの枠線を描画する
		if(calcNodelenses){
			//std::list<uleap>//マップ座標でのレンズ中心
			for (std::decay<decltype(rowNum)>::type rd = 0; rd < rowNum; rd++) {
				const ureal tlati = eachRowsDistance * rd - (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//lati方向の現在位置
				const bool eachFlag = rd % 2;//交互に切り替わるフラグ
				for (std::decay<decltype(lensNumInCollum)>::type ld = 0; ld < lensNumInCollum; ld++) {

					auto color = HsvToRgb({ uleap({0.,1.},ld / (ureal)lensNumInCollum),1.,1. });//色を行方向に変える
					//六角形を収めるバッファをクリア
					ResetPyVecSeries(pypltSeries);

					//lonn方向の現在位置
					const ureal tlonn = uleap(PairMinusPlus(rowLength / 2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);
					const auto localcenterInMapDesigned = uvec2(tlonn, tlati);//要素レンズの中央 ローカルマップ座標
					const auto localcenterInMap = DesignedMapToMap.prograte() * localcenterInMapDesigned;
					const auto localcenterInBalllocalPolar = MapToLocalPolar(localcenterInMap);
					const uvec3 localcenterInBalllocal = PolarToXyz(localcenterInBalllocalPolar);

					//要素レンズを描画していく
					
					//つぎに極座標で要素レンズを計算する
					ResetPyVecSeries(nlensSeries);//ノードレンズ
					for (std::decay<decltype(nodeLensResolution.second)>::type nlla = 0; nlla < nodeLensResolution.second; nlla++) {

						ResetPyVecSeries(mlabSeries);//mlabSeriesはグリッドの一行を格納する
						for (std::decay<decltype(nodeLensResolution.first)>::type nllo = 0; nllo < nodeLensResolution.first; nllo++) {
							const uvec2 localpos(uleap(PairMinusPlus(pi), nllo / (ureal)(nodeLensResolution.first - 1)),
								uleap(PairMinusPlus(pi / 2.), nlla / (ureal)(nodeLensResolution.second - 1)));//要素レンズローカルでの極座標

							const uvec3 nodelensShape = (nodeLensRadius * fabs(cos(localcenterInBalllocalPolar.y()))) * PolarToXyz(localpos);//これが球になるはず
							AppendPyVecSeries(mlabSeries, nodelensShape + localcenterInBalllocal);
						}
						if (drawNodelenses)py::s("nlx.append(mlabvx)\nnly.append(mlabvy)\nnlz.append(mlabvz)\n");//これでメッシュになると思うんやけど
					}

					if (drawNodelenses)py::sf("mlab.mesh(%s,color=(%f,%f,%f))", GetPySeriesForPlot(nlensSeries), color[0], color[1], color[2]);
					//頂点を転送して描く
					ResetPyVecSeries(mlabSeries);
					for (const auto& v : hexverticesNodelensOuter) {
						const uvec2 designedVertex = (v + localcenterInMapDesigned);
						const uvec2 vertex = DesignedMapToMap.prograte() * designedVertex;//傾けてマップ座標にする
						AppendPyVecSeries(pypltSeries, designedVertex);
						const auto polarpos = MapToLocalPolar(vertex);//つぎに極座標を得る
						AppendPyVecSeries(mlabSeries, PolarToXyz(polarpos));
					}

					if (drawNodelensEdges)py::sf("plt.plot(%s,color=(%f,%f,%f))", GetPySeriesForPlot(pypltSeries), color[0], color[1], color[2]);
					if (drawNodelensEdges)py::sf("mlab.plot3d(%s,color=(%f,%f,%f),tube_radius=0.01)", GetPySeriesForPlot(mlabSeries), color[0], color[1], color[2]);
				}
			}
		};


		//スキャンをする
		constexpr size_t projectorResInTheta = 720;//プロジェクタの縦がわ解像度
		constexpr ureal projectorHalfAngle = 60. / 180. * pi;//プロジェクトの投映角
		constexpr size_t numOfProjectionPerACycle = 720;//一回転での投影数
		const ureal nodeLensFocalLength = nodeLensRadius * 1.5;//要素レンズの中心から焦点までの距離

		constexpr size_t scanThreadNum = 19;//スキャンに使うスレッド数
		std::array<uptr<std::thread>, scanThreadNum> scanThreads;//実行スレッド
		std::array<std::atomic_bool, scanThreadNum> scanThreadIsFin;//すきゃんがおわったことを報告

		constexpr bool scanLenses = true;//レンズボールに対するレイトレーシングを行う
		constexpr bool drawRefractionDirectionOfARay = false;//あるレイの屈折方向を描画する
		if(scanLenses){
			ResetPyVecSeries<6>(quiverSeries,quiverPrefix);//ベクトル場をお掃除
			ResetPyVecSeries(pypltSeries);//ベクトル場をお掃除

			//回転角度ごとにスレッドを割り付ける
			const auto scanAScene = [&](const std::decay<decltype(numOfProjectionPerACycle)>::type rd, decltype(scanThreadIsFin)::iterator finflag) {

				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(numOfProjectionPerACycle)) + (2. * pi / (ureal)(numOfProjectionPerACycle + 1) / 2.);//ボールの回転角度
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//グローバルからレンズボールローカルへの変換 XYZ座標

				for (std::decay<decltype(projectorResInTheta)>::type pd = 0; pd < projectorResInTheta; pd++) {//プロジェクタの注目画素ごとに
					const ureal rayThetaInGlobal = uleap(PairMinusPlus(projectorHalfAngle), pd / (ureal)(projectorResInTheta - 1));//プロジェクタ座標系での注目画素からでるレイのθ

					const uvec2 rayDirInBallInBalllocalPolar(-ballRotation, rayThetaInGlobal);//ボールの回転角度画からボールローカルでのレイの方向(極座標がわかる)
					const uvec2 rayDirInMap = PolarToMap(rayDirInBallInBalllocalPolar);//マップ座標はここ 傾いたあと
					const uvec2 rayDirInMapDesigned = (DesignedMapToMap.untiprograte()) * rayDirInMap;//傾ける前 デザインマップ


					//py::sf("plt.scatter(%f,%f,color=(0,0,0))", rayDirInBallLocalMapDesigned.x(), rayDirInBallLocalMapDesigned.y());
					//デザインマップのシータからrowがわかる
					//const ureal tlati = eachRowsDistance * rd-(eachRowsDistance*(ureal)(rowNum-1)/2.);//lati方向の現在位置
					//シータは-eachRowsDistance*(ureal)(rowNum-1)/2~eachRowDistance*(rownum-1)/2まで
					//まずrayDirInMapDのlatitudeを正規化する 一番下のrowの高さちょうどのとき 0　一番上のrowの高さちょうどのときrowNum-1になるようにする
					const ureal regRayDirLati = [&] {
						const ureal zerothRowlati = -(eachRowsDistance * (ureal)(rowNum - 1) / 2.);//zero番目の行の高さ
						const ureal zeroSetRayDir = rayDirInMapDesigned.y() - zerothRowlati;//0番目の行の高さに始点を合わせたレイの高さ
						const ureal finalRowlati = (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//rowNum-1番目の行の高さ
						//スケーリング　fin~0までのスケールがrowNum-1~0までのスケールになって欲しい
						const ureal thisscale = (ureal)(rowNum - 1 - 0) / (finalRowlati - zerothRowlati);

						return thisscale * zeroSetRayDir;
					}();


					const int centerRawIndex = round(regRayDirLati);//四捨五入するともっともらしいインデックスがわかる
					const int neibourRawIndex = (regRayDirLati - (ureal)centerRawIndex) > 0. ? centerRawIndex + 1 : centerRawIndex - 1;//隣り合う行のもっともらしいインデックスもわかる
					//printf("row index %d,%d\n", centerRawIndex, neibourRawIndex);

					//ではここから行中の当たり判定を始める
					optional<std::pair<uvec2, uvec2>> hitLensCenterAndHitDistInMapDesigned;//対象のレンズの中心位置
					for (const auto& rid : { centerRawIndex,neibourRawIndex }) {

						//つぎにphiから何番目のレンズかを考える
						const ureal regRayDirLonn = [&] {
							//const ureal tlonn = uleap(PairMinusPlus(rowLength/2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);

							const ureal zerothlenslonn = -rowLength / 2.;//-rowLength/2.が最初のレンズの位置　場合によって前後するけどね
							const ureal zeroSetRayDir = rayDirInMapDesigned.x() - zerothlenslonn;//zero番目のレンズの場所をzeroに
							const ureal finlenslonn = uleap(PairMinusPlus(rowLength / 2.), (lensNumInCollum - 1) / (ureal)lensNumInCollum);//最後のレンズの場所
							//スケーリング　fin~0までのスケールがlensNumInCollum-1~0までのスケールになって欲しい
							const ureal thisscale = (ureal)(lensNumInCollum - 1 - 0) / (finlenslonn - zerothlenslonn);

							return thisscale * zeroSetRayDir;
						}();
						const bool eachFlag = rid % 2;//交互に切り替わるフラグ 立っているときは行が半周進んでる

						const int centerLensIndex = round(regRayDirLonn - (eachFlag ? 0.5 : 0.));//これはオフセットがない　つまりよりマイナス側から始まっている行にいる場合のインデックス そうでなければ-0.5してから丸める←やりました
						const int neibourLensIndex = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? centerLensIndex + 1 : centerLensIndex - 1;//隣り合うレンズのもっともらしいインデックスもわかる
						//printf("lens index %d,%d\n", centerLensIndex, neibourLensIndex);

						//ではレンズの当たり判定を始める
						for (const auto& lid : { centerLensIndex,neibourLensIndex }) {
							//これでレンズの場所が確定するはず
							const auto hitlensCenterInMapDesigned = uvec2(uleap(PairMinusPlus(rowLength / 2.), lid / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.), eachRowsDistance * rid - (eachRowsDistance * (ureal)(rowNum - 1) / 2.));
							const uvec2 hitDistInMapDesigned = rayDirInMapDesigned - hitlensCenterInMapDesigned;//相対ヒット位置 もちろんマップD上
							//本当かしら とりあえず概形の中に入っているかどうかを判定
							if (NaihouHantei(hitDistInMapDesigned, hexverticesNodelensOuter)) {
								if (hitDistInMapDesigned.norm() >= nodeLensRadius)throw logic_error("判定がだめ");
								//printf("ok r=%d l=%d\n", rid, lid);

								//とりあえずテスト　これがcenterでない可能性はあるの?
								if (rid != centerRawIndex || lid != centerLensIndex)std::runtime_error("別にエラーではないけど想像したのと違う!");

								hitLensCenterAndHitDistInMapDesigned = make_pair(hitlensCenterInMapDesigned, hitDistInMapDesigned);
								break;
							}
						}

						if (hitLensCenterAndHitDistInMapDesigned)break;
					}

					//ここまでで結果が出ないとやばい
					if (!hitLensCenterAndHitDistInMapDesigned)std::runtime_error("要素レンズの検索に失敗");

					//焦点の場所は要素レンズが確定すれば計算できる
					const auto focalposInBalllocalXYZ = Polar3DToXyz(ExtendUvec2(MapToLocalPolar(DesignedMapToMap.prograte() * hitLensCenterAndHitDistInMapDesigned.value().first), sphereRadius + nodeLensFocalLength));

					const auto hitPosInBalllocalXYZ = PolarToXyz(rayDirInBallInBalllocalPolar);//ここがボールローカルでのヒット点
					const auto refractRayDirInBalllocalXYZ = (focalposInBalllocalXYZ - hitPosInBalllocalXYZ);//衝突点と焦点の位置が分かれば光の方向がわかるね　暫定的に
					//これをグローバルに戻す
					const uvec3 refractRayDirInGlobal = GlobalToBallLocal.untiprograte() * refractRayDirInBalllocalXYZ;

					//プロットします　ヒットポイントに
					if (pd == 0 && drawRefractionDirectionOfARay) {
						const auto color = HsvToRgb({ rd / (ureal)(numOfProjectionPerACycle - 1),1.,1. });
						py::sf("mlab.quiver3d(0,0,0,%f,%f,%f,mode=\"arrow\",color=(%f,%f,%f))", refractRayDirInGlobal.x(), refractRayDirInGlobal.y(), refractRayDirInGlobal.z(), color[0], color[1], color[2]);
						py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", hitLensCenterAndHitDistInMapDesigned.value().second.x(), hitLensCenterAndHitDistInMapDesigned.value().second.y(), color[0], color[1], color[2]);
					}
				}

				*finflag = true;
			};

			for (std::decay<decltype(numOfProjectionPerACycle)>::type rdgen = 0; rdgen < numOfProjectionPerACycle; rdgen++) {

				//このrdgenでの処理を開いているスレッドに割り付けたい
				bool isfound = false;
				while (!isfound) {//割り付けられなければ繰り返す
					for (size_t t=0;t<scanThreadNum;t++)
						if (!scanThreads.at(t)) {//空きなら割付
							if (!isfound) {//一つのインデックスには一回だけ割り付ける
								isfound = true;
								scanThreadIsFin.at(t) = false;//フラグをセット

								scanThreads.at(t).reset(new std::thread(scanAScene, rdgen, scanThreadIsFin.begin() + t));//スレッド実行開始
							}
						}
						else if (scanThreadIsFin.at(t)) {//空いてなくて終わってるなら
							scanThreads.at(t).get()->join();
							scanThreads.at(t).release();//リソースを開放
						}
				}
			}

			//全部のスレッドが終わっていることを確認
			for (auto& t : scanThreads)
				if (t) {
					t.get()->join();
					t.release();
				}
		};
		

		//表示する 3d 2dの順
		py::s("mlab.show()");
		py::s("plt.show()");
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		system("pause");//なんか入れたら終わり

		py::Terminate();
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		system("pause");//なんか入れたら終わり

		py::Terminate();
		return -2;
	}

	cout << "The work is complete...Wait rendering by Python" << endl;
	py::Terminate();
	return 0;
}
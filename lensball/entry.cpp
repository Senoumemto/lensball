#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"
#include <fstream>

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/binary.hpp>

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

template < typename T > constexpr T sqrt_constexpr(T s){
	T x = s / 2.0;
	T prev = 0.0;

	while (x != prev)
	{
		prev = x;
		x = (x + s / x) / 2.0;
	}
	return x;
}

template<class Archive>
void serialize(Archive& archive, std::pair<Eigen::Vector3i, arrow3>& dic)
{
	archive(dic.first.x(), dic.first.y(), dic.first.z(),
		dic.second.org().x(), dic.second.org().y(), dic.second.org().z(),
		dic.second.dir().x(), dic.second.dir().y(), dic.second.dir().z());
}

int main() {

	try {
		
		//開始時刻を記録してスタート
		const auto startTimePoint = std::chrono::system_clock::now();
		cout << "Start: " << startTimePoint << endl;

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

		//ハードウェアスペック
		constexpr size_t projectorFps = 22727;//フレームレート
		constexpr size_t numOfProjectionPerACycle = 1024;//一周ごとの投映数
		constexpr ureal rotationSpeed = projectorFps / (ureal)numOfProjectionPerACycle;//回転速度
		//解像度
		constexpr size_t verticalDirectionResolution = sqrt_constexpr(numOfProjectionPerACycle);//垂直解像度(一周に並んでいるレンズの数)
		constexpr size_t horizontalDirectionResolution = numOfProjectionPerACycle / verticalDirectionResolution;//水平解像度　位置レンズあたりの投映数


		//レンズアレイを作成、描画する
		//六角形でタイリングする　偶数行を書いてから奇数行を書くって感じ
		constexpr size_t lensNumInARow = verticalDirectionResolution;

		constexpr ureal rowAngle = 0.02706659 * 2.;//行の角度 theta=atan(-1.5*1/(cos(theta)*sqrt(3))*1/lensnum)
		const ureal rowLength = 2. * pi * cos(rowAngle);//行の長さ
		const ureal lensEdgeWidth = rowLength / (ureal)lensNumInARow / 2.;
		const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInARow;//六角形の一変だけシフトする
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
		constexpr bool drawNodelenses = calcNodelenses & false;//要素レンズを描画する
		constexpr bool drawNodelensEdges = calcNodelenses & true;//ノードレンズの枠線を描画する
		if(calcNodelenses){
			//std::list<uleap>//マップ座標でのレンズ中心
			for (std::decay<decltype(rowNum)>::type rd = 0; rd < rowNum; rd++) {
				const ureal tlati = eachRowsDistance * rd - (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//lati方向の現在位置
				const bool eachFlag = rd % 2;//交互に切り替わるフラグ
				for (std::decay<decltype(lensNumInARow)>::type ld = 0; ld < lensNumInARow; ld++) {

					auto color = HsvToRgb({ uleap({0.,1.},ld / (ureal)lensNumInARow),1.,1. });//色を行方向に変える
					//六角形を収めるバッファをクリア
					ResetPyVecSeries(pypltSeries);

					//lonn方向の現在位置
					const ureal tlonn = uleap(PairMinusPlus(rowLength / 2.), ld / (ureal)lensNumInARow) + (eachFlag ? ((rowLength) / (ureal)lensNumInARow / 2.) : 0.);
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
		constexpr size_t projectorResInTheta = 768;//プロジェクタの縦がわ解像度
		constexpr ureal projectorHalfAngleTheta = 60. / 180. * pi;//プロジェクトの投映角
		constexpr size_t projectorResInPhi = 1024;
		constexpr ureal projectorHalfAnglePhi = projectorHalfAngleTheta * (projectorResInPhi / (ureal)projectorResInTheta);//プロジェクトの投映角
		const ureal nodeLensFocalLength = nodeLensRadius * 1.5;//要素レンズの中心から焦点までの距離

		//std::unordered_map<Eigen::Vector3i, arrow3> projectorRefractRayMap;

		constexpr size_t scanThreadNum = 16;//スキャンに使うスレッド数
		constexpr size_t sendStorageThreshold = 0;//各スレッドごとにこれを超えるとストレージに保存を行う

		constexpr bool scanLenses = true;//レンズボールに対するレイトレーシングを行う
		constexpr bool drawRefractionDirectionOfARay = false;//あるレイの屈折方向を描画する
		constexpr bool logWarningInScan = false;//scan中の警告を表示する

		bool onetime = false;

		if(scanLenses){

			//並列処理用のいろいろ
			std::array<uptr<std::thread>, scanThreadNum> scanThreads;//実行スレッド
			std::array<std::unordered_map<Eigen::Vector3i, arrow3>, scanThreadNum> scanResultOfEachScanThread;//各スレッドごとに結果をためていく//レンズボールからどのようなレイが出るのか　そのレイはプロジェクタのどこと(h,v,t)対応づくのか
			std::array<bool, scanThreadNum> finFlagOfEachScanThread;//スキャンがおわったことを報告
			//結果格納ストリームを用意する
			std::array<uptr<std::ofstream>, scanThreadNum> storageOfEachScanThread;
			for (size_t sd = 0; sd < scanThreadNum; sd++) {
				storageOfEachScanThread.at(sd).reset(new std::ofstream(rezpath + branchpath + "projRefRayMapPart" + to_string(sd) + ".bin"));
			}

			ResetPyVecSeries<6>(quiverSeries,quiverPrefix);//ベクトル場をお掃除
			ResetPyVecSeries(pypltSeries);//ベクトル場をお掃除


			//指定された結果を指定されたストレージへ送信する 転送したあとはRezはクリアされます
			const std::function<void(std::unordered_map<Eigen::Vector3i,arrow3>&,ofstream*)> TransRezToStorage = [&](decltype(scanResultOfEachScanThread)::value_type& rezOfSt, ofstream* storageofStPtr) {
				cereal::BinaryOutputArchive o_archive(*storageofStPtr);

				for (std::pair<Eigen::Vector3i, arrow3> dicp : rezOfSt)
					o_archive(dicp);

				rezOfSt.clear();
			};
			//回転角度ごとにスキャンを行う
			const auto ScanAScene = [&](const std::decay<decltype(numOfProjectionPerACycle)>::type rd, decltype(scanResultOfEachScanThread)::iterator rezOfStIte, decltype(finFlagOfEachScanThread)::iterator finflagOfStIte, decltype(storageOfEachScanThread)::iterator storageOfStIte) {

				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(numOfProjectionPerACycle)) + (2. * pi / (ureal)(numOfProjectionPerACycle + 1) / 2.);//ボールの回転角度
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//グローバルからレンズボールローカルへの変換 XYZ座標

				//各ピクセルから飛び出るレイと回転角度rdの球との当たり判定を行う
				for (std::decay<decltype(projectorResInPhi)>::type hpd = 0; hpd < projectorResInPhi; hpd++) {
					const ureal rayPhiInGlobal = uleap(PairMinusPlus(projectorHalfAnglePhi), hpd / (ureal)(projectorResInPhi - 1));//プロジェクタ座標系での注目画素からでるレイのφ

					for (std::decay<decltype(projectorResInTheta)>::type vpd = 0; vpd < projectorResInTheta; vpd++) {//プロジェクタの注目画素ごとに
						const ureal rayThetaInGlobal = uleap(PairMinusPlus(projectorHalfAngleTheta), vpd / (ureal)(projectorResInTheta - 1));//プロジェクタ座標系での注目画素からでるレイのθ
						const uvec3 rayDirInGlobal = PolarToXyz(uvec2(rayPhiInGlobal, rayThetaInGlobal));

						const uvec2 rayDirInBallInBalllocalPolar(rayPhiInGlobal - ballRotation, rayThetaInGlobal);//ボールの回転角度画からボールローカルでのレイの方向(極座標がわかる)
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
								const ureal finlenslonn = uleap(PairMinusPlus(rowLength / 2.), (lensNumInARow - 1) / (ureal)lensNumInARow);//最後のレンズの場所
								//スケーリング　fin~0までのスケールがlensNumInCollum-1~0までのスケールになって欲しい
								const ureal thisscale = (ureal)(lensNumInARow - 1 - 0) / (finlenslonn - zerothlenslonn);

								return thisscale * zeroSetRayDir;
							}();
							const bool eachFlag = rid % 2;//交互に切り替わるフラグ 立っているときは行が半周進んでる

							const int centerLensIndex = round(regRayDirLonn - (eachFlag ? 0.5 : 0.));//これはオフセットがない　つまりよりマイナス側から始まっている行にいる場合のインデックス そうでなければ-0.5してから丸める←やりました
							const int neibourLensIndex = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? centerLensIndex + 1 : centerLensIndex - 1;//隣り合うレンズのもっともらしいインデックスもわかる
							//printf("lens index %d,%d\n", centerLensIndex, neibourLensIndex);

							//ではレンズの当たり判定を始める
							for (const auto& lid : { centerLensIndex,neibourLensIndex }) {
								//これでレンズの場所が確定するはず
								const auto hitlensCenterInMapDesigned = uvec2(uleap(PairMinusPlus(rowLength / 2.), lid / (ureal)lensNumInARow) + (eachFlag ? ((rowLength) / (ureal)lensNumInARow / 2.) : 0.), eachRowsDistance * rid - (eachRowsDistance * (ureal)(rowNum - 1) / 2.));
								const uvec2 hitDistInMapDesigned = rayDirInMapDesigned - hitlensCenterInMapDesigned;//相対ヒット位置 もちろんマップD上
								//本当かしら とりあえず概形の中に入っているかどうかを判定
								if (NaihouHantei(hitDistInMapDesigned, hexverticesNodelensOuter)) {
									if (hitDistInMapDesigned.norm() >= nodeLensRadius)throw logic_error("判定がだめ");
									//printf("ok r=%d l=%d\n", rid, lid);

									//とりあえずテスト　これがcenterでない可能性はあるの?
									if (rid != centerRawIndex || lid != centerLensIndex)
										if (logWarningInScan)cout << "別にエラーではないけど想像したのと違う!" << endl;

									hitLensCenterAndHitDistInMapDesigned = make_pair(hitlensCenterInMapDesigned, hitDistInMapDesigned);
									break;
								}
							}

							if (hitLensCenterAndHitDistInMapDesigned)break;
						}

						//ここまでで結果が出ないとやばい
						if (!hitLensCenterAndHitDistInMapDesigned)throw std::runtime_error("要素レンズの検索に失敗");

						//焦点の場所は要素レンズが確定すれば計算できる
						const auto focalposInBalllocalXYZ = Polar3DToXyz(ExtendUvec2(MapToLocalPolar(DesignedMapToMap.prograte() * hitLensCenterAndHitDistInMapDesigned.value().first), sphereRadius + nodeLensFocalLength));

						const auto hitPosInBalllocalXYZ = PolarToXyz(rayDirInBallInBalllocalPolar);//ここがボールローカルでのヒット点
						const auto refractRayDirInBalllocalXYZ = (focalposInBalllocalXYZ - hitPosInBalllocalXYZ);//衝突点と焦点の位置が分かれば光の方向がわかるね　暫定的に
						//これをグローバルに戻す
						const uvec3 refractRayDirInGlobal = GlobalToBallLocal.untiprograte() * refractRayDirInBalllocalXYZ;

						//結果を追加
						(*rezOfStIte)[Eigen::Vector3i(hpd, vpd, rd)] = arrow3(rayDirInGlobal, refractRayDirInGlobal);

						//プロットします　ヒットポイントに
						if (vpd == 0 && drawRefractionDirectionOfARay) {
							const auto color = HsvToRgb({ rd / (ureal)(numOfProjectionPerACycle - 1),1.,1. });
							py::sf("mlab.quiver3d(0,0,0,%f,%f,%f,mode=\"arrow\",color=(%f,%f,%f))", refractRayDirInGlobal.x(), refractRayDirInGlobal.y(), refractRayDirInGlobal.z(), color[0], color[1], color[2]);
							py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", hitLensCenterAndHitDistInMapDesigned.value().second.x(), hitLensCenterAndHitDistInMapDesigned.value().second.y(), color[0], color[1], color[2]);
						}
					}
				}

				//閾値を超えた場合はストレージに転送する
				if ((*rezOfStIte).size() >= sendStorageThreshold)
					TransRezToStorage(rezOfStIte.operator*(), storageOfStIte.operator*().get());
				*finflagOfStIte = true;
			};

			//回転角度を変えながらスキャンする
			for (std::decay<decltype(numOfProjectionPerACycle)>::type rdgen = 0; rdgen < numOfProjectionPerACycle; rdgen++) {
				cout << "count: " << rdgen << endl;
				//このrdgenでの処理を開いているスレッドに割り付けたい
				bool isfound = false;
				while (!isfound) {//割り付けられなければ繰り返す
					for (size_t th=0;th<scanThreadNum;th++)
						if (!scanThreads.at(th)) {//空きなら割付
							if (!isfound) {//一つのインデックスには一回だけ割り付ける
								isfound = true;
								finFlagOfEachScanThread.at(th) = false;//フラグをクリアして

								scanThreads.at(th).reset(new std::thread(ScanAScene, rdgen, scanResultOfEachScanThread.begin() + th, finFlagOfEachScanThread.begin() + th, storageOfEachScanThread.begin() + th));//スレッド実行開始
							}
						}
						else if (finFlagOfEachScanThread.at(th)) {//空いてなくて終わってるなら
							scanThreads.at(th).get()->join();
							scanThreads.at(th).release();//リソースを開放
						}
				}
			}

			//これ以上は結果は追加されない
			for (size_t th = 0; th < scanThreadNum; th++) {

				//まだ開放されていなければjoinして開放する
				if (scanThreads.at(th)) {
					scanThreads.at(th).get()->join();
					scanThreads.at(th).release();
				}

				//残っている結果を転送する
				scanThreads.at(th).reset(new std::thread(TransRezToStorage, std::ref(scanResultOfEachScanThread.at(th)), (ofstream*)(storageOfEachScanThread.at(th).get())));//スレッド実行開始

			}

			//スレッドを全部joinする
			for (auto&t:scanThreads) {
				t->join();
				t.release();
			}
		};
		

		//表示する 3d 2dの順
		py::s("mlab.show()");
		py::s("plt.show()");


		const auto endTimePoint = std::chrono::system_clock::now();
		cout << "Finish: " << endTimePoint << endl;
		cout << "Process time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTimePoint - startTimePoint).count() / 1000. << " [s]" << endl;
		cout << "The work is complete...Wait rendering by Python" << endl;
		py::Terminate();
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//なんか入れたら終わり

		py::Terminate();
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//なんか入れたら終わり

		py::Terminate();
		return -2;
	}

	return 0;
}
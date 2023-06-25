#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"
#include "bmpLib/bmp.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//結果を格納するフォルダ
const std::string branchpath = "SimVis/";//このbranchの結果を格納するフォルダ

using py = pythonRuntime;

//ある半直線(ray)と線分(line)の当たり判定
bool IntersectLineAndWay(const std::pair<uvec2,uvec2>& line,const arrow2& ray) {
	const uvec2 dist = line.first - line.second;//これが線分の傾き

	umat2 expression;//連立方程式を解く
	uvec2 ans;
	for (size_t a = 0; a < 2; a++) {
		expression.row(a)[0] = dist[a];
		expression.row(a)[1] = -ray.dir()[a];
		ans[a] = -line.second[a] + ray.org()[a];
	}
	if (expression.determinant() == 0.)throw runtime_error("この式は溶けない!!");
	uvec2 trueans = expression.inverse()* ans;

	return trueans.x() >= 0. && trueans.x() <= 1. && trueans.y() >= 0.;//線分のtが0~1なら かつ半直線のtが正なら
}
bool NaihouHanteiX(const uvec2& p, const std::list<uvec2>& vs) {
	constexpr ureal naihouDist = 0.001;

	auto ite = vs.cbegin();//末尾から一個前
	size_t xcount = 0, ycount = 0;
	for (size_t i = 0; i < 6; i++) {
		//一つ次の頂点をチェック
		auto nite = std::next(ite);

		//これからなる点
		const uvec2 ver = *ite;
		const uvec2 nver = *nite;

		//x方向に伸ばします
		if (IntersectLineAndWay(make_pair(ver, nver), arrow2(p, uvec2::UnitX())))xcount++;
		if (IntersectLineAndWay(make_pair(ver, nver), arrow2(p, uvec2::UnitY())))ycount++;
		ite++;
	}
	//x/ycountはそれぞれの方向に線分を伸ばしたときに交差した回数
	return xcount % 2 && ycount % 2;
}

//球とarrowが交差するか判定する
std::optional<ureal> IntersectSphereAndArrow(const sphereParam& s, const arrow3& r) {
	//二次方程式の係数たち　ただし球ローカル
	const ureal a = [&] {
		ureal ret = 0.;
		for (size_t i = 0; i < 3; i++)
			ret += pow(r.dir()[i], 2);

		return ret;
	}();
	const ureal b = [&] {
		ureal ret = 0.;
		for (size_t i = 0; i < 3; i++)
			ret += r.dir()[i] * (r.org()[i] - s.first[i]);
		return ret * 2.;
	}();
	const ureal c = [&] {
		ureal ret = 0.;
		for (size_t i = 0; i < 3; i++)
			ret += pow(r.org()[i] - s.first[i], 2);
		return ret - pow(s.second, 2);
	}();

	//解の公式を解けばok
	const auto hanbetu = pow(b, 2) - 4. * a * c;
	if (hanbetu < 0.)return std::nullopt;

	const auto ans0 = (-b + sqrt(hanbetu)) / (2. * a);
	const auto ans1 = (-b - sqrt(hanbetu)) / (2. * a);

	//両方正なら
	if (ans1 > 0. && ans0 > 0.)return std::min(ans0, ans1);

	//球の中が始点なら一方が過去である
	if (ans0 <= 0. && ans1 > 0.)return ans1;
	else if (ans1 <= 0. && ans0 > 0.)return ans0;

	//球の表面をかすめるならどっちでも一緒
	return ans0;
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

//プロジェクター辞書をデコードする
void DeserializeProjRefraDic(const std::string& path) {
	//ヘッダをロードする
	projRefraDicHeader header;
	{
		ifstream ifs(path + ".head", std::ios::binary);
		cereal::BinaryInputArchive iarch(ifs);

		iarch(header);

		std::cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
	}


	//プロジェクターの色を計算することができるよ　各フレームごとに
	for (size_t sd = 0; sd < header.rotationRes; sd++) {
		//シーンのレイリストを計算する
		std::list<arrow3> raylist;
		{
			ifstream ifs(path + ".part" + to_string(sd), std::ios::binary);
			if (!ifs)throw std::runtime_error("ファイルを読めない");

			cereal::BinaryInputArchive iarch(ifs);
			iarch(raylist);
		}

		//これをたどればOK
		auto listite = raylist.cbegin();
		for(size_t hd=0;hd<header.horizontalRes;hd++,listite++)
			for (size_t vd = 0; vd < header.verticalRes; vd++) {
				const arrow3 refraction = (*listite);//このピクセルに対応するレイ



			}

	}
}


ureal NormalizeAngle(ureal Angle)
{
	long ofs = (*(long*)&Angle & 0x80000000) | 0x3F000000;
	return (Angle - ((int)(Angle * (1./pi) + *(ureal*)&ofs) * (2.*pi)));
}

//0~始まるインデックスを、ある中心から両側に検索するような形に変換する
size_t GetBisideIndex(size_t lini,size_t center, int way,const size_t indSiz) {
	//まず両側インデックスを計算する
	const size_t bilocalSiz = (lini+1)/2;//中心からの相対インデックスの大きさ
	//符号を計算する
	const int sign = lini % 2 ? way : -way;

	//-max/2まで行くかな
	int signedIndex = (sign * (int)bilocalSiz + center);
	//マイナスならmaxを足す
	return (signedIndex < 0 ? signedIndex + indSiz : signedIndex) % indSiz;
}

int main() {

	try {

		//開始時刻を記録してスタート
		const auto startTimePoint = std::chrono::system_clock::now();
		std::cout << "Start: " << startTimePoint << endl;

		//python plt用のベクトル系列名
		const pyVecSeries<3> mlabSeries("mlabv");//mlabプロット用に使う3Dベクトル配列
		const pyVecSeries<3> nlensSeries("nl");//要素レンズのグリッド
		const pyVecSeries<2> pypltSeries("pypltv");//二次元プロット用
		const pyVecSeries<6> quiverSeries("mlabquiver");//ベクトル場用
		const std::array<const string, 6> quiverPrefix = { "x","y","z","a","b","c" };

		//ハードウェアスペック
		constexpr size_t projectorFps = 22727;//フレームレート
		constexpr size_t numOfProjectionPerACycle = 1024;//一周ごとの投映数
		constexpr ureal rotationSpeed = projectorFps / (ureal)numOfProjectionPerACycle;//回転速度
		//解像度
		constexpr size_t verticalDirectionResolution = sqrt_constexpr(numOfProjectionPerACycle);//垂直解像度(一周に並んでいるレンズの数)
		constexpr size_t horizontalDirectionResolution = numOfProjectionPerACycle / verticalDirectionResolution;//水平解像度　位置レンズあたりの投映数

		//プロジェクタのパラメータ
		constexpr size_t projectorResInTheta = 768;//プロジェクタの縦側解像度 ホントはXGA
		constexpr ureal projectorHalfAngleTheta = 60. / 180. * pi;//プロジェクトの投映角
		constexpr size_t projectorResInPhi = 1024; // プロジェクタの横側解像度 ホントはXGA
		constexpr ureal projectorHalfAnglePhi = projectorHalfAngleTheta * (projectorResInPhi / (ureal)projectorResInTheta);//プロジェクトの投映角

		constexpr bool drawSphere = false;//レンズボール概形を描画する
		//Pythonをセットアップしてからレンズボールの概形を書く
		constexpr ureal sphereRadius = 1.;
		const sphereParam lensballParam(uvec3::Zero(), sphereRadius);
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
		constexpr size_t lensNumInARow = verticalDirectionResolution;

		constexpr ureal rowAngle = 0.02706659 * 2.;//行の角度 theta=atan(-1.5*1/(cos(theta)*sqrt(3))*1/lensnum)
		const ureal rowLength = 2. * pi * cos(rowAngle);//行の長さ
		const ureal lensEdgeWidth = rowLength / (ureal)lensNumInARow / 2.;
		const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInARow;//六角形の一変だけシフトする
		const auto DesignedMapToMap = bitrans<Eigen::Rotation2D<ureal>>(Eigen::Rotation2D<ureal>(rowAngle));//レンズアレイを傾斜させる前から傾斜させたあとにする

		const ureal nodeLensRadius = 2. * lensEdgeWidth / sqrt(3.);//要素レンズ形状を作成　球の半径
		const ureal nodeLensRadiusSq = pow(nodeLensRadius, 2);//要素レンズ半径の二条
		const std::pair<size_t,size_t> nodeLensResolution = make_pair(20*2,20);//要素レンズの分割数
		constexpr size_t rowNum = 15;//奇数にしてね 行の数

		//要素レンズの概形は六角形になるはず
		auto hexverticesNodelensOuter = [&] {
			auto hexvjunk = MakeHexagon(lensEdgeWidth);//六角形の頂点
			hexvjunk.push_back(hexvjunk.front());//一周するために最初の点を末尾に挿入

			return hexvjunk;
		}();

		constexpr bool calcNodelenses = true;//ノードレンズの位置を計算してレンズボールを形成する
		constexpr bool drawNodelenses = calcNodelenses & false;//要素レンズを描画する
		constexpr bool drawNodelensEdges = calcNodelenses & false;//ノードレンズの枠線を描画する

		//この計算で要素レンズリストがわかるよ
		std::optional<std::unordered_map<std::pair<size_t, size_t>, sphereParam, HashPair>> nodelensParamsInBalllocal = std::nullopt;//行番号　レンズ番号がキーでパラメータをBalllocalで保存する
		if(calcNodelenses){
			//opt計算できます
			nodelensParamsInBalllocal = decltype(nodelensParamsInBalllocal)::value_type();
			auto& nodelensParamsRez = nodelensParamsInBalllocal.value();

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

					//パラメータを登録
					nodelensParamsRez[make_pair(rd, ld)] = sphereParam(localcenterInBalllocal, 1.1*nodeLensRadius*fabs(cos(localcenterInBalllocalPolar.y())));

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







		//デベロップセクション
		constexpr ureal nodelensEta = 1.5;//ノードレンズの比屈折率
		const sphereParam apertureProjector(uvec3::Zero(), sphereRadius/10.);//ここにあたったらプロジェクターから出たってこと
		const auto regularHexagon = [&] {
			auto hexvjunk = MakeHexagon(sqrt(3.)/2.);//六角形の頂点
			hexvjunk.push_back(hexvjunk.front());//一周するために最初の点を末尾に挿入

			return hexvjunk;
		}();//外接円の半径が1になるような六角形
		constexpr size_t searchAreaInALen = lensNumInARow;//同じ行のレンズをどれだけ深追いして検索するか
		constexpr size_t searchAreaInARow = rowNum;//列をどれだけ深追いして検索するか
		//あたりを付けたノードレンズから探索する範囲
		//まずヘッダを読み出す
		const std::string framePath = R"(C:\local\user\lensball\lensball\resultsX\projectorFrames\)";
		const std::string dicHeaderPath = R"(C:\local\user\lensball\lensball\resultsX\HexBall\projRefRayMap.head)";//辞書ヘッダが収まっている場所
		constexpr ureal nodelensExpand = 1. + 1.e-4;//ノードレンズのギリギリに入射したときに判定できるようにする拡大率
		constexpr size_t devThreadNum = 1;//現像をどれだけのスレッドで実行するか

		//現像に使うカメラ
		constexpr ureal fovHalf = 3. / 180. * pi;
		const uvec3 cameraPos(uvec3(30, 0., 0.));//カメラ位置
		constexpr size_t cameraResW = 16, cameraResH = 16;

		constexpr bool developImage = true;
		constexpr bool printMessagesInDevelopping = false;//デベロップ中のメッセージを出力するか
		std::list<std::list<Eigen::Vector3i>> pixListForCam(cameraResW* cameraResH);//結果 カメラ画素に対する、プロジェクタ画素
		if (developImage) {
			projRefraDicHeader header;
			{
				ifstream ifs(dicHeaderPath, std::ios::binary);
				cereal::BinaryInputArchive iarch(ifs);

				iarch(header);

				std::cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
			}

			std::list<arrow3>cameraRayList;
			for (size_t y = 0; y < cameraResH; y++) {
				const ureal scy = uleap(PairMinusPlus(1.), y / (ureal)(cameraResH - 1));
				for (size_t x = 0; x < cameraResW; x++) {
					//スクリーンの位置は((2/res)*i+(1/res))-1 ｽｸﾘｰﾝサイズは多分2*2

					const ureal scx = uleap(PairMinusPlus(1. * (cameraResW / cameraResH)), x / (ureal)(cameraResW - 1));
					double scz = 1. / tan(fovHalf);//視野角を決める事ができる

					//orgが0 wayがスクリーンの正規化
					Eigen::Vector3d scnormed = Eigen::Vector3d(-scz, scy, scx).normalized();

					cameraRayList.push_back(arrow3(cameraPos, scnormed));
					//py::sf("mlab.quiver3d(%f,%f,%f,%f,%f,%f)", cameraPos.x(), cameraPos.y(), cameraPos.z(), scnormed.x(), scnormed.y(), scnormed.z());

				}
			}

			//解像度とかがわかる
			//つぎに視点ごとにレイトレースしてどの画素に当たるか調べたい
			//並列処理用のいろいろ
			std::array<bool, devThreadNum> finFlagOfEachDevThread;//スキャンがおわったことを報告

			const auto FindPixelsFromARay = [&](const arrow3& devTargetRayInGlobal, decltype(finFlagOfEachDevThread)::iterator finflagOfStIte,decltype(pixListForCam)::iterator rez) {

				//もしこのレイがボールに当たらなければそもそもいらんよ
				if (!IntersectSphereAndArrow(lensballParam, devTargetRayInGlobal));
				else for (size_t rd = 0; rd < header.rotationRes; rd++) {
					////まずはフレームを読み出す
					//const auto thisFrame = make_unique<bmpLib::img>();
					//bmpLib::ReadBmp((framePath + "frame" + to_string(rd) + ".bmp").c_str(), thisFrame.get());

					//つぎにローカルグローバル変換を計算する
					const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(header.rotationRes)) + (2. * pi / (ureal)(header.rotationRes + 1) / 2.);//ボールの回転角度
					const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//グローバルからレンズボールローカルへの変換 XYZ座標

					//レイのローカルを計算できる
					const arrow3 targetInBalllocal(GlobalToBallLocal.prograte() * devTargetRayInGlobal.org(), GlobalToBallLocal.prograte() * devTargetRayInGlobal.dir());

					//このレイがプロジェクタに当たるか
					const auto targetTOpt = IntersectSphere(targetInBalllocal, lensballParam.first, lensballParam.second);//レイの大まかな着弾点を計算するSphereのどこに当たりますか
					if (targetTOpt.isHit) {
						const auto& targetTVsBall = targetTOpt.t;
						const uvec3 hitPosVsSphereInBalllocal = targetTOpt.pos;//Balllocalでの交差位置　レンズボールとターゲットの
						const uvec2 hitposVsSphereInBalllocalPolar = XyzToPolar(hitPosVsSphereInBalllocal);
						const uvec2 hitposVsSphereInMap = PolarToMap(hitposVsSphereInBalllocalPolar);
						const uvec2 hitposVsSphereInMapDesigned = DesignedMapToMap.untiprograte() * hitposVsSphereInMap;
						//if (printMessagesInDevelopping)py::sf("plt.scatter(%f,%f,color=(%f,0,0))", hitposVsSphereInMapD.x(), hitposVsSphereInMapD.y(), rd == 11 ? 1. : 0.);

						//行に当たりをつける
						const ureal regRayDirLati = [&] {
							const ureal zerothRowlati = -(eachRowsDistance * (ureal)(rowNum - 1) / 2.);//zero番目の行の高さ
							const ureal zeroSetRayDir = hitposVsSphereInMapDesigned.y() - zerothRowlati;//0番目の行の高さに始点を合わせたレイの高さ
							const ureal finalRowlati = (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//rowNum-1番目の行の高さ
							//スケーリング　fin~0までのスケールがrowNum-1~0までのスケールになって欲しい
							const ureal thisscale = (ureal)(rowNum - 1 - 0) / (finalRowlati - zerothRowlati);

							return thisscale * zeroSetRayDir;
						}();
						const int centerRawIndex = round(regRayDirLati);//四捨五入するともっともらしいインデックスがわかる
						const int rawSearchWay = (regRayDirLati - (ureal)centerRawIndex) > 0. ? +1 : -1;//検索方向

						//ではここからレンズに当たりをつける
						ureal closestT = std::numeric_limits<ureal>::infinity();//見つかったレンズの距離
						std::pair<size_t, size_t> hitlensIds;
						optional<std::pair<sphereParam, uvec3>> hitlensParamInBalllocal;//対象のレンズパラメータと衝突法線　ボールローカルで
						bool escapeLensSearching = false;//このフラグがたったたらレンズの検索をやめる

						for (size_t rilini = 0; rilini < searchAreaInARow&&(!escapeLensSearching); rilini++) {//検索範囲は全部の行
							const size_t rid = GetBisideIndex(rilini, centerRawIndex, rawSearchWay, rowNum);//当たりをつけたところから放射状に探索する

							//つぎにphiから何番目のレンズかを考える
							const ureal regRayDirLonn = [&] {
								//const ureal tlonn = uleap(PairMinusPlus(rowLength/2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);

								const ureal zerothlenslonn = -rowLength / 2.;//-rowLength/2.が最初のレンズの位置　場合によって前後するけどね
								const ureal zeroSetRayDir = hitposVsSphereInMapDesigned.x() - zerothlenslonn;//zero番目のレンズの場所をzeroに
								const ureal finlenslonn = uleap(PairMinusPlus(rowLength / 2.), (lensNumInARow - 1) / (ureal)lensNumInARow);//最後のレンズの場所
								//スケーリング　fin~0までのスケールがlensNumInCollum-1~0までのスケールになって欲しい
								const ureal thisscale = (ureal)(lensNumInARow - 1 - 0) / (finlenslonn - zerothlenslonn);

								return thisscale * zeroSetRayDir;
							}();
							const bool eachFlag = rid % 2;//交互に切り替わるフラグ 立っているときは行が半周進んでる
							const int centerLensIndex = round(regRayDirLonn - (eachFlag ? 0.5 : 0.));//これはオフセットがない　つまりよりマイナス側から始まっている行にいる場合のインデックス そうでなければ-0.5してから丸める←やりました
							const int lensSearchWay = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? +1 : -1;//隣り合うレンズのもっともらしいインデックスもわかる

							//ではレンズの当たり判定を始める
							for (size_t lidlini = 0; lidlini < searchAreaInALen && (!escapeLensSearching); lidlini++) {
								const size_t lid = GetBisideIndex(lidlini, centerLensIndex, lensSearchWay, lensNumInARow);//当たりをつけたところから放射状に探索する

								const auto thislensparamInBalllocal = nodelensParamsInBalllocal.value()[make_pair(rid, lid)];

								//球と当たり判定する
								const auto thisnodeT = IntersectSphere(targetInBalllocal, thislensparamInBalllocal.first, thislensparamInBalllocal.second);
								if (thisnodeT.isHit) {
									//やっぱり一番近いレンズを見つけておわり
									if (thisnodeT.t < closestT) {
										hitlensParamInBalllocal = make_pair(thislensparamInBalllocal, thisnodeT.norm);
										closestT = thisnodeT.t;
									}
								}
							}

						}

						//要素レンズがないところに当たったってこと
						if (!hitlensParamInBalllocal) {
							continue;//回せば当たるかもよ
						}

						//つぎにスネルの法則で球を演算する
						ray3 targetSeriesInBalllocal(targetInBalllocal);
						const auto fIntersect = IntersectSphere(targetSeriesInBalllocal.back(), hitlensParamInBalllocal.value().first.first, hitlensParamInBalllocal.value().first.second);
						if (!fIntersect.isHit)throw logic_error("当たり判定のロジックがバグってます");
						fIntersect.ApplyToRay(targetSeriesInBalllocal);
						if (!RefractSnell(targetSeriesInBalllocal, fIntersect.norm, nodelensEta)) {
							if (printMessagesInDevelopping)cout << "全反射が起きた(入射時)" << endl;
							escapeLensSearching = true;
							continue;
						}

						//さらに対面の判定をする
						const auto bIntersect = IntersectSphere(targetSeriesInBalllocal.back(), hitlensParamInBalllocal.value().first.first, hitlensParamInBalllocal.value().first.second);
						if (!bIntersect.isHit)throw logic_error("当たり判定のロジックがバグってます");
						bIntersect.ApplyToRay(targetSeriesInBalllocal);
						if (!RefractSnell(targetSeriesInBalllocal, -bIntersect.norm, 1. / nodelensEta)) {
							if (printMessagesInDevelopping)cout << "全反射が起きた(出射時)" << endl;
							escapeLensSearching = true;
							continue;
						}

						//これでレンズボール内での方向がわかった
						const auto& refractedArrow = targetSeriesInBalllocal.back();
						//ResetPyVecSeries(mlabSeries);
						//for (const auto& a : targetSeriesInBalllocal)
						//	AppendPyVecSeries(mlabSeries, a.org());
						//py::sf("mlab.plot3d(%s,tube_radius=0.01)", GetPySeriesForPlot(mlabSeries));
						
						//つぎにプロジェクターのどの画素に当たるかを解く
						//まず開口に当たるかい
						rez->push_back(Eigen::Vector3i(0, 0, 0));
						auto apertureT = IntersectSphereAndArrow(apertureProjector, refractedArrow);
						if (!apertureT) {
							//プロジェクタには入社しなかった
							if (printMessagesInDevelopping)cout << "プロジェクタには入射しなかった" << endl;
							escapeLensSearching = true;
							continue;
						}

						//開口にあたったらレイの向きで画素を判断できる
						const uvec3 refractedDirInGlobal = GlobalToBallLocal.untiprograte() * refractedArrow.dir();
						//限界の角度(プロジェクタの投映角度)との比率でUV座標を算出できる
						const uvec2 refractedDirInGlobalPolar = XyzToPolar(refractedDirInGlobal);//投映はx方向(phi=0)に行われるのだからプロジェクタ中心は(phi=pi theta=0.)

						//各座標を0~1に正規化してくれるはず
						const uvec2 regpos = 0.5 * (uvec2(NormalizeAngle((refractedDirInGlobalPolar.x()-pi)) / projectorHalfAnglePhi, NormalizeAngle(refractedDirInGlobalPolar.y()) / projectorHalfAngleTheta) + uvec2(1., 1.));

						//座標にする
						Eigen::Vector2i pixpos(regpos.x() * projectorResInPhi, regpos.y() * projectorResInTheta);
						cout << pixpos.x() << "\t" << pixpos.y() <<"\t"<< NormalizeAngle((refractedDirInGlobalPolar.x() - pi)) << endl;

						//無効な座標でなければリストに入れる
						if (pixpos.x() >= 0 && pixpos.x() < projectorResInPhi && pixpos.y() >= 0 && pixpos.y() < projectorResInTheta) {

							rez->push_back(Eigen::Vector3i(pixpos.x(), pixpos.y(), rd));
						}
					}
					else break;//レンズボールに当たらなかったらあんま意味ない

				}

				(*finflagOfStIte) = true;
			};

			auto cameraRayIte = cameraRayList.cbegin();
			auto cameraRezIte = pixListForCam.begin();
			for (size_t y = 0; y < cameraResH; y++)
				for (size_t x = 0; x < cameraResW; x++) {

					FindPixelsFromARay(*cameraRayIte, finFlagOfEachDevThread.begin() + 0, cameraRezIte);
					cameraRezIte++;
					cameraRayIte++;
					////このrdgenでの処理を開いているスレッドに割り付けたい
					//bool isfound = false;
					//while (!isfound) {//割り付けられなければ繰り返す
					//	for (size_t th = 0; th < devThreadNum; th++)
					//		if (!devThreads.at(th)) {//空きなら割付
					//			if (!isfound) {//一つのインデックスには一回だけ割り付ける
					//				isfound = true;
					//				finFlagOfEachDevThread.at(th) = false;//フラグをクリアして

					//				//マップに登録して
					//				//cout << x << "\t" << y << endl;
					//				devThreads.at(th).reset(new std::thread(FindPixelsFromARay, *cameraRayIte, finFlagOfEachDevThread.begin() + th, cameraRezIte));//スレッド実行開始

					//				cameraRezIte++;
					//				cameraRayIte++;
					//			}
					//		}
					//		else if (finFlagOfEachDevThread.at(th)) {//空いてなくて終わってるなら
					//			devThreads.at(th).get()->join();
					//			devThreads.at(th).release();//リソースを開放
					//		}
					//}
				}

			//とりあえず映像が見えるゾーンをマッピング
			auto cameraRezIteWrite = pixListForCam.cbegin();
			bmpLib::img myImage;
			myImage.width = cameraResW;
			myImage.data.resize(cameraResH);

			myImage.height = cameraResH;
			for (int y = 0; y < myImage.height; y++) {
				myImage.data.at(myImage.height - 1 - y).resize(cameraResW);
				for (int x = 0; x < myImage.width; x++) {

					myImage.data[myImage.height - 1 - y][x] = bmpLib::color((*cameraRezIteWrite).size() ? 255 : 0, 0, 0);
					cameraRezIteWrite++;
				}
			}
			bmpLib::WriteBmp((rezpath+branchpath+"final.bmp").c_str(), &myImage);

		}









		//スキャンをする
		const ureal nodeLensFocalLength = nodeLensRadius * 1.5;//要素レンズの中心から焦点までの距離

		//std::unordered_map<Eigen::Vector3i, arrow3> projectorRefractRayMap;

		constexpr size_t scanThreadNum = 16;//スキャンに使うスレッド数
		const std::string resultDicPrefix = "projRefRayMap";

		constexpr bool scanLenses = false;//レンズボールに対するレイトレーシングを行う
		constexpr bool drawRefractionDirectionOfARay = false;//あるレイの屈折方向を描画する
		constexpr bool logWarningInScan = false;//scan中の警告を表示する

		bool onetime = false;

		if(scanLenses){

			//並列処理用のいろいろ
			std::array<uptr<std::thread>, scanThreadNum> scanThreads;//実行スレッド
			std::array<bool, scanThreadNum> finFlagOfEachScanThread;//スキャンがおわったことを報告

			//格納結果の形式を示すヘッダを作って保存する
			const projRefraDicHeader storageheader(projectorResInPhi, projectorResInTheta, numOfProjectionPerACycle);
			storageheader.SaveHeader(rezpath + branchpath + resultDicPrefix);

			ResetPyVecSeries<6>(quiverSeries,quiverPrefix);//ベクトル場をお掃除
			ResetPyVecSeries(pypltSeries);//ベクトル場をお掃除


			//指定された結果を指定されたストレージへ送信する 転送したあとはRezはクリアされます
			const auto TransRezToStorage = [&](std::list<arrow3>& rezOfSt, ofstream& storageofStPtr) {
				cereal::BinaryOutputArchive o_archive(storageofStPtr);
				o_archive(rezOfSt);

				rezOfSt.clear();
			};
			//回転角度ごとにスキャンを行う
			const auto ScanAScene = [&](const std::decay<decltype(numOfProjectionPerACycle)>::type rd, decltype(finFlagOfEachScanThread)::iterator finflagOfStIte) {

				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(numOfProjectionPerACycle)) + (2. * pi / (ureal)(numOfProjectionPerACycle + 1) / 2.);//ボールの回転角度
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//グローバルからレンズボールローカルへの変換 XYZ座標

				//結果格納メモリとストレージを用意する
				std::list<arrow3> rezMem;
				std::ofstream storageStream(rezpath + branchpath + resultDicPrefix + ".part" + to_string(rd), std::ios::binary);

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
								if (NaihouHanteiX(hitDistInMapDesigned, hexverticesNodelensOuter)) {
									if (hitDistInMapDesigned.norm() >= nodeLensRadius)throw logic_error("判定がだめ");
									//printf("ok r=%d l=%d\n", rid, lid);

									//とりあえずテスト　これがcenterでない可能性はあるの?
									if (rid != centerRawIndex || lid != centerLensIndex)
										if (logWarningInScan)std::cout << "別にエラーではないけど想像したのと違う!" << endl;

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
						rezMem.push_back(arrow3(rayDirInGlobal, refractRayDirInGlobal));

						//プロットします　ヒットポイントに
						if (vpd == 0 && drawRefractionDirectionOfARay) {
							const auto color = HsvToRgb({ rd / (ureal)(numOfProjectionPerACycle - 1),1.,1. });
							py::sf("mlab.quiver3d(0,0,0,%f,%f,%f,mode=\"arrow\",color=(%f,%f,%f))", refractRayDirInGlobal.x(), refractRayDirInGlobal.y(), refractRayDirInGlobal.z(), color[0], color[1], color[2]);
							py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", hitLensCenterAndHitDistInMapDesigned.value().second.x(), hitLensCenterAndHitDistInMapDesigned.value().second.y(), color[0], color[1], color[2]);
						}
					}
				}

				//スキャンが終わったらセーブ
				TransRezToStorage(rezMem, storageStream);

				*finflagOfStIte = true;
			};

			//複数スレッドに回転角度を変えながら割り当てる
			for (std::decay<decltype(numOfProjectionPerACycle)>::type rdgen = 0; rdgen < numOfProjectionPerACycle; rdgen++) {
				std::cout << "count: " << rdgen << endl;
				//このrdgenでの処理を開いているスレッドに割り付けたい
				bool isfound = false;
				while (!isfound) {//割り付けられなければ繰り返す
					for (size_t th=0;th<scanThreadNum;th++)
						if (!scanThreads.at(th)) {//空きなら割付
							if (!isfound) {//一つのインデックスには一回だけ割り付ける
								isfound = true;
								finFlagOfEachScanThread.at(th) = false;//フラグをクリアして

								scanThreads.at(th).reset(new std::thread(ScanAScene, rdgen,  finFlagOfEachScanThread.begin() + th));//スレッド実行開始
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
			}
		};






		//表示する 3d 2dの順
		//py::s("mlab.show()");
		py::s("plt.show()");

		const auto endTimePoint = std::chrono::system_clock::now();
		std::cout << "Finish: " << endTimePoint << endl;
		std::cout << "Process time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTimePoint - startTimePoint).count() / 1000. << " [s]" << endl;
		std::cout << "The work is complete...Wait rendering by Python" << endl;
		py::Terminate();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << endl;
		std::cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//なんか入れたら終わり

		py::Terminate();
		return -1;
	}
	catch (...) {
		std::cout << "unknown err" << endl;
		std::cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//なんか入れたら終わり

		py::Terminate();
		return -2;
	}

	return 0;
}
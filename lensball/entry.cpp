#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"
#include "bmpLib/bmp.hpp"

using namespace std;

const std::string appContextPath(R"(./lensball.context)");//コンテキスト置き場　かえないで
constexpr size_t appContextVersion=1;//コンテキストのバージョン　内容が変わったらこれを更新してください

std::string rezpath = "./";//結果を格納するフォルダ(コンテキストに支配されます)
const std::string branchpath = "ExLens/";//このbranchの結果を格納するフォルダ

using py = pythonRuntime;

//ハードウェアのスペック
namespace hardwareParams {
	//ハードウェアスペック
	constexpr size_t projectorFps = 22727;//フレームレート
	constexpr size_t numOfProjectionPerACycle = 1024;//一周ごとの投映数
	constexpr ureal rotationSpeed = projectorFps / (ureal)numOfProjectionPerACycle;//回転速度
	//解像度
	constexpr size_t verticalDirectionResolution = sqrt_constexpr((ureal)numOfProjectionPerACycle);//垂直解像度(一周に並んでいるレンズの数)
	constexpr size_t horizontalDirectionResolution = numOfProjectionPerACycle / verticalDirectionResolution;//水平解像度　位置レンズあたりの投映数

	//プロジェクタのパラメータ
	constexpr size_t projectorResInTheta = 768;//プロジェクタの縦側解像度 ホントはXGA
	constexpr ureal projectorHalfAngleTheta = 60. / 180. * pi;//プロジェクトの投映角
	constexpr size_t projectorResInPhi = 768; // プロジェクタの横側解像度 ホントはXGA
	constexpr ureal projectorHalfAnglePhi = projectorHalfAngleTheta * (projectorResInPhi / (ureal)projectorResInTheta);//プロジェクトの投映角

};
//レンズボールのデザインパラメータ
namespace lensballDesignParams {

	constexpr ureal sphereRadiusInner = 1.;//レンズボール内径の直径
	const sphereParam lensballParamInner(uvec3::Zero(), sphereRadiusInner);//レンズボールのパラメータ
	const sphereParam lensballParamOuter(uvec3::Zero(), sphereRadiusInner);//レンズボールのパラメータ
	
	constexpr size_t lensNumInARow = hardwareParams::verticalDirectionResolution;//一行あたりの行の数

	constexpr ureal rowAngle = 0.02706659 * 2.;//行の角度 theta=atan(-1.5*1/(cos(theta)*sqrt(3))*1/lensnum)
	const ureal rowLength = 2. * pi * cos(rowAngle);//行の長さ
	const ureal lensEdgeWidth = rowLength / (ureal)lensNumInARow / 2.;
	const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInARow;//六角形の一変だけシフトする
	const bitrans<Eigen::Rotation2D<ureal>> DesignedMapToMap = bitrans<Eigen::Rotation2D<ureal>>(Eigen::Rotation2D<ureal>(rowAngle));//レンズアレイを傾斜させる前から傾斜させたあとにする

	//const ureal nodeLensRadius = 2. * lensEdgeWidth / sqrt(3.);//要素レンズ形状を作成　球の半径
	const std::pair<size_t, size_t> nodeLensResolution = make_pair(20 * 2, 20);//要素レンズの分割数
	constexpr size_t rowNum = 15;//奇数にしてね 行の数

	//要素レンズの概形は六角形になるはず
	const auto hexverticesNodelensOuter = [&] {
		auto hexvjunk = MakeHexagon(lensballDesignParams::lensEdgeWidth);//六角形の頂点
		hexvjunk.push_back(hexvjunk.front());//一周するために最初の点を末尾に挿入

		return hexvjunk;
	}();


	constexpr ureal nodelensEta = 1.2;//ノードレンズの比屈折率
};
//現像時のパラメータ
namespace developperParams {
	const ureal apertureRadius = lensballDesignParams::sphereRadiusInner / 300.;//ここにあたったらプロジェクターから出たってこと
	const auto regularHexagon = [&] {
		auto hexvjunk = MakeHexagon(sqrt(3.) / 2.);//六角形の頂点
		hexvjunk.push_back(hexvjunk.front());//一周するために最初の点を末尾に挿入

		return hexvjunk;
	}();//外接円の半径が1になるような六角形
	constexpr size_t searchAreaInALen = 8;//同じ行のレンズをどれだけ深追いして検索するか
	constexpr size_t searchAreaInARow = 8;//列をどれだけ深追いして検索するか
	//あたりを付けたノードレンズから探索する範囲
	//まずヘッダを読み出す
	const std::string framePrefixX = "frames\\frame";//フレームを格納しているフォルダのprefix <prefix><id>.bmpみたいな名前にしてね
	const std::string dicHeaderPathX = "dic\\dic.head";//辞書ヘッダのパス
	const std::string developRezPath = "dev";//branchフォルダ内のここに結果を保存する

	constexpr ureal nodelensExpand = 1. + 1.e-4;//ノードレンズのギリギリに入射したときに判定できるようにする拡大率
	size_t devThreadNum = 1;//現像をどれだけのスレッドで実行するか(コンテキストに支配されます)

	//現像に使うカメラ
	constexpr ureal fovHalf = 1.5 / 180. * pi;
	constexpr size_t antialiasInH = 1;//y方向にこれだけサンプルしてから縮小する
	constexpr size_t cameraResW = 1024, cameraResH = cameraResW* antialiasInH;//あるレイに代表するからね
	constexpr ureal brightnessCoef = 3.;//明るさ係数　これだけ明るくなる

	constexpr size_t subStepRes=10;//より細かくボールを回す

	auto cameraToGlobal = Eigen::Affine3d(Eigen::AngleAxis<ureal>(0. / 180. * pi, uvec3::UnitY())*Eigen::AngleAxis<ureal>(0./180.*pi ,uvec3::UnitZ())* Eigen::Translation<ureal, 3>(uvec3(30.,0.,0.)));//カメラの変換 カメラは-xを視線方向 zを上方向にする
};
//スキャン時のパラメータ
namespace scanParams {
	size_t scanThreadNum = 1;//スキャンに使うスレッド数(コンテキストに支配されます)
	const std::string resultDicPrefix = "dic\\dic";
	constexpr size_t searchAreaInALen = 5;//同じ行のレンズをどれだけ深追いして検索するか
	constexpr size_t searchAreaInARow = 5;//列をどれだけ深追いして検索するか
}

ureal GetLatitudeInMapDFromRowIndex(size_t row) {
	return lensballDesignParams::eachRowsDistance * row - (lensballDesignParams::eachRowsDistance * (ureal)(lensballDesignParams::rowNum - 1) / 2.);
}
ureal GetLongitudeInMapDFromLensIndexAndRowFlag(size_t lensIdInARow, bool isOddRow) {
	return uleap(PairMinusPlus(lensballDesignParams::rowLength / 2.), lensIdInARow / (ureal)lensballDesignParams::lensNumInARow) + (isOddRow ? ((lensballDesignParams::rowLength) / (ureal)lensballDesignParams::lensNumInARow / 2.) : 0.);
}

//画像の(w0,h0)は(-asp,-1)
//プロジェクタの画素から放出されるレイの向きをもとめる uv座標系　高さがz、横がy軸　+x方向に放射される　原点から
uvec3 GetRayDirFromProjectorPix(const ivec2& pix) {
	const auto aspect = hardwareParams::projectorResInPhi / (ureal)hardwareParams::projectorResInPhi;
	//この二次元座標から表示面上での座標がわかる
	const uvec2 posInDisplay(uleap(PairMinusPlus(aspect), pix.x() / (ureal)(hardwareParams::projectorResInPhi - 1)), uleap(PairMinusPlus(1.), pix.y() / (ureal)(hardwareParams::projectorResInTheta - 1)));
	//表示面の位置を求める
	const ureal dist = 1. / tan(hardwareParams::projectorHalfAngleTheta);

	//原点を通ってここにたどり着くレイ
	return uvec3(dist, posInDisplay.x(), posInDisplay.y()).normalized();
}
//入射方向からプロジェクタピクセルを特定する
ivec2 GetPixPosFromEnteredRay(const uvec3& enteredDir) {
	//表示面のx位置
	const ureal dist = 1. / tan(hardwareParams::projectorHalfAngleTheta);

	//アパーチャを絞れば原点を通ると考えられるから交差の式を解く
	const ureal t = dist / enteredDir.x();
	const uvec3 crosspos = enteredDir * t;//ここが表示面との交差

	const uvec2 posInDisplay(crosspos.y(), crosspos.z());

	//逆算をするとこういうこと
	const uvec2 realpos = 0.5 * (posInDisplay + uvec2(1., 1.));
	return  ivec2(round(realpos.x() * (ureal)(hardwareParams::projectorResInPhi - 1)), round(realpos.y() * (ureal)(hardwareParams::projectorResInTheta - 1)));
}



enum rayIncidentWay {
	toInner,//ボール内側から
	toOuter//ボール外側から
};
class doubleFaceLensParam :private std::pair<sphereParam, sphereParam> {
	using super = std::pair<sphereParam, sphereParam>;
public:
	sphereParam& inSurf() { return this->first; }//内側の面
	sphereParam& outSurf() { return this->second; }//内側の面
	const sphereParam& inSurf() const{ return this->first; }//内側の面
	const sphereParam& outSurf() const{ return this->second; }//内側の面

	doubleFaceLensParam() = default;
	doubleFaceLensParam(const doubleFaceLensParam&) = default;
	//コピーできます
	doubleFaceLensParam(const super& s) :super(s){
		//*this = s;
	}
	//代入できます
	doubleFaceLensParam& operator=(const super& s) {
		super::operator=(s);
		return *this;
	}
};


//要素レンズパラメータ辞書　レンズIDから前面と後面の球面パラメータを検索できる
using nodeLensDic = std::unordered_map<std::pair<size_t, size_t>, doubleFaceLensParam, HashPair>;

//ノードレンズを検索した結果そのレンズパラメータと交差場所を返してくれる
class resultSearchNodeLens :private std::pair<doubleFaceLensParam, resultIntersecteSphere> {
	using super = std::pair<doubleFaceLensParam, resultIntersecteSphere>;
public:
	doubleFaceLensParam& lensParam() { return this->first; }
	resultIntersecteSphere& hitParam() { return this->second; }
	const doubleFaceLensParam& lensParam() const{ return this->first; }
	const resultIntersecteSphere& hitParam() const{ return this->second; }


	resultSearchNodeLens(const super& s) :super(s){
	}
	resultSearchNodeLens& operator=(const super& s) {
		super::operator=(s);

		return *this;
	}

	resultSearchNodeLens() = default;

};

//Search Node lens関数が最初に当たりをつけたレンズからどれほど離れたレンズを結果として返したかを格納する　これがリミット寸前ならリミットの設定がおかしいかもしれない
class accuracyOfSearchNodelenses:private std::pair<size_t,size_t> {
	using super = std::pair<size_t, size_t>;
public:

	super::first_type& distOfRow() { return this->first; }
	super::first_type& distOfLens() { return this->second; }
	const super::first_type& distOfRow() const{ return this->first; }
	const super::first_type& distOfLens() const{ return this->second; }

	//与えられたaccuracyと比較してこのaccurasyを最悪に更新する
	accuracyOfSearchNodelenses& UpdateThisToWorth(const accuracyOfSearchNodelenses& op) {
		//各要素ごとの最大をとる
		this->distOfRow() = std::max(this->distOfRow(), op.distOfRow());
		this->distOfLens() = std::max(this->distOfLens(), op.distOfLens());

		return *this;
	}
	std::string str() {
		stringstream ss;
		ss << "row: " << this->distOfRow() << ",\t lens: " << this->distOfLens();
		return ss.str();
	}

	accuracyOfSearchNodelenses(const super& s) :super(s) {
	}
	accuracyOfSearchNodelenses& operator=(const super& s) {
		super::operator=(s);

		return *this;
	}

	accuracyOfSearchNodelenses() :super(0, 0){}
};

//要素レンズを検索する 内側からか外側からかで振る舞いが変わります 	
optional<std::pair<resultSearchNodeLens, accuracyOfSearchNodelenses>> SearchNodeLensHitByARayInBalllocalX(const arrow3& targetInBalllocal, const resultIntersecteSphere& hitRezVsSphere, const nodeLensDic& nodelensParamsInBalllocal, const size_t searchAreaInARow, const size_t searchAreaInALen,const rayIncidentWay& enterFrom) {

	//レイとレンズボールの交差位置をいろんな座標系で計算
	//const auto& targetTVsBall = hitRezVsSphere.t;
	const uvec3 hitPosVsSphereInBalllocal = hitRezVsSphere.pos;
	const uvec2 hitposVsSphereInBalllocalPolar = XyzToPolar(hitPosVsSphereInBalllocal);
	const uvec2 hitposVsSphereInMap = PolarToMap(hitposVsSphereInBalllocalPolar);
	const uvec2 hitposVsSphereInMapDesigned = lensballDesignParams::DesignedMapToMap.untiprograte() * hitposVsSphereInMap;
	//if (printMessagesInDevelopping)py::sf("plt.scatter(%f,%f,color=(%f,0,0))", hitposVsSphereInMapD.x(), hitposVsSphereInMapD.y(), rd == 11 ? 1. : 0.);

	//行に当たりをつける
	const ureal regRayDirLati = [&] {
		const ureal zerothRowlati = GetLatitudeInMapDFromRowIndex(0);//zero番目の行の高さ
		const ureal zeroSetRayDir = hitposVsSphereInMapDesigned.y() - zerothRowlati;//0番目の行の高さに始点を合わせたレイの高さ
		const ureal finalRowlati = GetLatitudeInMapDFromRowIndex(lensballDesignParams::rowNum - 1);//rowNum-1番目の行の高さ
		//スケーリング　fin~0までのスケールがrowNum-1~0までのスケールになって欲しい
		const ureal thisscale = (ureal)(lensballDesignParams::rowNum - 1 - 0) / (finalRowlati - zerothRowlati);

		return thisscale * zeroSetRayDir;
	}();
	const int centerRawIndex = NormalizeIntger<signed int>((signed int)round(regRayDirLati), lensballDesignParams::rowNum);//四捨五入するともっともらしいインデックスがわかる
	const int rawSearchWay = (regRayDirLati - (ureal)centerRawIndex) > 0. ? +1 : -1;//検索方向


	//ではここからレンズに当たりをつける
	ureal closestT = std::numeric_limits<ureal>::infinity();//見つかったレンズの距離
	std::pair<size_t, size_t> hitlensIds;
	optional<std::pair<resultSearchNodeLens, accuracyOfSearchNodelenses>> hitlensParamInBalllocal;//対象のレンズパラメータと衝突法線　ボールローカルで

	for (size_t rilini = 0; rilini < searchAreaInARow; rilini++) {//検索範囲は全部の行
		const size_t rid = GetBisideIndex(rilini, centerRawIndex, rawSearchWay, lensballDesignParams::rowNum);//当たりをつけたところから放射状に探索する

		//つぎにphiから何番目のレンズかを考える
		const ureal regRayDirLonn = [&] {
			//const ureal tlonn = uleap(PairMinusPlus(rowLength/2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);

			const ureal zerothlenslonn = GetLongitudeInMapDFromLensIndexAndRowFlag(0, rid % 2);//-rowLength/2.が最初のレンズの位置　場合によって前後するけどね
			const ureal zeroSetRayDir = hitposVsSphereInMapDesigned.x() - zerothlenslonn;//zero番目のレンズの場所をzeroに
			const ureal finlenslonn = GetLongitudeInMapDFromLensIndexAndRowFlag(lensballDesignParams::lensNumInARow - 1, rid % 2);//最後のレンズの場所
			//スケーリング　fin~0までのスケールがlensNumInCollum-1~0までのスケールになって欲しい
			const ureal thisscale = (ureal)(lensballDesignParams::lensNumInARow - 1 - 0) / (finlenslonn - zerothlenslonn);

			return thisscale * zeroSetRayDir;
		}();
		const bool eachFlag = rid % 2;//交互に切り替わるフラグ 立っているときは行が半周進んでる
		const int centerLensIndex = NormalizeIntger<signed int>((signed int)round(regRayDirLonn - (eachFlag ? 0.5 : 0.)), lensballDesignParams::lensNumInARow);//これはオフセットがない　つまりよりマイナス側から始まっている行にいる場合のインデックス そうでなければ-0.5してから丸める←やりました
		const int lensSearchWay = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? +1 : -1;//隣り合うレンズのもっともらしいインデックスもわかる

		//ではレンズの当たり判定を始める
		for (size_t lidlini = 0; lidlini < searchAreaInALen; lidlini++) {
			const size_t lid = GetBisideIndex(lidlini, centerLensIndex, lensSearchWay, lensballDesignParams::lensNumInARow);//当たりをつけたところから放射状に探索する

			const auto thislensparamInBalllocal = nodelensParamsInBalllocal.at(make_pair(rid, lid));//これが怪しいレンズのパラメータ

			//球と当たり判定する
			const auto hitRezVsANode = [&] {

				//内側か外側かによって分岐
				if (enterFrom == rayIncidentWay::toInner) return IntersectSphere(targetInBalllocal, thislensparamInBalllocal.inSurf().first, thislensparamInBalllocal.inSurf().second);
				else return IntersectSphere(targetInBalllocal, thislensparamInBalllocal.outSurf().first, thislensparamInBalllocal.outSurf().second);
			}();
			//レンズボール概形よりも絶対近い場所のはず これは内側外側問わずね
			if (hitRezVsANode.isHit && hitRezVsANode.t < hitRezVsSphere.t) {
				//やっぱり一番近いレンズを見つけておわり
				if (hitRezVsANode.t < closestT) {
					hitlensParamInBalllocal = make_pair(make_pair(thislensparamInBalllocal, hitRezVsANode), make_pair(rilini, lidlini));

					if (hitlensParamInBalllocal.value().first.lensParam().inSurf().second == 0.)
						int ad = 0;
					closestT = hitRezVsANode.t;
				}
			}
		}

	}

	//要素レンズがないところに当たったってこと
	return hitlensParamInBalllocal;
}

//スネルの法則で光の経路を計算 どっちから入射したかが大事
std::optional<arrow3> GetRefractedRayWithASphericalLensX(const arrow3& targetInBalllocal, const resultSearchNodeLens& hitlensParamInBalllocal, const bool printMessagesInDevelopping,const rayIncidentWay& inciWay) {
	//入射方向によって入射面と出射面が変わる
	const sphereParam& inciSurf = (inciWay == rayIncidentWay::toInner) ? hitlensParamInBalllocal.lensParam().inSurf() : hitlensParamInBalllocal.lensParam().outSurf();
	const sphereParam& emitSurf = (inciWay == rayIncidentWay::toInner) ? hitlensParamInBalllocal.lensParam().outSurf() : hitlensParamInBalllocal.lensParam().inSurf();

	ray3 targetSeriesInBalllocal(targetInBalllocal);
	const auto fIntersect = IntersectSphere(targetSeriesInBalllocal.back(), inciSurf.first, inciSurf.second);
	if (!fIntersect.isHit)
		throw logic_error("当たり判定のロジックがバグってます");
	fIntersect.ApplyToRay(targetSeriesInBalllocal);
	if (!RefractSnell(targetSeriesInBalllocal, fIntersect.norm, lensballDesignParams::nodelensEta)) {
		if (printMessagesInDevelopping)cout << "全反射が起きた(入射時)" << endl;
		return std::optional<arrow3>();//このシーンではだめだったので次のレイ
	}

	//さらに対面の判定をする
	const auto bIntersect = IntersectSphere(targetSeriesInBalllocal.back(), emitSurf.first, emitSurf.second);
	if (!bIntersect.isHit)throw logic_error("当たり判定のロジックがバグってます");
	bIntersect.ApplyToRay(targetSeriesInBalllocal);
	if (!RefractSnell(targetSeriesInBalllocal, -bIntersect.norm, 1. / lensballDesignParams::nodelensEta)) {
		if (printMessagesInDevelopping)cout << "全反射が起きた(出射時)" << endl;
		return std::optional<arrow3>();//このシーンではだめだったので次のレイ
	}



	//これでレンズボール内での方向がわかった
	const auto& refractedArrow = targetSeriesInBalllocal.back();
	//ResetPyVecSeries(mlabSeries);
	//for (const auto& a : targetSeriesInBalllocal)
	//	AppendPyVecSeries(mlabSeries, a.org());
	//py::sf("mlab.plot3d(%s,tube_radius=0.01)", GetPySeriesForPlot(mlabSeries));

	return std::optional<arrow3>(refractedArrow);
}

//カメラ映像を保存する
void WriteBmpOfCamera(const std::unordered_map<ivec2, uvec3>& colorList, const std::unordered_map < ivec2, ureal >& colorSiz, const std::string& filename, const bool useTimestamp=true) {

	bmpLib::img picture;//カメラからの映像
	picture.width = developperParams::cameraResW;
	picture.data.resize(developperParams::cameraResH / developperParams::antialiasInH);
	picture.height = developperParams::cameraResH / developperParams::antialiasInH;
	bmpLib::img maskPic;//どこに値が存在するか
	maskPic.width = developperParams::cameraResW;
	maskPic.data.resize(developperParams::cameraResH / developperParams::antialiasInH);
	maskPic.height = developperParams::cameraResH / developperParams::antialiasInH;
	for (int yy = 0; yy < picture.height; yy++) {
		picture.data.at(yy).resize(developperParams::cameraResW);
		maskPic.data.at(yy).resize(developperParams::cameraResW);
		for (int x = 0; x < picture.width; x++) {
			//まず対象の位置を初期化して
			picture.data[yy][x] = bmpLib::color(0, 0, 0);
			maskPic.data[yy][x] = bmpLib::color(0, 0, 0);
			for (int a = 0; a < developperParams::antialiasInH; a++) {
				const auto pixIte = colorList.find(ivec2(x, yy * developperParams::antialiasInH + a));
				const auto sizeIte = colorSiz.find(ivec2(x, yy * developperParams::antialiasInH + a));//対応したピクセルを設置
				if (pixIte != colorList.cend()) {//ちゃんと色があれば
					picture.data[yy][x] += bmpLib::color(pixIte->second.x() * (developperParams::brightnessCoef / developperParams::antialiasInH), pixIte->second.y() * (developperParams::brightnessCoef / developperParams::antialiasInH), pixIte->second.z() * (developperParams::brightnessCoef / developperParams::antialiasInH));//そもそもレイが放たれている範囲をうっすら色付け
					maskPic.data[yy][x] = bmpLib::color(0, clamp<int>(sizeIte->second, 0, 255), 0);
				}
			}
		}
	}

	const std::string filenameSufix = [&] {
		//名前にタイムスタンプをつける
		if (useTimestamp) {
			const auto timestampText = [&] {
				std::chrono::zoned_time zonedTimestamp{ std::chrono::current_zone(), std::chrono::system_clock::now() };
				auto truncated = std::chrono::time_point_cast<std::chrono::milliseconds>(zonedTimestamp.get_local_time());


				stringstream ss;
				ss << std::format("({:%y%m%d_%H%M%S})", truncated);

				return (std::string)ss.str();
			}();
			return timestampText;
		}
		else return std::string("");
	}();

	bmpLib::WriteBmp((rezpath + branchpath + filename + filenameSufix + ".bmp").c_str(), &picture);
	bmpLib::WriteBmp((rezpath + branchpath + "_mask" + filename + filenameSufix + ".bmp").c_str(), &maskPic);
}

//アパーチャ
bool ThroughAperture(const arrow3& global) {
	constexpr ureal espAperture = 0.0001;
	if (fabs(global.dir().x()) <= espAperture)return false;//アパーチャに平行ならどうしようもない
	const ureal t = -global.org().x() / global.dir().x();//x=0との好転を求める
	const uvec3 hitpos = global.dir() * t + global.org();

	//このノルムが半径未満なら
	return hitpos.norm() <= developperParams::apertureRadius;
}

//シーンidから回転角度を計算する　substepを含めて実数で渡してね
ureal GetRotationAngleFromRd(const ureal rd) {
	return uleap({ 0.,2. * pi }, rd / (ureal)(hardwareParams::numOfProjectionPerACycle));
}

int main(int argc, char* argv[]) {

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

		//コンテキストファイルを読み込んでデバイスに普遍的な処理をする
		constexpr bool useContext = true;
		std::list<std::string> appArgs;//アプリケーションの引数はここに格納される
		if (useContext) {
			appContext usercon;
			{
				ifstream ifs(appContextPath, std::ios::binary);

				cereal::JSONInputArchive iarch(ifs);
				iarch(usercon);
			}

			//コンテキストを適用する
			rezpath = usercon.rezpath;
			developperParams::devThreadNum = usercon.threadNumMax;
			scanParams::scanThreadNum = usercon.threadNumMax;


			//引数処理をする
			if (argc) {//引数が与えられていれば
				for (size_t ad = 0; ad < argc; ad++)
					appArgs.push_back(std::string(argv[ad]));
			}
			else appArgs = usercon.defaultArg;
		}

		//Pythonをセットアップしてからレンズボールの概形を書く
		constexpr bool drawSphere = false;//レンズボール概形を描画する
		constexpr bool drawAparture = false;//アパーチャを描画する
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
mlab.mesh(%f*spx, %f*spy, %f*spz ,color=(1.,1.,1.) ,representation="wireframe" )  
)", sphereResolution, sphereResolution, lensballDesignParams::sphereRadiusInner, lensballDesignParams::sphereRadiusInner, lensballDesignParams::sphereRadiusInner);

			//アパーチャを球で近似して描画する
			if (drawAparture)py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
spx = np.cos(sphphi)*np.sin(sphtheta)
spy = np.sin(sphphi)*np.sin(sphtheta)
spz = np.cos(sphtheta)
mlab.mesh(%f*spx, %f*spy, %f*spz ,color=(0.,1.,0.) )  
)", sphereResolution, sphereResolution, developperParams::apertureRadius, developperParams::apertureRadius, developperParams::apertureRadius);
		};






		//レンズアレイを作成、描画する
		//六角形でタイリングする　偶数行を書いてから奇数行を書くって感じ
		constexpr bool calcNodelenses = true;//ノードレンズの位置を計算してレンズボールを形成する
		constexpr bool drawNodelenses = calcNodelenses & true;//要素レンズを描画する
		constexpr bool drawNodelensEdges = calcNodelenses & true;//ノードレンズの枠線を描画する

		//この計算で要素レンズリストがわかるよ
		std::optional<nodeLensDic> nodelensParamsFrontBackInBalllocal = std::nullopt;//行番号　レンズ番号がキーでパラメータをBalllocalで保存する 入射面と出射面で別々のパラメータを割り当てられる
		if(calcNodelenses){
			//opt計算できます
			nodelensParamsFrontBackInBalllocal = decltype(nodelensParamsFrontBackInBalllocal)::value_type();
			auto& nodelensParamsFrontBackRez = nodelensParamsFrontBackInBalllocal.value();

			//std::list<uleap>//マップ座標でのレンズ中心
			for (std::decay<decltype(lensballDesignParams::rowNum)>::type rd = 0; rd < lensballDesignParams::rowNum; rd++) {
				const ureal tlati = GetLatitudeInMapDFromRowIndex(rd);//lati方向の現在位置
				const bool eachFlag = rd % 2;//交互に切り替わるフラグ
				for (std::decay<decltype(lensballDesignParams::lensNumInARow)>::type ld = 0; ld < lensballDesignParams::lensNumInARow; ld++) {

					auto color = HsvToRgb({ uleap({0.,1.},ld / (ureal)lensballDesignParams::lensNumInARow),1.,1. });//色を行方向に変える
					//六角形を収めるバッファをクリア
					ResetPyVecSeries(pypltSeries);

					//lonn方向の現在位置
					const ureal tlonn = GetLongitudeInMapDFromLensIndexAndRowFlag(ld, eachFlag);
					const auto localcenterInMapDesigned = uvec2(tlonn, tlati);//要素レンズの中央 ローカルマップ座標
					const auto localcenterInMap = lensballDesignParams::DesignedMapToMap.prograte() * localcenterInMapDesigned;
					const auto localcenterInBalllocalPolar = MapToLocalPolar(localcenterInMap);
					const uvec3 localcenterInBalllocal = PolarToXyz(localcenterInBalllocalPolar);

					//レンズの半径を全反射が生じないようなサイズにして解く
					const ureal lensRadius=[&]{
						const ureal lensWidthCrossHalfInMapD = lensballDesignParams::lensEdgeWidth * (2. / sqrt_constexpr(3.));//マップでのレンズの対角幅の半分
						lensballDesignParams::hexverticesNodelensOuter;
						const uvec2 lensWidthHalfVecInMap = lensballDesignParams::DesignedMapToMap.prograte() * uvec2(0., lensWidthCrossHalfInMapD);//Map上でのレンズの対角ベクトルの半分


						//最終的にθ方向の拡がり角が分かればOK
						const ureal lensWidthInTheta = acos(PolarToXyz(MapToLocalPolar(localcenterInMap + lensWidthHalfVecInMap)).dot(PolarToXyz(MapToLocalPolar(localcenterInMap - lensWidthHalfVecInMap))));//レンズの上から下を引いたらbシータの角度差のはず
						const ureal lensWidthGenLength = 2. * lensballDesignParams::lensballParamInner.second * sin(lensWidthInTheta / 2.);//そいつの弦の長さ
						return lensWidthGenLength;


						//つぎに全反射角を決めたい
						const ureal rinkaiAngle = asin(1. / lensballDesignParams::nodelensEta);

						//頑張って解くと半径がこのように求まるらしい
						const ureal dammyRadius = lensWidthGenLength / (2. * sin(rinkaiAngle - lensWidthInTheta / 2.))*1.;
						return dammyRadius;
					}();

					//パラメータを登録
					nodelensParamsFrontBackRez[make_pair(rd, ld)].operator=(make_pair(sphereParam(localcenterInBalllocal, lensRadius), sphereParam(localcenterInBalllocal, lensRadius)));

					int a = 0;

					//要素レンズを描画していく
					if (drawNodelenses || drawNodelensEdges) {

						//つぎに極座標で要素レンズを計算する
						ResetPyVecSeries(nlensSeries);//ノードレンズ
						for (std::decay<decltype(lensballDesignParams::nodeLensResolution.second)>::type nlla = 0; nlla < lensballDesignParams::nodeLensResolution.second; nlla++) {

							ResetPyVecSeries(mlabSeries);//mlabSeriesはグリッドの一行を格納する
							for (std::decay<decltype(lensballDesignParams::nodeLensResolution.first)>::type nllo = 0; nllo < lensballDesignParams::nodeLensResolution.first; nllo++) {
								const uvec2 localpos(uleap(PairMinusPlus(pi), nllo / (ureal)(lensballDesignParams::nodeLensResolution.first - 1)),
									uleap(PairMinusPlus(pi / 2.), nlla / (ureal)(lensballDesignParams::nodeLensResolution.second - 1)));//要素レンズローカルでの極座標

								const uvec3 nodelensShape = (lensRadius * fabs(cos(localcenterInBalllocalPolar.y()))) * PolarToXyz(localpos);//これが球になるはず
								AppendPyVecSeries(mlabSeries, nodelensShape + localcenterInBalllocal);
							}
							if (drawNodelenses)py::s("nlx.append(mlabvx)\nnly.append(mlabvy)\nnlz.append(mlabvz)\n");//これでメッシュになると思うんやけど
						}

						if (drawNodelenses)py::sf("mlab.mesh(%s,color=(%f,%f,%f))", GetPySeriesForPlot(nlensSeries), color[0], color[1], color[2]);
						//頂点を転送して描く
						ResetPyVecSeries(mlabSeries);
						for (const auto& v : lensballDesignParams::hexverticesNodelensOuter) {
							const uvec2 designedVertex = (v + localcenterInMapDesigned);
							const uvec2 vertex = lensballDesignParams::DesignedMapToMap.prograte() * designedVertex;//傾けてマップ座標にする
							AppendPyVecSeries(pypltSeries, designedVertex);
							const auto polarpos = MapToLocalPolar(vertex);//つぎに極座標を得る
							AppendPyVecSeries(mlabSeries, PolarToXyz(polarpos));
						}

						if (drawNodelensEdges)py::sf("plt.plot(%s,color=(%f,%f,%f))", GetPySeriesForPlot(pypltSeries), color[0], color[1], color[2]);
						if (drawNodelensEdges)py::sf("mlab.plot3d(%s,color=(%f,%f,%f),tube_radius=0.01)", GetPySeriesForPlot(mlabSeries), color[0], color[1], color[2]);
					}
				}
			}
		};


		//スキャンをする
		constexpr bool scanLenses = true;//レンズボールに対するレイトレーシングを行う
		constexpr bool drawRefractionDirectionOfARay = false;//あるレイの屈折方向を描画する
		constexpr bool logWarningInScan = false;//scan中の警告を表示する
		if (scanLenses) {

			//並列処理用のいろいろ
			std::vector<uptr<std::thread>> scanThreads(scanParams::scanThreadNum);//実行スレッド
			std::vector<bool> finFlagOfEachScanThread(scanParams::scanThreadNum);//スキャンがおわったことを報告

			//格納結果の形式を示すヘッダを作って保存する
			const projRefraDicHeader storageheader(hardwareParams::projectorResInPhi, hardwareParams::projectorResInTheta, hardwareParams::numOfProjectionPerACycle);
			storageheader.SaveHeader(rezpath + branchpath + scanParams::resultDicPrefix);


			ResetPyVecSeries<6>(quiverSeries, quiverPrefix);//ベクトル場をお掃除
			ResetPyVecSeries(pypltSeries);//ベクトル場をお掃除

			//このスキャンの正確さ
			mutexedVariant<accuracyOfSearchNodelenses> searchAccuracyOfTheScanX;
			mutexedVariant<std::list<uvec3>> refractWays;

			//指定された結果を指定されたストレージへ送信する 転送したあとはRezはクリアされます
			const auto TransRezToStorage = [&](std::list<arrow3>& rezOfSt, ofstream& storageofStPtr) {
				cereal::BinaryOutputArchive o_archive(storageofStPtr);
				o_archive(rezOfSt);

				rezOfSt.clear();
			};
			//回転角度ごとにスキャンを行う
			const auto ScanAScene = [&](const std::decay<decltype(hardwareParams::numOfProjectionPerACycle)>::type rd, decltype(finFlagOfEachScanThread)::iterator finflagOfStIte) {

				const ureal ballRotation = GetRotationAngleFromRd(rd);//ボールの回転角度
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//グローバルからレンズボールローカルへの変換 XYZ座標

				//結果格納メモリとストレージを用意する
				std::list<arrow3> rezMem;
				std::ofstream storageStream(rezpath + branchpath + scanParams::resultDicPrefix + ".part" + to_string(rd), std::ios::binary);

				//要素レンズ検索の正確さインジゲータ(最初に当たりをつけた部分からどれだけ離れた位置を検索したか)
				accuracyOfSearchNodelenses searchAccuracyOfTheScene;

				//各ピクセルから飛び出るレイと回転角度rdの球との当たり判定を行う
				for (std::decay<decltype(hardwareParams::projectorResInTheta)>::type vpd = hardwareParams::projectorResInTheta/2; vpd < hardwareParams::projectorResInTheta; vpd++) {//プロジェクタの注目画素ごとに
					for (std::decay<decltype(hardwareParams::projectorResInPhi)>::type hpd = hardwareParams::projectorResInPhi/2; hpd < hardwareParams::projectorResInPhi; hpd++) {

						const uvec3 rayDirInGlobal = GetRayDirFromProjectorPix(ivec2(hpd, vpd));
						const arrow3 rayArrowInBalllocal(uvec3::Zero(), GlobalToBallLocal.prograte() * rayDirInGlobal);//ローカルで表したレイ

						//このレイがレンズボールに当たるか
						const auto hitRezVsSphere = IntersectSphere(rayArrowInBalllocal, lensballDesignParams::lensballParamInner.first, lensballDesignParams::lensballParamInner.second);//レイの大まかな着弾点を計算するSphereのどこに当たりますか
						if (!hitRezVsSphere.isHit)throw logic_error("プロジェクタから放出されているんだから絶対当たるはずなの");

						//要素レンズをサーチ
						const auto hitRezVsANodelens = SearchNodeLensHitByARayInBalllocalX(rayArrowInBalllocal, hitRezVsSphere, nodelensParamsFrontBackInBalllocal.value(), scanParams::searchAreaInARow, scanParams::searchAreaInALen, rayIncidentWay::toInner);//プロジェクターは内側!
						if (!hitRezVsANodelens) {
							//もし要素レンズがなければ計算する必要はない
							if (logWarningInScan)cout << "要素レンズなかったよ" << endl;
							rezMem.push_back(arrow3(uvec3::Zero(), uvec3::Zero()));
						}
						else {
							//正確さを更新しとく
							searchAccuracyOfTheScene.UpdateThisToWorth(hitRezVsANodelens.value().second);

							//焦点の場所は要素レンズが確定すれば計算できる
							const auto refractedInBalllocal = GetRefractedRayWithASphericalLensX(rayArrowInBalllocal, hitRezVsANodelens.value().first, logWarningInScan, rayIncidentWay::toInner);
							if (!refractedInBalllocal) {
								//全反射するなら計算する必要がない
								if (logWarningInScan)cout << "ふつうに屈折して出れなかったよ" << endl;
								rezMem.push_back(arrow3(uvec3::Zero(), uvec3::Zero()));
							}
							else {
								//これをグローバルに戻す
								const arrow3 refractRayDirInGlobal(GlobalToBallLocal.untiprograte() * refractedInBalllocal.value().org(), GlobalToBallLocal.untiprograte() * refractedInBalllocal.value().dir());

								//プロットしましょう
								//py::sf("mlab.quiver3d(%f,%f,%f,%f,%f,%f,mode=\"arrow\")", refractRayDirInGlobal.org().x(), refractRayDirInGlobal.org().y(), refractRayDirInGlobal.org().z(),refractRayDirInGlobal.dir().x(), refractRayDirInGlobal.dir().y(), refractRayDirInGlobal.dir().z());
								//cout << "a" << endl;
								PlotRayInMlab(refractRayDirInGlobal,"color = (0, 0, 1), tube_radius = 0.01");
								refractWays.GetAndLock()->push_back(refractRayDirInGlobal.dir());
								refractWays.unlock();

								//結果を追加
								rezMem.push_back(refractRayDirInGlobal);

								////プロットします　ヒットポイントに
								//if (vpd == 0 && drawRefractionDirectionOfARay) {
								//	const auto color = HsvToRgb({ rd / (ureal)(hardwareParams::numOfProjectionPerACycle - 1),1.,1. });
								//	py::sf("mlab.quiver3d(0,0,0,%f,%f,%f,mode=\"arrow\",color=(%f,%f,%f))", refractRayDirInGlobal.x(), refractRayDirInGlobal.y(), refractRayDirInGlobal.z(), color[0], color[1], color[2]);
								//	py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", hitLensCenterAndHitDistInMapDesigned.value().second.x(), hitLensCenterAndHitDistInMapDesigned.value().second.y(), color[0], color[1], color[2]);
								//}
							}
						}

						break;
					}
					break;
				}

				//スキャンが終わったらセーブ
				TransRezToStorage(rezMem, storageStream);
				//一応worthも更新
				searchAccuracyOfTheScanX.GetAndLock()->UpdateThisToWorth(searchAccuracyOfTheScene);
				searchAccuracyOfTheScanX.unlock();

				*finflagOfStIte = true;
			};

			//複数スレッドに回転角度を変えながら割り当てる
			for (std::decay<decltype(hardwareParams::numOfProjectionPerACycle)>::type rdgen = 0; rdgen < hardwareParams::numOfProjectionPerACycle; rdgen++) {
				std::cout << "count: " << rdgen << endl;
				//このrdgenでの処理を開いているスレッドに割り付けたい
				bool isfound = false;
				while (!isfound) {//割り付けられなければ繰り返す
					for (size_t th = 0; th < scanParams::scanThreadNum; th++)
						if (!scanThreads.at(th)) {//空きなら割付
							if (!isfound) {//一つのインデックスには一回だけ割り付ける
								isfound = true;
								finFlagOfEachScanThread.at(th) = false;//フラグをクリアして

								scanThreads.at(th).reset(new std::thread(ScanAScene, rdgen, finFlagOfEachScanThread.begin() + th));//スレッド実行開始
							}
						}
						else if (finFlagOfEachScanThread.at(th)) {//空いてなくて終わってるなら
							scanThreads.at(th).get()->join();
							scanThreads.at(th).release();//リソースを開放
						}
				}
			}

			//これ以上は結果は追加されない
			for (size_t th = 0; th < scanParams::scanThreadNum; th++) {

				//まだ開放されていなければjoinして開放する
				if (scanThreads.at(th)) {
					scanThreads.at(th).get()->join();
					scanThreads.at(th).release();
				}
			}

			//さいごに有効角度を計算する
			const auto& refptr = refractWays.GetAndLock();
			ureal maxangle = 0.;
			for(const auto& a:*refptr)
				for (const auto& b : *refptr) {
					//角度を計算
					const auto nowangle = fabs(acos(a.dot(b)));
					maxangle = max(maxangle, nowangle);
				}

			cout <<"MAX: " << maxangle*180./pi<<" deg" << endl;
		};





		//デベロップセクション
		constexpr bool developImage = false;
		constexpr bool printMessagesInDevelopping = developImage && false;//デベロップ中のメッセージを出力するか
		if (developImage) {
			projRefraDicHeader header;
			{
				ifstream ifs(rezpath + branchpath + developperParams::dicHeaderPathX, std::ios::binary);
				cereal::BinaryInputArchive iarch(ifs);

				iarch(header);

				std::cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
			}

			//解像度とかがわかる
			//つぎに視点ごとにレイトレースしてどの画素に当たるか調べたい

			//シーンの中でマルチスレッド化する
			std::vector<uptr<std::thread>> devThreads(developperParams::devThreadNum);
			std::vector<bool> finFlagOfEachDevThread(developperParams::devThreadNum);//スキャンがおわったことを報告
			
			//現像での要素レンズ検索の正確さインジゲータ(最初に当たりをつけた部分からどれだけ離れた位置を検索したか)
			mutexedVariant<accuracyOfSearchNodelenses> searchAccuracyOfTheDevelopping;

			//カメラの変換を変えながらレンダリングする
			std::vector<std::pair<ureal, ureal>> cameraTransAngleSets = {
				std::make_pair(0.,0.)/*, std::make_pair(15.,0.), std::make_pair(30.,0.), std::make_pair(15.,0.),
				std::make_pair(0.,0.), std::make_pair(-15.,0.), std::make_pair(-30.,0.), std::make_pair(-15.,0.),
				std::make_pair(0.,0.), std::make_pair(0., 15.), std::make_pair(0., 30.), std::make_pair(0., 15.),
				std::make_pair(0.,0.), std::make_pair(0., -15.), std::make_pair(0., -30.), std::make_pair(0., -15.) */};//Z軸中心の回転角度、Y軸中心の回転角度の順に[deg]で表記する
			for (size_t ctd = 0; ctd < cameraTransAngleSets.size();ctd++) {
				developperParams::cameraToGlobal = Eigen::Affine3d(Eigen::AngleAxis<ureal>(cameraTransAngleSets.at(ctd).second / 180. * pi, uvec3::UnitY()) * Eigen::AngleAxis<ureal>(cameraTransAngleSets.at(ctd).first / 180. * pi, uvec3::UnitZ()) * Eigen::Translation<ureal, 3>(uvec3(30., 0., 0.)));//カメラの変換をセットする
				
				//カメラを生成 アスペクト比は1で固定です　縦横の解像度に関わらず拡がり角は等しくなります
				std::list<arrow3>cameraRayList;
				for (size_t y = 0; y < developperParams::cameraResH; y++) {
					const ureal scy = uleap(PairMinusPlus(1.), y / (ureal)(developperParams::cameraResH - 1));
					for (size_t x = 0; x < developperParams::cameraResW; x++) {
						//スクリーンの位置は((2/res)*i+(1/res))-1 ｽｸﾘｰﾝサイズは多分2*2

						const ureal scx = uleap(PairMinusPlus(1.), x / (ureal)(developperParams::cameraResW - 1));
						double scz = 1. / tan(developperParams::fovHalf);//視野角を決める事ができる

						//orgが0 wayがスクリーンの正規化
						Eigen::Vector3d scnormed = Eigen::Vector3d(-scz, scx, scy).normalized();

						cameraRayList.push_back(arrow3(developperParams::cameraToGlobal * uvec3(0., 0., 0.), developperParams::cameraToGlobal.rotation() * scnormed));
						//py::sf("mlab.quiver3d(%f,%f,%f,%f,%f,%f)", cameraRayList.back().org().x(), cameraRayList.back().org().y(), cameraRayList.back().org().z(), cameraRayList.back().dir().x(), cameraRayList.back().dir().y(), cameraRayList.back().dir().z());
						//py::sf("mlab.plot3d([%f,%f],[%f,%f],[%f,%f],color=(1,0,0))", cameraRayList.back().org().x(), cameraRayList.back().dir().x()*30.+ cameraRayList.back().org().x(), cameraRayList.back().org().y(), cameraRayList.back().dir().y()*30.+ cameraRayList.back().org().y(),cameraRayList.back().org().z(), cameraRayList.back().dir().z()*30.+ cameraRayList.back().org().z());
					}
				}

				std::mutex colorListMutex;
				std::unordered_map<ivec2, uvec3> colorList;//カメラの受光素子ごとの色の合計
				std::unordered_map<ivec2, ureal> colorSiz;//受光素子に何シーン光が入射したか
				//あるシーンでのカメラ映像をけいさんする関数
				const auto GetAFrameOfAScene = [&](const size_t rdx, const decltype(finFlagOfEachDevThread)::iterator finflag) {
					//この計算での//要素レンズ検索の正確さインジゲータ(最初に当たりをつけた部分からどれだけ離れた位置を検索したか)
					accuracyOfSearchNodelenses searchAccuracyOfTheScene;

					////まずはフレームを読み出す
					const auto thisFrame = make_unique<bmpLib::img>();
					bmpLib::ReadBmp((rezpath + branchpath + developperParams::framePrefixX + to_string(rdx / developperParams::subStepRes) + ".bmp").c_str(), thisFrame.get());

					//つぎにローカルグローバル変換を計算する
					const ureal ballRotation = GetRotationAngleFromRd((ureal)rdx / (ureal)(developperParams::subStepRes));//ボールの回転角度
					const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//グローバルからレンズボールローカルへの変換 XYZ座標

					//あるレイが当たるピクセルの色を求める
					const auto GetColorOfCamPix = [&](const decltype(cameraRayList)::const_iterator cameraRayIte, const ivec2& pixOfCam) {//レイを特定してローカルを計算
						const arrow3 targetInBalllocal(GlobalToBallLocal.prograte() * (*cameraRayIte).org(), GlobalToBallLocal.prograte() * (*cameraRayIte).dir());
						//このレイがレンズボールに当たるか
						const auto hitRezVsSphere = IntersectSphere(targetInBalllocal, lensballDesignParams::lensballParamOuter.first, lensballDesignParams::lensballParamOuter.second);//レイの大まかな着弾点を計算するSphereのどこに当たりますか

						if (hitRezVsSphere.isHit) {

							//このレイが要素レンズに当たるか検索する　もちろんローカル座標系での話
							const auto hitlensParamInBalllocal = SearchNodeLensHitByARayInBalllocalX(targetInBalllocal, hitRezVsSphere, nodelensParamsFrontBackInBalllocal.value(), developperParams::searchAreaInARow, developperParams::searchAreaInALen, rayIncidentWay::toOuter);//カメラは外側!

							if (hitlensParamInBalllocal) {//要素レンズに当たったら
								//accuracyを更新
								searchAccuracyOfTheScene.UpdateThisToWorth(hitlensParamInBalllocal.value().second);

								//屈折
								const auto refractedRay = GetRefractedRayWithASphericalLensX(targetInBalllocal, hitlensParamInBalllocal.value().first, printMessagesInDevelopping, rayIncidentWay::toOuter);
								if (refractedRay) {
									const arrow3 refractedArrowInGlobal(GlobalToBallLocal.untiprograte() * refractedRay.value().org(), GlobalToBallLocal.untiprograte() * refractedRay.value().dir());//レイの向きを戻す

//つぎにプロジェクターのどの画素に当たるかを解く
//まず開口に当たるかい
if (!ThroughAperture(refractedArrowInGlobal)) {
	//プロジェクタには入社しなかった
	//if (rdx % (int)(developperParams::subStepRes*2.24) == 2)PlotRayInMlab(refractedArrowInGlobal,"color=(0,0,1), tube_radius=0.01");

	if (printMessagesInDevelopping)cout << "プロジェクタには入射しなかった" << endl;
	return (std::optional<uvec3>)(std::nullopt);//このシーンではだめだったので次のレイ
}

//開口にあたったらレイの向きで画素を判断できる
const auto pixpos = GetPixPosFromEnteredRay(refractedArrowInGlobal.dir());
//アパーチャに入ったら違う色で描画してやる
//PlotRayInMlab(refractedArrowInGlobal, "color=(1,0,1), tube_radius=0.01");

//無効な座標でなければリストに入れる
if (pixpos.x() >= 0 && pixpos.x() < hardwareParams::projectorResInPhi && pixpos.y() >= 0 && pixpos.y() < hardwareParams::projectorResInTheta) {
	const auto pixColor = thisFrame.get()->data.at(pixpos.y()).at(pixpos.x());//フレームから色を取り出す
	//cout << "rez: " << pixpos.x() << "\t" << pixpos.y() << "\t";
	return std::optional<uvec3>(uvec3(pixColor.r, pixColor.g, pixColor.b));
}
else {
	//cout << "invalid: " << pixpos.x() << "\t" << pixpos.y() << "\t";
}
								}
							}

						}

						//結果が特に挿入されることがなければ
						return (std::optional<uvec3>)(std::nullopt);
					};

					//カメラの色をフレームから読み出す
					auto cameraRayIte = cameraRayList.cbegin();
					for (size_t camY = 0; camY < developperParams::cameraResH; camY++)
						for (size_t camX = 0; camX < developperParams::cameraResW; camX++, cameraRayIte++) {
							const auto poskey = ivec2(camX, camY);//今のカメラ画素位置
							if (cameraRayIte.operator*().dir() == uvec3::Zero());// cout << "skip:" << endl;
							else {
								//色をゲットする
								const optional<uvec3> thiscolor = GetColorOfCamPix(cameraRayIte, poskey);//スレッド実行開始
								if (thiscolor) {

									//cout << "poskey: " << poskey.x() << "\t" << poskey.y() << endl;
									lock_guard guard(colorListMutex);

									const auto pixIte = colorList.find(poskey);
									if (pixIte == colorList.cend())colorList[poskey] = uvec3::Zero();
									colorList[poskey] += thiscolor.value() / developperParams::subStepRes;//各シーンの色を足し合わせてやればいい

									const auto sizIte = colorSiz.find(poskey);//何フレームでゲットできたかをゲット
									if (sizIte == colorSiz.cend())colorSiz[poskey] = 0;
									colorSiz[poskey] += 5.;
								}
								else
									int a = 0;
							}
						}

					searchAccuracyOfTheDevelopping.GetAndLock()->UpdateThisToWorth(searchAccuracyOfTheScene);
					searchAccuracyOfTheDevelopping.unlock();
					*finflag = true;
				};


				//ファイル読み込みの観点からシーンごとにやる
				size_t rd = 0;
				bool threadLoopFin = false;
				while (!threadLoopFin) {
					for (size_t th = 0; th < developperParams::devThreadNum; th++) {
						if (!devThreads.at(th)) {
							finFlagOfEachDevThread.at(th) = false;//リセットする　フラグを
							cout << "dev: " << rd / (ureal)developperParams::subStepRes << endl;
							devThreads.at(th).reset(new std::thread(GetAFrameOfAScene, rd, finFlagOfEachDevThread.begin() + th));//処理をセットする
							rd++;
							//処理終わりです
							if (rd >= header.rotationRes * developperParams::subStepRes) {
								threadLoopFin = true;
								break;
							}
						}
						else if (finFlagOfEachDevThread.at(th)) {//thが実行されていて、かつ終わっていたら
							devThreads.at(th)->join();
							devThreads.at(th).release();
						}
					}
				}
				for (size_t th = 0; th < developperParams::devThreadNum; th++) {
					if (devThreads.at(th)) {//thが実行されていたら
						devThreads.at(th)->join();
						devThreads.at(th).release();
					}
				}

				//映像をBMPとして書き出す
				WriteBmpOfCamera(colorList, colorSiz, StringFormat("rez%d", ctd));
			}

			//Accuracyでも表示してみる
			cout <<"Search accuracy In developping: "<< searchAccuracyOfTheDevelopping.GetAndLock()->str() << endl;
			searchAccuracyOfTheDevelopping.unlock();
		}




		//表示する 3d 2dの順
		py::s("mlab.show()");
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
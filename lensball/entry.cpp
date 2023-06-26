#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"
#include "bmpLib/bmp.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "SimVis/";//����branch�̌��ʂ��i�[����t�H���_

using py = pythonRuntime;

//�n�[�h�E�F�A�̃X�y�b�N
namespace hardwareParams {
	//�n�[�h�E�F�A�X�y�b�N
	constexpr size_t projectorFps = 22727;//�t���[�����[�g
	constexpr size_t numOfProjectionPerACycle = 1024;//������Ƃ̓��f��
	constexpr ureal rotationSpeed = projectorFps / (ureal)numOfProjectionPerACycle;//��]���x
	//�𑜓x
	constexpr size_t verticalDirectionResolution = sqrt_constexpr(numOfProjectionPerACycle);//�����𑜓x(����ɕ���ł��郌���Y�̐�)
	constexpr size_t horizontalDirectionResolution = numOfProjectionPerACycle / verticalDirectionResolution;//�����𑜓x�@�ʒu�����Y������̓��f��

	//�v���W�F�N�^�̃p�����[�^
	constexpr size_t projectorResInTheta = 768;//�v���W�F�N�^�̏c���𑜓x �z���g��XGA
	constexpr ureal projectorHalfAngleTheta = 60. / 180. * pi;//�v���W�F�N�g�̓��f�p
	constexpr size_t projectorResInPhi = 768; // �v���W�F�N�^�̉����𑜓x �z���g��XGA
	constexpr ureal projectorHalfAnglePhi = projectorHalfAngleTheta * (projectorResInPhi / (ureal)projectorResInTheta);//�v���W�F�N�g�̓��f�p

};
//�����Y�{�[���̃f�U�C���p�����[�^
namespace lensballDesignParams {

	constexpr ureal sphereRadius = 1.;//�����Y�{�[���̒��a
	const sphereParam lensballParam(uvec3::Zero(), sphereRadius);//�����Y�{�[���̃p�����[�^
	
	constexpr size_t lensNumInARow = hardwareParams::verticalDirectionResolution;//��s������̍s�̐�

	constexpr ureal rowAngle = 0.02706659 * 2.;//�s�̊p�x theta=atan(-1.5*1/(cos(theta)*sqrt(3))*1/lensnum)
	const ureal rowLength = 2. * pi * cos(rowAngle);//�s�̒���
	const ureal lensEdgeWidth = rowLength / (ureal)lensNumInARow / 2.;
	const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInARow;//�Z�p�`�̈�ς����V�t�g����
	const bitrans<Eigen::Rotation2D<ureal>> DesignedMapToMap = bitrans<Eigen::Rotation2D<ureal>>(Eigen::Rotation2D<ureal>(rowAngle));//�����Y�A���C���X�΂�����O����X�΂��������Ƃɂ���

	const ureal nodeLensRadius = 2. * lensEdgeWidth / sqrt(3.);//�v�f�����Y�`����쐬�@���̔��a
	const ureal nodeLensRadiusSq = pow(nodeLensRadius, 2);//�v�f�����Y���a�̓��
	const std::pair<size_t, size_t> nodeLensResolution = make_pair(20 * 2, 20);//�v�f�����Y�̕�����
	constexpr size_t rowNum = 15;//��ɂ��Ă� �s�̐�

	//�v�f�����Y�̊T�`�͘Z�p�`�ɂȂ�͂�
	const auto hexverticesNodelensOuter = [&] {
		auto hexvjunk = MakeHexagon(lensballDesignParams::lensEdgeWidth);//�Z�p�`�̒��_
		hexvjunk.push_back(hexvjunk.front());//������邽�߂ɍŏ��̓_�𖖔��ɑ}��

		return hexvjunk;
	}();


	constexpr ureal nodelensEta = 1.5;//�m�[�h�����Y�̔���ܗ�
};
//�������̃p�����[�^
namespace developperParams {
	const sphereParam apertureProjector(uvec3::Zero(), lensballDesignParams::sphereRadius / 40.);//�����ɂ���������v���W�F�N�^�[����o�����Ă���
	const auto regularHexagon = [&] {
		auto hexvjunk = MakeHexagon(sqrt(3.) / 2.);//�Z�p�`�̒��_
		hexvjunk.push_back(hexvjunk.front());//������邽�߂ɍŏ��̓_�𖖔��ɑ}��

		return hexvjunk;
	}();//�O�ډ~�̔��a��1�ɂȂ�悤�ȘZ�p�`
	constexpr size_t searchAreaInALen = 5;//�����s�̃����Y���ǂꂾ���[�ǂ����Č������邩
	constexpr size_t searchAreaInARow = 5;//����ǂꂾ���[�ǂ����Č������邩
	//�������t�����m�[�h�����Y����T������͈�
	//�܂��w�b�_��ǂݏo��
	const std::string framePath = R"(C:\local\user\lensball\lensball\resultsX\projectorFramesX768\)";
	const std::string dicHeaderPath = R"(C:\local\user\lensball\lensball\resultsX\dicX768\projRefRayMap.head)";//�����w�b�_�����܂��Ă���ꏊ
	constexpr ureal nodelensExpand = 1. + 1.e-4;//�m�[�h�����Y�̃M���M���ɓ��˂����Ƃ��ɔ���ł���悤�ɂ���g�嗦
	constexpr size_t devThreadNum = 16;//�������ǂꂾ���̃X���b�h�Ŏ��s���邩

	//�����Ɏg���J����
	constexpr ureal fovHalf = 3. / 180. * pi;
	const uvec3 cameraPos(uvec3(30, 0., 0.));//�J�����ʒu
	constexpr size_t cameraResW = 128, cameraResH = 128;
};
//�X�L�������̃p�����[�^
namespace scanParams {
	const ureal nodeLensFocalLength = lensballDesignParams::nodeLensRadius * 1.5;//�v�f�����Y�̒��S����œ_�܂ł̋���
	constexpr size_t scanThreadNum = 16;//�X�L�����Ɏg���X���b�h��
	const std::string resultDicPrefix = "projRefRayMap";
	constexpr size_t searchAreaInALen = 5;//�����s�̃����Y���ǂꂾ���[�ǂ����Č������邩
	constexpr size_t searchAreaInARow = 5;//����ǂꂾ���[�ǂ����Č������邩
}

//���锼����(ray)�Ɛ���(line)�̓����蔻��
bool IntersectLineAndWay(const std::pair<uvec2,uvec2>& line,const arrow2& ray) {
	const uvec2 dist = line.first - line.second;//���ꂪ�����̌X��

	umat2 expression;//�A��������������
	uvec2 ans;
	for (size_t a = 0; a < 2; a++) {
		expression.row(a)[0] = dist[a];
		expression.row(a)[1] = -ray.dir()[a];
		ans[a] = -line.second[a] + ray.org()[a];
	}
	if (expression.determinant() == 0.)throw runtime_error("���̎��͗n���Ȃ�!!");
	uvec2 trueans = expression.inverse()* ans;

	return trueans.x() >= 0. && trueans.x() <= 1. && trueans.y() >= 0.;//������t��0~1�Ȃ� ����������t�����Ȃ�
}
bool NaihouHanteiX(const uvec2& p, const std::list<uvec2>& vs) {
	constexpr ureal naihouDist = 0.001;

	auto ite = vs.cbegin();//���������O
	size_t xcount = 0, ycount = 0;
	for (size_t i = 0; i < 6; i++) {
		//����̒��_���`�F�b�N
		auto nite = std::next(ite);

		//���ꂩ��Ȃ�_
		const uvec2 ver = *ite;
		const uvec2 nver = *nite;

		//x�����ɐL�΂��܂�
		if (IntersectLineAndWay(make_pair(ver, nver), arrow2(p, uvec2::UnitX())))xcount++;
		if (IntersectLineAndWay(make_pair(ver, nver), arrow2(p, uvec2::UnitY())))ycount++;
		ite++;
	}
	//x/ycount�͂��ꂼ��̕����ɐ�����L�΂����Ƃ��Ɍ���������
	return xcount % 2 && ycount % 2;
}


//2d�x�N�g���ɗv�f��������
uvec3 ExtendUvec2(const uvec2& v, const ureal& z) {
	return uvec3(v.x(), v.y(), z);
}
//arrow��vec6�ɕς���
uvec6 ArrowToUVec6(const arrow<3>& v) {
	return uvec6(v.org().x(), v.org().y(), v.org().z(), v.dir().x(), v.dir().y(), v.dir().z());
}


//�v���W�F�N�^�[�������f�R�[�h����
void DeserializeProjRefraDic(const std::string& path) {
	//�w�b�_�����[�h����
	projRefraDicHeader header;
	{
		ifstream ifs(path + ".head", std::ios::binary);
		cereal::BinaryInputArchive iarch(ifs);

		iarch(header);

		std::cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
	}


	//�v���W�F�N�^�[�̐F���v�Z���邱�Ƃ��ł����@�e�t���[�����Ƃ�
	for (size_t sd = 0; sd < header.rotationRes; sd++) {
		//�V�[���̃��C���X�g���v�Z����
		std::list<arrow3> raylist;
		{
			ifstream ifs(path + ".part" + to_string(sd), std::ios::binary);
			if (!ifs)throw std::runtime_error("�t�@�C����ǂ߂Ȃ�");

			cereal::BinaryInputArchive iarch(ifs);
			iarch(raylist);
		}

		//��������ǂ��OK
		auto listite = raylist.cbegin();
		for(size_t hd=0;hd<header.horizontalRes;hd++,listite++)
			for (size_t vd = 0; vd < header.verticalRes; vd++) {
				const arrow3 refraction = (*listite);//���̃s�N�Z���ɑΉ����郌�C



			}

	}
}

//�p�x�𐳋K������
ureal NormalizeAngle(ureal Angle) {
	const int syuki = (int)(Angle / (2. * pi));//2pi�������܂܂�Ă��邩
	Angle -= syuki * (2. * pi);//�����+-2pi�ȉ��ɂ͂Ȃ����͂�
	if (fabs(Angle) <= pi)return Angle;
	//��Βl��pi�𒴂��Ă�����
	else return (2. * pi) + Angle;
}

//���낢��Ȃ��̂𐳋K������@������
template<typename intger>intger NormalizeIntger(const intger& i,const intger& siz) {
	//���Ȃ�mod����΂悵
	if (i > 0)return i % siz;
	if (i % siz==0)return 0;//siz�̐����{�Ȃ���0
	else return siz + (i % siz);//���Ȃ�S�̂�������Ηǂ�
}

//0~�n�܂�C���f�b�N�X���A���钆�S���痼���Ɍ�������悤�Ȍ`�ɕϊ�����
size_t GetBisideIndex(size_t lini,size_t center, int way,const size_t indSiz) {
	//�܂������C���f�b�N�X���v�Z����
	const size_t bilocalSiz = (lini+1)/2;//���S����̑��΃C���f�b�N�X�̑傫��
	//�������v�Z����
	const int sign = lini % 2 ? way : -way;

	//-max/2�܂ōs������
	int signedIndex = (sign * (int)bilocalSiz + center);
	//�}�C�i�X�Ȃ�max�𑫂�
	return NormalizeIntger<signed int>(signedIndex, indSiz);
}

ureal GetLatitudeInMapDFromRowIndex(size_t row) {
	return lensballDesignParams::eachRowsDistance * row - (lensballDesignParams::eachRowsDistance * (ureal)(lensballDesignParams::rowNum - 1) / 2.);
}
ureal GetLongitudeInMapDFromLensIndexAndRowFlag(size_t lensIdInARow, bool isOddRow) {
	return uleap(PairMinusPlus(lensballDesignParams::rowLength / 2.), lensIdInARow / (ureal)lensballDesignParams::lensNumInARow) + (isOddRow ? ((lensballDesignParams::rowLength) / (ureal)lensballDesignParams::lensNumInARow / 2.) : 0.);
}

//�摜��(w0,h0)��(-asp,-1)
//�v���W�F�N�^�̉�f������o����郌�C�̌��������Ƃ߂� uv���W�n�@������z�A����y���@+x�����ɕ��˂����@���_����
uvec3 GetRayDirFromProjectorPix(const ivec2& pix) {
	const auto aspect = hardwareParams::projectorResInPhi / (ureal)hardwareParams::projectorResInPhi;
	//���̓񎟌����W����\���ʏ�ł̍��W���킩��
	const uvec2 posInDisplay(uleap(PairMinusPlus(aspect), pix.x() / (ureal)(hardwareParams::projectorResInPhi - 1)), uleap(PairMinusPlus(1.), pix.y() / (ureal)(hardwareParams::projectorResInTheta - 1)));
	//�\���ʂ̈ʒu�����߂�
	const ureal dist = 1./tan(hardwareParams::projectorHalfAngleTheta);

	//���_��ʂ��Ă����ɂ��ǂ蒅�����C
	return uvec3(dist, posInDisplay.x(), posInDisplay.y()).normalized();
}
//���˕�������s�N�Z������肷��
ivec2 GetPixPosFromEnteredRay(const uvec3& enteredDir) {
	//�\���ʂ�x�ʒu
	const ureal dist = 1. / tan(hardwareParams::projectorHalfAngleTheta);

	//�A�p�[�`�����i��Ό��_��ʂ�ƍl�����邩������̎�������
	const ureal t = dist / enteredDir.x();
	const uvec3 crosspos = enteredDir * t;//�������\���ʂƂ̌���

	const uvec2 posInDisplay(crosspos.y(), crosspos.z());

	//�t�Z������Ƃ�����������
	const uvec2 realpos = 0.5 * (posInDisplay + uvec2(1., 1.));
	return  ivec2(round(realpos.x()* (ureal)(hardwareParams::projectorResInPhi - 1)), round(realpos.y()* (ureal)(hardwareParams::projectorResInTheta - 1)));
}

//�v�f�����Y����������
optional<std::pair<sphereParam, uvec3>> SearchNodeLensHitByARayInBalllocal(const arrow3& targetInBalllocal, const resultIntersecteSphere& hitRezVsSphere,const std::unordered_map<std::pair<size_t, size_t>, sphereParam, HashPair>& nodelensParamsInBalllocal,const size_t searchAreaInARow, const size_t searchAreaInALen) {

	//���C�ƃ����Y�{�[���̌����ʒu�������ȍ��W�n�Ōv�Z
	const auto& targetTVsBall = hitRezVsSphere.t;
	const uvec3 hitPosVsSphereInBalllocal = hitRezVsSphere.pos;
	const uvec2 hitposVsSphereInBalllocalPolar = XyzToPolar(hitPosVsSphereInBalllocal);
	const uvec2 hitposVsSphereInMap = PolarToMap(hitposVsSphereInBalllocalPolar);
	const uvec2 hitposVsSphereInMapDesigned = lensballDesignParams::DesignedMapToMap.untiprograte() * hitposVsSphereInMap;
	//if (printMessagesInDevelopping)py::sf("plt.scatter(%f,%f,color=(%f,0,0))", hitposVsSphereInMapD.x(), hitposVsSphereInMapD.y(), rd == 11 ? 1. : 0.);

	//�s�ɓ����������
	const ureal regRayDirLati = [&] {
		const ureal zerothRowlati = GetLatitudeInMapDFromRowIndex(0);//zero�Ԗڂ̍s�̍���
		const ureal zeroSetRayDir = hitposVsSphereInMapDesigned.y() - zerothRowlati;//0�Ԗڂ̍s�̍����Ɏn�_�����킹�����C�̍���
		const ureal finalRowlati = GetLatitudeInMapDFromRowIndex(lensballDesignParams::rowNum - 1);//rowNum-1�Ԗڂ̍s�̍���
		//�X�P�[�����O�@fin~0�܂ł̃X�P�[����rowNum-1~0�܂ł̃X�P�[���ɂȂ��ė~����
		const ureal thisscale = (ureal)(lensballDesignParams::rowNum - 1 - 0) / (finalRowlati - zerothRowlati);

		return thisscale * zeroSetRayDir;
	}();
	const int centerRawIndex = NormalizeIntger<signed int>(round(regRayDirLati), lensballDesignParams::rowNum);//�l�̌ܓ�����Ƃ����Ƃ��炵���C���f�b�N�X���킩��
	const int rawSearchWay = (regRayDirLati - (ureal)centerRawIndex) > 0. ? +1 : -1;//��������


	//�ł͂������烌���Y�ɓ����������
	ureal closestT = std::numeric_limits<ureal>::infinity();//�������������Y�̋���
	std::pair<size_t, size_t> hitlensIds;
	optional<std::pair<sphereParam, uvec3>> hitlensParamInBalllocal;//�Ώۂ̃����Y�p�����[�^�ƏՓ˖@���@�{�[�����[�J����

	for (size_t rilini = 0; rilini < searchAreaInARow; rilini++) {//�����͈͂͑S���̍s
		const size_t rid = GetBisideIndex(rilini, centerRawIndex, rawSearchWay, lensballDesignParams::rowNum);//������������Ƃ��납����ˏ�ɒT������

		//����phi���牽�Ԗڂ̃����Y�����l����
		const ureal regRayDirLonn = [&] {
			//const ureal tlonn = uleap(PairMinusPlus(rowLength/2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);

			const ureal zerothlenslonn = GetLongitudeInMapDFromLensIndexAndRowFlag(0, rid % 2);//-rowLength/2.���ŏ��̃����Y�̈ʒu�@�ꍇ�ɂ���đO�シ�邯�ǂ�
			const ureal zeroSetRayDir = hitposVsSphereInMapDesigned.x() - zerothlenslonn;//zero�Ԗڂ̃����Y�̏ꏊ��zero��
			const ureal finlenslonn = GetLongitudeInMapDFromLensIndexAndRowFlag(lensballDesignParams::lensNumInARow - 1, rid % 2);//�Ō�̃����Y�̏ꏊ
			//�X�P�[�����O�@fin~0�܂ł̃X�P�[����lensNumInCollum-1~0�܂ł̃X�P�[���ɂȂ��ė~����
			const ureal thisscale = (ureal)(lensballDesignParams::lensNumInARow - 1 - 0) / (finlenslonn - zerothlenslonn);

			return thisscale * zeroSetRayDir;
		}();
		const bool eachFlag = rid % 2;//���݂ɐ؂�ւ��t���O �����Ă���Ƃ��͍s�������i��ł�
		const int centerLensIndex = NormalizeIntger<signed int>(round(regRayDirLonn - (eachFlag ? 0.5 : 0.)), lensballDesignParams::lensNumInARow);//����̓I�t�Z�b�g���Ȃ��@�܂���}�C�i�X������n�܂��Ă���s�ɂ���ꍇ�̃C���f�b�N�X �����łȂ����-0.5���Ă���ۂ߂適���܂���
		const int lensSearchWay = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? +1 : -1;//�ׂ荇�������Y�̂����Ƃ��炵���C���f�b�N�X���킩��

		//�ł̓����Y�̓����蔻����n�߂�
		for (size_t lidlini = 0; lidlini < searchAreaInALen; lidlini++) {
			const size_t lid = GetBisideIndex(lidlini, centerLensIndex, lensSearchWay, lensballDesignParams::lensNumInARow);//������������Ƃ��납����ˏ�ɒT������

			const auto thislensparamInBalllocal = nodelensParamsInBalllocal.at(make_pair(rid, lid));//���ꂪ�����������Y�̃p�����[�^

			//���Ɠ����蔻�肷��
			const auto hitRezVsANode = IntersectSphere(targetInBalllocal, thislensparamInBalllocal.first, thislensparamInBalllocal.second);
			//�����Y�{�[���T�`������΋߂��ꏊ�̂͂�
			if (hitRezVsANode.isHit && hitRezVsANode.t < hitRezVsSphere.t) {
				//����ς��ԋ߂������Y�������Ă����
				if (hitRezVsANode.t < closestT) {
					hitlensParamInBalllocal = make_pair(thislensparamInBalllocal, hitRezVsANode.norm);
					closestT = hitRezVsANode.t;
				}
			}
		}

	}

	//�v�f�����Y���Ȃ��Ƃ���ɓ����������Ă���
	return hitlensParamInBalllocal;
}

//�X�l���̖@���Ō��̌o�H���v�Z
std::optional<arrow3> GetRefractedRayWithASphericalLens(const arrow3& targetInBalllocal, const std::pair<sphereParam, uvec3>& hitlensParamInBalllocal,const bool printMessagesInDevelopping) {
	ray3 targetSeriesInBalllocal(targetInBalllocal);
	const auto fIntersect = IntersectSphere(targetSeriesInBalllocal.back(), hitlensParamInBalllocal.first.first, hitlensParamInBalllocal.first.second);
	if (!fIntersect.isHit)throw logic_error("�����蔻��̃��W�b�N���o�O���Ă܂�");
	fIntersect.ApplyToRay(targetSeriesInBalllocal);
	if (!RefractSnell(targetSeriesInBalllocal, fIntersect.norm, lensballDesignParams::nodelensEta)) {
		if (printMessagesInDevelopping)cout << "�S���˂��N����(���ˎ�)" << endl;
		return std::optional<arrow3>();//���̃V�[���ł͂��߂������̂Ŏ��̃��C
	}

	//����ɑΖʂ̔��������
	const auto bIntersect = IntersectSphere(targetSeriesInBalllocal.back(), hitlensParamInBalllocal.first.first, hitlensParamInBalllocal.first.second);
	if (!bIntersect.isHit)throw logic_error("�����蔻��̃��W�b�N���o�O���Ă܂�");
	bIntersect.ApplyToRay(targetSeriesInBalllocal);
	if (!RefractSnell(targetSeriesInBalllocal, -bIntersect.norm, 1. / lensballDesignParams::nodelensEta)) {
		if (printMessagesInDevelopping)cout << "�S���˂��N����(�o�ˎ�)" << endl;
		return std::optional<arrow3>();//���̃V�[���ł͂��߂������̂Ŏ��̃��C
	}



	//����Ń����Y�{�[�����ł̕������킩����
	const auto& refractedArrow = targetSeriesInBalllocal.back();
	//ResetPyVecSeries(mlabSeries);
	//for (const auto& a : targetSeriesInBalllocal)
	//	AppendPyVecSeries(mlabSeries, a.org());
	//py::sf("mlab.plot3d(%s,tube_radius=0.01)", GetPySeriesForPlot(mlabSeries));

	return std::optional<arrow3>(refractedArrow);
}

//�J�����f����ۑ�����
void WriteBmpOfCamera(const std::unordered_map<ivec2, uvec3>& colorList,const std::unordered_map<ivec2, size_t>& colorSiz) {
	bmpLib::img picture;//�J��������̉f��
	picture.width = developperParams::cameraResW;
	picture.data.resize(developperParams::cameraResH);
	picture.height = developperParams::cameraResH;
	bmpLib::img maskPic;//�ǂ��ɒl�����݂��邩
	maskPic.width = developperParams::cameraResW;
	maskPic.data.resize(developperParams::cameraResH);
	maskPic.height = developperParams::cameraResH;
	for (int y = 0; y < picture.height; y++) {
		picture.data.at(picture.height - 1 - y).resize(developperParams::cameraResW);
		maskPic.data.at(picture.height - 1 - y).resize(developperParams::cameraResW);
		for (int x = 0; x < picture.width; x++) {
			const auto pixIte = colorList.find(ivec2(x, y));
			const auto sizeIte = colorSiz.find(ivec2(x, y));//�Ή������s�N�Z����ݒu
			if (pixIte != colorList.cend()) {//�����ƐF�������
				picture.data[picture.height - 1 - y][x] = bmpLib::color(pixIte->second.x(), pixIte->second.y(), pixIte->second.z());//�����������C��������Ă���͈͂���������F�t��
				maskPic.data[picture.height - 1 - y][x] = bmpLib::color(0, clamp<int>(sizeIte->second * 50, 0, 255), 0);
			}
			else {
				picture.data[picture.height - 1 - y][x] = bmpLib::color(0, 0, 0);
				maskPic.data[picture.height - 1 - y][x] = bmpLib::color(0, 0, 0);
			}
		}
	}
	bmpLib::WriteBmp((rezpath + branchpath + "pic.bmp").c_str(), &picture);
	bmpLib::WriteBmp((rezpath + branchpath + "mask.bmp").c_str(), &maskPic);
}


int main() {

	try {
		//�J�n�������L�^���ăX�^�[�g
		const auto startTimePoint = std::chrono::system_clock::now();
		std::cout << "Start: " << startTimePoint << endl;

		//python plt�p�̃x�N�g���n��
		const pyVecSeries<3> mlabSeries("mlabv");//mlab�v���b�g�p�Ɏg��3D�x�N�g���z��
		const pyVecSeries<3> nlensSeries("nl");//�v�f�����Y�̃O���b�h
		const pyVecSeries<2> pypltSeries("pypltv");//�񎟌��v���b�g�p
		const pyVecSeries<6> quiverSeries("mlabquiver");//�x�N�g����p
		const std::array<const string, 6> quiverPrefix = { "x","y","z","a","b","c" };


		//Python���Z�b�g�A�b�v���Ă��烌���Y�{�[���̊T�`������
		constexpr bool drawSphere = false;//�����Y�{�[���T�`��`�悷��
		{

			//python�����^�C�����������Ă��낢�돉������
			py::Init();

			py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

			//mayavi�̐ݒ�
			const std::pair<size_t, size_t> figResolution(800, 600);
			py::sf("fig = mlab.figure(\'Refract dir In Global\', size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

			//matplotlib�̐ݒ�
			py::s("fig, ax = plt.subplots()\nax.set_aspect(\"equal\")\nplt.title('Lens dist In MapD')");

			//����`�悷��
			constexpr size_t sphereResolution = 20;
			if (drawSphere)py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
spx = np.cos(sphphi)*np.sin(sphtheta)
spy = np.sin(sphphi)*np.sin(sphtheta)
spz = np.cos(sphtheta)
mlab.mesh(%f*spx, %f*spy, %f*spz ,color=(1.,1.,1.) )  
)", sphereResolution, sphereResolution, lensballDesignParams::sphereRadius, lensballDesignParams::sphereRadius, lensballDesignParams::sphereRadius);
		};








		//�����Y�A���C���쐬�A�`�悷��
		//�Z�p�`�Ń^�C�����O����@�����s�������Ă����s���������Ċ���
		constexpr bool calcNodelenses = true;//�m�[�h�����Y�̈ʒu���v�Z���ă����Y�{�[�����`������
		constexpr bool drawNodelenses = calcNodelenses & false;//�v�f�����Y��`�悷��
		constexpr bool drawNodelensEdges = calcNodelenses & false;//�m�[�h�����Y�̘g����`�悷��

		//���̌v�Z�ŗv�f�����Y���X�g���킩���
		std::optional<std::unordered_map<std::pair<size_t, size_t>, sphereParam, HashPair>> nodelensParamsInBalllocal = std::nullopt;//�s�ԍ��@�����Y�ԍ����L�[�Ńp�����[�^��Balllocal�ŕۑ�����
		if(calcNodelenses){
			//opt�v�Z�ł��܂�
			nodelensParamsInBalllocal = decltype(nodelensParamsInBalllocal)::value_type();
			auto& nodelensParamsRez = nodelensParamsInBalllocal.value();

			//std::list<uleap>//�}�b�v���W�ł̃����Y���S
			for (std::decay<decltype(lensballDesignParams::rowNum)>::type rd = 0; rd < lensballDesignParams::rowNum; rd++) {
				const ureal tlati = GetLatitudeInMapDFromRowIndex(rd);//lati�����̌��݈ʒu
				const bool eachFlag = rd % 2;//���݂ɐ؂�ւ��t���O
				for (std::decay<decltype(lensballDesignParams::lensNumInARow)>::type ld = 0; ld < lensballDesignParams::lensNumInARow; ld++) {

					auto color = HsvToRgb({ uleap({0.,1.},ld / (ureal)lensballDesignParams::lensNumInARow),1.,1. });//�F���s�����ɕς���
					//�Z�p�`�����߂�o�b�t�@���N���A
					ResetPyVecSeries(pypltSeries);

					//lonn�����̌��݈ʒu
					const ureal tlonn = GetLongitudeInMapDFromLensIndexAndRowFlag(ld, eachFlag);
					const auto localcenterInMapDesigned = uvec2(tlonn, tlati);//�v�f�����Y�̒��� ���[�J���}�b�v���W
					const auto localcenterInMap = lensballDesignParams::DesignedMapToMap.prograte() * localcenterInMapDesigned;
					const auto localcenterInBalllocalPolar = MapToLocalPolar(localcenterInMap);
					const uvec3 localcenterInBalllocal = PolarToXyz(localcenterInBalllocalPolar);

					//�p�����[�^��o�^
					nodelensParamsRez[make_pair(rd, ld)] = sphereParam(localcenterInBalllocal, 1.1* lensballDesignParams::nodeLensRadius*fabs(cos(localcenterInBalllocalPolar.y())));

					//�v�f�����Y��`�悵�Ă���
					if (drawNodelenses || drawNodelensEdges) {

						//���ɋɍ��W�ŗv�f�����Y���v�Z����
						ResetPyVecSeries(nlensSeries);//�m�[�h�����Y
						for (std::decay<decltype(lensballDesignParams::nodeLensResolution.second)>::type nlla = 0; nlla < lensballDesignParams::nodeLensResolution.second; nlla++) {

							ResetPyVecSeries(mlabSeries);//mlabSeries�̓O���b�h�̈�s���i�[����
							for (std::decay<decltype(lensballDesignParams::nodeLensResolution.first)>::type nllo = 0; nllo < lensballDesignParams::nodeLensResolution.first; nllo++) {
								const uvec2 localpos(uleap(PairMinusPlus(pi), nllo / (ureal)(lensballDesignParams::nodeLensResolution.first - 1)),
									uleap(PairMinusPlus(pi / 2.), nlla / (ureal)(lensballDesignParams::nodeLensResolution.second - 1)));//�v�f�����Y���[�J���ł̋ɍ��W

								const uvec3 nodelensShape = (lensballDesignParams::nodeLensRadius * fabs(cos(localcenterInBalllocalPolar.y()))) * PolarToXyz(localpos);//���ꂪ���ɂȂ�͂�
								AppendPyVecSeries(mlabSeries, nodelensShape + localcenterInBalllocal);
							}
							if (drawNodelenses)py::s("nlx.append(mlabvx)\nnly.append(mlabvy)\nnlz.append(mlabvz)\n");//����Ń��b�V���ɂȂ�Ǝv����₯��
						}

						if (drawNodelenses)py::sf("mlab.mesh(%s,color=(%f,%f,%f))", GetPySeriesForPlot(nlensSeries), color[0], color[1], color[2]);
						//���_��]�����ĕ`��
						ResetPyVecSeries(mlabSeries);
						for (const auto& v : lensballDesignParams::hexverticesNodelensOuter) {
							const uvec2 designedVertex = (v + localcenterInMapDesigned);
							const uvec2 vertex = lensballDesignParams::DesignedMapToMap.prograte() * designedVertex;//�X���ă}�b�v���W�ɂ���
							AppendPyVecSeries(pypltSeries, designedVertex);
							const auto polarpos = MapToLocalPolar(vertex);//���ɋɍ��W�𓾂�
							AppendPyVecSeries(mlabSeries, PolarToXyz(polarpos));
						}

						if (drawNodelensEdges)py::sf("plt.plot(%s,color=(%f,%f,%f))", GetPySeriesForPlot(pypltSeries), color[0], color[1], color[2]);
						if (drawNodelensEdges)py::sf("mlab.plot3d(%s,color=(%f,%f,%f),tube_radius=0.01)", GetPySeriesForPlot(mlabSeries), color[0], color[1], color[2]);
					}
				}
			}
		};







		//�f�x���b�v�Z�N�V����
		constexpr bool developImage = true;
		constexpr bool printMessagesInDevelopping = developImage && false;//�f�x���b�v���̃��b�Z�[�W���o�͂��邩
		if (developImage) {
			projRefraDicHeader header;
			{
				ifstream ifs(developperParams::dicHeaderPath, std::ios::binary);
				cereal::BinaryInputArchive iarch(ifs);

				iarch(header);

				std::cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
			}

			//�J�����𐶐�
			std::list<arrow3>cameraRayList;
			for (size_t y = 0; y < developperParams::cameraResH; y++) {
				const ureal scy = uleap(PairMinusPlus(1.), y / (ureal)(developperParams::cameraResH - 1));
				for (size_t x = 0; x < developperParams::cameraResW; x++) {
					//�X�N���[���̈ʒu��((2/res)*i+(1/res))-1 ��ذ݃T�C�Y�͑���2*2

					const ureal scx = uleap(PairMinusPlus(1. * (developperParams::cameraResW / developperParams::cameraResH)), x / (ureal)(developperParams::cameraResW - 1));
					double scz = 1. / tan(developperParams::fovHalf);//����p�����߂鎖���ł���

					//org��0 way���X�N���[���̐��K��
					Eigen::Vector3d scnormed = Eigen::Vector3d(-scz, scy, scx).normalized();

					cameraRayList.push_back(arrow3(developperParams::cameraPos, scnormed));
					//py::sf("mlab.quiver3d(%f,%f,%f,%f,%f,%f)", cameraPos.x(), cameraPos.y(), cameraPos.z(), scnormed.x(), scnormed.y(), scnormed.z());

				}
			}


			//�𑜓x�Ƃ����킩��
			//���Ɏ��_���ƂɃ��C�g���[�X���Ăǂ̉�f�ɓ����邩���ׂ���
			std::mutex colorListMutex;
			std::unordered_map<ivec2, uvec3> colorList;//�J�����̎���f�q���Ƃ̐F�̍��v
			std::unordered_map<ivec2, size_t> colorSiz;//����f�q�ɉ��V�[���������˂�����

			//�V�[���̒��Ń}���`�X���b�h������
			std::array<uptr<std::thread>, developperParams::devThreadNum> devThreads;
			std::array<bool, developperParams::devThreadNum> finFlagOfEachDevThread;//�X�L����������������Ƃ��
			//����V�[���ł̃J�����f�����������񂷂�
			const auto GetAFrameOfAScene = [&](const size_t rd,const decltype(finFlagOfEachDevThread)::iterator finflag) {
				////�܂��̓t���[����ǂݏo��
				const auto thisFrame = make_unique<bmpLib::img>();
				bmpLib::ReadBmp((developperParams::framePath + "frame" + to_string(rd) + ".bmp").c_str(), thisFrame.get());
				
				//���Ƀ��[�J���O���[�o���ϊ����v�Z����
				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(header.rotationRes)) + (2. * pi / (ureal)(header.rotationRes + 1) / 2.);//�{�[���̉�]�p�x
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//�O���[�o�����烌���Y�{�[�����[�J���ւ̕ϊ� XYZ���W

				//���郌�C��������s�N�Z���̐F�����߂�
				const auto GetColorOfCamPix = [&](const decltype(cameraRayList)::const_iterator cameraRayIte, const ivec2& pixOfCam) {//���C����肵�ă��[�J�����v�Z
					const arrow3 targetInBalllocal(GlobalToBallLocal.prograte() * (*cameraRayIte).org(), GlobalToBallLocal.prograte() * (*cameraRayIte).dir());
					//���̃��C�������Y�{�[���ɓ����邩
					const auto hitRezVsSphere = IntersectSphere(targetInBalllocal, lensballDesignParams::lensballParam.first, lensballDesignParams::lensballParam.second);//���C�̑�܂��Ȓ��e�_���v�Z����Sphere�̂ǂ��ɓ�����܂���

					if (hitRezVsSphere.isHit) {
						
						//���̃��C���v�f�����Y�ɓ����邩��������@������񃍁[�J�����W�n�ł̘b
						const auto hitlensParamInBalllocal = SearchNodeLensHitByARayInBalllocal(targetInBalllocal, hitRezVsSphere, nodelensParamsInBalllocal.value(), developperParams::searchAreaInARow, developperParams::searchAreaInALen);

						if (hitlensParamInBalllocal) {//�v�f�����Y�ɓ���������
							//����
							const auto refractedRay = GetRefractedRayWithASphericalLens(targetInBalllocal, hitlensParamInBalllocal.value(), printMessagesInDevelopping);
							if (refractedRay) {
								//���Ƀv���W�F�N�^�[�̂ǂ̉�f�ɓ����邩������
								//�܂��J���ɓ����邩��
								auto apertureT = IntersectSphere(refractedRay.value(), developperParams::apertureProjector.first, developperParams::apertureProjector.second);
								if (!apertureT.isHit) {
									//�v���W�F�N�^�ɂ͓��Ђ��Ȃ�����
									if (printMessagesInDevelopping)cout << "�v���W�F�N�^�ɂ͓��˂��Ȃ�����" << endl;
									return std::optional<uvec3>();//���̃V�[���ł͂��߂������̂Ŏ��̃��C
								}

								//�J���ɂ��������烌�C�̌����ŉ�f�𔻒f�ł���
								const uvec3 refractedDirInGlobal = GlobalToBallLocal.untiprograte() * refractedRay.value().dir();
								
								const auto pixpos = GetPixPosFromEnteredRay(refractedDirInGlobal);

								//�����ȍ��W�łȂ���΃��X�g�ɓ����
								if (pixpos.x() >= 0 && pixpos.x() < hardwareParams::projectorResInPhi && pixpos.y() >= 0 && pixpos.y() < hardwareParams::projectorResInTheta) {
									const auto pixColor = thisFrame.get()->data.at(pixpos.y()).at(pixpos.x());//�t���[������F�����o��
									return std::optional<uvec3>(uvec3(pixColor.r, pixColor.g, pixColor.b));
								}
							}
						}
					}
					else {
						return std::optional<uvec3>();//�����Y�{�[���ɓ�����Ȃ������炠��܈Ӗ��Ȃ� ���̃��C�ɍs��
					}
				};

				//�J�����̐F���t���[������ǂݏo��
				auto cameraRayIte = cameraRayList.cbegin();
				for (size_t camY = 0; camY < developperParams::cameraResH;camY++)
					for (size_t camX = 0; camX < developperParams::cameraResW; camX++,cameraRayIte++){
						const auto poskey = ivec2(camX, camY);//���̃J������f�ʒu
						//�F���Q�b�g����
						const auto thiscolor = GetColorOfCamPix(cameraRayIte, poskey);//�X���b�h���s�J�n
							if (thiscolor) {
								lock_guard guard(colorListMutex);

								const auto pixIte = colorList.find(poskey);
								if (pixIte == colorList.cend())colorList[poskey] = uvec3::Zero();
								colorList[poskey] = thiscolor.value();//�e�V�[���̐F�𑫂����킹�Ă��΂���

								const auto sizIte = colorSiz.find(poskey);//���t���[���ŃQ�b�g�ł��������Q�b�g
								if (sizIte == colorSiz.cend())colorSiz[poskey] = 0;
								colorSiz[poskey]++;
						}
					}

				*finflag = true;
			};

			//�t�@�C���ǂݍ��݂̊ϓ_����V�[�����Ƃɂ��
			size_t rd = 0;
			bool threadLoopFin = false;
			while(!threadLoopFin) {
				for (size_t th = 0; th < developperParams::devThreadNum; th++) {
					if (!devThreads.at(th)) {
						finFlagOfEachDevThread.at(th) = false;//���Z�b�g����@�t���O��
						devThreads.at(th).reset(new std::thread(GetAFrameOfAScene, rd, finFlagOfEachDevThread.begin() + th));//�������Z�b�g����
						rd++;
						//�����I���ł�
						if (rd >= header.rotationRes) {
							threadLoopFin = true;
							break;
						}
					}
					else if(finFlagOfEachDevThread.at(th)) {//th�����s����Ă��āA���I����Ă�����
						devThreads.at(th)->join();
						devThreads.at(th).release();
					}
				}
			}
			for (size_t th = 0; th < developperParams::devThreadNum; th++) {
				if(devThreads.at(th)) {//th�����s����Ă�����
					devThreads.at(th)->join();
					devThreads.at(th).release();
				}
			}

			//�f����BMP�Ƃ��ď����o��
			WriteBmpOfCamera(colorList, colorSiz);
		}







		//�X�L����������
		constexpr bool scanLenses = false;//�����Y�{�[���ɑ΂��郌�C�g���[�V���O���s��
		constexpr bool drawRefractionDirectionOfARay = false;//���郌�C�̋��ܕ�����`�悷��
		constexpr bool logWarningInScan = false;//scan���̌x����\������
		if(scanLenses){

			//���񏈗��p�̂��낢��
			std::array<uptr<std::thread>, scanParams::scanThreadNum> scanThreads;//���s�X���b�h
			std::array<bool, scanParams::scanThreadNum> finFlagOfEachScanThread;//�X�L����������������Ƃ��

			//�i�[���ʂ̌`���������w�b�_������ĕۑ�����
			const projRefraDicHeader storageheader(hardwareParams::projectorResInPhi, hardwareParams::projectorResInTheta, hardwareParams::numOfProjectionPerACycle);
			storageheader.SaveHeader(rezpath + branchpath + scanParams::resultDicPrefix);

			ResetPyVecSeries<6>(quiverSeries,quiverPrefix);//�x�N�g��������|��
			ResetPyVecSeries(pypltSeries);//�x�N�g��������|��


			//�w�肳�ꂽ���ʂ��w�肳�ꂽ�X�g���[�W�֑��M���� �]���������Ƃ�Rez�̓N���A����܂�
			const auto TransRezToStorage = [&](std::list<arrow3>& rezOfSt, ofstream& storageofStPtr) {
				cereal::BinaryOutputArchive o_archive(storageofStPtr);
				o_archive(rezOfSt);

				rezOfSt.clear();
			};
			//��]�p�x���ƂɃX�L�������s��
			const auto ScanAScene = [&](const std::decay<decltype(hardwareParams::numOfProjectionPerACycle)>::type rd, decltype(finFlagOfEachScanThread)::iterator finflagOfStIte) {

				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(hardwareParams::numOfProjectionPerACycle)) + (2. * pi / (ureal)(hardwareParams::numOfProjectionPerACycle + 1) / 2.);//�{�[���̉�]�p�x
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//�O���[�o�����烌���Y�{�[�����[�J���ւ̕ϊ� XYZ���W

				//���ʊi�[�������ƃX�g���[�W��p�ӂ���
				std::list<arrow3> rezMem;
				std::ofstream storageStream(rezpath + branchpath + scanParams::resultDicPrefix + ".part" + to_string(rd), std::ios::binary);

				//�e�s�N�Z�������яo�郌�C�Ɖ�]�p�xrd�̋��Ƃ̓����蔻����s��
				for (std::decay<decltype(hardwareParams::projectorResInTheta)>::type vpd = 0; vpd < hardwareParams::projectorResInTheta; vpd++) {//�v���W�F�N�^�̒��ډ�f���Ƃ�
					for (std::decay<decltype(hardwareParams::projectorResInPhi)>::type hpd = 0; hpd < hardwareParams::projectorResInPhi; hpd++) {
						
						const uvec3 rayDirInGlobal = GetRayDirFromProjectorPix(ivec2(hpd, vpd));
						const arrow3 rayArrowInBalllocal(uvec3::Zero(), GlobalToBallLocal.prograte()* rayDirInGlobal);//���[�J���ŕ\�������C

						//���̃��C�������Y�{�[���ɓ����邩
						const auto hitRezVsSphere = IntersectSphere(rayArrowInBalllocal, lensballDesignParams::lensballParam.first, lensballDesignParams::lensballParam.second);//���C�̑�܂��Ȓ��e�_���v�Z����Sphere�̂ǂ��ɓ�����܂���
						if (!hitRezVsSphere.isHit)throw logic_error("�v���W�F�N�^������o����Ă���񂾂����Γ�����͂��Ȃ�");

						//�v�f�����Y���T�[�`
						const auto hitRezVsANodelens = SearchNodeLensHitByARayInBalllocal(rayArrowInBalllocal, hitRezVsSphere, nodelensParamsInBalllocal.value(), scanParams::searchAreaInARow, scanParams::searchAreaInALen);
						if (!hitRezVsANodelens) {
							//�����v�f�����Y���Ȃ���Όv�Z����K�v�͂Ȃ�
							if(logWarningInScan)cout << "�v�f�����Y�Ȃ�������" << endl;
							rezMem.push_back(arrow3(uvec3::Zero(),uvec3::Zero()));
						}
						else {

							//�œ_�̏ꏊ�͗v�f�����Y���m�肷��Όv�Z�ł���
							const auto refractedInBalllocal = GetRefractedRayWithASphericalLens(rayArrowInBalllocal, hitRezVsANodelens.value(), logWarningInScan);
							if (!refractedInBalllocal) {
								//�S���˂���Ȃ�v�Z����K�v���Ȃ�
								if(logWarningInScan)cout << "�ӂ��ɋ��܂��ďo��Ȃ�������" << endl;
								rezMem.push_back(arrow3(uvec3::Zero(), uvec3::Zero()));
							}
							else {
								//������O���[�o���ɖ߂�
								const arrow3 refractRayDirInGlobal(GlobalToBallLocal.untiprograte() * refractedInBalllocal.value().org(), GlobalToBallLocal.untiprograte() * refractedInBalllocal.value().dir());

								//���ʂ�ǉ�
								rezMem.push_back(refractRayDirInGlobal);

								////�v���b�g���܂��@�q�b�g�|�C���g��
								//if (vpd == 0 && drawRefractionDirectionOfARay) {
								//	const auto color = HsvToRgb({ rd / (ureal)(hardwareParams::numOfProjectionPerACycle - 1),1.,1. });
								//	py::sf("mlab.quiver3d(0,0,0,%f,%f,%f,mode=\"arrow\",color=(%f,%f,%f))", refractRayDirInGlobal.x(), refractRayDirInGlobal.y(), refractRayDirInGlobal.z(), color[0], color[1], color[2]);
								//	py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", hitLensCenterAndHitDistInMapDesigned.value().second.x(), hitLensCenterAndHitDistInMapDesigned.value().second.y(), color[0], color[1], color[2]);
								//}
							}
						}
					}
				}

				//�X�L�������I�������Z�[�u
				TransRezToStorage(rezMem, storageStream);

				*finflagOfStIte = true;
			};

			//�����X���b�h�ɉ�]�p�x��ς��Ȃ��犄�蓖�Ă�
			for (std::decay<decltype(hardwareParams::numOfProjectionPerACycle)>::type rdgen = 0; rdgen < hardwareParams::numOfProjectionPerACycle; rdgen++) {
				std::cout << "count: " << rdgen << endl;
				//����rdgen�ł̏������J���Ă���X���b�h�Ɋ���t������
				bool isfound = false;
				while (!isfound) {//����t�����Ȃ���ΌJ��Ԃ�
					for (size_t th=0;th< scanParams::scanThreadNum;th++)
						if (!scanThreads.at(th)) {//�󂫂Ȃ犄�t
							if (!isfound) {//��̃C���f�b�N�X�ɂ͈�񂾂�����t����
								isfound = true;
								finFlagOfEachScanThread.at(th) = false;//�t���O���N���A����

								scanThreads.at(th).reset(new std::thread(ScanAScene, rdgen,  finFlagOfEachScanThread.begin() + th));//�X���b�h���s�J�n
							}
						}
						else if (finFlagOfEachScanThread.at(th)) {//�󂢂ĂȂ��ďI����Ă�Ȃ�
							scanThreads.at(th).get()->join();
							scanThreads.at(th).release();//���\�[�X���J��
						}
				}
			}

			//����ȏ�͌��ʂ͒ǉ�����Ȃ�
			for (size_t th = 0; th < scanParams::scanThreadNum; th++) {

				//�܂��J������Ă��Ȃ����join���ĊJ������
				if (scanThreads.at(th)) {
					scanThreads.at(th).get()->join();
					scanThreads.at(th).release();
				}
			}
		};






		//�\������ 3d 2d�̏�
		//py::s("mlab.show()");
		//py::s("plt.show()");

		const auto endTimePoint = std::chrono::system_clock::now();
		std::cout << "Finish: " << endTimePoint << endl;
		std::cout << "Process time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTimePoint - startTimePoint).count() / 1000. << " [s]" << endl;
		std::cout << "The work is complete...Wait rendering by Python" << endl;
		py::Terminate();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << endl;
		std::cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//�Ȃ񂩓��ꂽ��I���

		py::Terminate();
		return -1;
	}
	catch (...) {
		std::cout << "unknown err" << endl;
		std::cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//�Ȃ񂩓��ꂽ��I���

		py::Terminate();
		return -2;
	}

	return 0;
}
#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"
#include "bmpLib/bmp.hpp"
#include <random>

using namespace std;

const std::string appContextPath(R"(./lensball.context)");//�R���e�L�X�g�u����@�����Ȃ���
constexpr size_t appContextVersion=1;//�R���e�L�X�g�̃o�[�W�����@���e���ς�����炱����X�V���Ă�������

std::string rezpath = "./";//���ʂ��i�[����t�H���_(�R���e�L�X�g�Ɏx�z����܂�)
const std::string branchpath = "ExLens/";//����branch�̌��ʂ��i�[����t�H���_

using py = pythonRuntime;

std::pair<ureal, ureal> GetNodelensRadiusInDobuleface(const uvec2& localcenterInMap);

std::mt19937 dice([&] {std::random_device rand; return rand(); }());//���������̗��������C

//�n�[�h�E�F�A�̃X�y�b�N
namespace hardwareParams {
	//�n�[�h�E�F�A�X�y�b�N
	constexpr size_t projectorFps = 22727;//�t���[�����[�g
	constexpr size_t numOfProjectionPerACycle = 1024;//������Ƃ̓��f��
	constexpr ureal rotationSpeed = projectorFps / (ureal)numOfProjectionPerACycle;//��]���x
	//�𑜓x
	constexpr size_t verticalDirectionResolution = sqrt_constexpr((ureal)numOfProjectionPerACycle);//�����𑜓x(����ɕ���ł��郌���Y�̐�)
	constexpr size_t horizontalDirectionResolution = numOfProjectionPerACycle / verticalDirectionResolution;//�����𑜓x�@�ʒu�����Y������̓��f��

	//�v���W�F�N�^�̃p�����[�^
	constexpr size_t projectorResInTheta = 768;//�v���W�F�N�^�̏c���𑜓x �z���g��XGA
	constexpr ureal projectorHalfAngleTheta = 60. / 180. * pi;//�v���W�F�N�g�̓��f�p
	constexpr size_t projectorResInPhi = 768; // �v���W�F�N�^�̉����𑜓x �z���g��XGA
	constexpr ureal projectorHalfAnglePhi = projectorHalfAngleTheta * (projectorResInPhi / (ureal)projectorResInTheta);//�v���W�F�N�g�̓��f�p

};
//�����Y�{�[���̃f�U�C���p�����[�^
namespace lensballDesignParams {

	constexpr ureal sphereRadiusInner = 1.;//�����Y�{�[�����a�̒��a
	const sphereParam lensballParamInner(uvec3::Zero(), sphereRadiusInner);//�����Y�{�[���̃p�����[�^
	
	constexpr size_t lensNumInARow = hardwareParams::verticalDirectionResolution;//��s������̍s�̐�

	constexpr ureal rowAngle = 0.02706659 * 2.;//�s�̊p�x theta=atan(-1.5*1/(cos(theta)*sqrt(3))*1/lensnum)
	const ureal rowLength = 2. * pi * cos(rowAngle);//�s�̒���
	const ureal lensEdgeWidth = rowLength / (ureal)lensNumInARow / 2.;
	const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInARow;//�Z�p�`�̈�ς����V�t�g����
	const bitrans<Eigen::Rotation2D<ureal>> DesignedMapToMap = bitrans<Eigen::Rotation2D<ureal>>(Eigen::Rotation2D<ureal>(rowAngle));//�����Y�A���C���X�΂�����O����X�΂��������Ƃɂ���

	//const ureal nodeLensRadius = 2. * lensEdgeWidth / sqrt(3.);//�v�f�����Y�`����쐬�@���̔��a
	const std::pair<size_t, size_t> nodeLensResolution = make_pair(20 * 2, 20);//�v�f�����Y�̕�����
	constexpr size_t rowNum = 15;//��ɂ��Ă� �s�̐�

	//�v�f�����Y�̊T�`�͘Z�p�`�ɂȂ�͂�
	const auto hexverticesNodelensOuter = [&] {
		auto hexvjunk = MakeHexagon(lensballDesignParams::lensEdgeWidth);//�Z�p�`�̒��_
		hexvjunk.push_back(hexvjunk.front());//������邽�߂ɍŏ��̓_�𖖔��ɑ}��

		return hexvjunk;
	}();

	constexpr ureal nodelensEta = 1.5;//�m�[�h�����Y�̔���ܗ�

	//�����Y�{�[���̊T�`��ȉ~�̋ߎ�����
	//���̂��߂ɔ��a��m�肽��X,Y���a�͈ꏏ�A�����ԓ���̃����Y���a�Ō��܂�
	//z��̔��a��1�ɂȂ�񂶂�Ȃ�����
	const uvec3 lensballApproximateShapeElipsoid = [&] {
		const ureal outerRadiusInEquater = GetNodelensRadiusInDobuleface(uvec2::Zero()).second;//�ԓ��ł̃����Y�{�[���O�����a�����߂�
		const ureal radiusXY = lensballDesignParams::sphereRadiusInner + outerRadiusInEquater;//���񂾂��c������
		return uvec3(radiusXY, radiusXY, 1.); }();

};
//�������̃p�����[�^
namespace developperParams {
	const ureal apertureRadius = lensballDesignParams::sphereRadiusInner / 300.;//�����ɂ���������v���W�F�N�^�[����o�����Ă���
	const auto regularHexagon = [&] {
		auto hexvjunk = MakeHexagon(sqrt(3.) / 2.);//�Z�p�`�̒��_
		hexvjunk.push_back(hexvjunk.front());//������邽�߂ɍŏ��̓_�𖖔��ɑ}��

		return hexvjunk;
	}();//�O�ډ~�̔��a��1�ɂȂ�悤�ȘZ�p�`
	constexpr size_t searchAreaInALen = 5;// lensballDesignParams::lensNumInARow;//�����s�̃����Y���ǂꂾ���[�ǂ����Č������邩
	constexpr size_t searchAreaInARow =6;// lensballDesignParams::rowNum;//����ǂꂾ���[�ǂ����Č������邩
	//�������t�����m�[�h�����Y����T������͈�
	//�܂��w�b�_��ǂݏo��
	const std::string framePrefixX = "frames\\frame";//�t���[�����i�[���Ă���t�H���_��prefix <prefix><id>.bmp�݂����Ȗ��O�ɂ��Ă�
	const std::string dicHeaderPathX = "dic\\dic.head";//�����w�b�_�̃p�X
	const std::string developRezPath = "dev";//branch�t�H���_���̂����Ɍ��ʂ�ۑ�����

	constexpr ureal nodelensExpand = 1. + 1.e-4;//�m�[�h�����Y�̃M���M���ɓ��˂����Ƃ��ɔ���ł���悤�ɂ���g�嗦
	size_t devThreadNum = 1;//�������ǂꂾ���̃X���b�h�Ŏ��s���邩(�R���e�L�X�g�Ɏx�z����܂�)

	//�����Ɏg���J����
	constexpr ureal fovHalf = 1.5 / 180. * pi;
	constexpr size_t cameraResW = 2048, cameraResH = cameraResW;//���郌�C�ɑ�\���邩���
	constexpr ureal brightnessCoef = 3.;//���邳�W���@���ꂾ�����邭�Ȃ�

	constexpr size_t subStepRes=1;//���ׂ����{�[������

	auto cameraToGlobal = Eigen::Affine3d(Eigen::AngleAxis<ureal>(0. / 180. * pi, uvec3::UnitY())*Eigen::AngleAxis<ureal>(0./180.*pi ,uvec3::UnitZ())* Eigen::Translation<ureal, 3>(uvec3(30.,0.,0.)));//�J�����̕ϊ� �J������-x���������� z��������ɂ���
};
//�X�L�������̃p�����[�^
namespace scanParams {
	size_t scanThreadNum = 1;//�X�L�����Ɏg���X���b�h��(�R���e�L�X�g�Ɏx�z����܂�)
	const std::string resultDicPrefix = "dic\\dic";
	constexpr size_t searchAreaInALen = 7;//�����s�̃����Y���ǂꂾ���[�ǂ����Č������邩
	constexpr size_t searchAreaInARow =7;//����ǂꂾ���[�ǂ����Č������邩
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
	const ureal dist = 1. / tan(hardwareParams::projectorHalfAngleTheta);

	//���_��ʂ��Ă����ɂ��ǂ蒅�����C
	return uvec3(dist, posInDisplay.x(), posInDisplay.y()).normalized();
}
//���˕�������v���W�F�N�^�s�N�Z������肷��
ivec2 GetPixPosFromEnteredRay(const uvec3& enteredDir) {
	//�\���ʂ�x�ʒu
	const ureal dist = 1. / tan(hardwareParams::projectorHalfAngleTheta);

	//�A�p�[�`�����i��Ό��_��ʂ�ƍl�����邩������̎�������
	const ureal t = dist / enteredDir.x();
	const uvec3 crosspos = enteredDir * t;//�������\���ʂƂ̌���

	const uvec2 posInDisplay(crosspos.y(), crosspos.z());

	//�t�Z������Ƃ�����������
	const uvec2 realpos = 0.5 * (posInDisplay + uvec2(1., 1.));
	return  ivec2(round(realpos.x() * (ureal)(hardwareParams::projectorResInPhi - 1)), round(realpos.y() * (ureal)(hardwareParams::projectorResInTheta - 1)));
}



enum rayIncidentWay {
	toInner,//�{�[����������
	toOuter//�{�[���O������
};
class doubleFaceLensParam :private std::pair<sphereParam, sphereParam> {
	using super = std::pair<sphereParam, sphereParam>;
public:
	sphereParam& inSurf() { return this->first; }//�����̖�
	sphereParam& outSurf() { return this->second; }//�O���̖�
	const sphereParam& inSurf() const{ return this->first; }//�����̖�
	const sphereParam& outSurf() const{ return this->second; }//�O���̖�

	doubleFaceLensParam() = default;
	doubleFaceLensParam(const doubleFaceLensParam&) = default;
	//�R�s�[�ł��܂�
	doubleFaceLensParam(const super& s) :super(s){
		//*this = s;
	}
	//����ł��܂�
	doubleFaceLensParam& operator=(const super& s) {
		super::operator=(s);
		return *this;
	}
};


//�v�f�����Y�p�����[�^�����@�����YID����O�ʂƌ�ʂ̋��ʃp�����[�^�������ł���
using nodeLensDic = std::unordered_map<std::pair<size_t, size_t>, doubleFaceLensParam, HashPair>;

//�m�[�h�����Y�������������ʂ��̃����Y�p�����[�^�ƌ����ꏊ��Ԃ��Ă����
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

//Search Node lens�֐����ŏ��ɓ���������������Y����ǂ�قǗ��ꂽ�����Y�����ʂƂ��ĕԂ��������i�[����@���ꂪ���~�b�g���O�Ȃ烊�~�b�g�̐ݒ肪����������������Ȃ�
class accuracyOfSearchNodelenses:private std::pair<size_t,size_t> {
	using super = std::pair<size_t, size_t>;
public:

	super::first_type& distOfRow() { return this->first; }
	super::first_type& distOfLens() { return this->second; }
	const super::first_type& distOfRow() const{ return this->first; }
	const super::first_type& distOfLens() const{ return this->second; }

	//�^����ꂽaccuracy�Ɣ�r���Ă���accurasy���ň��ɍX�V����
	accuracyOfSearchNodelenses& UpdateThisToWorth(const accuracyOfSearchNodelenses& op) {
		//�e�v�f���Ƃ̍ő���Ƃ�
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


//�����ȗ����a�ƊO���ȗ����a�������Y�ʒu���狁�߂�
std::pair<ureal, ureal> GetNodelensRadiusInDobuleface(const uvec2& localcenterInMap) {

	//return (2. * lensballDesignParams::lensEdgeWidth / sqrt(3.)) * fabs(cos(localcenterInBalllocalPolar.y()));
	const ureal lensWidthCrossHalfInMap = lensballDesignParams::lensEdgeWidth * (2. / sqrt(3.));//�}�b�v�ł̃����Y�̑Ίp���̔���
	const uvec2 lensWidthHalfVecInMapD = uvec2(lensWidthCrossHalfInMap, 0.);//Map��ł̃����Y�̑Ίp�x�N�g���̔���


	//�ŏI�I�Ƀƕ����̊g����p���������OK
	const ureal lensWidthInTheta = acos(PolarToXyz(MapToLocalPolar(localcenterInMap + lensWidthHalfVecInMapD)).dot(PolarToXyz(MapToLocalPolar(localcenterInMap - lensWidthHalfVecInMapD))));//�����Y�̏ォ�牺����������b�V�[�^�̊p�x���̂͂�
	const ureal lensWidthGenLength = 2. * lensballDesignParams::lensballParamInner.second * sin(lensWidthInTheta / 2.);//�����̌��̒���
	const ureal lensWidthKoLength = lensballDesignParams::lensballParamInner.second * lensWidthInTheta;//�����̌ǂ̒���
	//return lensWidthGenLength /2.;
	//���̃T�C�Y����Ƃ��Ăǂꂾ���g�傷��ΑS���˂��N���Ȃ��̂��v�Z����

	//���ɑS���ˊp�����߂���
	const ureal rinkaiAngle = asin(1. / lensballDesignParams::nodelensEta);
	//�ՊE�p���킩��Ɩ@�������̊p�x�ƂȂ�Ƃ���x���W(���a��1�Ƃ����Ƃ���)���킩�� �v���X��
	const ureal xWhereANormisRinkai = (rinkaiAngle * sqrt(pow(rinkaiAngle, 2) + 1.)) / (pow(rinkaiAngle, 2) + 1.);
	//�܂肱�ꕪ�̈ꂷ��΃M���M���[�����ŗՊE�p�̂͂�
	const ureal radiusInner = lensWidthGenLength / 2. / xWhereANormisRinkai;
	//�O�����a�̓M���M���[�������M���M���[�����ɂȂ�悤�Ȕ��a
	const ureal radiusOuter = fabs(lensWidthGenLength) / 2.;

	return make_pair(radiusInner, radiusOuter);
}

//elipsoid�̌�_�����ʏ�ɓ��e����
resultIntersecteSphere GetHitResultOfSphereFromElipsoidIntersection(const uvec3& elipsoidRadius,const std::optional<ureal> intersectRez,const arrow3& ar) {
	if (!intersectRez) {
		resultIntersecteSphere rez;
		rez.isHit = false;
		return rez;
	}
	resultIntersecteSphere rez;
	rez.isHit = true;
	rez.t = intersectRez.value();
	rez.norm = (ar.dir() * rez.t + ar.org()).normalized();
	rez.pos = rez.norm * lensballDesignParams::sphereRadiusInner;

	return rez;
}

//�v�f�����Y���������� �������炩�O�����炩�ŐU�镑�����ς��܂� 	
optional<std::pair<resultSearchNodeLens, accuracyOfSearchNodelenses>> SearchNodeLensHitByARayInBalllocalX(const arrow3& targetInBalllocal, const resultIntersecteSphere& hitRezVsSphere, const nodeLensDic& nodelensParamsInBalllocal, const size_t searchAreaInARow, const size_t searchAreaInALen,const rayIncidentWay& enterFrom) {

	//���C�ƃ����Y�{�[���̌����ʒu�������ȍ��W�n�Ōv�Z
	//const auto& targetTVsBall = hitRezVsSphere.t;
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
	const int centerRawIndex = NormalizeIntger<signed int>((signed int)round(regRayDirLati), lensballDesignParams::rowNum);//�l�̌ܓ�����Ƃ����Ƃ��炵���C���f�b�N�X���킩��
	const int rawSearchWay = (regRayDirLati - (ureal)centerRawIndex) > 0. ? +1 : -1;//��������


	//�ł͂������烌���Y�ɓ����������
	ureal closestT = std::numeric_limits<ureal>::infinity();//�������������Y�̋���
	std::pair<size_t, size_t> hitlensIds;
	optional<std::pair<resultSearchNodeLens, accuracyOfSearchNodelenses>> hitlensParamInBalllocal;//�Ώۂ̃����Y�p�����[�^�ƏՓ˖@���@�{�[�����[�J����

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
		const int centerLensIndex = NormalizeIntger<signed int>((signed int)round(regRayDirLonn - (eachFlag ? 0.5 : 0.)), lensballDesignParams::lensNumInARow);//����̓I�t�Z�b�g���Ȃ��@�܂���}�C�i�X������n�܂��Ă���s�ɂ���ꍇ�̃C���f�b�N�X �����łȂ����-0.5���Ă���ۂ߂適���܂���
		const int lensSearchWay = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? +1 : -1;//�ׂ荇�������Y�̂����Ƃ��炵���C���f�b�N�X���킩��

		//�ł̓����Y�̓����蔻����n�߂�
		for (size_t lidlini = 0; lidlini < searchAreaInALen; lidlini++) {
			const size_t lid = GetBisideIndex(lidlini, centerLensIndex, lensSearchWay, lensballDesignParams::lensNumInARow);//������������Ƃ��납����ˏ�ɒT������

			const auto thislensparamInBalllocal = nodelensParamsInBalllocal.at(make_pair(rid, lid));//���ꂪ�����������Y�̃p�����[�^

			//���Ɠ����蔻�肷��
			const auto hitRezVsANode = [&] {

				//�������O�����ɂ���ĕ���
				if (enterFrom == rayIncidentWay::toInner) return IntersectSphere(targetInBalllocal, thislensparamInBalllocal.inSurf().first, thislensparamInBalllocal.inSurf().second);
				else return IntersectSphere(targetInBalllocal, thislensparamInBalllocal.outSurf().first, thislensparamInBalllocal.outSurf().second);
			}();
			if(hitRezVsANode.isHit)
				//����ς��ԋ߂������Y�������Ă����
				if (hitRezVsANode.t < closestT) {
					hitlensParamInBalllocal = make_pair(make_pair(thislensparamInBalllocal, hitRezVsANode), make_pair(rilini, lidlini));

					if (hitlensParamInBalllocal.value().first.lensParam().inSurf().second == 0.)
						int ad = 0;
					closestT = hitRezVsANode.t;
				}
		}

	}

	//�v�f�����Y���Ȃ��Ƃ���ɓ����������Ă���
	return hitlensParamInBalllocal;
}

//�X�l���̖@���Ō��̌o�H���v�Z �ǂ���������˂��������厖
std::optional<arrow3> GetRefractedRayWithASphericalLensX(const arrow3& targetInBalllocal, const resultSearchNodeLens& hitlensParamInBalllocal, const bool printMessagesInDevelopping,const rayIncidentWay& inciWay) {
	//���˕����ɂ���ē��˖ʂƏo�˖ʂ��ς��
	const sphereParam& inciSurf = (inciWay == rayIncidentWay::toInner) ? hitlensParamInBalllocal.lensParam().inSurf() : hitlensParamInBalllocal.lensParam().outSurf();
	const sphereParam& emitSurf = (inciWay == rayIncidentWay::toInner) ? hitlensParamInBalllocal.lensParam().outSurf() : hitlensParamInBalllocal.lensParam().inSurf();

	ray3 targetSeriesInBalllocal(targetInBalllocal);
	const auto fIntersect = IntersectSphere(targetSeriesInBalllocal.back(), inciSurf.first, inciSurf.second);
	if (!fIntersect.isHit)
		throw logic_error("�����蔻��̃��W�b�N���o�O���Ă܂�");
	fIntersect.ApplyToRay(targetSeriesInBalllocal);
	if (!RefractSnell(targetSeriesInBalllocal, fIntersect.norm, lensballDesignParams::nodelensEta)) {
		if (printMessagesInDevelopping)cout << "�S���˂��N����(���ˎ�)" << endl;
		return std::optional<arrow3>();//���̃V�[���ł͂��߂������̂Ŏ��̃��C
	}

	//����ɑΖʂ̔��������
	const auto bIntersect = __IntersectSphere_GetFarOne_TEMPORARYONE(targetSeriesInBalllocal.back(), emitSurf.first, emitSurf.second);//�Ƃ肠�����������̖ʂ��g���Ă܂�
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

	//mlab�ɂ��plt
	//const auto pltInMlab = [&](const ray3& target) {
	//	//�^�[�Q�b�g�̊Ԃ��v�Z����
	//	auto first = target.cbegin();
	//	//first++;

	//	while (1) {
	//		//��̏I�_���v�Z����
	//		auto second = std::next(first);
	//		if (second == target.cend())break;

	//		py::sf("mlab.plot3d([%f,%f],[%f,%f],[%f,%f])", first->org().x(), second->org().x(), first->org().y(), second->org().y(), first->org().z(),second->org().z());
	//		//DrawLine(plt, first->org(), second->org(), color);
	//		first = second;
	//	}
	//};
	//pltInMlab(targetSeriesInBalllocal);
	//PlotRayInMlab(refractedArrow, "color=(1,1,0)");

	return std::optional<arrow3>(refractedArrow);
}

//�J�����f����ۑ�����
void WriteBmpOfCamera(const developResult& devRez, const std::string& savepath) {

	//�܂��̓C���[�W�����
	bmpLib::img picture;//�J��������̉f��
	picture.width = developperParams::cameraResW;
	picture.data.resize(developperParams::cameraResH);
	picture.height = developperParams::cameraResH;
	bmpLib::img maskPic;//�ǂ��ɒl�����݂��邩
	maskPic.width = developperParams::cameraResW;
	maskPic.data.resize(developperParams::cameraResH);
	maskPic.height = developperParams::cameraResH;
	for (int yy = 0; yy < picture.height; yy++) {
		picture.data.at(yy).resize(developperParams::cameraResW);
		maskPic.data.at(yy).resize(developperParams::cameraResW);
		for (int x = 0; x < picture.width; x++) {
			//�܂��Ώۂ̈ʒu������������
			picture.data[yy][x] = bmpLib::color(0, 0, 0);
			maskPic.data[yy][x] = bmpLib::color(0, 0, 0);
		}
	}

	//�L�[�𑖍�����
	for (int yy = 0; yy < picture.height; yy++) {
		for (int x = 0; x < picture.width; x++) {
			const ivec2 key(x, yy);
			//����
			const auto& subIte = devRez.subpix.find(key);
			const auto& pixIte = devRez.colorSum.find(key);
			const auto& sizeIte = devRez.colorNum.find(key);//�Ή������s�N�Z����ݒu

			//9�̃s�N�Z���ɉe������
			for(int sy=0;sy<3;sy++)
				for (int sx = 0; sx < 3; sx++) {
					const ivec2 nowpixRer(sx - 1, sy - 1);
					const ivec2 nowpix = key + nowpixRer;
					//�Ȃ���Ζ���
					if (nowpix.x() < 0 || nowpix.x() >= developperParams::cameraResW)continue;
					if (nowpix.y() < 0 || nowpix.y() >= developperParams::cameraResH)continue;

					//x,y���Ƃ̍������߂�
					const uvec2 dist = subIte->second - uvec2(sx - 1, sy - 1);
					const ureal weight = max(1. - fabs(dist.x()), 0.) * max(1. - fabs(dist.y()), 0.);

					picture.data[yy+ nowpixRer.y()][x+nowpixRer.x()] += bmpLib::color(pixIte->second.x() * developperParams::brightnessCoef * weight, pixIte->second.y() * developperParams::brightnessCoef * weight, pixIte->second.z() * developperParams::brightnessCoef * weight);//�����������C��������Ă���͈͂���������F�t��
					maskPic.data[yy + nowpixRer.y()][x + nowpixRer.x()] += bmpLib::color(0, clamp<int>(sizeIte->second*weight, 0, 255), 0);
				}
		}
	}

	bmpLib::WriteBmp((savepath+".bmp").c_str(), &picture);
	bmpLib::WriteBmp((savepath+".mask.bmp").c_str(), &maskPic);
}

//�A�p�[�`��
bool ThroughAperture(const arrow3& global) {
	constexpr ureal espAperture = 0.0001;
	if (fabs(global.dir().x()) <= espAperture)return false;//�A�p�[�`���ɕ��s�Ȃ�ǂ����悤���Ȃ�
	const ureal t = -global.org().x() / global.dir().x();//x=0�Ƃ̍D�]�����߂�
	const uvec3 hitpos = global.dir() * t + global.org();

	//���̃m���������a�����Ȃ�
	return hitpos.norm() <= developperParams::apertureRadius;
}

//�V�[��id�����]�p�x���v�Z����@substep���܂߂Ď����œn���Ă�
ureal GetRotationAngleFromRd(const ureal rd) {
	return uleap({ 0.,2. * pi }, rd / (ureal)(hardwareParams::numOfProjectionPerACycle));
}

//���郌�C���{�[���ɒʉ߂������Ƃ��̊g����p����\���G���A�̔��a�����Ƃ߂�
ureal SolveVisibleAreaRadiusFromRefractAngle(const ureal ang) {
	//���̊p�x��������Ƃ��̌X��
	const auto slope = tan(ang / 2.);
	//���̂Ƃ��̔��a���ŏ��Ƃ���x
	const auto minx = -pow(slope, 2) / (1. + pow(slope, 2));
	//���̂Ƃ���y
	const auto miny = slope * minx + slope;

	return sqrt(pow(minx, 2) + pow(miny, 2));
};


int main(int argc, char* argv[]) {

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

		//�R���e�L�X�g�t�@�C����ǂݍ���Ńf�o�C�X�ɕ��ՓI�ȏ���������
		constexpr bool useContext = true;
		std::list<std::string> appArgs;//�A�v���P�[�V�����̈����͂����Ɋi�[�����
		if (useContext) {
			appContext usercon;
			{
				ifstream ifs(appContextPath, std::ios::binary);

				cereal::JSONInputArchive iarch(ifs);
				iarch(usercon);
			}

			//�R���e�L�X�g��K�p����
			rezpath = usercon.rezpath;
			developperParams::devThreadNum = usercon.threadNumMax;
			scanParams::scanThreadNum = usercon.threadNumMax;


			//��������������
			if (argc) {//�������^�����Ă����
				for (size_t ad = 0; ad < argc; ad++)
					appArgs.push_back(std::string(argv[ad]));
			}
			else appArgs = usercon.defaultArg;
		}

		//Python���Z�b�g�A�b�v���Ă��烌���Y�{�[���̊T�`������
		constexpr bool drawSphere = true;//�����Y�{�[���T�`��`�悷��
		constexpr bool drawAparture = false;//�A�p�[�`����`�悷��
		{

			//python�����^�C�����������Ă��낢�돉������
			py::Init();

			py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

			//mayavi�̐ݒ�
			const std::pair<size_t, size_t> figResolution(800, 600);
			py::sf("fig = mlab.figure(\'Refract dir In Global\', size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

			//matplotlib�̐ݒ�
			//py::s("fig, ax = plt.subplots()\nax.set_aspect(\"equal\")\nplt.title('Lens dist In MapD')");

			//����`�悷��
			constexpr size_t sphereResolution = 20;
			if (drawSphere)py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
spx = np.cos(sphphi)*np.sin(sphtheta)
spy = np.sin(sphphi)*np.sin(sphtheta)
spz = np.cos(sphtheta)
mlab.mesh(%f*spx, %f*spy, %f*spz ,color=(1.,1.,1.) )  
)", sphereResolution, sphereResolution, lensballDesignParams::sphereRadiusInner, lensballDesignParams::sphereRadiusInner, lensballDesignParams::sphereRadiusInner);

			//�A�p�[�`�������ŋߎ����ĕ`�悷��
			if (drawAparture)py::sf(R"(
[sphphi,sphtheta] = np.mgrid[0:2*np.pi:%dj,0:np.pi:%dj]
spx = np.cos(sphphi)*np.sin(sphtheta)
spy = np.sin(sphphi)*np.sin(sphtheta)
spz = np.cos(sphtheta)
mlab.mesh(%f*spx, %f*spy, %f*spz ,color=(0.,1.,0.) )  
)", sphereResolution, sphereResolution, developperParams::apertureRadius, developperParams::apertureRadius, developperParams::apertureRadius);
		};






		//�����Y�A���C���쐬�A�`�悷��
		//�Z�p�`�Ń^�C�����O����@�����s�������Ă����s���������Ċ���
		constexpr bool calcNodelenses = true;//�m�[�h�����Y�̈ʒu���v�Z���ă����Y�{�[�����`������
		constexpr bool drawNodelenses = calcNodelenses & false;//�v�f�����Y��`�悷��
		constexpr bool drawNodelensEdges = calcNodelenses & false;//�m�[�h�����Y�̘g����`�悷��

		//���̌v�Z�ŗv�f�����Y���X�g���킩���
		std::optional<nodeLensDic> nodelensParamsFrontBackInBalllocal = std::nullopt;//�s�ԍ��@�����Y�ԍ����L�[�Ńp�����[�^��Balllocal�ŕۑ����� ���˖ʂƏo�˖ʂŕʁX�̃p�����[�^�����蓖�Ă���
		if(calcNodelenses){
			//opt�v�Z�ł��܂�
			nodelensParamsFrontBackInBalllocal = decltype(nodelensParamsFrontBackInBalllocal)::value_type();
			auto& nodelensParamsFrontBackRez = nodelensParamsFrontBackInBalllocal.value();

			//std::list<uleap>//�}�b�v���W�ł̃����Y���S
			for (std::decay<decltype(lensballDesignParams::rowNum)>::type rd = 0; rd < lensballDesignParams::rowNum; rd++) {
				const ureal tlati = GetLatitudeInMapDFromRowIndex(rd);//lati�����̌��݈ʒu
				const bool eachFlag = rd % 2;//���݂ɐ؂�ւ��t���O
				for (std::decay<decltype(lensballDesignParams::lensNumInARow)>::type ld = 0; ld < lensballDesignParams::lensNumInARow; ld++) {

					//�Z�p�`�����߂�o�b�t�@���N���A
					ResetPyVecSeries(pypltSeries);

					//lonn�����̌��݈ʒu
					const ureal tlonn = GetLongitudeInMapDFromLensIndexAndRowFlag(ld, eachFlag);
					const uvec2 localcenterInMapDesigned = uvec2(tlonn, tlati);//�v�f�����Y�̒��� ���[�J���}�b�v���W
					const uvec2 localcenterInMap = lensballDesignParams::DesignedMapToMap.prograte() * localcenterInMapDesigned;
					const uvec2 localcenterInBalllocalPolar = MapToLocalPolar(localcenterInMap);
					const uvec3 localcenterInBalllocal = PolarToXyz(localcenterInBalllocalPolar);

					//�����Y�̔��a��S���˂������Ȃ��悤�ȃT�C�Y�ɂ��ĉ���
					const std::pair<ureal, ureal> lensRadiusDoubleface = GetNodelensRadiusInDobuleface(localcenterInMap);

					//�p�����[�^��o�^
					nodelensParamsFrontBackRez[make_pair(rd, ld)].operator=(make_pair(sphereParam(localcenterInBalllocal, lensRadiusDoubleface.first), sphereParam(localcenterInBalllocal, lensRadiusDoubleface.second)));

					//�v�f�����Y��`�悵�Ă���
					if ((drawNodelenses || drawNodelensEdges)) {
						const auto colorFactor=uleap(make_pair<ureal, ureal>(0., 1.), (ureal)(rd* lensballDesignParams::lensNumInARow + ld) / (ureal)(lensballDesignParams::rowNum * lensballDesignParams::lensNumInARow));
						const auto color = HsvToRgb({ colorFactor,1.,1. });//�F�𗆐��ɕς���

						//���ɋɍ��W�ŗv�f�����Y���v�Z����
						ResetPyVecSeries(nlensSeries);//�m�[�h�����Y
						for (std::decay<decltype(lensballDesignParams::nodeLensResolution.second)>::type nlla = 0; nlla < lensballDesignParams::nodeLensResolution.second; nlla++) {

							ResetPyVecSeries(mlabSeries);//mlabSeries�̓O���b�h�̈�s���i�[����
							for (std::decay<decltype(lensballDesignParams::nodeLensResolution.first)>::type nllo = 0; nllo < lensballDesignParams::nodeLensResolution.first; nllo++) {
								const uvec2 localpos(uleap(PairMinusPlus(pi), nllo / (ureal)(lensballDesignParams::nodeLensResolution.first - 1)),
									uleap(PairMinusPlus(pi / 2.), nlla / (ureal)(lensballDesignParams::nodeLensResolution.second - 1)));//�v�f�����Y���[�J���ł̋ɍ��W

								const uvec3 nodelensShape = (lensRadiusDoubleface.second * PolarToXyz(localpos));//���ꂪ���ɂȂ�͂� �Ƃ肠�����O�����a�ŕ`��
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


		//�X�L����������
		constexpr bool scanLenses = true;//�����Y�{�[���ɑ΂��郌�C�g���[�V���O���s��
		constexpr bool drawRefractionDirectionOfARay = false;//���郌�C�̋��ܕ�����`�悷��
		constexpr bool logWarningInScan = false;//scan���̌x����\������
		if (scanLenses) {

			//���񏈗��p�̂��낢��
			std::vector<uptr<std::thread>> scanThreads(scanParams::scanThreadNum);//���s�X���b�h
			std::vector<bool> finFlagOfEachScanThread(scanParams::scanThreadNum);//�X�L����������������Ƃ��

			//�i�[���ʂ̌`���������w�b�_������ĕۑ�����
			const projRefraDicHeader storageheader(hardwareParams::projectorResInPhi, hardwareParams::projectorResInTheta, hardwareParams::numOfProjectionPerACycle);
			storageheader.SaveHeader(rezpath + branchpath + scanParams::resultDicPrefix);


			ResetPyVecSeries<6>(quiverSeries, quiverPrefix);//�x�N�g��������|��
			ResetPyVecSeries(pypltSeries);//�x�N�g��������|��

			//���̃X�L�����̐��m��
			mutexedVariant<accuracyOfSearchNodelenses> searchAccuracyOfTheScanX;
			mutexedVariant<std::list<uvec3>> refractWays;

			//�w�肳�ꂽ���ʂ��w�肳�ꂽ�X�g���[�W�֑��M���� �]���������Ƃ�Rez�̓N���A����܂�
			const auto TransRezToStorage = [&](std::list<arrow3>& rezOfSt, ofstream& storageofStPtr) {
				cereal::BinaryOutputArchive o_archive(storageofStPtr);
				o_archive(rezOfSt);

				rezOfSt.clear();
			};
			//��]�p�x���ƂɃX�L�������s��
			const auto ScanAScene = [&](const std::decay<decltype(hardwareParams::numOfProjectionPerACycle)>::type rd, decltype(finFlagOfEachScanThread)::iterator finflagOfStIte) {

				const ureal ballRotation = GetRotationAngleFromRd(rd);//�{�[���̉�]�p�x
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//�O���[�o�����烌���Y�{�[�����[�J���ւ̕ϊ� XYZ���W

				//���ʊi�[�������ƃX�g���[�W��p�ӂ���
				std::list<arrow3> rezMem;
				std::ofstream storageStream(rezpath + branchpath + scanParams::resultDicPrefix + ".part" + to_string(rd), std::ios::binary);

				//�v�f�����Y�����̐��m���C���W�Q�[�^(�ŏ��ɓ������������������ǂꂾ�����ꂽ�ʒu������������)
				accuracyOfSearchNodelenses searchAccuracyOfTheScene;

				//�e�s�N�Z�������яo�郌�C�Ɖ�]�p�xrd�̋��Ƃ̓����蔻����s��
				for (std::decay<decltype(hardwareParams::projectorResInTheta)>::type vpd = 0; vpd < hardwareParams::projectorResInTheta; vpd++) {//�v���W�F�N�^�̒��ډ�f���Ƃ�
					for (std::decay<decltype(hardwareParams::projectorResInPhi)>::type hpd = 0; hpd < hardwareParams::projectorResInPhi; hpd++) {

						const uvec3 rayDirInGlobal = GetRayDirFromProjectorPix(ivec2(hpd, vpd));
						const arrow3 rayArrowInBalllocal(uvec3::Zero(), GlobalToBallLocal.prograte() * rayDirInGlobal);//���[�J���ŕ\�������C

						//���̃��C�������Y�{�[���ɓ����邩
						const auto hitRezVsSphere = IntersectSphere(rayArrowInBalllocal, lensballDesignParams::lensballParamInner.first, lensballDesignParams::lensballParamInner.second);//���C�̑�܂��Ȓ��e�_���v�Z����Sphere�̂ǂ��ɓ�����܂���
						if (!hitRezVsSphere.isHit)throw logic_error("�v���W�F�N�^������o����Ă���񂾂����Γ�����͂��Ȃ�");

						//�v�f�����Y���T�[�`
						const auto hitRezVsANodelens = SearchNodeLensHitByARayInBalllocalX(rayArrowInBalllocal, hitRezVsSphere, nodelensParamsFrontBackInBalllocal.value(), scanParams::searchAreaInARow, scanParams::searchAreaInALen, rayIncidentWay::toInner);//�v���W�F�N�^�[�͓���!
						if (!hitRezVsANodelens) {
							//�����v�f�����Y���Ȃ���Όv�Z����K�v�͂Ȃ�
							if (logWarningInScan)cout << "�v�f�����Y�Ȃ�������" << endl;
							rezMem.push_back(arrow3(uvec3::Zero(), uvec3::Zero()));
						}
						else {
							//���m�����X�V���Ƃ�
							searchAccuracyOfTheScene.UpdateThisToWorth(hitRezVsANodelens.value().second);

							//�œ_�̏ꏊ�͗v�f�����Y���m�肷��Όv�Z�ł���
							const auto refractedInBalllocal = GetRefractedRayWithASphericalLensX(rayArrowInBalllocal, hitRezVsANodelens.value().first, logWarningInScan, rayIncidentWay::toInner);
							if (!refractedInBalllocal) {
								//�S���˂���Ȃ�v�Z����K�v���Ȃ�
								if (logWarningInScan)cout << "�ӂ��ɋ��܂��ďo��Ȃ�������" << endl;
								rezMem.push_back(arrow3(uvec3::Zero(), uvec3::Zero()));
							}
							else {
								//������O���[�o���ɖ߂�
								const arrow3 refractRayDirInGlobal(GlobalToBallLocal.untiprograte() * refractedInBalllocal.value().org(), GlobalToBallLocal.untiprograte() * refractedInBalllocal.value().dir());

								//�v���b�g���܂��傤
								//py::sf("mlab.quiver3d(%f,%f,%f,%f,%f,%f,mode=\"arrow\")", refractRayDirInGlobal.org().x(), refractRayDirInGlobal.org().y(), refractRayDirInGlobal.org().z(),refractRayDirInGlobal.dir().x(), refractRayDirInGlobal.dir().y(), refractRayDirInGlobal.dir().z());
								//cout << "a" << endl;
								//PlotRayInMlab(refractRayDirInGlobal,"color = (0, 0, 1), tube_radius = 0.01");
								//

								//refractWays.GetAndLock()->push_back(refractRayDirInGlobal.dir());
								//refractWays.unlock();

								//���ʂ�ǉ�
								rezMem.push_back(refractRayDirInGlobal);

								if(vpd%100==0&& hpd%100==0)PlotRayInMlab(refractRayDirInGlobal, "tube_radius = 0.01, color=(1,0,0)");

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
				//TransRezToStorage(rezMem, storageStream);
				//�ꉞworth���X�V
				searchAccuracyOfTheScanX.GetAndLock()->UpdateThisToWorth(searchAccuracyOfTheScene);
				searchAccuracyOfTheScanX.unlock();

				*finflagOfStIte = true;
			};

			//�����X���b�h�ɉ�]�p�x��ς��Ȃ��犄�蓖�Ă�
			for (std::decay<decltype(hardwareParams::numOfProjectionPerACycle)>::type rdgen = 0; rdgen < hardwareParams::numOfProjectionPerACycle; rdgen++) {
				if (rdgen % 50 != 0)continue;
				std::cout << "count: " << rdgen << endl;
				//����rdgen�ł̏������J���Ă���X���b�h�Ɋ���t������
				bool isfound = false;
				while (!isfound) {//����t�����Ȃ���ΌJ��Ԃ�
					for (size_t th = 0; th < scanParams::scanThreadNum; th++)
						if (!scanThreads.at(th)) {//�󂫂Ȃ犄�t
							if (!isfound) {//��̃C���f�b�N�X�ɂ͈�񂾂�����t����
								isfound = true;
								finFlagOfEachScanThread.at(th) = false;//�t���O���N���A����

								scanThreads.at(th).reset(new std::thread(ScanAScene, rdgen, finFlagOfEachScanThread.begin() + th));//�X���b�h���s�J�n
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

			//�������ɗL���p�x���v�Z����
			const auto& refptr = refractWays.GetAndLock();
			ureal maxangle = 0.;
			for(const auto& a:*refptr)
				for (const auto& b : *refptr) {
					//�p�x���v�Z
					const auto nowangle = fabs(acos(a.dot(b)));
					maxangle = max(maxangle, nowangle);
				}
			
			//���ɍŏ��p�x�����߂�
			const ureal minangle = [&] {
				std::array<ureal, 6*12> classes;//�����̕������p�x�ɍ��킹�Čv�Z����
				for (size_t i = 0; i < classes.size(); i++)
					classes.at(i) = 0.;
				const auto GetAClass = [&](const ureal& theta) {
					const int classId = round((theta + pi) / (2. * pi) * (ureal)(classes.size() - 1));//0~36�ɐ��K������classpy
					return classId;
				};

				const uvec3 rayDirInGlobal = GetRayDirFromProjectorPix(ivec2(hardwareParams::projectorResInPhi / 2, hardwareParams::projectorResInTheta / 2));//�����C�̌���
				const uvec3 base0 = (rayDirInGlobal.cross(uvec3::UnitZ()));//����ɒ��s���邠����
				const uvec3 base1 = (rayDirInGlobal.cross(base0));//������̒��s���

				for (const auto& a : *refptr) {
					const uvec3 projected = a -((a.dot(rayDirInGlobal)) * rayDirInGlobal);//���e���܂����@�x�N�g����
					//���Ɋ��W�J����
					const uvec2 projectedOnBases(base0.dot(projected), base1.dot(projected));
					//�ɍ��W�ɕϊ�����
					const uvec2 projectedBInPolar(projectedOnBases.norm(), NormalizeAngle(atan2(projectedOnBases.y(), projectedOnBases.x())));
					classes.at(GetAClass(projectedBInPolar.y())) = max(classes.at(GetAClass(projectedBInPolar.y())), projectedBInPolar.x());//�N���X�ň�ԑ傫�����a��T��
				}

				for (size_t i = 0; i < classes.size(); i++)
					py::sf("plt.scatter(%f,%f)", (ureal)i, classes.at(i));

				const ureal min = *min_element(classes.cbegin(),classes.cend());//�ŏ��v�f��get ����͓��e����(sin(theta))
				return 2. * asin(min);
			}();
			refractWays.unlock();
			//�g����p��visible area radius��\��
			cout <<"MAX ANG: " << maxangle*180./pi<<" deg\tMIN ANG: " << minangle * 180. / pi<<endl;
			cout << "MAX RAD: " << SolveVisibleAreaRadiusFromRefractAngle(maxangle) << "\tMIN RAD: " << SolveVisibleAreaRadiusFromRefractAngle(minangle) << endl;

			//���m�����\������
			const auto& accu = searchAccuracyOfTheScanX.GetAndLock();
			cout << accu->distOfRow() << "\t" << accu->distOfLens() << endl;
			searchAccuracyOfTheScanX.unlock();
		};





		//�f�x���b�v�Z�N�V����
		constexpr bool developImage = false;
		constexpr bool printMessagesInDevelopping = developImage && false;//�f�x���b�v���̃��b�Z�[�W���o�͂��邩
		if (developImage) {
			projRefraDicHeader header;
			{
				ifstream ifs(rezpath + branchpath + developperParams::dicHeaderPathX, std::ios::binary);
				cereal::BinaryInputArchive iarch(ifs);

				iarch(header);

				std::cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
			}

			//�𑜓x�Ƃ����킩��
			//���Ɏ��_���ƂɃ��C�g���[�X���Ăǂ̉�f�ɓ����邩���ׂ���

			//�V�[���̒��Ń}���`�X���b�h������
			std::vector<uptr<std::thread>> devThreads(developperParams::devThreadNum);
			std::vector<bool> finFlagOfEachDevThread(developperParams::devThreadNum);//�X�L����������������Ƃ��
			
			//�����ł̗v�f�����Y�����̐��m���C���W�Q�[�^(�ŏ��ɓ������������������ǂꂾ�����ꂽ�ʒu������������)
			mutexedVariant<accuracyOfSearchNodelenses> searchAccuracyOfTheDevelopping;

			//�J�����̕ϊ���ς��Ȃ��烌���_�����O����
			std::vector<std::pair<ureal, ureal>> cameraTransAngleSets = {
				std::make_pair(-30.,+30.), std::make_pair(0.,+30.), std::make_pair(+30.,+30.),
				std::make_pair(-30.,0.), std::make_pair(0.,0.), std::make_pair(+30.,0.),
				std::make_pair(-30.,-30.), std::make_pair(0.,-30.), std::make_pair(+30.,-30.)};//Z�����S�̉�]�p�x�AY�����S�̉�]�p�x�̏���[deg]�ŕ\�L����

			if (developperParams::cameraResW != developperParams::cameraResH)throw std::logic_error("");
			std::uniform_real_distribution<> regularUniform(-1./ (ureal)developperParams::cameraResW, +1. / (ureal)developperParams::cameraResW);//�v���}�C1�͈̔͂Ɍ���

			for (size_t ctd = 0; ctd < cameraTransAngleSets.size();ctd++) {
				developperParams::cameraToGlobal = Eigen::Affine3d(Eigen::AngleAxis<ureal>(cameraTransAngleSets.at(ctd).second / 180. * pi, uvec3::UnitY()) * Eigen::AngleAxis<ureal>(cameraTransAngleSets.at(ctd).first / 180. * pi, uvec3::UnitZ()) * Eigen::Translation<ureal, 3>(uvec3(30., 0., 0.)));//�J�����̕ϊ����Z�b�g����
				
				mutexedVariant<developResult> devRez;//�f�x���b�v���ʂ̐��摜

				//�J�����𐶐� �A�X�y�N�g���1�ŌŒ�ł��@�c���̉𑜓x�Ɋւ�炸�g����p�͓������Ȃ�܂�
				std::list<arrow3>cameraRayList;
				for (size_t y = 0; y < developperParams::cameraResH; y++) {
					const ureal scy = uleap(PairMinusPlus(1.), y / (ureal)(developperParams::cameraResH - 1));//�T�u�s�N�Z����������
					for (size_t x = 0; x < developperParams::cameraResW; x++) {
						//�X�N���[���̈ʒu��((2/res)*i+(1/res))-1 ��ذ݃T�C�Y�͑���2*2
						const ureal scx = uleap(PairMinusPlus(1.), x / (ureal)(developperParams::cameraResW - 1));
						double scz = 1. / tan(developperParams::fovHalf);//����p�����߂鎖���ł���

						//�T�u�s�N�Z�������
						//const uvec2 subpix(regularUniform(dice), regularUniform(dice));
						const uvec2 subpix(0., 0.);
						devRez.GetAndLock()->subpix[ivec2(x, y)] = subpix;
						devRez.unlock();

						//org��0 way���X�N���[���̐��K��
						Eigen::Vector3d scnormed = Eigen::Vector3d(-scz, scx + subpix.x(), scy + subpix.y()).normalized();

						cameraRayList.push_back(arrow3(developperParams::cameraToGlobal * uvec3(0., 0., 0.), developperParams::cameraToGlobal.rotation() * scnormed));
						//py::sf("mlab.quiver3d(%f,%f,%f,%f,%f,%f)", cameraRayList.back().org().x(), cameraRayList.back().org().y(), cameraRayList.back().org().z(), cameraRayList.back().dir().x(), cameraRayList.back().dir().y(), cameraRayList.back().dir().z());
						//py::sf("mlab.plot3d([%f,%f],[%f,%f],[%f,%f],color=(1,0,0))", cameraRayList.back().org().x(), cameraRayList.back().dir().x()*30.+ cameraRayList.back().org().x(), cameraRayList.back().org().y(), cameraRayList.back().dir().y()*30.+ cameraRayList.back().org().y(),cameraRayList.back().org().z(), cameraRayList.back().dir().z()*30.+ cameraRayList.back().org().z());
					}
				}
				
				//����V�[���ł̃J�����f�����������񂷂�֐�
				const auto GetAFrameOfAScene = [&](const size_t rdx, const decltype(finFlagOfEachDevThread)::iterator finflag) {
					//���̌v�Z�ł�//�v�f�����Y�����̐��m���C���W�Q�[�^(�ŏ��ɓ������������������ǂꂾ�����ꂽ�ʒu������������)
					accuracyOfSearchNodelenses searchAccuracyOfTheScene;

					////�܂��̓t���[����ǂݏo��
					const auto thisFrame = make_unique<bmpLib::img>();
					bmpLib::ReadBmp((rezpath + branchpath + developperParams::framePrefixX + to_string(rdx / developperParams::subStepRes) + ".bmp").c_str(), thisFrame.get());

					//���Ƀ��[�J���O���[�o���ϊ����v�Z����
					const ureal ballRotation = GetRotationAngleFromRd((ureal)rdx / (ureal)(developperParams::subStepRes));//�{�[���̉�]�p�x
					const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//�O���[�o�����烌���Y�{�[�����[�J���ւ̕ϊ� XYZ���W

					//���郌�C��������s�N�Z���̐F�����߂�
					const auto GetColorOfCamPix = [&](const decltype(cameraRayList)::const_iterator cameraRayIte, const ivec2& pixOfCam) {//���C����肵�ă��[�J�����v�Z
						const arrow3 targetInBalllocal(GlobalToBallLocal.prograte() * (*cameraRayIte).org(), GlobalToBallLocal.prograte() * (*cameraRayIte).dir());
						//���̃��C�������Y�{�[���ɓ����邩
						//const auto hitRezVsSphere = IntersectSphere(targetInBalllocal, lensballDesignParams::lensballParamInner.first, lensballDesignParams::lensballParamInner.second);//���C�̑�܂��Ȓ��e�_���v�Z����Sphere�̂ǂ��ɓ�����܂���
						const auto hitRezVsElipsoid = IntersectArrowAndElipsoid(targetInBalllocal, lensballDesignParams::lensballApproximateShapeElipsoid);
						const auto hitRezVsSphere = GetHitResultOfSphereFromElipsoidIntersection(lensballDesignParams::lensballApproximateShapeElipsoid, hitRezVsElipsoid, targetInBalllocal);
						if (hitRezVsSphere.isHit) {
							//���̃��C���v�f�����Y�ɓ����邩��������@������񃍁[�J�����W�n�ł̘b
							const auto hitlensParamInBalllocal = SearchNodeLensHitByARayInBalllocalX(targetInBalllocal, hitRezVsSphere, nodelensParamsFrontBackInBalllocal.value(), developperParams::searchAreaInARow, developperParams::searchAreaInALen, rayIncidentWay::toOuter);//�J�����͊O��!
							
							if (hitlensParamInBalllocal) {//�v�f�����Y�ɓ���������
								//accuracy���X�V
								searchAccuracyOfTheScene.UpdateThisToWorth(hitlensParamInBalllocal.value().second);

								//����
								const auto refractedRay = GetRefractedRayWithASphericalLensX(targetInBalllocal, hitlensParamInBalllocal.value().first, printMessagesInDevelopping, rayIncidentWay::toOuter);
								if (refractedRay) {
									const arrow3 refractedArrowInGlobal(GlobalToBallLocal.untiprograte() * refractedRay.value().org(), GlobalToBallLocal.untiprograte() * refractedRay.value().dir());//���C�̌�����߂�

									//���Ƀv���W�F�N�^�[�̂ǂ̉�f�ɓ����邩������
									//�܂��J���ɓ����邩��
									if (!ThroughAperture(refractedArrowInGlobal)) {
										//�v���W�F�N�^�ɂ͓��Ђ��Ȃ�����
										//if (rdx % (int)(developperParams::subStepRes*2.24) == 2)PlotRayInMlab(refractedArrowInGlobal,"color=(0,0,1), tube_radius=0.01");

										if (printMessagesInDevelopping)cout << "�v���W�F�N�^�ɂ͓��˂��Ȃ�����" << endl;
										return (std::optional<uvec3>)(std::nullopt);//���̃V�[���ł͂��߂������̂Ŏ��̃��C
									}

									//�J���ɂ��������烌�C�̌����ŉ�f�𔻒f�ł���
									const auto pixpos = GetPixPosFromEnteredRay(refractedArrowInGlobal.dir());
									//�A�p�[�`���ɓ�������Ⴄ�F�ŕ`�悵�Ă��
									//PlotRayInMlab(refractedArrowInGlobal, "color=(1,0,1), tube_radius=0.01");

									//�����ȍ��W�łȂ���΃��X�g�ɓ����
									if (pixpos.x() >= 0 && pixpos.x() < hardwareParams::projectorResInPhi && pixpos.y() >= 0 && pixpos.y() < hardwareParams::projectorResInTheta) {
										const auto pixColor = thisFrame.get()->data.at(pixpos.y()).at(pixpos.x());//�t���[������F�����o��
										//cout << "rez: " << pixpos.x() << "\t" << pixpos.y() << "\t";
										return std::optional<uvec3>(uvec3(pixColor.r, pixColor.g, pixColor.b));
									}
									else {
										//cout << "invalid: " << pixpos.x() << "\t" << pixpos.y() << "\t";
									}
								}
							}

						}

						//���ʂ����ɑ}������邱�Ƃ��Ȃ����
						return (std::optional<uvec3>)(std::nullopt);
					};

					//�J�����̐F���t���[������ǂݏo��
					auto cameraRayIte = cameraRayList.cbegin();
					for (size_t camY = 0; camY < developperParams::cameraResH; camY++)
						for (size_t camX = 0; camX < developperParams::cameraResW; camX++, cameraRayIte++) {
							const auto poskey = ivec2(camX, camY);//���̃J������f�ʒu
							if (cameraRayIte.operator*().dir() == uvec3::Zero());// cout << "skip:" << endl;
							else {
								//�F���Q�b�g����
								const optional<uvec3> thiscolor = GetColorOfCamPix(cameraRayIte, poskey);//�X���b�h���s�J�n
								if (thiscolor) {

									//cout << "poskey: " << poskey.x() << "\t" << poskey.y() << endl;
									const auto& devRezVal = devRez.GetAndLock();

									const auto pixIte = devRezVal->colorSum.find(poskey);
									if (pixIte == devRezVal->colorSum.cend())devRezVal->colorSum[poskey] = uvec3::Zero();
									devRezVal->colorSum[poskey] += thiscolor.value() / developperParams::subStepRes;//�e�V�[���̐F�𑫂����킹�Ă��΂���

									const auto sizIte = devRezVal->colorNum.find(poskey);//���t���[���ŃQ�b�g�ł��������Q�b�g
									if (sizIte == devRezVal->colorNum.cend())devRezVal->colorNum[poskey] = 0;
									devRezVal->colorNum[poskey] += 5.;

									devRez.unlock();
								}
								else
									int a = 0;
							}
						}

					searchAccuracyOfTheDevelopping.GetAndLock()->UpdateThisToWorth(searchAccuracyOfTheScene);
					searchAccuracyOfTheDevelopping.unlock();
					*finflag = true;
				};


				//�t�@�C���ǂݍ��݂̊ϓ_����V�[�����Ƃɂ��
				size_t rd = 0;
				bool threadLoopFin = false;
				while (!threadLoopFin) {
					for (size_t th = 0; th < developperParams::devThreadNum; th++) {
						if (!devThreads.at(th)) {
							finFlagOfEachDevThread.at(th) = false;//���Z�b�g����@�t���O��
							cout << "dev: " << rd / (ureal)developperParams::subStepRes << endl;
							devThreads.at(th).reset(new std::thread(GetAFrameOfAScene, rd, finFlagOfEachDevThread.begin() + th));//�������Z�b�g����
							rd++;
							//�����I���ł�
							if (rd >= header.rotationRes * developperParams::subStepRes) {
								threadLoopFin = true;
								break;
							}
						}
						else if (finFlagOfEachDevThread.at(th)) {//th�����s����Ă��āA���I����Ă�����
							devThreads.at(th)->join();
							devThreads.at(th).release();
						}
					}
				}
				for (size_t th = 0; th < developperParams::devThreadNum; th++) {
					if (devThreads.at(th)) {//th�����s����Ă�����
						devThreads.at(th)->join();
						devThreads.at(th).release();
					}
				}
				//���ʂ̊i�[�ꏊ�����
				const auto filePathWithTimeStamp = rezpath+branchpath+StringFormat("rez%d", ctd)+ [&] {
					std::chrono::zoned_time zonedTimestamp{ std::chrono::current_zone(), std::chrono::system_clock::now() };
					auto truncated = std::chrono::time_point_cast<std::chrono::milliseconds>(zonedTimestamp.get_local_time());

					stringstream ss;
					ss << std::format("({:%y%m%d_%H%M%S})", truncated);

					return (std::string)ss.str();
				}();
				//���ʂ����b�N���ď�������
				const auto& devRezVal = devRez.GetAndLock();
				//Row�t�@�C���Ƃ��ĕۑ�����
				{
					ofstream ofs(filePathWithTimeStamp +".devrez", std::ios::binary);
					cereal::BinaryOutputArchive bin(ofs);
					bin(*devRezVal);
					devRez.unlock();
				}
				//�f����BMP�Ƃ��ď����o��
				WriteBmpOfCamera(devRezVal.operator*(), filePathWithTimeStamp);
			}

			//Accuracy�ł��\�����Ă݂�
			cout <<"Search accuracy In developping: "<< searchAccuracyOfTheDevelopping.GetAndLock()->str() << endl;
			searchAccuracyOfTheDevelopping.unlock();
		}




		//�\������ 3d 2d�̏�
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

#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"
#include "bmpLib/bmp.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "SimVis/";//����branch�̌��ʂ��i�[����t�H���_

using py = pythonRuntime;


//�v�f�����Y�ɂ���_�������Ă��邩�ǂ������`�F�b�N����
bool NaihouHantei(const uvec2& p ,const std::list<uvec2>& vs) {
	constexpr ureal naihouDist = 0.001;

	auto ite =vs.cbegin();//���������O
	ureal sumAngle = 0.;
	for (size_t i = 0; i < 6;i++) {
		//����̒��_���`�F�b�N
		auto nite = std::next(ite);

		//ite p��p nite�̊p�x�����߂�
		const uvec2 e0 = (*ite - p).normalized(),
			e1 = (*nite - p).normalized();
		
		sumAngle += acos(e0.dot(e1));
		ite++;
	}

	//���v��2pi���ۂ�
	if (fabs(2. * pi - fabs(sumAngle)) < naihouDist)return true;
	return false;
}

//2d�x�N�g���ɗv�f��������
uvec3 ExtendUvec2(const uvec2& v, const ureal& z) {
	return uvec3(v.x(), v.y(), z);
}
//arrow��vec6�ɕς���
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

//�v���W�F�N�^�[�������f�R�[�h����
void DeserializeProjRefraDic(const std::string& path) {
	//�w�b�_�����[�h����
	projRefraDicHeader header;
	{
		ifstream ifs(path + ".head", std::ios::binary);
		cereal::BinaryInputArchive iarch(ifs);

		iarch(header);

		cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
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

int main() {

	try {
		
		//�J�n�������L�^���ăX�^�[�g
		const auto startTimePoint = std::chrono::system_clock::now();
		cout << "Start: " << startTimePoint << endl;

		//python plt�p�̃x�N�g���n��
		const pyVecSeries<3> mlabSeries("mlabv");//mlab�v���b�g�p�Ɏg��3D�x�N�g���z��
		const pyVecSeries<3> nlensSeries("nl");//�v�f�����Y�̃O���b�h
		const pyVecSeries<2> pypltSeries("pypltv");//�񎟌��v���b�g�p
		const pyVecSeries<6> quiverSeries("mlabquiver");//�x�N�g����p
		const std::array<const string, 6> quiverPrefix = { "x","y","z","a","b","c" };

		constexpr bool drawSphere = false;//�����Y�{�[���T�`��`�悷��
		//Python���Z�b�g�A�b�v���Ă��烌���Y�{�[���̊T�`������
		constexpr ureal sphereRadius = 1.;
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
)", sphereResolution, sphereResolution, sphereRadius, sphereRadius, sphereRadius);
		};

		//�n�[�h�E�F�A�X�y�b�N
		constexpr size_t projectorFps = 22727;//�t���[�����[�g
		constexpr size_t numOfProjectionPerACycle = 1024;//������Ƃ̓��f��
		constexpr ureal rotationSpeed = projectorFps / (ureal)numOfProjectionPerACycle;//��]���x
		//�𑜓x
		constexpr size_t verticalDirectionResolution = sqrt_constexpr(numOfProjectionPerACycle);//�����𑜓x(����ɕ���ł��郌���Y�̐�)
		constexpr size_t horizontalDirectionResolution = numOfProjectionPerACycle / verticalDirectionResolution;//�����𑜓x�@�ʒu�����Y������̓��f��


		//�����Y�A���C���쐬�A�`�悷��
		//�Z�p�`�Ń^�C�����O����@�����s�������Ă����s���������Ċ���
		constexpr size_t lensNumInARow = verticalDirectionResolution;

		constexpr ureal rowAngle = 0.02706659 * 2.;//�s�̊p�x theta=atan(-1.5*1/(cos(theta)*sqrt(3))*1/lensnum)
		const ureal rowLength = 2. * pi * cos(rowAngle);//�s�̒���
		const ureal lensEdgeWidth = rowLength / (ureal)lensNumInARow / 2.;
		const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInARow;//�Z�p�`�̈�ς����V�t�g����
		const auto DesignedMapToMap = bitrans<Eigen::Rotation2D<ureal>>(Eigen::Rotation2D<ureal>(rowAngle));//�����Y�A���C���X�΂�����O����X�΂��������Ƃɂ���

		const ureal nodeLensRadius = 2. * lensEdgeWidth / sqrt(3.);//�v�f�����Y�`����쐬�@���̔��a
		const ureal nodeLensRadiusSq = pow(nodeLensRadius, 2);//�v�f�����Y���a�̓��
		const std::pair<size_t,size_t> nodeLensResolution = make_pair(5*2,5);//�v�f�����Y�̕�����
		constexpr size_t rowNum = 15;//��ɂ��Ă� �s�̐�

		//�v�f�����Y�̊T�`�͘Z�p�`�ɂȂ�͂�
		auto hexverticesNodelensOuter = [&] {
			auto hexvjunk = MakeHexagon(lensEdgeWidth);//�Z�p�`�̒��_
			hexvjunk.push_back(hexvjunk.front());//������邽�߂ɍŏ��̓_�𖖔��ɑ}��

			return hexvjunk;
		}();

		constexpr bool calcNodelenses = true;//�m�[�h�����Y�̈ʒu���v�Z���ă����Y�{�[�����`������
		constexpr bool drawNodelenses = calcNodelenses & false;//�v�f�����Y��`�悷��
		constexpr bool drawNodelensEdges = calcNodelenses & true;//�m�[�h�����Y�̘g����`�悷��

		//���̌v�Z�ŗv�f�����Y���X�g���킩���
		std::optional<std::unordered_map<std::pair<size_t, size_t>, sphereParam, HashPair>> nodelensParamsInBalllocal = std::nullopt;//�s�ԍ��@�����Y�ԍ����L�[�Ńp�����[�^��Balllocal�ŕۑ�����
		if(calcNodelenses){
			//opt�v�Z�ł��܂�
			nodelensParamsInBalllocal = decltype(nodelensParamsInBalllocal)::value_type();
			auto& nodelensParamsRez = nodelensParamsInBalllocal.value();

			//std::list<uleap>//�}�b�v���W�ł̃����Y���S
			for (std::decay<decltype(rowNum)>::type rd = 0; rd < rowNum; rd++) {
				const ureal tlati = eachRowsDistance * rd - (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//lati�����̌��݈ʒu
				const bool eachFlag = rd % 2;//���݂ɐ؂�ւ��t���O
				for (std::decay<decltype(lensNumInARow)>::type ld = 0; ld < lensNumInARow; ld++) {

					auto color = HsvToRgb({ uleap({0.,1.},ld / (ureal)lensNumInARow),1.,1. });//�F���s�����ɕς���
					//�Z�p�`�����߂�o�b�t�@���N���A
					ResetPyVecSeries(pypltSeries);

					//lonn�����̌��݈ʒu
					const ureal tlonn = uleap(PairMinusPlus(rowLength / 2.), ld / (ureal)lensNumInARow) + (eachFlag ? ((rowLength) / (ureal)lensNumInARow / 2.) : 0.);
					const auto localcenterInMapDesigned = uvec2(tlonn, tlati);//�v�f�����Y�̒��� ���[�J���}�b�v���W
					const auto localcenterInMap = DesignedMapToMap.prograte() * localcenterInMapDesigned;
					const auto localcenterInBalllocalPolar = MapToLocalPolar(localcenterInMap);
					const uvec3 localcenterInBalllocal = PolarToXyz(localcenterInBalllocalPolar);

					//�p�����[�^��o�^
					nodelensParamsRez[make_pair(rd, ld)] = sphereParam(localcenterInBalllocal, fabs(cos(localcenterInBalllocalPolar.y())));

					//�v�f�����Y��`�悵�Ă���
					
					//���ɋɍ��W�ŗv�f�����Y���v�Z����
					ResetPyVecSeries(nlensSeries);//�m�[�h�����Y
					for (std::decay<decltype(nodeLensResolution.second)>::type nlla = 0; nlla < nodeLensResolution.second; nlla++) {

						ResetPyVecSeries(mlabSeries);//mlabSeries�̓O���b�h�̈�s���i�[����
						for (std::decay<decltype(nodeLensResolution.first)>::type nllo = 0; nllo < nodeLensResolution.first; nllo++) {
							const uvec2 localpos(uleap(PairMinusPlus(pi), nllo / (ureal)(nodeLensResolution.first - 1)),
								uleap(PairMinusPlus(pi / 2.), nlla / (ureal)(nodeLensResolution.second - 1)));//�v�f�����Y���[�J���ł̋ɍ��W

							const uvec3 nodelensShape = (nodeLensRadius * fabs(cos(localcenterInBalllocalPolar.y()))) * PolarToXyz(localpos);//���ꂪ���ɂȂ�͂�
							AppendPyVecSeries(mlabSeries, nodelensShape + localcenterInBalllocal);
						}
						if (drawNodelenses)py::s("nlx.append(mlabvx)\nnly.append(mlabvy)\nnlz.append(mlabvz)\n");//����Ń��b�V���ɂȂ�Ǝv����₯��
					}

					if (drawNodelenses)py::sf("mlab.mesh(%s,color=(%f,%f,%f))", GetPySeriesForPlot(nlensSeries), color[0], color[1], color[2]);
					//���_��]�����ĕ`��
					ResetPyVecSeries(mlabSeries);
					for (const auto& v : hexverticesNodelensOuter) {
						const uvec2 designedVertex = (v + localcenterInMapDesigned);
						const uvec2 vertex = DesignedMapToMap.prograte() * designedVertex;//�X���ă}�b�v���W�ɂ���
						AppendPyVecSeries(pypltSeries, designedVertex);
						const auto polarpos = MapToLocalPolar(vertex);//���ɋɍ��W�𓾂�
						AppendPyVecSeries(mlabSeries, PolarToXyz(polarpos));
					}

					if (drawNodelensEdges)py::sf("plt.plot(%s,color=(%f,%f,%f))", GetPySeriesForPlot(pypltSeries), color[0], color[1], color[2]);
					if (drawNodelensEdges)py::sf("mlab.plot3d(%s,color=(%f,%f,%f),tube_radius=0.01)", GetPySeriesForPlot(mlabSeries), color[0], color[1], color[2]);
				}
			}
		};


		//�f�x���b�v�Z�N�V����
		//�܂��w�b�_��ǂݏo��
		const std::string framePath = R"(C:\local\user\lensball\lensball\resultsX\projectorFrames\)";
		const std::string dicHeaderPath = R"(C:\local\user\lensball\lensball\resultsX\HexBall\projRefRayMap.head)";//�����w�b�_�����܂��Ă���ꏊ
		projRefraDicHeader header;
		{
			ifstream ifs(dicHeaderPath, std::ios::binary);
			cereal::BinaryInputArchive iarch(ifs);

			iarch(header);

			cout << "Loaded header\nh: " << header.horizontalRes << "\nv: " << header.verticalRes << "\nt: " << header.rotationRes << endl;
		}

		//�𑜓x�Ƃ����킩��
		//���Ɏx�X���ƂɃ��C�g���[�X���Ăǂ̉�f�ɓ����邩���ׂ���
		const arrow3 devTargetRayInGlobal(uvec3(0,-5,0),uvec3(0,1,0));//�����Ō������� �O���[�o�����W�n
		for (size_t rd = 0; rd < header.rotationRes; rd++) {
			//�܂��̓t���[����ǂݏo��
			const auto thisFrame = make_unique<bmpLib::img>();
			bmpLib::ReadBmp((framePath + "frame" + to_string(rd) + ".bmp").c_str(), thisFrame.get());

			//���Ƀ��[�J���O���[�o���ϊ����v�Z����
			const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(numOfProjectionPerACycle)) + (2. * pi / (ureal)(numOfProjectionPerACycle + 1) / 2.);//�{�[���̉�]�p�x
			const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//�O���[�o�����烌���Y�{�[�����[�J���ւ̕ϊ� XYZ���W

			//���C�̃��[�J�����v�Z�ł���
			const arrow3 targetInBalllocal(GlobalToBallLocal.prograte() * devTargetRayInGlobal.org(), GlobalToBallLocal.prograte() * devTargetRayInGlobal.dir());

			//���C�̑�܂��Ȓ��e�_���v�Z����@���ꂩ��
		}



		//�X�L����������
		constexpr size_t projectorResInTheta = 256;//�v���W�F�N�^�̏c���𑜓x �z���g��XGA
		constexpr ureal projectorHalfAngleTheta = 60. / 180. * pi;//�v���W�F�N�g�̓��f�p
		constexpr size_t projectorResInPhi = 256; // �v���W�F�N�^�̉����𑜓x �z���g��XGA
		constexpr ureal projectorHalfAnglePhi = projectorHalfAngleTheta * (projectorResInPhi / (ureal)projectorResInTheta);//�v���W�F�N�g�̓��f�p
		const ureal nodeLensFocalLength = nodeLensRadius * 1.5;//�v�f�����Y�̒��S����œ_�܂ł̋���

		//std::unordered_map<Eigen::Vector3i, arrow3> projectorRefractRayMap;

		constexpr size_t scanThreadNum = 16;//�X�L�����Ɏg���X���b�h��
		const std::string resultDicPrefix = "projRefRayMap";

		constexpr bool scanLenses = false;//�����Y�{�[���ɑ΂��郌�C�g���[�V���O���s��
		constexpr bool drawRefractionDirectionOfARay = false;//���郌�C�̋��ܕ�����`�悷��
		constexpr bool logWarningInScan = false;//scan���̌x����\������

		bool onetime = false;

		if(scanLenses){

			//���񏈗��p�̂��낢��
			std::array<uptr<std::thread>, scanThreadNum> scanThreads;//���s�X���b�h
			std::array<bool, scanThreadNum> finFlagOfEachScanThread;//�X�L����������������Ƃ��

			//�i�[���ʂ̌`���������w�b�_������ĕۑ�����
			const projRefraDicHeader storageheader(projectorResInPhi, projectorResInTheta, numOfProjectionPerACycle);
			storageheader.SaveHeader(rezpath + branchpath + resultDicPrefix);

			ResetPyVecSeries<6>(quiverSeries,quiverPrefix);//�x�N�g��������|��
			ResetPyVecSeries(pypltSeries);//�x�N�g��������|��


			//�w�肳�ꂽ���ʂ��w�肳�ꂽ�X�g���[�W�֑��M���� �]���������Ƃ�Rez�̓N���A����܂�
			const auto TransRezToStorage = [&](std::list<arrow3>& rezOfSt, ofstream& storageofStPtr) {
				cereal::BinaryOutputArchive o_archive(storageofStPtr);
				o_archive(rezOfSt);

				rezOfSt.clear();
			};
			//��]�p�x���ƂɃX�L�������s��
			const auto ScanAScene = [&](const std::decay<decltype(numOfProjectionPerACycle)>::type rd, decltype(finFlagOfEachScanThread)::iterator finflagOfStIte) {

				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(numOfProjectionPerACycle)) + (2. * pi / (ureal)(numOfProjectionPerACycle + 1) / 2.);//�{�[���̉�]�p�x
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//�O���[�o�����烌���Y�{�[�����[�J���ւ̕ϊ� XYZ���W

				//���ʊi�[�������ƃX�g���[�W��p�ӂ���
				std::list<arrow3> rezMem;
				std::ofstream storageStream(rezpath + branchpath + resultDicPrefix + ".part" + to_string(rd), std::ios::binary);

				//�e�s�N�Z�������яo�郌�C�Ɖ�]�p�xrd�̋��Ƃ̓����蔻����s��
				for (std::decay<decltype(projectorResInPhi)>::type hpd = 0; hpd < projectorResInPhi; hpd++) {
					const ureal rayPhiInGlobal = uleap(PairMinusPlus(projectorHalfAnglePhi), hpd / (ureal)(projectorResInPhi - 1));//�v���W�F�N�^���W�n�ł̒��ډ�f����ł郌�C�̃�

					for (std::decay<decltype(projectorResInTheta)>::type vpd = 0; vpd < projectorResInTheta; vpd++) {//�v���W�F�N�^�̒��ډ�f���Ƃ�
						const ureal rayThetaInGlobal = uleap(PairMinusPlus(projectorHalfAngleTheta), vpd / (ureal)(projectorResInTheta - 1));//�v���W�F�N�^���W�n�ł̒��ډ�f����ł郌�C�̃�
						const uvec3 rayDirInGlobal = PolarToXyz(uvec2(rayPhiInGlobal, rayThetaInGlobal));

						const uvec2 rayDirInBallInBalllocalPolar(rayPhiInGlobal - ballRotation, rayThetaInGlobal);//�{�[���̉�]�p�x�悩��{�[�����[�J���ł̃��C�̕���(�ɍ��W���킩��)
						const uvec2 rayDirInMap = PolarToMap(rayDirInBallInBalllocalPolar);//�}�b�v���W�͂��� �X��������
						const uvec2 rayDirInMapDesigned = (DesignedMapToMap.untiprograte()) * rayDirInMap;//�X����O �f�U�C���}�b�v


						//py::sf("plt.scatter(%f,%f,color=(0,0,0))", rayDirInBallLocalMapDesigned.x(), rayDirInBallLocalMapDesigned.y());
						//�f�U�C���}�b�v�̃V�[�^����row���킩��
						//const ureal tlati = eachRowsDistance * rd-(eachRowsDistance*(ureal)(rowNum-1)/2.);//lati�����̌��݈ʒu
						//�V�[�^��-eachRowsDistance*(ureal)(rowNum-1)/2~eachRowDistance*(rownum-1)/2�܂�
						//�܂�rayDirInMapD��latitude�𐳋K������ ��ԉ���row�̍������傤�ǂ̂Ƃ� 0�@��ԏ��row�̍������傤�ǂ̂Ƃ�rowNum-1�ɂȂ�悤�ɂ���
						const ureal regRayDirLati = [&] {
							const ureal zerothRowlati = -(eachRowsDistance * (ureal)(rowNum - 1) / 2.);//zero�Ԗڂ̍s�̍���
							const ureal zeroSetRayDir = rayDirInMapDesigned.y() - zerothRowlati;//0�Ԗڂ̍s�̍����Ɏn�_�����킹�����C�̍���
							const ureal finalRowlati = (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//rowNum-1�Ԗڂ̍s�̍���
							//�X�P�[�����O�@fin~0�܂ł̃X�P�[����rowNum-1~0�܂ł̃X�P�[���ɂȂ��ė~����
							const ureal thisscale = (ureal)(rowNum - 1 - 0) / (finalRowlati - zerothRowlati);

							return thisscale * zeroSetRayDir;
						}();


						const int centerRawIndex = round(regRayDirLati);//�l�̌ܓ�����Ƃ����Ƃ��炵���C���f�b�N�X���킩��
						const int neibourRawIndex = (regRayDirLati - (ureal)centerRawIndex) > 0. ? centerRawIndex + 1 : centerRawIndex - 1;//�ׂ荇���s�̂����Ƃ��炵���C���f�b�N�X���킩��
						//printf("row index %d,%d\n", centerRawIndex, neibourRawIndex);

						//�ł͂�������s���̓����蔻����n�߂�
						optional<std::pair<uvec2, uvec2>> hitLensCenterAndHitDistInMapDesigned;//�Ώۂ̃����Y�̒��S�ʒu
						for (const auto& rid : { centerRawIndex,neibourRawIndex }) {

							//����phi���牽�Ԗڂ̃����Y�����l����
							const ureal regRayDirLonn = [&] {
								//const ureal tlonn = uleap(PairMinusPlus(rowLength/2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);

								const ureal zerothlenslonn = -rowLength / 2.;//-rowLength/2.���ŏ��̃����Y�̈ʒu�@�ꍇ�ɂ���đO�シ�邯�ǂ�
								const ureal zeroSetRayDir = rayDirInMapDesigned.x() - zerothlenslonn;//zero�Ԗڂ̃����Y�̏ꏊ��zero��
								const ureal finlenslonn = uleap(PairMinusPlus(rowLength / 2.), (lensNumInARow - 1) / (ureal)lensNumInARow);//�Ō�̃����Y�̏ꏊ
								//�X�P�[�����O�@fin~0�܂ł̃X�P�[����lensNumInCollum-1~0�܂ł̃X�P�[���ɂȂ��ė~����
								const ureal thisscale = (ureal)(lensNumInARow - 1 - 0) / (finlenslonn - zerothlenslonn);

								return thisscale * zeroSetRayDir;
							}();
							const bool eachFlag = rid % 2;//���݂ɐ؂�ւ��t���O �����Ă���Ƃ��͍s�������i��ł�

							const int centerLensIndex = round(regRayDirLonn - (eachFlag ? 0.5 : 0.));//����̓I�t�Z�b�g���Ȃ��@�܂���}�C�i�X������n�܂��Ă���s�ɂ���ꍇ�̃C���f�b�N�X �����łȂ����-0.5���Ă���ۂ߂適���܂���
							const int neibourLensIndex = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? centerLensIndex + 1 : centerLensIndex - 1;//�ׂ荇�������Y�̂����Ƃ��炵���C���f�b�N�X���킩��
							//printf("lens index %d,%d\n", centerLensIndex, neibourLensIndex);

							//�ł̓����Y�̓����蔻����n�߂�
							for (const auto& lid : { centerLensIndex,neibourLensIndex }) {
								//����Ń����Y�̏ꏊ���m�肷��͂�
								const auto hitlensCenterInMapDesigned = uvec2(uleap(PairMinusPlus(rowLength / 2.), lid / (ureal)lensNumInARow) + (eachFlag ? ((rowLength) / (ureal)lensNumInARow / 2.) : 0.), eachRowsDistance * rid - (eachRowsDistance * (ureal)(rowNum - 1) / 2.));
								const uvec2 hitDistInMapDesigned = rayDirInMapDesigned - hitlensCenterInMapDesigned;//���΃q�b�g�ʒu �������}�b�vD��
								//�{�������� �Ƃ肠�����T�`�̒��ɓ����Ă��邩�ǂ����𔻒�
								if (NaihouHantei(hitDistInMapDesigned, hexverticesNodelensOuter)) {
									if (hitDistInMapDesigned.norm() >= nodeLensRadius)throw logic_error("���肪����");
									//printf("ok r=%d l=%d\n", rid, lid);

									//�Ƃ肠�����e�X�g�@���ꂪcenter�łȂ��\���͂����?
									if (rid != centerRawIndex || lid != centerLensIndex)
										if (logWarningInScan)cout << "�ʂɃG���[�ł͂Ȃ����Ǒz�������̂ƈႤ!" << endl;

									hitLensCenterAndHitDistInMapDesigned = make_pair(hitlensCenterInMapDesigned, hitDistInMapDesigned);
									break;
								}
							}

							if (hitLensCenterAndHitDistInMapDesigned)break;
						}

						//�����܂łŌ��ʂ��o�Ȃ��Ƃ�΂�
						if (!hitLensCenterAndHitDistInMapDesigned)throw std::runtime_error("�v�f�����Y�̌����Ɏ��s");

						//�œ_�̏ꏊ�͗v�f�����Y���m�肷��Όv�Z�ł���
						const auto focalposInBalllocalXYZ = Polar3DToXyz(ExtendUvec2(MapToLocalPolar(DesignedMapToMap.prograte() * hitLensCenterAndHitDistInMapDesigned.value().first), sphereRadius + nodeLensFocalLength));

						const auto hitPosInBalllocalXYZ = PolarToXyz(rayDirInBallInBalllocalPolar);//�������{�[�����[�J���ł̃q�b�g�_
						const auto refractRayDirInBalllocalXYZ = (focalposInBalllocalXYZ - hitPosInBalllocalXYZ);//�Փ˓_�Əœ_�̈ʒu��������Ό��̕������킩��ˁ@�b��I��
						//������O���[�o���ɖ߂�
						const uvec3 refractRayDirInGlobal = GlobalToBallLocal.untiprograte() * refractRayDirInBalllocalXYZ;

						//���ʂ�ǉ�
						rezMem.push_back(arrow3(rayDirInGlobal, refractRayDirInGlobal));

						//�v���b�g���܂��@�q�b�g�|�C���g��
						if (vpd == 0 && drawRefractionDirectionOfARay) {
							const auto color = HsvToRgb({ rd / (ureal)(numOfProjectionPerACycle - 1),1.,1. });
							py::sf("mlab.quiver3d(0,0,0,%f,%f,%f,mode=\"arrow\",color=(%f,%f,%f))", refractRayDirInGlobal.x(), refractRayDirInGlobal.y(), refractRayDirInGlobal.z(), color[0], color[1], color[2]);
							py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", hitLensCenterAndHitDistInMapDesigned.value().second.x(), hitLensCenterAndHitDistInMapDesigned.value().second.y(), color[0], color[1], color[2]);
						}
					}
				}

				//�X�L�������I�������Z�[�u
				TransRezToStorage(rezMem, storageStream);

				*finflagOfStIte = true;
			};

			//�����X���b�h�ɉ�]�p�x��ς��Ȃ��犄�蓖�Ă�
			for (std::decay<decltype(numOfProjectionPerACycle)>::type rdgen = 0; rdgen < numOfProjectionPerACycle; rdgen++) {
				cout << "count: " << rdgen << endl;
				//����rdgen�ł̏������J���Ă���X���b�h�Ɋ���t������
				bool isfound = false;
				while (!isfound) {//����t�����Ȃ���ΌJ��Ԃ�
					for (size_t th=0;th<scanThreadNum;th++)
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
			for (size_t th = 0; th < scanThreadNum; th++) {

				//�܂��J������Ă��Ȃ����join���ĊJ������
				if (scanThreads.at(th)) {
					scanThreads.at(th).get()->join();
					scanThreads.at(th).release();
				}
			}
		};
		
		//�f�R�[�h����
		//DeserializeProjRefraDic(rezpath + branchpath + resultDicPrefix);

		//�\������ 3d 2d�̏�
		//py::s("mlab.show()");
		//py::s("plt.show()");


		const auto endTimePoint = std::chrono::system_clock::now();
		cout << "Finish: " << endTimePoint << endl;
		cout << "Process time: " << std::chrono::duration_cast<std::chrono::milliseconds>(endTimePoint - startTimePoint).count() / 1000. << " [s]" << endl;
		cout << "The work is complete...Wait rendering by Python" << endl;
		py::Terminate();
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//�Ȃ񂩓��ꂽ��I���

		py::Terminate();
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		cout << "ERROR: " << std::chrono::system_clock::now() << endl;

		system("pause");//�Ȃ񂩓��ꂽ��I���

		py::Terminate();
		return -2;
	}

	return 0;
}
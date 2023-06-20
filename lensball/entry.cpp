#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "HexBall/";//����branch�̌��ʂ��i�[����t�H���_

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

int main() {

	try {
		//python plt�p�̃x�N�g���n��
		const pyVecSeries<3> mlabSeries("mlabv");//mlab�v���b�g�p�Ɏg��3D�x�N�g���z��
		const pyVecSeries<3> nlensSeries("nl");//�v�f�����Y�̃O���b�h
		const pyVecSeries<2> pypltSeries("pypltv");//�񎟌��v���b�g�p
		const pyVecSeries<6> quiverSeries("mlabquiver");//�x�N�g����p
		const std::array<const string, 6> quiverPrefix = { "x","y","z","a","b","c" };

		constexpr bool drawSphere = true;//�����Y�{�[���T�`��`�悷��
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



		//�����Y�A���C���쐬�A�`�悷��
		//�Z�p�`�Ń^�C�����O����@�����s�������Ă����s���������Ċ���
		constexpr size_t lensNumInCollum = 20;

		constexpr ureal rowAngle = 0.04331481 * 2.;//�s�̊p�x
		const ureal rowLength = 2. * pi * cos(rowAngle);//�s�̒���
		const ureal lensEdgeWidth = rowLength / (ureal)lensNumInCollum / 2.;
		const ureal eachRowsDistance = 1.5 * rowLength / sqrt(3.) / (ureal)lensNumInCollum;//�Z�p�`�̈�ς����V�t�g����
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

		constexpr bool calcNodelenses = false;//�m�[�h�����Y�̈ʒu���v�Z���ă����Y�{�[�����`������
		constexpr bool drawNodelenses = calcNodelenses & true;//�v�f�����Y��`�悷��
		constexpr bool drawNodelensEdges = calcNodelenses & true;//�m�[�h�����Y�̘g����`�悷��
		if(calcNodelenses){
			//std::list<uleap>//�}�b�v���W�ł̃����Y���S
			for (std::decay<decltype(rowNum)>::type rd = 0; rd < rowNum; rd++) {
				const ureal tlati = eachRowsDistance * rd - (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//lati�����̌��݈ʒu
				const bool eachFlag = rd % 2;//���݂ɐ؂�ւ��t���O
				for (std::decay<decltype(lensNumInCollum)>::type ld = 0; ld < lensNumInCollum; ld++) {

					auto color = HsvToRgb({ uleap({0.,1.},ld / (ureal)lensNumInCollum),1.,1. });//�F���s�����ɕς���
					//�Z�p�`�����߂�o�b�t�@���N���A
					ResetPyVecSeries(pypltSeries);

					//lonn�����̌��݈ʒu
					const ureal tlonn = uleap(PairMinusPlus(rowLength / 2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);
					const auto localcenterInMapDesigned = uvec2(tlonn, tlati);//�v�f�����Y�̒��� ���[�J���}�b�v���W
					const auto localcenterInMap = DesignedMapToMap.prograte() * localcenterInMapDesigned;
					const auto localcenterInBalllocalPolar = MapToLocalPolar(localcenterInMap);
					const uvec3 localcenterInBalllocal = PolarToXyz(localcenterInBalllocalPolar);

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


		//�X�L����������
		constexpr size_t projectorResInTheta = 720;//�v���W�F�N�^�̏c����𑜓x
		constexpr ureal projectorHalfAngle = 60. / 180. * pi;//�v���W�F�N�g�̓��f�p
		constexpr size_t numOfProjectionPerACycle = 720;//���]�ł̓��e��
		const ureal nodeLensFocalLength = nodeLensRadius * 1.5;//�v�f�����Y�̒��S����œ_�܂ł̋���

		constexpr size_t scanThreadNum = 19;//�X�L�����Ɏg���X���b�h��
		std::array<uptr<std::thread>, scanThreadNum> scanThreads;//���s�X���b�h
		std::array<std::atomic_bool, scanThreadNum> scanThreadIsFin;//������񂪂���������Ƃ��

		constexpr bool scanLenses = true;//�����Y�{�[���ɑ΂��郌�C�g���[�V���O���s��
		constexpr bool drawRefractionDirectionOfARay = false;//���郌�C�̋��ܕ�����`�悷��
		if(scanLenses){
			ResetPyVecSeries<6>(quiverSeries,quiverPrefix);//�x�N�g��������|��
			ResetPyVecSeries(pypltSeries);//�x�N�g��������|��

			//��]�p�x���ƂɃX���b�h������t����
			const auto scanAScene = [&](const std::decay<decltype(numOfProjectionPerACycle)>::type rd, decltype(scanThreadIsFin)::iterator finflag) {

				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(numOfProjectionPerACycle)) + (2. * pi / (ureal)(numOfProjectionPerACycle + 1) / 2.);//�{�[���̉�]�p�x
				const bitrans<Eigen::AngleAxis<ureal>> GlobalToBallLocal(Eigen::AngleAxis<ureal>(-ballRotation, uvec3::UnitZ()));//�O���[�o�����烌���Y�{�[�����[�J���ւ̕ϊ� XYZ���W

				for (std::decay<decltype(projectorResInTheta)>::type pd = 0; pd < projectorResInTheta; pd++) {//�v���W�F�N�^�̒��ډ�f���Ƃ�
					const ureal rayThetaInGlobal = uleap(PairMinusPlus(projectorHalfAngle), pd / (ureal)(projectorResInTheta - 1));//�v���W�F�N�^���W�n�ł̒��ډ�f����ł郌�C�̃�

					const uvec2 rayDirInBallInBalllocalPolar(-ballRotation, rayThetaInGlobal);//�{�[���̉�]�p�x�悩��{�[�����[�J���ł̃��C�̕���(�ɍ��W���킩��)
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
							const ureal finlenslonn = uleap(PairMinusPlus(rowLength / 2.), (lensNumInCollum - 1) / (ureal)lensNumInCollum);//�Ō�̃����Y�̏ꏊ
							//�X�P�[�����O�@fin~0�܂ł̃X�P�[����lensNumInCollum-1~0�܂ł̃X�P�[���ɂȂ��ė~����
							const ureal thisscale = (ureal)(lensNumInCollum - 1 - 0) / (finlenslonn - zerothlenslonn);

							return thisscale * zeroSetRayDir;
						}();
						const bool eachFlag = rid % 2;//���݂ɐ؂�ւ��t���O �����Ă���Ƃ��͍s�������i��ł�

						const int centerLensIndex = round(regRayDirLonn - (eachFlag ? 0.5 : 0.));//����̓I�t�Z�b�g���Ȃ��@�܂���}�C�i�X������n�܂��Ă���s�ɂ���ꍇ�̃C���f�b�N�X �����łȂ����-0.5���Ă���ۂ߂適���܂���
						const int neibourLensIndex = (regRayDirLonn - (ureal)centerLensIndex) > 0. ? centerLensIndex + 1 : centerLensIndex - 1;//�ׂ荇�������Y�̂����Ƃ��炵���C���f�b�N�X���킩��
						//printf("lens index %d,%d\n", centerLensIndex, neibourLensIndex);

						//�ł̓����Y�̓����蔻����n�߂�
						for (const auto& lid : { centerLensIndex,neibourLensIndex }) {
							//����Ń����Y�̏ꏊ���m�肷��͂�
							const auto hitlensCenterInMapDesigned = uvec2(uleap(PairMinusPlus(rowLength / 2.), lid / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.), eachRowsDistance * rid - (eachRowsDistance * (ureal)(rowNum - 1) / 2.));
							const uvec2 hitDistInMapDesigned = rayDirInMapDesigned - hitlensCenterInMapDesigned;//���΃q�b�g�ʒu �������}�b�vD��
							//�{�������� �Ƃ肠�����T�`�̒��ɓ����Ă��邩�ǂ����𔻒�
							if (NaihouHantei(hitDistInMapDesigned, hexverticesNodelensOuter)) {
								if (hitDistInMapDesigned.norm() >= nodeLensRadius)throw logic_error("���肪����");
								//printf("ok r=%d l=%d\n", rid, lid);

								//�Ƃ肠�����e�X�g�@���ꂪcenter�łȂ��\���͂����?
								if (rid != centerRawIndex || lid != centerLensIndex)std::runtime_error("�ʂɃG���[�ł͂Ȃ����Ǒz�������̂ƈႤ!");

								hitLensCenterAndHitDistInMapDesigned = make_pair(hitlensCenterInMapDesigned, hitDistInMapDesigned);
								break;
							}
						}

						if (hitLensCenterAndHitDistInMapDesigned)break;
					}

					//�����܂łŌ��ʂ��o�Ȃ��Ƃ�΂�
					if (!hitLensCenterAndHitDistInMapDesigned)std::runtime_error("�v�f�����Y�̌����Ɏ��s");

					//�œ_�̏ꏊ�͗v�f�����Y���m�肷��Όv�Z�ł���
					const auto focalposInBalllocalXYZ = Polar3DToXyz(ExtendUvec2(MapToLocalPolar(DesignedMapToMap.prograte() * hitLensCenterAndHitDistInMapDesigned.value().first), sphereRadius + nodeLensFocalLength));

					const auto hitPosInBalllocalXYZ = PolarToXyz(rayDirInBallInBalllocalPolar);//�������{�[�����[�J���ł̃q�b�g�_
					const auto refractRayDirInBalllocalXYZ = (focalposInBalllocalXYZ - hitPosInBalllocalXYZ);//�Փ˓_�Əœ_�̈ʒu��������Ό��̕������킩��ˁ@�b��I��
					//������O���[�o���ɖ߂�
					const uvec3 refractRayDirInGlobal = GlobalToBallLocal.untiprograte() * refractRayDirInBalllocalXYZ;

					//�v���b�g���܂��@�q�b�g�|�C���g��
					if (pd == 0 && drawRefractionDirectionOfARay) {
						const auto color = HsvToRgb({ rd / (ureal)(numOfProjectionPerACycle - 1),1.,1. });
						py::sf("mlab.quiver3d(0,0,0,%f,%f,%f,mode=\"arrow\",color=(%f,%f,%f))", refractRayDirInGlobal.x(), refractRayDirInGlobal.y(), refractRayDirInGlobal.z(), color[0], color[1], color[2]);
						py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", hitLensCenterAndHitDistInMapDesigned.value().second.x(), hitLensCenterAndHitDistInMapDesigned.value().second.y(), color[0], color[1], color[2]);
					}
				}

				*finflag = true;
			};

			for (std::decay<decltype(numOfProjectionPerACycle)>::type rdgen = 0; rdgen < numOfProjectionPerACycle; rdgen++) {

				//����rdgen�ł̏������J���Ă���X���b�h�Ɋ���t������
				bool isfound = false;
				while (!isfound) {//����t�����Ȃ���ΌJ��Ԃ�
					for (size_t t=0;t<scanThreadNum;t++)
						if (!scanThreads.at(t)) {//�󂫂Ȃ犄�t
							if (!isfound) {//��̃C���f�b�N�X�ɂ͈�񂾂�����t����
								isfound = true;
								scanThreadIsFin.at(t) = false;//�t���O���Z�b�g

								scanThreads.at(t).reset(new std::thread(scanAScene, rdgen, scanThreadIsFin.begin() + t));//�X���b�h���s�J�n
							}
						}
						else if (scanThreadIsFin.at(t)) {//�󂢂ĂȂ��ďI����Ă�Ȃ�
							scanThreads.at(t).get()->join();
							scanThreads.at(t).release();//���\�[�X���J��
						}
				}
			}

			//�S���̃X���b�h���I����Ă��邱�Ƃ��m�F
			for (auto& t : scanThreads)
				if (t) {
					t.get()->join();
					t.release();
				}
		};
		

		//�\������ 3d 2d�̏�
		py::s("mlab.show()");
		py::s("plt.show()");
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		system("pause");//�Ȃ񂩓��ꂽ��I���

		py::Terminate();
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		system("pause");//�Ȃ񂩓��ꂽ��I���

		py::Terminate();
		return -2;
	}

	cout << "The work is complete...Wait rendering by Python" << endl;
	py::Terminate();
	return 0;
}
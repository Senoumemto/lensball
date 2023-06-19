#include <iostream>
#include "application.hpp"
#include "matplotwrapper.hpp"
#include "general.hpp"

using namespace std;

const std::string rezpath = "C:/local/user/lensball/lensball/resultsX/";//���ʂ��i�[����t�H���_
const std::string branchpath = "SphereCoord/";//����branch�̌��ʂ��i�[����t�H���_

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

int main() {

	try {
		//python plt�p�̃x�N�g���n��
		const pyVecSeries<3> mlabSeries("mlabv");//mlab�v���b�g�p�Ɏg��3D�x�N�g���z��
		const pyVecSeries<3> nlensSeries("nl");//�v�f�����Y�̃O���b�h
		const pyVecSeries<2> pypltSeries("pypltv");//�񎟌��v���b�g�p


		//Python���Z�b�g�A�b�v���Ă��烌���Y�{�[���̊T�`������
		constexpr ureal sphereRadius = 1.;
		{

			//python�����^�C�����������Ă��낢�돉������
			py::Init();

			py::s("import numpy as np\nfrom mayavi import mlab\nimport matplotlib.pyplot as plt");

			//mayavi�̐ݒ�
			const std::pair<size_t, size_t> figResolution(800, 600);
			py::sf("fig = mlab.figure( size=(%d,%d), bgcolor=(0,0,0) )", figResolution.first, figResolution.second);

			//matplotlib�̐ݒ�
			py::s("fig, ax = plt.subplots()\nax.set_aspect(\"equal\")");

			//����`�悷��
			constexpr size_t sphereResolution = 20;
			py::sf(R"(
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
		if(1){
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
					const auto localcenter = uvec2(tlonn, tlati);//�v�f�����Y�̒��� ���[�J���}�b�v���W

					//�v�f�����Y��`�悵�Ă���
					
					//���ɋɍ��W�ŗv�f�����Y���v�Z����
					std::list<uvec3> nodeLensVertices;
					//ResetPyVecSeries(mlabSeries);
					ResetPyVecSeries(nlensSeries);//�m�[�h�����Y
					for (std::decay<decltype(nodeLensResolution.second)>::type nlla = 0; nlla < nodeLensResolution.second; nlla++) {

						ResetPyVecSeries(mlabSeries);//mlabSeries�̓O���b�h�̈�s���i�[����
						for (std::decay<decltype(nodeLensResolution.first)>::type nllo = 0; nllo < nodeLensResolution.first; nllo++) {
							const uvec2 localpos(uleap(PairMinusPlus(pi), nllo / (ureal)(nodeLensResolution.first - 1)),
								uleap(PairMinusPlus(pi / 2.), nlla / (ureal)(nodeLensResolution.second - 1)));//�v�f�����Y���[�J���ł̋ɍ��W

							const uvec3 nodelensShape = nodeLensRadius * PolarToXyz(localpos);//���ꂪ�~�ɂȂ�͂�
							const uvec2 nodelensGrobalMap = (DesignedMapToMap.prograte()) * (uvec2(nodelensShape.x(), nodelensShape.y()) + localcenter);//�}�b�v���[�J���ł̗v�f�����Y


							const auto polarpos = MapToPolar(nodelensGrobalMap);//���Ƀ��[�J���ɍ��W�𓾂�
							const auto localHeight = nodelensShape.z() * cos(polarpos.y());//���[�J�����W�ō��������߂�
							const auto globalpos = Polar3DToXyz(uvec3(polarpos.x(), polarpos.y(), sphereRadius + localHeight));//xyz���W�n�ł̈ʒu���v�Z
							AppendPyVecSeries(mlabSeries, globalpos);
						}
						py::s("nlx.append(mlabvx)\nnly.append(mlabvy)\nnlz.append(mlabvz)\n");//����Ń��b�V���ɂȂ�Ǝv����₯��
					}

					py::sf("mlab.mesh(%s,color=(%f,%f,%f))", GetPySeriesForPlot(nlensSeries), color[0], color[1], color[2]);
					//���_��]�����ĕ`��
					ResetPyVecSeries(mlabSeries);
					for (const auto& v : hexverticesNodelensOuter) {
						const uvec2 designedVertex = (v + localcenter);
						const uvec2 vertex = DesignedMapToMap.prograte() * designedVertex;//�X���ă}�b�v���W�ɂ���
						AppendPyVecSeries(pypltSeries, designedVertex);
						const auto polarpos = MapToPolar(vertex);//���ɋɍ��W�𓾂�
						AppendPyVecSeries(mlabSeries, PolarToXyz(polarpos));
					}

					py::sf("plt.plot(%s,color=(0,0,0))", GetPySeriesForPlot(pypltSeries));
					py::sf("mlab.plot3d(%s,color=(%f,%f,%f),tube_radius=0.01)", GetPySeriesForPlot(mlabSeries), color[0], color[1], color[2]);
				}
			}
		};


		//�X�L����������
		constexpr size_t projectorResInTheta = 2;//�v���W�F�N�^�̏c����𑜓x
		constexpr ureal projectorHalfAngle = 60. / 180. * pi;//�v���W�F�N�g�̓��f�p
		constexpr size_t numOfProjectionPerACycle = 720;//���]�ł̓��e��
		const ureal nodeLensFocalLength = nodeLensRadius * 1.5;//�v�f�����Y�̒��S����œ_�܂ł̋���
		{
			for (std::decay<decltype(numOfProjectionPerACycle)>::type rd = 0; rd < numOfProjectionPerACycle; rd++) {
				const ureal ballRotation = uleap(PairMinusPlus(pi), rd / (ureal)(numOfProjectionPerACycle)) + (2. * pi / (ureal)(numOfProjectionPerACycle + 1) / 2.);//�{�[���̉�]�p�x
				for (std::decay<decltype(projectorResInTheta)>::type pd = 0; pd < projectorResInTheta; pd++) {//�v���W�F�N�^�̒��ډ�f
					const ureal rayThetaInProjectorLocal = uleap(PairMinusPlus(projectorHalfAngle), pd / (ureal)(projectorResInTheta - 1));//�v���W�F�N�^���W�n�ł̒��ډ�f����ł郌�C�̃�

					const uvec2 rayDirInBallLocalPolar(-ballRotation, rayThetaInProjectorLocal);//�{�[���̉�]�p�x�悩��{�[�����[�J���ł̃��C�̕���(�ɍ��W���킩��)
					const uvec2 rayDirInBallLocalMap = PolarToMap(rayDirInBallLocalPolar);//�}�b�v���W�͂��� �X��������
					const uvec2 rayDirInBallLocalMapDesigned = (DesignedMapToMap.untiprograte()) * rayDirInBallLocalMap;//�X����O �f�U�C���}�b�v
					
					//�f�U�C���}�b�v�̃V�[�^����row���킩��
					//const ureal tlati = eachRowsDistance * rd-(eachRowsDistance*(ureal)(rowNum-1)/2.);//lati�����̌��݈ʒu
					//�V�[�^��-eachRowsDistance*(ureal)(rowNum-1)/2~eachRowDistance*(rownum-1)/2�܂�
					//�܂������ɐ��K������ ��ԉ���row�̍������傤�ǂ̂Ƃ� 0�@��ԏ��row�̍������傤�ǂ̂Ƃ�rowNum-1�ɂȂ�悤�ɂ���
					const ureal regRayDirLati = [&] {
						const ureal zerothRowlati = -(eachRowsDistance * (ureal)(rowNum - 1) / 2.);//zero�Ԗڂ̍s�̍���
						const ureal zeroSetRayDir = rayDirInBallLocalMapDesigned.y() - zerothRowlati;//0�Ԗڂ̍s�̍����Ɏn�_�����킹�����C�̍���
						const ureal finalRowlati = (eachRowsDistance * (ureal)(rowNum - 1) / 2.);//rowNum-1�Ԗڂ̍s�̍���
						//�X�P�[�����O�@fin~0�܂ł̃X�P�[����rowNum-1~0�܂ł̃X�P�[���ɂȂ��ė~����
						const ureal thisscale = (ureal)(rowNum - 1 - 0) / (finalRowlati - zerothRowlati);

						return thisscale * zeroSetRayDir;
					}();

					
					const int centerRawIndex = round(regRayDirLati);//�l�̌ܓ�����Ƃ����Ƃ��炵���C���f�b�N�X���킩��
					const int neibourRawIndex = (regRayDirLati - (ureal)centerRawIndex) > 0. ? centerRawIndex + 1 : centerRawIndex - 1;//�ׂ荇���s�̂����Ƃ��炵���C���f�b�N�X���킩��
					//printf("row index %d,%d\n", centerRawIndex, neibourRawIndex);
					
					//�ł͂�������s���̓����蔻����n�߂�
					optional<std::pair<uvec2, uvec2>> hitLensCenterAndHitDist;//�Ώۂ̃����Y�̒��S�ʒu
					for (const auto& rid : { centerRawIndex,neibourRawIndex }) {

						//����phi���牽�Ԗڂ̃����Y�����l����
						const ureal regRayDirLonn = [&] {
							//const ureal tlonn = uleap(PairMinusPlus(rowLength/2.), ld / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.);

							const ureal zerothlenslonn = -rowLength / 2.;//-rowLength/2.���ŏ��̃����Y�̈ʒu�@�ꍇ�ɂ���đO�シ�邯�ǂ�
							const ureal zeroSetRayDir = rayDirInBallLocalMapDesigned.x() - zerothlenslonn;//zero�Ԗڂ̃����Y�̏ꏊ��zero��
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
							const auto thiscenter = uvec2(uleap(PairMinusPlus(rowLength / 2.), lid / (ureal)lensNumInCollum) + (eachFlag ? ((rowLength) / (ureal)lensNumInCollum / 2.) : 0.), eachRowsDistance * rid - (eachRowsDistance * (ureal)(rowNum - 1) / 2.));
							const uvec2 hitDist = rayDirInBallLocalMapDesigned - thiscenter;//���΃q�b�g�ʒu
							//�{�������� �Ƃ肠�����T�`�̒��ɓ����Ă��邩�ǂ����𔻒�
							if (!NaihouHantei(hitDist, hexverticesNodelensOuter)) {

								//printf("ok r=%d l=%d\n", rid, lid);

								//�Ƃ肠�����e�X�g�@���ꂪcenter�łȂ��\���͂����?
								if (rid != centerRawIndex || lid != centerLensIndex)std::runtime_error("yayayaya!");

								hitLensCenterAndHitDist = make_pair(thiscenter, hitDist);
								break;
							}
						}

						if (hitLensCenterAndHitDist)break;
					}

					//�����܂łŌ��ʂ��o�Ȃ��Ƃ�΂�
					if (!hitLensCenterAndHitDist)std::runtime_error("�v�f�����Y�̌����Ɏ��s");

					//�œ_�̏ꏊ�͗v�f�����Y���m�肷��Όv�Z�ł���
					const auto focalposXYZLocal = Polar3DToXyz(ExtendUvec2(MapToPolar(hitLensCenterAndHitDist.value().first), sphereRadius + nodeLensFocalLength));

					const auto hitPosXYZLocal = PolarToXyz(rayDirInBallLocalPolar);
					const auto refractRayDirXYZLocal = (focalposXYZLocal-hitPosXYZLocal).normalized();//�Փ˓_�Əœ_�̈ʒu��������Ό��̕������킩��ˁ@�b��I��

					//�v���b�g���܂��@�q�b�g�|�C���g�ɐF�ʂ�
					py::sf("plt.scatter(%f,%f,color=(%f,%f,%f))", rayDirInBallLocalMapDesigned.x(), rayDirInBallLocalMapDesigned.y(), fabs(refractRayDirXYZLocal.x()), fabs(refractRayDirXYZLocal.y()), fabs(refractRayDirXYZLocal.z()));

				}
			}
		};
		

		//�\������ 3d 2d�̏�
		py::s("plt.show()");
	}
	catch (std::exception& ex) {
		cout << ex.what() << endl;
		py::Terminate();
		return -1;
	}
	catch (...) {
		cout << "unknown err" << endl;
		py::Terminate();
		return -2;
	}

	system("pause");//�Ȃ񂩓��ꂽ��I���
	py::Terminate();
	return 0;
}
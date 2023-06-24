#include "application.hpp"

using namespace std;

uptr<matplotlib> SetupPythonRuntime() {
	uptr<matplotlib> ret = make_unique<matplotlib>();
	ret->open(R"(
import numpy as np
from mayavi import mlab

#�m���u���b�L���O�ŕ\��
plt.show(block=False)
##�\���̈�̐ݒ�
fig = plt.figure(figsize = (8, 6))
ax = fig.add_subplot(111, projection = '3d')
ax.set_box_aspect((1, 1, 1))
# �����x���̐ݒ�
ax.set_xlabel("x", fontsize = 16)
ax.set_ylabel("y", fontsize = 16)
ax.set_zlabel("z", fontsize = 16))");

	return ret;
}

void DefinePythonFunctions(uptr<matplotlib>& plt) {
	//��{�֐�
	plt->send_command(R"(
m=3#����

#���K�����܂��傤
def normalize(vec):
  norm=np.linalg.norm(vec)
  return vec/norm

#�c�x�N�g�����ꎟ���z��ɂ��܂��傤
def v(vec):
  return (vec.T)[0]
#�ꎟ���z����c�x�N�g���ɂ��܂��傤
def iv(arr):
  return np.array([arr]).T

#�ʉߓ_�A���x�ȃx�N�g���𑬓x�̂ݐ��K������
def normalizeV(ray):
  velo=normalize(ray[m:m+m])
  return np.vstack((ray[0:m],velo)))");

	//��������
	plt->send_command(R"(
#���������@���S c,���a r, �`��plot ax
def drawCircle(ax,c,r,splitnum=10,col="lightgreen",alpha=1.):
  # Make data
  u = np.linspace(0, 2 * np.pi, splitnum)
  v = np.linspace(0, np.pi, splitnum)
  x = r * np.outer(np.cos(u), np.sin(v))+c[0]
  y = r * np.outer(np.sin(u), np.sin(v))+c[1]
  z = r * np.outer(np.ones(np.size(u)), np.cos(v))+c[2]

  # Plot the surface
  ax.plot_surface(x, y, z,color=col,rcount=splitnum, ccount=splitnum, antialiased=False,alpha=alpha))");
}

void DrawSphere(uptr<matplotlib>& plt, uvec3 center, ureal r, int splitnum, const std::string& color, ureal alpha) {

	//�������ڂ��ւ���
	//std::stringstream ss;
	plt->send_command(StringFormat("drawCircle(ax,iv([%f,%f,%f]),%f,%i,col=%s,alpha=%f)", center.x(), center.y(), center.z(), r, splitnum, color, alpha).c_str());

}

uptr<std::list<sphereParam>> CalcSmallLensPosAndRadius() {
	//����z�񂵂Ă���...
	constexpr unsigned int babynum = 600;
	//���̍��W�Ɣ��a������
	auto smallBallsParams = make_unique<std::list<sphereParam>>(babynum);
	auto saved = smallBallsParams->begin();//�v�Z�����p�����[�^�̕ۑ���
	for (unsigned int i = 0; i < babynum; i++) {
		/*����
		���ʏ�ɒ��S������͂�
		�l�W��ɂ�����Ƃ������͂�?*/

		//�܂��̓l�W���
		//x=rcos(at),y=rsin(at)���Ɖ~
		//z=bt��ǉ�����ƃR�C����

		ureal t = (ureal)i / (ureal)babynum;//0.~less than 1.

		//�ʒu��
		constexpr ureal spin = 20;//����
		constexpr ureal len = 2.;//�R�C���̒���
		constexpr ureal pitch = len / spin;//��������Ƃ��ɐi�ދ���
		const uvec3 org(0., 0., -1.);//�J�n�ʒu
		uvec3 babypos;//�����̈ʒu
		babypos.z() = spin * pitch * [&] {
			//�񎟊֐��I��plt
			if (t < 0.5) {
				return pow(t * 2., 2) / 2.;
			}
			else {
				return -(pow((1 - t) * 2., 2) / 2.) + 1.;
			}
		}();//t��1��spin�����Ă�@�܂�spin*pitch
		//�������甼�a�����߂���ł��傤? ���Ƀt�B�b�e�B���O����悤��
		ureal radius = sin(acos(babypos.z() - 1.));
		babypos.x() = radius * cos(2 * std::numbers::pi * t * spin);
		babypos.y() = radius * sin(2 * std::numbers::pi * t * spin);

		//���ɏ����̔��a�����߂��� �܂��������Ԃɉ��u������H
		ureal numOfSmallBallInCircle = (ureal)babynum / (ureal)spin;
		//�勅��̔��a���킩�邩��~����~���l�߂��悤�ȑ傫���ɂ���
		constexpr ureal expandSmallBall = 1.5;//������Ƃ݂��݂��Ă��ق���Good�Ȃ̂ŏ������g�傷��
		ureal smallradius = 2 * std::numbers::pi * radius / numOfSmallBallInCircle / 2. * expandSmallBall;
		//�����̕ۑ�
		saved.operator*() = make_pair(babypos + org, smallradius);
		saved++;
	}

	return smallBallsParams;
}

void DrawLine(uptr<matplotlib>& plt, const uvec3& a, const uvec3& b, const std::string& color) {
	plt->send_command(StringFormat("plt.plot([%f,%f],[%f,%f],[%f,%f],color=%s)", a.x(), b.x(), a.y(), b.y(), a.z(), b.z(), color).c_str());
}

void DrawRay(uptr<matplotlib>& plt, const ray3& target, const std::string& color) {
	//�^�[�Q�b�g�̊Ԃ��v�Z����
	auto first = target.begin();

	while (1) {
		//��̏I�_���v�Z����
		auto second = std::next(first);
		if (second == target.end())break;

		DrawLine(plt, first->org(), second->org(), color);
		first = second;
	}
}
void DrawRaySkipFirstArrow(uptr<matplotlib>& plt, const ray3& target, const std::string& color) {
	//�^�[�Q�b�g�̊Ԃ��v�Z����
	auto first = target.begin();
	//first++;

	while (1) {
		//��̏I�_���v�Z����
		auto second = std::next(first);
		if (second == target.end())break;

		DrawLine(plt, first->org(), second->org(), color);
		first = second;
	}
}

resultIntersecteSphere IntersectSphere(const arrow3& rayback, const uvec3& c, const ureal r) {
	constexpr ureal crossThreshold = 0.01;

	//�e�������v�Z����
	ureal A = 0., B = 0., C = -pow(r, 2);
	for (int i = 0; i < 3; i++) {
		A += pow(rayback.dir()[i], 2);
		B += 2. * rayback.org()[i] * rayback.dir()[i] - 2. * rayback.dir()[i] * c[i];
		C += pow(rayback.org()[i], 2) - 2. * rayback.org()[i] * c[i] + pow(c[i], 2);
	}
	//���ʎ�
	const ureal hanbetu = pow(B, 2) - 4 * A * C;

	//����������
	if (hanbetu >= 0.) {
		//�������Ԃ����߂�
		const ureal t0 = (-B + sqrt(hanbetu)) / (2. * A);
		const ureal t1 = (-B - sqrt(hanbetu)) / (2. * A);

		//�ŏ��̌������Ԃ����߂�@�����������������疳��
		ureal t = std::min(t0, t1);
		if (t <= crossThreshold)t = std::max(t0, t1);

		//�����_�����߂�
		const uvec3 kai = rayback.dir() * t + rayback.org();
		//��_�̖@�������߂�
		const uvec3 norm = (kai - c).normalized();

		return resultIntersecteSphere(kai, norm, t);
	}
	//�������Ȃ�������
	else {
		return resultIntersecteSphere();
	}
}

resultIntersecteSphere::resultIntersecteSphere() {
	this->isHit = false;
}
resultIntersecteSphere::resultIntersecteSphere(const uvec3& pos, const uvec3& norm,const ureal t) {
	this->isHit = true;
	this->pos = pos;
	this->norm = norm;
	this->t = t;
}
ray3& resultIntersecteSphere::ApplyToRay(ray3& target) const{
	if (!this->isHit)return target;
	//�V���ȓ_
	arrow3 newone = target.back();
	newone.org() = pos;

	target.push_back(newone);
	return target;
}

ray3& ReflectMirror(ray3& target, const uvec3& norm) {
	const uvec3 newway = (target.back().dir() + 2*(-target.back().dir().dot(norm)) * norm).normalized();

	arrow3 newone = target.back();
	newone.dir() = newway;

	target.push_back(newone);
	return target;
}

//���܂���
bool RefractSnell(ray3& target, const uvec3& norm, const ureal eta) {
	const auto& ins = target.back();//���ˌ�
	//cout << ins.dir().norm() << endl;
	//cout <<"hei "<< (norm.dot(-ins.dir())) << endl;
	const auto theta = acos(clamp(norm.dot(-ins.dir()),-1.,1.));//���ˊp

	//�ՊE�p���ǂ������肷��
	if (sin(theta) * eta > 1.) {
		//�ՊE�p�Ȃ̂ŋ��܂͂��Ȃ�
		return false;
	}

	const auto phi = asin(sin(theta) / eta);//�o�ˊp

	ureal alpha;
	//�o�ˊp��0�łȂ����=sin(phi)��0�łȂ����
	if (sin(phi) != 0.)
		alpha = sin(theta - phi) / sin(phi);
	else
		alpha = 0.;

	//�܂�o�˕�����(alpha��0�Ȃ���܂��Ȃ����Ă���)
	const uvec3 e = ins.dir() - alpha * norm;

	//�ǉ����ďI���
	target.push_back(arrow3(ins.org(), e.normalized()));
	return true;
}


//�A��gif�����
void MakeGifAnim(const std::string& palletfile, const std::string& outputfile, const std::string& inputfile,const size_t fps) {
	const std::string MakePallet = StringFormat("ffmpeg -i %s -vf palettegen -y %s", inputfile, palletfile);
	const std::string MakeAnim = StringFormat("ffmpeg -r %d -i %s -i %s -lavfi paletteuse -y %s", fps, inputfile, palletfile, outputfile);

	system(MakePallet.c_str());
	system(MakeAnim.c_str());
}

std::array<ureal, 3> RgbToHsv(const std::array<ureal, 3>& rgb) {
	std::array<ureal, 3> hsv;
	auto max = std::max_element(rgb.cbegin(), rgb.cend());
	auto min = std::min_element(rgb.cbegin(), rgb.cend());

	hsv.at(2) = *max;
	hsv.at(1) = (*max - *min) / *max;

	switch (std::distance(rgb.cbegin(), max)) {
	case 0:
		hsv.at(0) = 60. * (rgb.at(2) - rgb.at(1)) / (*max - *min);
		break;
	case 1:
		hsv.at(0) = 60. * (2. + (rgb.at(0) - rgb.at(2)) / (*max - *min));
		break;
	case 2:
		hsv.at(0) = 60. * (3 + (rgb.at(1) - rgb.at(0)) / (*max - *min));
		break;
	}
	//hsv���X�P�[�����O
	hsv.at(0) = hsv.at(0) / 360.;

	return hsv;
}

std::array<ureal, 3> HsvToRgb(const std::array<ureal, 3>& hsv) {
	std::array<ureal, 3> rgb;

	int tempi;
	ureal tempm, tempn, tempk, tempf;


	if (hsv.at(1) == 0.) {
		rgb.at(0) = rgb.at(1) = rgb.at(2) = hsv.at(2);
	}
	else {
		tempi = (decltype(tempi))floor(hsv.at(0) * 6.);
		tempf = hsv.at(0) * 6. - tempi;
		tempm = hsv.at(2) * (1. - hsv.at(1));
		tempn = hsv.at(2) * (1. - (hsv.at(1)) * tempf);
		tempk = hsv.at(2) * (1. - (hsv.at(1)) * (1. - tempf));

		switch (tempi) {
		case 0:
			rgb.at(0) = hsv.at(2);
			rgb.at(1) = tempk;
			rgb.at(2) = tempm;
			break;
		case 1:
			rgb.at(0) = tempn;
			rgb.at(1) = hsv.at(2);
			rgb.at(2) = tempm;
			break;
		case 2:
			rgb.at(0) = tempm;
			rgb.at(1) = hsv.at(2);
			rgb.at(2) = tempk;
			break;
		case 3:
			rgb.at(0) = tempm;
			rgb.at(1) = tempn;
			rgb.at(2) = hsv.at(2);
			break;
		case 4:
			rgb.at(0) = tempk;
			rgb.at(1) = tempm;
			rgb.at(2) = hsv.at(2);
			break;
		case 5:
			rgb.at(0) = hsv.at(2);
			rgb.at(1) = tempm;
			rgb.at(2) = tempn;
		}
	}
	return rgb;
}



uvec3 PolarToXyz(const uvec2& spolar) {
	return uvec3(cos(spolar.y()) * cos(spolar.x()),
		cos(spolar.y()) * sin(spolar.x()),
		sin(spolar.y()));
}
uvec2 XyzToPolar(const uvec3& xyz) {
	return uvec2((xyz.y() > 0. ? 1. : -1.) * acos(xyz.x() / sqrt(pow(xyz.x(), 2) + pow(xyz.y(), 2))), -acos(xyz.z() / sqrt(pow(xyz.x(), 2) + pow(xyz.y(), 2) + pow(xyz.z(), 2))) + (pi / 2.));
}
uvec3 Polar3DToXyz(const uvec3& phiThetaRadius) {
	return uvec3(phiThetaRadius.z() * cos(phiThetaRadius.y()) * cos(phiThetaRadius.x()),
		phiThetaRadius.z() * cos(phiThetaRadius.y()) * sin(phiThetaRadius.x()),
		phiThetaRadius.z() * sin(phiThetaRadius.y()));
}

uvec2 MapToLocalPolar(const uvec2& xy) {
	return uvec2(xy.x(), 2. * atan(-pow(std::numbers::e, -xy.y())) + pi / 2.);
}
uvec2 PolarToMap(const uvec2& xy) {
	return uvec2(xy.x(), -log(abs(tan(xy.y() / 2. - pi / 4.))));
}

//��ҕ�����Z�p�`�����
std::list<uvec2> MakeHexagon(const ureal& edgeWidth) {
	//�O�ڋ��̔��a���o������
	const ureal radius = 2. * edgeWidth / sqrt(3.);
	std::list<uvec2> ret;

	//�_��������Əo���Ă���
	for (size_t i = 0; i < 6; i++) {
		const auto t = uleap(PairMinusPlus(pi), i / 6.);

		const uvec2 pos = radius * uvec2(sin(t), cos(t));
		ret.push_back(pos);
	}

	return ret;
}


//��������pythonRuntime
safe_queue::safe_queue<std::string> pythonRuntime::combuffer;
//�R�}���h�����X���b�h
uptr<std::thread> pythonRuntime::pyRunThread;

std::atomic_bool pythonRuntime::FlagContiPyRun;

void pythonRuntime::PyRunLoop() {

	Py_Initialize();

	while (FlagContiPyRun||!combuffer.empty()) {
		try {
			if (combuffer.empty())std::this_thread::sleep_for(std::chrono::microseconds(100));//�Ȃ��������x��
			else {
				PyRun_SimpleString(combuffer.pop().get()->c_str());//���s
			}
		}
		catch (std::exception& ex) {
			cout << ex.what() << endl;
		}
	}

	Py_Finalize();
	return;
}

void pythonRuntime::Init() {
	FlagContiPyRun = true;

	pyRunThread.reset(new std::thread(PyRunLoop));

}
void pythonRuntime::Terminate() {
	FlagContiPyRun = false;
	pyRunThread->join();
	pyRunThread.release();
}

void pythonRuntime::SendCommand(const std::string& command) {
	combuffer.push(command);
}
void pythonRuntime::s(const std::string& s) {
	SendCommand(s);
}
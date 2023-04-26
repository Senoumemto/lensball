#include <iostream>
#include "matplotwrapper.hpp"

using namespace std;

uptr<matplotlib> SetupPythonRuntime() {
	uptr<matplotlib> ret = make_unique<matplotlib>();
	ret->open(R"(
import numpy as np

#�m���u���b�L���O�ŕ\��
plt.show(block=False)
##�\���̈�̐ݒ�
fig = plt.figure(figsize = (8, 6))
ax = fig.add_subplot(111, projection = '3d')
ax.set_box_aspect((1, 1, 1))
# �����x���̐ݒ�
ax.set_xlabel("x", fontsize = 16)
ax.set_ylabel("y", fontsize = 16)
ax.set_zlabel("z", fontsize = 16)
# ���͈͂̐ݒ�
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_zlim(-1.5, 1.5))");

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

void DrawSphere(uptr<matplotlib>& plt,uvec3 center,ureal r,int splitnum=10,const std::string& color="\"red\"",ureal alpha=1.) {

	//�������ڂ��ւ���
	std::stringstream ss;
	plt->send_command(StringFormat("drawCircle(ax,iv([%f,%f,%f]),%f,%i,col=%s,alpha=%f)", center.x(), center.y(), center.z(), r, splitnum, color, alpha).c_str());

}

int main() {

	auto plotter = SetupPythonRuntime();//python���Z�b�g�A�b�v����
	DefinePythonFunctions(plotter);//�~�{�I��{�֐����`

	//�܂��K�C�h�p�̋���u��
	//DrawSphere(plotter, uvec3(0, 0, 0.), 1., 25, R"("lightgreen")", 1.);


	//����z�񂵂Ă���...
	int babynum = 120;
	for (decltype(babynum) i = 0; i < babynum; i++) {
		/*����
		���ʏ�ɒ��S������͂�
		�l�W��ɂ�����Ƃ������͂�?*/

		//�܂��̓l�W���
		//x=rcos(at),y=rsin(at)���Ɖ~
		//z=bt��ǉ�����ƃR�C����

		ureal t = (ureal)i / (ureal)babynum;//0.~less than 1.

		//�ʒu��
		constexpr ureal spin = 10;//����
		constexpr ureal len = 2.;//�R�C���̒���
		constexpr ureal pitch = len/spin;//��������Ƃ��ɐi�ދ���
		const uvec3 org(0., 0., -1.);//�J�n�ʒu
		uvec3 babypos;
		babypos.z() = spin * pitch * [&]{
			//�񎟊֐��I��plt
			if (t < 0.5) {
				return pow(t * 2., 2) / 2.;
			}
			else {
				return -(pow((t-1.) * 2., 2) / 2.)+1.;
			}
		}();//t��1��spin�����Ă�@�܂�spin*pitch
		//�������甼�a�����߂���ł��傤? ���Ƀt�B�b�e�B���O����悤��
		ureal radius = sin(acos(babypos.z()-1.));
		babypos.x() = radius * cos(2 * std::numbers::pi * t * spin);
		babypos.y() = radius * sin(2 * std::numbers::pi * t * spin);

		//���ɏ����̔��a�����߂��� �܂��������Ԃɉ��u������H
		ureal numOfSmallBallInCircle = (ureal)babynum / (ureal)spin;
		//�勅��̔��a���킩�邩��~����~���l�߂��悤�ȑ傫���ɂ���
		ureal smallradius = 2 * std::numbers::pi * radius / numOfSmallBallInCircle / 2.;
		//�����̕`��
		DrawSphere(plotter, babypos + org, smallradius, 10);
	}

	plotter->show();

	plotter->close();//�I���@����܂�Ӗ����Ȃ�
	return 0;
}
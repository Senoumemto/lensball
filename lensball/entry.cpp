#include <iostream>
#include "matplotwrapper.hpp"

using namespace std;

uptr<matplotlib> SetupPythonRuntime() {
	uptr<matplotlib> ret = make_unique<matplotlib>();
	ret->open(R"(
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
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_zlim(-5, 5))");

	return ret;
}

int main() {
	//python���Z�b�g�A�b�v����
	auto plotter = SetupPythonRuntime();

	//���C�����[�v
	while (1) {
		plotter->show();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	plotter->close();
	return 0;
}
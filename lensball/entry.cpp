#include <iostream>
#include "matplotwrapper.hpp"

using namespace std;

uptr<matplotlib> SetupPythonRuntime() {
	uptr<matplotlib> ret = make_unique<matplotlib>();
	ret->open(R"(
#ノンブロッキングで表示
plt.show(block=False)
##表示領域の設定
fig = plt.figure(figsize = (8, 6))
ax = fig.add_subplot(111, projection = '3d')
ax.set_box_aspect((1, 1, 1))
# 軸ラベルの設定
ax.set_xlabel("x", fontsize = 16)
ax.set_ylabel("y", fontsize = 16)
ax.set_zlabel("z", fontsize = 16)
# 軸範囲の設定
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_zlim(-5, 5))");

	return ret;
}

int main() {
	//pythonをセットアップする
	auto plotter = SetupPythonRuntime();

	//メインループ
	while (1) {
		plotter->show();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	plotter->close();
	return 0;
}
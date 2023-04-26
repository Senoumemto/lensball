#include <iostream>
#include "matplotwrapper.hpp"

using namespace std;

uptr<matplotlib> SetupPythonRuntime() {
	uptr<matplotlib> ret = make_unique<matplotlib>();
	ret->open(R"(
import numpy as np

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
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_zlim(-1.5, 1.5))");

	return ret;
}

void DefinePythonFunctions(uptr<matplotlib>& plt) {
	//基本関数
	plt->send_command(R"(
m=3#次元

#正規化しましょう
def normalize(vec):
  norm=np.linalg.norm(vec)
  return vec/norm

#縦ベクトルを一次元配列にしましょう
def v(vec):
  return (vec.T)[0]
#一次元配列を縦ベクトルにしましょう
def iv(arr):
  return np.array([arr]).T

#通過点、速度なベクトルを速度のみ正規化する
def normalizeV(ray):
  velo=normalize(ray[m:m+m])
  return np.vstack((ray[0:m],velo)))");

	//球を書く
	plt->send_command(R"(
#球を書く　中心 c,半径 r, 描画plot ax
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

	//引数を移し替える
	std::stringstream ss;
	plt->send_command(StringFormat("drawCircle(ax,iv([%f,%f,%f]),%f,%i,col=%s,alpha=%f)", center.x(), center.y(), center.z(), r, splitnum, color, alpha).c_str());

}

int main() {

	auto plotter = SetupPythonRuntime();//pythonをセットアップする
	DefinePythonFunctions(plotter);//梅本的基本関数を定義

	//まずガイド用の球を置く
	//DrawSphere(plotter, uvec3(0, 0, 0.), 1., 25, R"("lightgreen")", 1.);


	//球を配列していく...
	int babynum = 120;
	for (decltype(babynum) i = 0; i < babynum; i++) {
		/*条件
		球面上に中心が来るはず
		ネジ状にちょっとずつずれるはず?*/

		//まずはネジ状に
		//x=rcos(at),y=rsin(at)だと円
		//z=btを追加するとコイルに

		ureal t = (ureal)i / (ureal)babynum;//0.~less than 1.

		//位置は
		constexpr ureal spin = 10;//巻数
		constexpr ureal len = 2.;//コイルの長さ
		constexpr ureal pitch = len/spin;//一周したときに進む距離
		const uvec3 org(0., 0., -1.);//開始位置
		uvec3 babypos;
		babypos.z() = spin * pitch * [&]{
			//二次関数的にplt
			if (t < 0.5) {
				return pow(t * 2., 2) / 2.;
			}
			else {
				return -(pow((t-1.) * 2., 2) / 2.)+1.;
			}
		}();//tが1でspin回回ってる　つまりspin*pitch
		//ここから半径を求められるでしょう? 球にフィッティングするように
		ureal radius = sin(acos(babypos.z()-1.));
		babypos.x() = radius * cos(2 * std::numbers::pi * t * spin);
		babypos.y() = radius * sin(2 * std::numbers::pi * t * spin);

		//つぎに昇給の半径を求めたい まず一周する間に何個置かれるん？
		ureal numOfSmallBallInCircle = (ureal)babynum / (ureal)spin;
		//大球上の半径がわかるから円周を敷き詰めれるような大きさにする
		ureal smallradius = 2 * std::numbers::pi * radius / numOfSmallBallInCircle / 2.;
		//豆球の描画
		DrawSphere(plotter, babypos + org, smallradius, 10);
	}

	plotter->show();

	plotter->close();//終了　あんまり意味がない
	return 0;
}
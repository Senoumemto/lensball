#include "application.hpp"

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
ax.set_xlim(-3.,3.)
ax.set_ylim(-3.,3.)
ax.set_zlim(-3.,3.))");

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

void DrawSphere(uptr<matplotlib>& plt, uvec3 center, ureal r, int splitnum, const std::string& color, ureal alpha) {

	//引数を移し替える
	//std::stringstream ss;
	plt->send_command(StringFormat("drawCircle(ax,iv([%f,%f,%f]),%f,%i,col=%s,alpha=%f)", center.x(), center.y(), center.z(), r, splitnum, color, alpha).c_str());

}

uptr<std::list<sphereParam>> CalcSmallLensPosAndRadius() {
	//球を配列していく...
	constexpr unsigned int babynum = 600;
	//球の座標と半径を入れる
	auto smallBallsParams = make_unique<std::list<sphereParam>>(babynum);
	auto saved = smallBallsParams->begin();//計算したパラメータの保存先
	for (unsigned int i = 0; i < babynum; i++) {
		/*条件
		球面上に中心が来るはず
		ネジ状にちょっとずつずれるはず?*/

		//まずはネジ状に
		//x=rcos(at),y=rsin(at)だと円
		//z=btを追加するとコイルに

		ureal t = (ureal)i / (ureal)babynum;//0.~less than 1.

		//位置は
		constexpr ureal spin = 20;//巻数
		constexpr ureal len = 2.;//コイルの長さ
		constexpr ureal pitch = len / spin;//一周したときに進む距離
		const uvec3 org(0., 0., -1.);//開始位置
		uvec3 babypos;//昇給の位置
		babypos.z() = spin * pitch * [&] {
			//二次関数的にplt
			if (t < 0.5) {
				return pow(t * 2., 2) / 2.;
			}
			else {
				return -(pow((1 - t) * 2., 2) / 2.) + 1.;
			}
		}();//tが1でspin回回ってる　つまりspin*pitch
		//ここから半径を求められるでしょう? 球にフィッティングするように
		ureal radius = sin(acos(babypos.z() - 1.));
		babypos.x() = radius * cos(2 * std::numbers::pi * t * spin);
		babypos.y() = radius * sin(2 * std::numbers::pi * t * spin);

		//つぎに昇給の半径を求めたい まず一周する間に何個置かれるん？
		ureal numOfSmallBallInCircle = (ureal)babynum / (ureal)spin;
		//大球上の半径がわかるから円周を敷き詰めれるような大きさにする
		constexpr ureal expandSmallBall = 1.5;//ちょっとみちみちてたほうがGoodなので小球を拡大する
		ureal smallradius = 2 * std::numbers::pi * radius / numOfSmallBallInCircle / 2. * expandSmallBall;
		//豆球の保存
		saved.operator*() = make_pair(babypos + org, smallradius);
		saved++;
	}

	return smallBallsParams;
}

void DrawLine(uptr<matplotlib>& plt, const uvec3& a, const uvec3& b, const std::string& color) {
	plt->send_command(StringFormat("plt.plot([%f,%f],[%f,%f],[%f,%f],color=%s)", a.x(), b.x(), a.y(), b.y(), a.z(), b.z(), color).c_str());
}

void DrawRay(uptr<matplotlib>& plt, const ray3& target, const std::string& color) {
	//ターゲットの間を計算する
	auto first = target.begin();

	while (1) {
		//先の終点を計算する
		auto second = std::next(first);
		if (second == target.end())break;

		DrawLine(plt, first->org(), second->org(), color);
		first = second;
	}
}
void DrawRaySkipFirstArrow(uptr<matplotlib>& plt, const ray3& target, const std::string& color) {
	//ターゲットの間を計算する
	auto first = target.begin();
	//first++;

	while (1) {
		//先の終点を計算する
		auto second = std::next(first);
		if (second == target.end())break;

		DrawLine(plt, first->org(), second->org(), color);
		first = second;
	}
}

resultIntersecteSphere IntersectSphere(const arrow3& rayback, const uvec3& c, const ureal r) {
	constexpr ureal crossThreshold = 0.01;

	//各こうを計算する
	ureal A = 0., B = 0., C = -pow(r, 2);
	for (int i = 0; i < 3; i++) {
		A += pow(rayback.dir()[i], 2);
		B += 2. * rayback.org()[i] * rayback.dir()[i] - 2. * rayback.dir()[i] * c[i];
		C += pow(rayback.org()[i], 2) - 2. * rayback.org()[i] * c[i] + pow(c[i], 2);
	}
	//判別式
	const ureal hanbetu = pow(B, 2) - 4 * A * C;

	//交差したら
	if (hanbetu >= 0.) {
		//交差時間を求める
		const ureal t0 = (-B + sqrt(hanbetu)) / (2. * A);
		const ureal t1 = (-B - sqrt(hanbetu)) / (2. * A);

		//最小の交差時間を求める　ただし小さすぎたら無効
		ureal t = std::min(t0, t1);
		if (t <= crossThreshold)t = std::max(t0, t1);

		//交差点を求める
		const uvec3 kai = rayback.dir() * t + rayback.org();
		//交点の法線を求める
		const uvec3 norm = (kai - c).normalized();

		return resultIntersecteSphere(kai, norm, t);
	}
	//交差しなかったら
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
	//新たな点
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

//屈折する
bool RefractSnell(ray3& target, const uvec3& norm, const ureal eta) {
	const auto& ins = target.back();//入射光
	cout << (norm.dot(-ins.dir())) << endl;
	const auto theta = acos(clamp(norm.dot(-ins.dir()),-1.,1.));//入射角

	//臨界角かどうか判定する
	if (sin(theta) / eta > 1.) {
		//臨界角なので屈折はしない
		return false;
	}

	const auto phi = asin(sin(theta) / eta);//出射角

	ureal alpha;
	//出射角が0でなければ=sin(phi)が0でなければ
	if (sin(phi) != 0.)
		alpha = sin(theta - phi) / sin(phi);
	else
		alpha = 0.;

	//つまり出射方向は(alphaが0なら屈折しないってこと)
	const uvec3 e = ins.dir() - alpha * norm;

	//追加して終わり
	target.push_back(arrow3(ins.org(), e.normalized()));
	return true;
}
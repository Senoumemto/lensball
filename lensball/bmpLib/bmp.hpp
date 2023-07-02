#ifndef BMPLINB_INCLUDEGUARD
#define BMPLINB_INCLUDEGUARD

/*****************************************************************************/
/*                                                                           */
/*     bmp.h: bmp ファイル処理のライブラリのためのヘッダファイル             */
/*                                                                           */
/*             Kazutoshi Ando (Shizuoka Univ.)                               */
/*                                                                           */
/*                  Ver. 2004.08.18                                          */
/*                  Ver. 2004.08.17                                          */
/*                  Ver. 2003.11.04                                          */
/*                                                                           */
/*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>

/* x と y の交換のための マクロ関数 */
#define SWAP(x,y) {typeof(x) temp; temp=x; x=y; y=temp;}

namespace bmpLib {

	struct color {                      /* 1ピクセルあたりの赤緑青の各輝度     */
		unsigned char r;
		unsigned char g;
		unsigned char b;

		color(unsigned char _r, unsigned char _g, unsigned char _b):r(_r),g(_g),b(_b){}
		color() = default;

		//足します
		color& operator+=(const color& b) {
			this->r += b.r;
			this->g += b.g;
			this->b += b.b;

			return *this;
		}
		color operator*(const double& b) const {

			return color(this->r * b, this->g * b, this->b * b);
		}
	} ;

	typedef struct {
		long height;
		long width;
		std::vector<std::vector<color>> data;
	} img;

	void ReadBmp(const char* filename, img* imgp);
	void WriteBmp(const char* filename, img* tp);
	void PrintBmpInfo(const char* filename);

};
#endif
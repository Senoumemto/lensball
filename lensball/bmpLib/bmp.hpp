#ifndef BMPLINB_INCLUDEGUARD
#define BMPLINB_INCLUDEGUARD

/*****************************************************************************/
/*                                                                           */
/*     bmp.h: bmp �t�@�C�������̃��C�u�����̂��߂̃w�b�_�t�@�C��             */
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

/* x �� y �̌����̂��߂� �}�N���֐� */
#define SWAP(x,y) {typeof(x) temp; temp=x; x=y; y=temp;}

namespace bmpLib {

	typedef struct {                      /* 1�s�N�Z��������̐ԗΐ̊e�P�x     */
		unsigned char r;
		unsigned char g;
		unsigned char b;
	} color;

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
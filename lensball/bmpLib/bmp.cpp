/*****************************************************************************/
/*                                                                           */
/*     bmp.c: bmp �t�@�C�������̃��C�u����                                   */
/*                                                                           */
/*             Kazutoshi Ando (Shizuoka Univ.)                               */
/*                                                                           */
/*                  Ver. 2004.11.30                                          */
/*                      WriteBmp: �w�b�_���̌������C��.                    */
/*                  Ver. 2004.11.26                                          */
/*                      Diminish:  ����(?)���[�`����ǉ�.                    */
/*                      PrintBmpInfo: bmp�t�@�C���ȊO�̃t�@�C����ǂ񂾂Ƃ�  */
/*                               ��, �G���[���o�͂���悤�ɕύX.             */
/*                  Ver. 2004.08.20                                          */
/*                      ReadBmp: 24bit�F��bmp�t�@�C���ȊO�̃t�@�C����        */
/*                               �ǂ񂾂Ƃ���, �G���[���o�͂���悤�ɕύX.   */
/*                      PrintBmpInfo: ����, �����𑜓x���o�͂���悤�ɕύX.  */
/*                      WriteBmp: �w�b�_���̌������C��.                    */
/*                  Ver. 2004.08.18                                          */
/*                        Gray ��ǉ�.                                       */
/*                  Ver. 2004.08.17                                          */
/*                        4byte ���E�ɍ��킹�邽�߂̌v�Z��������,            */
/*                        Rotate90, Shrink, Mosaic ��ǉ�.                   */
/*                        �G���[���b�Z�[�W�̏o�͐��W���G���[�o�͂ɕύX.    */
/*                  Ver. 2003.11.04                                          */
/*                  Ver. 2010.10.19                                          */
/*                        long int �� int �ŏo�͂��Ă���ƌx�����ł�̂ŏC��        */
/*                                                                           */
/*****************************************************************************/
#include "bmp.hpp"


constexpr size_t HEADERSIZE = 54;               /* �w�b�_�̃T�C�Y 54 = 14 + 40         */
constexpr size_t PALLETSIZE = 1024;               /* �p���b�g�̃T�C�Y                    */

using namespace bmpLib;

/*
   �֐���: ReadBmp
   ����  : char *filename, img *imgp
   �Ԃ�l: void
   ����  : bmp�`���̃t�@�C�� filename ���J����, ���̉摜�f�[�^��
           2�����z�� imgp->data �Ɋi�[����. ������, �w�b�_����ǂݍ��܂ꂽ
           �摜�̕��ƍ������O���[�o���ϐ� Bmp_width ��Bmp_height �ɃZ�b�g����.
*/
void bmpLib::ReadBmp(const char* filename, img* imgp) {
    int i, j;
    int Real_width;
    FILE* Bmp_Fp;
    fopen_s(&Bmp_Fp, filename, "rb");  /* �o�C�i�����[�h�ǂݍ��ݗp�ɃI�[�v��  */
    unsigned char* Bmp_Data;           /* �摜�f�[�^��1�s���i�[               */

    unsigned char Bmp_headbuf[HEADERSIZE];/* �w�b�_���i�[���邽�߂̕ϐ�          */
    unsigned char Bmp_Pallet[PALLETSIZE]; /* �J���[�p���b�g���i�[                */

    char Bmp_type[2];                     /* �t�@�C���^�C�v "BM"                 */
    unsigned long Bmp_size;               /* bmp�t�@�C���̃T�C�Y (�o�C�g)        */
    unsigned int Bmp_info_header_size; /* ���w�b�_�̃T�C�Y = 40             */
    unsigned int Bmp_header_size;      /* �w�b�_�T�C�Y = 54*/
    long Bmp_height;                      /* ���� (�s�N�Z��)                     */
    long Bmp_width;                       /* ��   (�s�N�Z��)                     */
    unsigned short Bmp_planes;          /* �v���[���� ��� 1                   */
    unsigned short Bmp_color;          /* �F (�r�b�g)     24                  */
    long Bmp_comp;                        /* ���k���@         0                  */
    long Bmp_image_size;                  /* �摜�����̃t�@�C���T�C�Y (�o�C�g)   */
    long Bmp_xppm;                        /* �����𑜓x (ppm)                    */
    long Bmp_yppm;                        /* �����𑜓x (ppm)                    */

    if (Bmp_Fp == NULL) {
        fprintf(stderr, "Error: file %s couldn\'t open for read!.\n", filename);
        exit(1);
    }

    /* �w�b�_�ǂݍ��� */
    fread(Bmp_headbuf, sizeof(unsigned char), HEADERSIZE, Bmp_Fp);

    memcpy(&Bmp_type, Bmp_headbuf, sizeof(Bmp_type));
    if (strncmp(Bmp_type, "BM", 2) != 0) {
        fprintf(stderr, "Error: %s is not a bmp file.\n", filename);
        exit(1);
    }

    memcpy(&imgp->width, Bmp_headbuf + 18, sizeof(Bmp_width));
    memcpy(&imgp->height, Bmp_headbuf + 22, sizeof(Bmp_height));
    memcpy(&Bmp_color, Bmp_headbuf + 28, sizeof(Bmp_color));

    if (Bmp_color != 24) {
        fprintf(stderr, "Error: Bmp_color = %d is not implemented in this program.\n", Bmp_color);
        exit(1);
    }

    //�ł͊m��
    imgp->data.resize(imgp->height);
    for (size_t hd = 0; hd < imgp->height; hd++)
        imgp->data[hd].resize(imgp->width);

    Real_width = imgp->width * 3 + imgp->width % 4; /* 4byte ���E�ɂ��킹�邽�߂Ɏ��ۂ̕��̌v�Z */

    /* �z��̈�̓��I�m��. ���s�����ꍇ�̓G���[���b�Z�[�W���o�͂��ďI�� */
    if ((Bmp_Data = (unsigned char*)calloc(Real_width, sizeof(unsigned char))) == NULL) {
        fprintf(stderr, "Error: Memory allocation failed for Bmp_Data!\n");
        exit(1);
    }

    /* �摜�f�[�^�ǂݍ��� */
    for (i = 0; i < imgp->height; i++) {
        fread(Bmp_Data, 1, Real_width, Bmp_Fp);
        for (j = 0; j < imgp->width; j++) {
            imgp->data[imgp->height - i - 1][j].b = Bmp_Data[j * 3];
            imgp->data[imgp->height - i - 1][j].g = Bmp_Data[j * 3 + 1];
            imgp->data[imgp->height - i - 1][j].r = Bmp_Data[j * 3 + 2];
        }
    }

    /* ���I�Ɋm�ۂ����z��̈�̉�� */
    free(Bmp_Data);

    /* �t�@�C���N���[�Y */
    fclose(Bmp_Fp);
}

/*
   �֐���: WriteBmp
   ����  : char *filename, img *tp
   �Ԃ�l: void
   ����  : 2�����z�� tp->data �̓��e���摜�f�[�^�Ƃ���, 24�r�b�g
           bmp�`���̃t�@�C�� filename �ɏ����o��.
*/
void bmpLib::WriteBmp(const char* filename, img* tp) {

    unsigned char Bmp_headbuf[HEADERSIZE];/* �w�b�_���i�[���邽�߂̕ϐ�          */
    unsigned char Bmp_Pallet[PALLETSIZE]; /* �J���[�p���b�g���i�[                */

    char Bmp_type[2];                     /* �t�@�C���^�C�v "BM"                 */
    unsigned long Bmp_size;               /* bmp�t�@�C���̃T�C�Y (�o�C�g)        */
    unsigned int Bmp_info_header_size; /* ���w�b�_�̃T�C�Y = 40             */
    unsigned int Bmp_header_size;      /* �w�b�_�T�C�Y = 54*/
    long Bmp_height;                      /* ���� (�s�N�Z��)                     */
    long Bmp_width;                       /* ��   (�s�N�Z��)                     */
    unsigned short Bmp_planes;          /* �v���[���� ��� 1                   */
    unsigned short Bmp_color;          /* �F (�r�b�g)     24                  */
    long Bmp_comp;                        /* ���k���@         0                  */
    long Bmp_image_size;                  /* �摜�����̃t�@�C���T�C�Y (�o�C�g)   */
    long Bmp_xppm;                        /* �����𑜓x (ppm)                    */
    long Bmp_yppm;                        /* �����𑜓x (ppm)                    */

    int i, j;
    int Real_width;
    FILE* Out_Fp;
    fopen_s(&Out_Fp, filename, "wb");  /* �t�@�C���I�[�v�� */
    unsigned char* Bmp_Data;     /* �摜�f�[�^��1�s���i�[               */

    if (Out_Fp == NULL) {
        fprintf(stderr, "Error: file %s couldn\'t open for write!\n", filename);
        exit(1);
    }

    Bmp_color = 24;
    Bmp_header_size = HEADERSIZE;
    Bmp_info_header_size = 40;
    Bmp_planes = 1;
    Bmp_comp = 0;

    Real_width = tp->width * 3 + tp->width % 4;  /* 4byte ���E�ɂ��킹�邽�߂Ɏ��ۂ̕��̌v�Z */

    /* �z��̈�̓��I�m��. ���s�����ꍇ�̓G���[���b�Z�[�W���o�͂��ďI�� */
    if ((Bmp_Data = (unsigned char*)calloc(Real_width, sizeof(unsigned char))) == NULL) {
        fprintf(stderr, "Error: Memory allocation failed for Bmp_Data!\n");
        exit(1);
    }

    /* �w�b�_���̏��� */
    Bmp_xppm = Bmp_yppm = 0;
    Bmp_image_size = tp->height * Real_width;
    Bmp_size = Bmp_image_size + HEADERSIZE;
    Bmp_headbuf[0] = 'B'; Bmp_headbuf[1] = 'M';
    memcpy(Bmp_headbuf + 2, &Bmp_size, sizeof(Bmp_size));
    Bmp_headbuf[6] = Bmp_headbuf[7] = Bmp_headbuf[8] = Bmp_headbuf[9] = 0;
    memcpy(Bmp_headbuf + 10, &Bmp_header_size, sizeof(Bmp_header_size));
    Bmp_headbuf[11] = Bmp_headbuf[12] = Bmp_headbuf[13] = 0;
    memcpy(Bmp_headbuf + 14, &Bmp_info_header_size, sizeof(Bmp_info_header_size));
    Bmp_headbuf[15] = Bmp_headbuf[16] = Bmp_headbuf[17] = 0;
    memcpy(Bmp_headbuf + 18, &tp->width, sizeof(Bmp_width));
    memcpy(Bmp_headbuf + 22, &tp->height, sizeof(Bmp_height));
    memcpy(Bmp_headbuf + 26, &Bmp_planes, sizeof(Bmp_planes));
    memcpy(Bmp_headbuf + 28, &Bmp_color, sizeof(Bmp_color));
    memcpy(Bmp_headbuf + 30, &Bmp_comp, sizeof(Bmp_comp));
    memcpy(Bmp_headbuf + 34, &Bmp_image_size, sizeof(Bmp_image_size));
    memcpy(Bmp_headbuf + 38, &Bmp_xppm, sizeof(Bmp_xppm));
    memcpy(Bmp_headbuf + 42, &Bmp_yppm, sizeof(Bmp_yppm));
    Bmp_headbuf[46] = Bmp_headbuf[47] = Bmp_headbuf[48] = Bmp_headbuf[49] = 0;
    Bmp_headbuf[50] = Bmp_headbuf[51] = Bmp_headbuf[52] = Bmp_headbuf[53] = 0;

    /* �w�b�_��񏑂��o�� */
    fwrite(Bmp_headbuf, sizeof(unsigned char), HEADERSIZE, Out_Fp);

    /* �摜�f�[�^�����o�� */
    for (i = 0; i < tp->height; i++) {
        for (j = 0; j < tp->width; j++) {
            Bmp_Data[j * 3] = tp->data[tp->height - i - 1][j].b;
            Bmp_Data[j * 3 + 1] = tp->data[tp->height - i - 1][j].g;
            Bmp_Data[j * 3 + 2] = tp->data[tp->height - i - 1][j].r;
        }
        for (j = tp->width * 3; j < Real_width; j++) {
            Bmp_Data[j] = 0;
        }
        fwrite(Bmp_Data, sizeof(unsigned char), Real_width, Out_Fp);
    }

    /* ���I�Ɋm�ۂ����z��̈�̉�� */
    free(Bmp_Data);

    /* �t�@�C���N���[�Y */
    fclose(Out_Fp);
}

/*
   �֐���: PrintBmpInfo
   ����  : char *filename
   �Ԃ�l: void
   ����  : �����Ƃ��ė^������t�@�C���������� bmp �`���̉摜�t�@�C��
           �̑�������ʂɏo�͂���.
*/
void bmpLib::PrintBmpInfo(const char* filename) {

    unsigned char Bmp_headbuf[HEADERSIZE];/* �w�b�_���i�[���邽�߂̕ϐ�          */
    unsigned char Bmp_Pallet[PALLETSIZE]; /* �J���[�p���b�g���i�[                */

    char Bmp_type[2];                     /* �t�@�C���^�C�v "BM"                 */
    unsigned long Bmp_size;               /* bmp�t�@�C���̃T�C�Y (�o�C�g)        */
    unsigned int Bmp_info_header_size; /* ���w�b�_�̃T�C�Y = 40             */
    unsigned int Bmp_header_size;      /* �w�b�_�T�C�Y = 54*/
    long Bmp_height;                      /* ���� (�s�N�Z��)                     */
    long Bmp_width;                       /* ��   (�s�N�Z��)                     */
    unsigned short Bmp_planes;          /* �v���[���� ��� 1                   */
    unsigned short Bmp_color;          /* �F (�r�b�g)     24                  */
    long Bmp_comp;                        /* ���k���@         0                  */
    long Bmp_image_size;                  /* �摜�����̃t�@�C���T�C�Y (�o�C�g)   */
    long Bmp_xppm;                        /* �����𑜓x (ppm)                    */
    long Bmp_yppm;                        /* �����𑜓x (ppm)                    */

    FILE* Bmp_Fp;
    fopen_s(&Bmp_Fp ,filename, "rb");        /* �o�C�i�����[�h�ǂݍ��ݗp�ɃI�[�v��  */
    if (Bmp_Fp == NULL) {
        fprintf(stderr, "Error: file %s couldn\'t open for write!\n", filename);
        exit(1);
    }

    fread(Bmp_headbuf, sizeof(unsigned char), HEADERSIZE, Bmp_Fp);

    memcpy(&Bmp_type, Bmp_headbuf, sizeof(Bmp_type));
    if (strncmp(Bmp_type, "BM", 2) != 0) {
        fprintf(stderr, "Error: %s is not a bmp file.\n", filename);
        exit(1);
    }
    memcpy(&Bmp_size, Bmp_headbuf + 2, sizeof(Bmp_size));
    memcpy(&Bmp_width, Bmp_headbuf + 18, sizeof(Bmp_width));
    memcpy(&Bmp_height, Bmp_headbuf + 22, sizeof(Bmp_height));
    memcpy(&Bmp_color, Bmp_headbuf + 28, sizeof(Bmp_color));
    memcpy(&Bmp_comp, Bmp_headbuf + 30, sizeof(Bmp_comp));
    memcpy(&Bmp_image_size, Bmp_headbuf + 34, sizeof(Bmp_size));
    memcpy(&Bmp_xppm, Bmp_headbuf + 38, sizeof(Bmp_xppm));
    memcpy(&Bmp_yppm, Bmp_headbuf + 42, sizeof(Bmp_yppm));


    printf("�t�@�C����       = %s \n", filename);
    printf("�t�@�C���^�C�v   = %c%c \n", Bmp_type[0], Bmp_type[1]);
    printf("�t�@�C���T�C�Y   = %ld (byte)\n", Bmp_size);
    printf("��               = %ld (pixel)\n", Bmp_width);
    printf("����             = %ld (pixel)\n", Bmp_height);
    printf("�F               = %d (bit)\n", Bmp_color);
    printf("���k             = %ld\n", Bmp_comp);
    printf("�摜�����̃T�C�Y = %ld (byte)\n", Bmp_image_size);
    printf("�����𑜓x       = %ld (ppm)\n", Bmp_xppm);
    printf("�����𑜓x       = %ld (ppm)\n", Bmp_yppm);

    fclose(Bmp_Fp);
}

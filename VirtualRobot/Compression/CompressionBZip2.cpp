#include "Logging.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#include "../VirtualRobotException.h"
#include "CompressionBZip2.h"

#define BZ_RUN 0
#define BZ_FLUSH 1
#define BZ_FINISH 2

#define BZ_OK 0
#define BZ_RUN_OK 1
#define BZ_FLUSH_OK 2
#define BZ_FINISH_OK 3
#define BZ_STREAM_END 4
#define BZ_SEQUENCE_ERROR (-1)
#define BZ_PARAM_ERROR (-2)
#define BZ_MEM_ERROR (-3)
#define BZ_DATA_ERROR (-4)
#define BZ_DATA_ERROR_MAGIC (-5)
#define BZ_IO_ERROR (-6)
#define BZ_UNEXPECTED_EOF (-7)
#define BZ_OUTBUFF_FULL (-8)
#define BZ_CONFIG_ERROR (-9)

#define BZ_True ((Bool)1)
#define BZ_False ((Bool)0)

#define BZ_M_IDLE 1
#define BZ_M_RUNNING 2
#define BZ_M_FLUSHING 3
#define BZ_M_FINISHING 4

#define BZ_S_OUTPUT 1
#define BZ_S_INPUT 2

#define VPrintf0(zf) fprintf(stderr, zf)
#define VPrintf1(zf, za1) fprintf(stderr, zf, za1)
#define VPrintf2(zf, za1, za2) fprintf(stderr, zf, za1, za2)
#define VPrintf3(zf, za1, za2, za3) fprintf(stderr, zf, za1, za2, za3)
#define VPrintf4(zf, za1, za2, za3, za4) fprintf(stderr, zf, za1, za2, za3, za4)
#define VPrintf5(zf, za1, za2, za3, za4, za5) fprintf(stderr, zf, za1, za2, za3, za4, za5)

/*-- Header bytes. --*/

#define BZ_HDR_B 0x42 /* 'B' */
#define BZ_HDR_Z 0x5a /* 'Z' */
#define BZ_HDR_h 0x68 /* 'h' */
#define BZ_HDR_0 0x30 /* '0' */


#define BZ_X_IDLE 1
#define BZ_X_OUTPUT 2

#define BZ_X_MAGIC_1 10
#define BZ_X_MAGIC_2 11
#define BZ_X_MAGIC_3 12
#define BZ_X_MAGIC_4 13
#define BZ_X_BLKHDR_1 14
#define BZ_X_BLKHDR_2 15
#define BZ_X_BLKHDR_3 16
#define BZ_X_BLKHDR_4 17
#define BZ_X_BLKHDR_5 18
#define BZ_X_BLKHDR_6 19
#define BZ_X_BCRC_1 20
#define BZ_X_BCRC_2 21
#define BZ_X_BCRC_3 22
#define BZ_X_BCRC_4 23
#define BZ_X_RANDBIT 24
#define BZ_X_ORIGPTR_1 25
#define BZ_X_ORIGPTR_2 26
#define BZ_X_ORIGPTR_3 27
#define BZ_X_MAPPING_1 28
#define BZ_X_MAPPING_2 29
#define BZ_X_SELECTOR_1 30
#define BZ_X_SELECTOR_2 31
#define BZ_X_SELECTOR_3 32
#define BZ_X_CODING_1 33
#define BZ_X_CODING_2 34
#define BZ_X_CODING_3 35
#define BZ_X_MTF_1 36
#define BZ_X_MTF_2 37
#define BZ_X_MTF_3 38
#define BZ_X_MTF_4 39
#define BZ_X_MTF_5 40
#define BZ_X_MTF_6 41
#define BZ_X_ENDHDR_2 42
#define BZ_X_ENDHDR_3 43
#define BZ_X_ENDHDR_4 44
#define BZ_X_ENDHDR_5 45
#define BZ_X_ENDHDR_6 46
#define BZ_X_CCRC_1 47
#define BZ_X_CCRC_2 48
#define BZ_X_CCRC_3 49
#define BZ_X_CCRC_4 50

#define BZ_N_RADIX 2
#define BZ_N_QSORT 12
#define BZ_N_SHELL 18
#define BZ_N_OVERSHOOT (BZ_N_RADIX + BZ_N_QSORT + BZ_N_SHELL + 2)


#define SET_LL4(i, n)                                                                              \
    {                                                                                              \
        if (((i) & 0x1) == 0)                                                                      \
            s->ll4[(i) >> 1] = (s->ll4[(i) >> 1] & 0xf0) | (n);                                    \
        else                                                                                       \
            s->ll4[(i) >> 1] = (s->ll4[(i) >> 1] & 0x0f) | ((n) << 4);                             \
    }

#define GET_LL4(i) ((((UInt32)(s->ll4[(i) >> 1])) >> (((i) << 2) & 0x4)) & 0xF)

#define SET_LL(i, n)                                                                               \
    {                                                                                              \
        s->ll16[i] = (UInt16)(n & 0x0000ffff);                                                     \
        SET_LL4(i, n >> 16);                                                                       \
    }

#define GET_LL(i) (((UInt32)s->ll16[i]) | (GET_LL4(i) << 16))

#define BZ_GET_SMALL(cccc)                                                                         \
    /* c_tPos is unsigned, hence test < 0 is pointless. */                                         \
    if (s->tPos >= (UInt32)100000 * (UInt32)s->blockSize100k)                                      \
        return BZ_True;                                                                            \
    cccc = BZ2_indexIntoF(s->tPos, s->cftab);                                                      \
    s->tPos = GET_LL(s->tPos);


#define BZ_SETERR(eee)                                                                             \
    {                                                                                              \
        if (bzerror != NULL)                                                                       \
            *bzerror = eee;                                                                        \
        if (bzf != NULL)                                                                           \
            bzf->lastErr = eee;                                                                    \
    }

#define BZ_RAND_DECLS                                                                              \
    Int32 rNToGo;                                                                                  \
    Int32 rTPos

#define BZ_RAND_INIT_MASK                                                                          \
    s->rNToGo = 0;                                                                                 \
    s->rTPos = 0

#define BZ_RAND_MASK ((s->rNToGo == 1) ? 1 : 0)

#define BZ_RAND_UPD_MASK                                                                           \
    if (s->rNToGo == 0)                                                                            \
    {                                                                                              \
        s->rNToGo = BZ2_rNums[s->rTPos];                                                           \
        s->rTPos++;                                                                                \
        if (s->rTPos == 512)                                                                       \
            s->rTPos = 0;                                                                          \
    }                                                                                              \
    s->rNToGo--;


#define BZ_GET_FAST(cccc)                                                                          \
    /* c_tPos is unsigned, hence test < 0 is pointless. */                                         \
    if (s->tPos >= (UInt32)100000 * (UInt32)s->blockSize100k)                                      \
        return BZ_True;                                                                            \
    s->tPos = s->tt[s->tPos];                                                                      \
    cccc = (UChar)(s->tPos & 0xff);                                                                \
    s->tPos >>= 8;

#define BZ_GET_FAST_C(cccc)                                                                        \
    /* c_tPos is unsigned, hence test < 0 is pointless. */                                         \
    if (c_tPos >= (UInt32)100000 * (UInt32)ro_blockSize100k)                                       \
        return BZ_True;                                                                            \
    c_tPos = c_tt[c_tPos];                                                                         \
    cccc = (UChar)(c_tPos & 0xff);                                                                 \
    c_tPos >>= 8;


#define BZ_INITIALISE_CRC(crcVar)                                                                  \
    {                                                                                              \
        crcVar = 0xffffffffL;                                                                      \
    }

#define BZ_FINALISE_CRC(crcVar)                                                                    \
    {                                                                                              \
        crcVar = ~(crcVar);                                                                        \
    }

#define BZ_UPDATE_CRC(crcVar, cha)                                                                 \
    {                                                                                              \
        crcVar = (crcVar << 8) ^ BZ2_crc32Table[(crcVar >> 24) ^ ((UChar)cha)];                    \
    }


/*---------------------------------------------------*/
#define ADD_CHAR_TO_BLOCK(zs, zchh0)                                                               \
    {                                                                                              \
        UInt32 zchh = (UInt32)(zchh0);                                                             \
        /*-- fast track the common case --*/                                                       \
        if (zchh != zs->state_in_ch && zs->state_in_len == 1)                                      \
        {                                                                                          \
            UChar ch = (UChar)(zs->state_in_ch);                                                   \
            BZ_UPDATE_CRC(zs->blockCRC, ch);                                                       \
            zs->inUse[zs->state_in_ch] = BZ_True;                                                  \
            zs->block[zs->nblock] = (UChar)ch;                                                     \
            zs->nblock++;                                                                          \
            zs->state_in_ch = zchh;                                                                \
        }                                                                                          \
        else                                                                                       \
            /*-- general, uncommon cases --*/                                                      \
            if (zchh != zs->state_in_ch || zs->state_in_len == 255)                                \
            {                                                                                      \
                if (zs->state_in_ch < 256)                                                         \
                    add_pair_to_block(zs);                                                         \
                zs->state_in_ch = zchh;                                                            \
                zs->state_in_len = 1;                                                              \
            }                                                                                      \
            else                                                                                   \
            {                                                                                      \
                zs->state_in_len++;                                                                \
            }                                                                                      \
    }


#define BZALLOC(nnn) (strm->bzalloc)(strm->opaque, (nnn), 1)
#define BZFREE(ppp) (strm->bzfree)(strm->opaque, (ppp))


#ifndef NDEBUG
#define AssertD(cond, msg)                                                                         \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            fprintf(stderr, "\n\nlibbzip2(debug build): internal error\n\t%s\n", msg);             \
            exit(1);                                                                               \
        }                                                                                          \
    }
#define AssertH(cond, errcode)                                                                     \
    {                                                                                              \
        if (!(cond))                                                                               \
            VR_ERROR << errcode;                                                                   \
    }
#else
#define AssertD(cond, msg) /* */
#define AssertH(cond, errcode) /* */
#endif

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    const CompressionBZip2::UInt32 CompressionBZip2::BZ2_crc32Table[256] = {

        /*-- Ugly, innit? --*/

        0x00000000L, 0x04c11db7L, 0x09823b6eL, 0x0d4326d9L, 0x130476dcL, 0x17c56b6bL, 0x1a864db2L,
        0x1e475005L, 0x2608edb8L, 0x22c9f00fL, 0x2f8ad6d6L, 0x2b4bcb61L, 0x350c9b64L, 0x31cd86d3L,
        0x3c8ea00aL, 0x384fbdbdL, 0x4c11db70L, 0x48d0c6c7L, 0x4593e01eL, 0x4152fda9L, 0x5f15adacL,
        0x5bd4b01bL, 0x569796c2L, 0x52568b75L, 0x6a1936c8L, 0x6ed82b7fL, 0x639b0da6L, 0x675a1011L,
        0x791d4014L, 0x7ddc5da3L, 0x709f7b7aL, 0x745e66cdL, 0x9823b6e0L, 0x9ce2ab57L, 0x91a18d8eL,
        0x95609039L, 0x8b27c03cL, 0x8fe6dd8bL, 0x82a5fb52L, 0x8664e6e5L, 0xbe2b5b58L, 0xbaea46efL,
        0xb7a96036L, 0xb3687d81L, 0xad2f2d84L, 0xa9ee3033L, 0xa4ad16eaL, 0xa06c0b5dL, 0xd4326d90L,
        0xd0f37027L, 0xddb056feL, 0xd9714b49L, 0xc7361b4cL, 0xc3f706fbL, 0xceb42022L, 0xca753d95L,
        0xf23a8028L, 0xf6fb9d9fL, 0xfbb8bb46L, 0xff79a6f1L, 0xe13ef6f4L, 0xe5ffeb43L, 0xe8bccd9aL,
        0xec7dd02dL, 0x34867077L, 0x30476dc0L, 0x3d044b19L, 0x39c556aeL, 0x278206abL, 0x23431b1cL,
        0x2e003dc5L, 0x2ac12072L, 0x128e9dcfL, 0x164f8078L, 0x1b0ca6a1L, 0x1fcdbb16L, 0x018aeb13L,
        0x054bf6a4L, 0x0808d07dL, 0x0cc9cdcaL, 0x7897ab07L, 0x7c56b6b0L, 0x71159069L, 0x75d48ddeL,
        0x6b93dddbL, 0x6f52c06cL, 0x6211e6b5L, 0x66d0fb02L, 0x5e9f46bfL, 0x5a5e5b08L, 0x571d7dd1L,
        0x53dc6066L, 0x4d9b3063L, 0x495a2dd4L, 0x44190b0dL, 0x40d816baL, 0xaca5c697L, 0xa864db20L,
        0xa527fdf9L, 0xa1e6e04eL, 0xbfa1b04bL, 0xbb60adfcL, 0xb6238b25L, 0xb2e29692L, 0x8aad2b2fL,
        0x8e6c3698L, 0x832f1041L, 0x87ee0df6L, 0x99a95df3L, 0x9d684044L, 0x902b669dL, 0x94ea7b2aL,
        0xe0b41de7L, 0xe4750050L, 0xe9362689L, 0xedf73b3eL, 0xf3b06b3bL, 0xf771768cL, 0xfa325055L,
        0xfef34de2L, 0xc6bcf05fL, 0xc27dede8L, 0xcf3ecb31L, 0xcbffd686L, 0xd5b88683L, 0xd1799b34L,
        0xdc3abdedL, 0xd8fba05aL, 0x690ce0eeL, 0x6dcdfd59L, 0x608edb80L, 0x644fc637L, 0x7a089632L,
        0x7ec98b85L, 0x738aad5cL, 0x774bb0ebL, 0x4f040d56L, 0x4bc510e1L, 0x46863638L, 0x42472b8fL,
        0x5c007b8aL, 0x58c1663dL, 0x558240e4L, 0x51435d53L, 0x251d3b9eL, 0x21dc2629L, 0x2c9f00f0L,
        0x285e1d47L, 0x36194d42L, 0x32d850f5L, 0x3f9b762cL, 0x3b5a6b9bL, 0x0315d626L, 0x07d4cb91L,
        0x0a97ed48L, 0x0e56f0ffL, 0x1011a0faL, 0x14d0bd4dL, 0x19939b94L, 0x1d528623L, 0xf12f560eL,
        0xf5ee4bb9L, 0xf8ad6d60L, 0xfc6c70d7L, 0xe22b20d2L, 0xe6ea3d65L, 0xeba91bbcL, 0xef68060bL,
        0xd727bbb6L, 0xd3e6a601L, 0xdea580d8L, 0xda649d6fL, 0xc423cd6aL, 0xc0e2d0ddL, 0xcda1f604L,
        0xc960ebb3L, 0xbd3e8d7eL, 0xb9ff90c9L, 0xb4bcb610L, 0xb07daba7L, 0xae3afba2L, 0xaafbe615L,
        0xa7b8c0ccL, 0xa379dd7bL, 0x9b3660c6L, 0x9ff77d71L, 0x92b45ba8L, 0x9675461fL, 0x8832161aL,
        0x8cf30badL, 0x81b02d74L, 0x857130c3L, 0x5d8a9099L, 0x594b8d2eL, 0x5408abf7L, 0x50c9b640L,
        0x4e8ee645L, 0x4a4ffbf2L, 0x470cdd2bL, 0x43cdc09cL, 0x7b827d21L, 0x7f436096L, 0x7200464fL,
        0x76c15bf8L, 0x68860bfdL, 0x6c47164aL, 0x61043093L, 0x65c52d24L, 0x119b4be9L, 0x155a565eL,
        0x18197087L, 0x1cd86d30L, 0x029f3d35L, 0x065e2082L, 0x0b1d065bL, 0x0fdc1becL, 0x3793a651L,
        0x3352bbe6L, 0x3e119d3fL, 0x3ad08088L, 0x2497d08dL, 0x2056cd3aL, 0x2d15ebe3L, 0x29d4f654L,
        0xc5a92679L, 0xc1683bceL, 0xcc2b1d17L, 0xc8ea00a0L, 0xd6ad50a5L, 0xd26c4d12L, 0xdf2f6bcbL,
        0xdbee767cL, 0xe3a1cbc1L, 0xe760d676L, 0xea23f0afL, 0xeee2ed18L, 0xf0a5bd1dL, 0xf464a0aaL,
        0xf9278673L, 0xfde69bc4L, 0x89b8fd09L, 0x8d79e0beL, 0x803ac667L, 0x84fbdbd0L, 0x9abc8bd5L,
        0x9e7d9662L, 0x933eb0bbL, 0x97ffad0cL, 0xafb010b1L, 0xab710d06L, 0xa6322bdfL, 0xa2f33668L,
        0xbcb4666dL, 0xb8757bdaL, 0xb5365d03L, 0xb1f740b4L};
    const CompressionBZip2::Int32 CompressionBZip2::BZ2_rNums[512] = {
        619, 720, 127, 481, 931, 816, 813, 233, 566, 247, 985, 724, 205, 454, 863, 491, 741, 242,
        949, 214, 733, 859, 335, 708, 621, 574, 73,  654, 730, 472, 419, 436, 278, 496, 867, 210,
        399, 680, 480, 51,  878, 465, 811, 169, 869, 675, 611, 697, 867, 561, 862, 687, 507, 283,
        482, 129, 807, 591, 733, 623, 150, 238, 59,  379, 684, 877, 625, 169, 643, 105, 170, 607,
        520, 932, 727, 476, 693, 425, 174, 647, 73,  122, 335, 530, 442, 853, 695, 249, 445, 515,
        909, 545, 703, 919, 874, 474, 882, 500, 594, 612, 641, 801, 220, 162, 819, 984, 589, 513,
        495, 799, 161, 604, 958, 533, 221, 400, 386, 867, 600, 782, 382, 596, 414, 171, 516, 375,
        682, 485, 911, 276, 98,  553, 163, 354, 666, 933, 424, 341, 533, 870, 227, 730, 475, 186,
        263, 647, 537, 686, 600, 224, 469, 68,  770, 919, 190, 373, 294, 822, 808, 206, 184, 943,
        795, 384, 383, 461, 404, 758, 839, 887, 715, 67,  618, 276, 204, 918, 873, 777, 604, 560,
        951, 160, 578, 722, 79,  804, 96,  409, 713, 940, 652, 934, 970, 447, 318, 353, 859, 672,
        112, 785, 645, 863, 803, 350, 139, 93,  354, 99,  820, 908, 609, 772, 154, 274, 580, 184,
        79,  626, 630, 742, 653, 282, 762, 623, 680, 81,  927, 626, 789, 125, 411, 521, 938, 300,
        821, 78,  343, 175, 128, 250, 170, 774, 972, 275, 999, 639, 495, 78,  352, 126, 857, 956,
        358, 619, 580, 124, 737, 594, 701, 612, 669, 112, 134, 694, 363, 992, 809, 743, 168, 974,
        944, 375, 748, 52,  600, 747, 642, 182, 862, 81,  344, 805, 988, 739, 511, 655, 814, 334,
        249, 515, 897, 955, 664, 981, 649, 113, 974, 459, 893, 228, 433, 837, 553, 268, 926, 240,
        102, 654, 459, 51,  686, 754, 806, 760, 493, 403, 415, 394, 687, 700, 946, 670, 656, 610,
        738, 392, 760, 799, 887, 653, 978, 321, 576, 617, 626, 502, 894, 679, 243, 440, 680, 879,
        194, 572, 640, 724, 926, 56,  204, 700, 707, 151, 457, 449, 797, 195, 791, 558, 945, 679,
        297, 59,  87,  824, 713, 663, 412, 693, 342, 606, 134, 108, 571, 364, 631, 212, 174, 643,
        304, 329, 343, 97,  430, 751, 497, 314, 983, 374, 822, 928, 140, 206, 73,  263, 980, 736,
        876, 478, 430, 305, 170, 514, 364, 692, 829, 82,  855, 953, 676, 246, 369, 970, 294, 750,
        807, 827, 150, 790, 288, 923, 804, 378, 215, 828, 592, 281, 565, 555, 710, 82,  896, 831,
        547, 261, 524, 462, 293, 465, 502, 56,  661, 821, 976, 991, 658, 869, 905, 758, 745, 193,
        768, 550, 608, 933, 378, 286, 215, 979, 792, 961, 61,  688, 793, 644, 986, 403, 106, 366,
        905, 644, 372, 567, 466, 434, 645, 210, 389, 550, 919, 135, 780, 773, 635, 389, 707, 100,
        626, 958, 165, 504, 920, 176, 193, 713, 857, 265, 203, 50,  668, 108, 645, 990, 626, 197,
        510, 357, 358, 850, 858, 364, 936, 638};

    CompressionBZip2::CompressionBZip2(std::ostream* ofs) // , CompressionMode mode);
    {
        this->mode = eCompress;
        this->ofs = ofs;
        ifs = nullptr;
        THROW_VR_EXCEPTION_IF(!ofs, "Stream NULL");
        THROW_VR_EXCEPTION_IF(!this->ofs->good(), "Stream not good");
        bzFileData = nullptr;
        bzFileData = BZ2_bzWriteOpen(&currentError, this->ofs);
        THROW_VR_EXCEPTION_IF(!bzFileData, "Could not initialize compression...");
    }

    CompressionBZip2::CompressionBZip2(std::istream* ifs) // , CompressionMode mode);
    {
        this->mode = eUncompress;
        this->ifs = ifs;
        ofs = nullptr;
        THROW_VR_EXCEPTION_IF(!ifs, "Stream NULL");
        THROW_VR_EXCEPTION_IF(!this->ifs->good(), "Stream not good");
        bzFileData = nullptr;
        bzFileData = BZ2_bzReadOpen(&currentError, this->ifs, 0, 0, nullptr, 0);
        THROW_VR_EXCEPTION_IF(!bzFileData, "Could not initialize compression...");
    }

    CompressionBZip2::~CompressionBZip2()
    {
        close();
    }

    bool
    CompressionBZip2::close()
    {
        if (mode == eCompress && bzFileData && ofs)
        {
            BZ2_bzWriteClose(&currentError, bzFileData, 0, nullptr, nullptr);
            //ofs->close();
            //fclose (dataFile);
            bzFileData = nullptr;

            if (currentError < 0) //== BZ_IO_ERROR) {
            {
                VR_ERROR << "Could not close file?!" << std::endl;
                return false;
            }
        }
        else if (mode == eUncompress && bzFileData && ifs)
        {
            BZ2_bzReadClose(&currentError, bzFileData);
            bzFileData = nullptr;
            //fclose (dataFile);
        }

        return true;
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_bsInitWrite(EState* s)
    {
        s->bsLive = 0;
        s->bsBuff = 0;
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::bsFinishWrite(EState* s)
    {
        while (s->bsLive > 0)
        {
            s->zbits[s->numZ] = (UChar)(s->bsBuff >> 24);
            s->numZ++;
            s->bsBuff <<= 8;
            s->bsLive -= 8;
        }
    }

    /*---------------------------------------------------*/
#define bsNEEDW(nz)                                                                                \
    {                                                                                              \
        while (s->bsLive >= 8)                                                                     \
        {                                                                                          \
            s->zbits[s->numZ] = (UChar)(s->bsBuff >> 24);                                          \
            s->numZ++;                                                                             \
            s->bsBuff <<= 8;                                                                       \
            s->bsLive -= 8;                                                                        \
        }                                                                                          \
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::bsW(EState* s, Int32 n, UInt32 v)
    {
        bsNEEDW(n);
        s->bsBuff |= (v << (32 - s->bsLive - n));
        s->bsLive += n;
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::bsPutUInt32(EState* s, UInt32 u)
    {
        bsW(s, 8, (u >> 24) & 0xffL);
        bsW(s, 8, (u >> 16) & 0xffL);
        bsW(s, 8, (u >> 8) & 0xffL);
        bsW(s, 8, u & 0xffL);
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::bsPutUChar(EState* s, UChar c)
    {
        bsW(s, 8, (UInt32)c);
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::makeMaps_e(EState* s)
    {
        Int32 i;
        s->nInUse = 0;

        for (i = 0; i < 256; i++)
            if (s->inUse[i])
            {
                s->unseqToSeq[i] = s->nInUse;
                s->nInUse++;
            }
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::generateMTFValues(EState* s)
    {
        UChar yy[256];
        Int32 i, j;
        Int32 zPend;
        Int32 wr;
        Int32 EOB;

        /*
           After sorting (eg, here),
              s->arr1 [ 0 .. s->nblock-1 ] holds sorted order,
              and
              ((UChar*)s->arr2) [ 0 .. s->nblock-1 ]
              holds the original block data.

           The first thing to do is generate the MTF values,
           and put them in
              ((UInt16*)s->arr1) [ 0 .. s->nblock-1 ].
           Because there are strictly fewer or equal MTF values
           than block values, ptr values in this area are overwritten
           with MTF values only when they are no longer needed.

           The final compressed bitstream is generated into the
           area starting at
              (UChar*) (&((UChar*)s->arr2)[s->nblock])

           These storage aliases are set up in bzCompressInit(),
           except for the last one, which is arranged in
           compressBlock().
        */
        UInt32* ptr = s->ptr;
        UChar* block = s->block;
        UInt16* mtfv = s->mtfv;

        makeMaps_e(s);
        EOB = s->nInUse + 1;

        for (i = 0; i <= EOB; i++)
        {
            s->mtfFreq[i] = 0;
        }

        wr = 0;
        zPend = 0;

        for (i = 0; i < s->nInUse; i++)
        {
            yy[i] = (UChar)i;
        }

        for (i = 0; i < s->nblock; i++)
        {
            UChar ll_i;
            AssertD(wr <= i, "generateMTFValues(1)");
            j = ptr[i] - 1;

            if (j < 0)
            {
                j += s->nblock;
            }

            ll_i = s->unseqToSeq[block[j]];
            AssertD(ll_i < s->nInUse, "generateMTFValues(2a)");

            if (yy[0] == ll_i)
            {
                zPend++;
            }
            else
            {

                if (zPend > 0)
                {
                    zPend--;

                    while (BZ_True)
                    {
                        if (zPend & 1)
                        {
                            mtfv[wr] = BZ_RUNB;
                            wr++;
                            s->mtfFreq[BZ_RUNB]++;
                        }
                        else
                        {
                            mtfv[wr] = BZ_RUNA;
                            wr++;
                            s->mtfFreq[BZ_RUNA]++;
                        }

                        if (zPend < 2)
                        {
                            break;
                        }

                        zPend = (zPend - 2) / 2;
                    };

                    zPend = 0;
                }

                {
                    UChar rtmp;
                    UChar* ryy_j;
                    UChar rll_i;
                    rtmp = yy[1];
                    yy[1] = yy[0];
                    ryy_j = &(yy[1]);
                    rll_i = ll_i;

                    while (rll_i != rtmp)
                    {
                        UChar rtmp2;
                        ryy_j++;
                        rtmp2 = rtmp;
                        rtmp = *ryy_j;
                        *ryy_j = rtmp2;
                    };

                    yy[0] = rtmp;

                    j = ryy_j - &(yy[0]);

                    mtfv[wr] = j + 1;

                    wr++;

                    s->mtfFreq[j + 1]++;
                }
            }
        }

        if (zPend > 0)
        {
            zPend--;

            while (BZ_True)
            {
                if (zPend & 1)
                {
                    mtfv[wr] = BZ_RUNB;
                    wr++;
                    s->mtfFreq[BZ_RUNB]++;
                }
                else
                {
                    mtfv[wr] = BZ_RUNA;
                    wr++;
                    s->mtfFreq[BZ_RUNA]++;
                }

                if (zPend < 2)
                {
                    break;
                }

                zPend = (zPend - 2) / 2;
            };

            zPend = 0;
        }

        mtfv[wr] = EOB;
        wr++;
        s->mtfFreq[EOB]++;

        s->nMTF = wr;
    }

    /*---------------------------------------------------*/
#define BZ_LESSER_ICOST 0
#define BZ_GREATER_ICOST 15

    void
    CompressionBZip2::sendMTFValues(EState* s)
    {
        Int32 v, t, i, j, gs, ge, totc, bt, bc, iter;
        Int32 nSelectors, alphaSize, minLen, maxLen, selCtr;
        Int32 nGroups, nBytes;

        /*--
        UChar  len [BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
        is a global since the decoder also needs it.

        Int32  code[BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
        Int32  rfreq[BZ_N_GROUPS][BZ_MAX_ALPHA_SIZE];
        are also globals only used in this proc.
        Made global to keep stack frame size small.
        --*/


        UInt16 cost[BZ_N_GROUPS];
        Int32 fave[BZ_N_GROUPS];

        UInt16* mtfv = s->mtfv;

        if (s->verbosity >= 3)
            VPrintf3("      %d in block, %d after MTF & 1-2 coding, "
                     "%d+2 syms in use\n",
                     s->nblock,
                     s->nMTF,
                     s->nInUse);

        alphaSize = s->nInUse + 2;

        for (t = 0; t < BZ_N_GROUPS; t++)
            for (v = 0; v < alphaSize; v++)
            {
                s->len[t][v] = BZ_GREATER_ICOST;
            }

        /*--- Decide how many coding tables to use ---*/
        AssertH(s->nMTF > 0, 3001);

        if (s->nMTF < 200)
        {
            nGroups = 2;
        }
        else if (s->nMTF < 600)
        {
            nGroups = 3;
        }
        else if (s->nMTF < 1200)
        {
            nGroups = 4;
        }
        else if (s->nMTF < 2400)
        {
            nGroups = 5;
        }
        else
        {
            nGroups = 6;
        }

        /*--- Generate an initial set of coding tables ---*/
        {
            Int32 nPart, remF, tFreq, aFreq;

            nPart = nGroups;
            remF = s->nMTF;
            gs = 0;

            while (nPart > 0)
            {
                tFreq = remF / nPart;
                ge = gs - 1;
                aFreq = 0;

                while (aFreq < tFreq && ge < alphaSize - 1)
                {
                    ge++;
                    aFreq += s->mtfFreq[ge];
                }

                if (ge > gs && nPart != nGroups && nPart != 1 && ((nGroups - nPart) % 2 == 1))
                {
                    aFreq -= s->mtfFreq[ge];
                    ge--;
                }

                if (s->verbosity >= 3)
                    VPrintf5("      initial group %d, [%d .. %d], "
                             "has %d syms (%4.1f%%)\n",
                             nPart,
                             gs,
                             ge,
                             aFreq,
                             (100.0 * (float)aFreq) / (float)(s->nMTF));

                for (v = 0; v < alphaSize; v++)
                    if (v >= gs && v <= ge)
                    {
                        s->len[nPart - 1][v] = BZ_LESSER_ICOST;
                    }
                    else
                    {
                        s->len[nPart - 1][v] = BZ_GREATER_ICOST;
                    }

                nPart--;
                gs = ge + 1;
                remF -= aFreq;
            }
        }

        /*---
           Iterate up to BZ_N_ITERS times to improve the tables.
        ---*/
        for (iter = 0; iter < BZ_N_ITERS; iter++)
        {

            for (t = 0; t < nGroups; t++)
            {
                fave[t] = 0;
            }

            for (t = 0; t < nGroups; t++)
                for (v = 0; v < alphaSize; v++)
                {
                    s->rfreq[t][v] = 0;
                }

            /*---
              Set up an auxiliary length table which is used to fast-track
            the common case (nGroups == 6).
                 ---*/
            if (nGroups == 6)
            {
                for (v = 0; v < alphaSize; v++)
                {
                    s->len_pack[v][0] = (s->len[1][v] << 16) | s->len[0][v];
                    s->len_pack[v][1] = (s->len[3][v] << 16) | s->len[2][v];
                    s->len_pack[v][2] = (s->len[5][v] << 16) | s->len[4][v];
                }
            }

            nSelectors = 0;
            totc = 0;
            gs = 0;

            while (BZ_True)
            {

                /*--- Set group start & end marks. --*/
                if (gs >= s->nMTF)
                {
                    break;
                }

                ge = gs + BZ_G_SIZE - 1;

                if (ge >= s->nMTF)
                {
                    ge = s->nMTF - 1;
                }

                /*--
                   Calculate the cost of this group as coded
                   by each of the coding tables.
                --*/
                for (t = 0; t < nGroups; t++)
                {
                    cost[t] = 0;
                }

                if (nGroups == 6 && 50 == ge - gs + 1)
                {
                    /*--- fast track the common case ---*/
                    UInt32 cost01, cost23, cost45;
                    UInt16 icv;
                    cost01 = cost23 = cost45 = 0;

#define BZ_ITER(nn)                                                                                \
    icv = mtfv[gs + (nn)];                                                                         \
    cost01 += s->len_pack[icv][0];                                                                 \
    cost23 += s->len_pack[icv][1];                                                                 \
    cost45 += s->len_pack[icv][2];

                    BZ_ITER(0);
                    BZ_ITER(1);
                    BZ_ITER(2);
                    BZ_ITER(3);
                    BZ_ITER(4);
                    BZ_ITER(5);
                    BZ_ITER(6);
                    BZ_ITER(7);
                    BZ_ITER(8);
                    BZ_ITER(9);
                    BZ_ITER(10);
                    BZ_ITER(11);
                    BZ_ITER(12);
                    BZ_ITER(13);
                    BZ_ITER(14);
                    BZ_ITER(15);
                    BZ_ITER(16);
                    BZ_ITER(17);
                    BZ_ITER(18);
                    BZ_ITER(19);
                    BZ_ITER(20);
                    BZ_ITER(21);
                    BZ_ITER(22);
                    BZ_ITER(23);
                    BZ_ITER(24);
                    BZ_ITER(25);
                    BZ_ITER(26);
                    BZ_ITER(27);
                    BZ_ITER(28);
                    BZ_ITER(29);
                    BZ_ITER(30);
                    BZ_ITER(31);
                    BZ_ITER(32);
                    BZ_ITER(33);
                    BZ_ITER(34);
                    BZ_ITER(35);
                    BZ_ITER(36);
                    BZ_ITER(37);
                    BZ_ITER(38);
                    BZ_ITER(39);
                    BZ_ITER(40);
                    BZ_ITER(41);
                    BZ_ITER(42);
                    BZ_ITER(43);
                    BZ_ITER(44);
                    BZ_ITER(45);
                    BZ_ITER(46);
                    BZ_ITER(47);
                    BZ_ITER(48);
                    BZ_ITER(49);

#undef BZ_ITER

                    cost[0] = cost01 & 0xffff;
                    cost[1] = cost01 >> 16;
                    cost[2] = cost23 & 0xffff;
                    cost[3] = cost23 >> 16;
                    cost[4] = cost45 & 0xffff;
                    cost[5] = cost45 >> 16;
                }
                else
                {
                    /*--- slow version which correctly handles all situations ---*/
                    for (i = gs; i <= ge; i++)
                    {
                        UInt16 icv = mtfv[i];

                        for (t = 0; t < nGroups; t++)
                        {
                            cost[t] += s->len[t][icv];
                        }
                    }
                }

                /*--
                   Find the coding table which is best for this group,
                   and record its identity in the selector table.
                --*/
                bc = 999999999;
                bt = -1;

                for (t = 0; t < nGroups; t++)
                    if (cost[t] < bc)
                    {
                        bc = cost[t];
                        bt = t;
                    };

                totc += bc;

                fave[bt]++;

                s->selector[nSelectors] = bt;

                nSelectors++;

                /*--
                   Increment the symbol frequencies for the selected table.
                 --*/
                if (nGroups == 6 && 50 == ge - gs + 1)
                {
                    /*--- fast track the common case ---*/

#define BZ_ITUR(nn) s->rfreq[bt][mtfv[gs + (nn)]]++

                    BZ_ITUR(0);
                    BZ_ITUR(1);
                    BZ_ITUR(2);
                    BZ_ITUR(3);
                    BZ_ITUR(4);
                    BZ_ITUR(5);
                    BZ_ITUR(6);
                    BZ_ITUR(7);
                    BZ_ITUR(8);
                    BZ_ITUR(9);
                    BZ_ITUR(10);
                    BZ_ITUR(11);
                    BZ_ITUR(12);
                    BZ_ITUR(13);
                    BZ_ITUR(14);
                    BZ_ITUR(15);
                    BZ_ITUR(16);
                    BZ_ITUR(17);
                    BZ_ITUR(18);
                    BZ_ITUR(19);
                    BZ_ITUR(20);
                    BZ_ITUR(21);
                    BZ_ITUR(22);
                    BZ_ITUR(23);
                    BZ_ITUR(24);
                    BZ_ITUR(25);
                    BZ_ITUR(26);
                    BZ_ITUR(27);
                    BZ_ITUR(28);
                    BZ_ITUR(29);
                    BZ_ITUR(30);
                    BZ_ITUR(31);
                    BZ_ITUR(32);
                    BZ_ITUR(33);
                    BZ_ITUR(34);
                    BZ_ITUR(35);
                    BZ_ITUR(36);
                    BZ_ITUR(37);
                    BZ_ITUR(38);
                    BZ_ITUR(39);
                    BZ_ITUR(40);
                    BZ_ITUR(41);
                    BZ_ITUR(42);
                    BZ_ITUR(43);
                    BZ_ITUR(44);
                    BZ_ITUR(45);
                    BZ_ITUR(46);
                    BZ_ITUR(47);
                    BZ_ITUR(48);
                    BZ_ITUR(49);

#undef BZ_ITUR
                }
                else
                {
                    /*--- slow version which correctly handles all situations ---*/
                    for (i = gs; i <= ge; i++)
                    {
                        s->rfreq[bt][mtfv[i]]++;
                    }
                }

                gs = ge + 1;
            }

            if (s->verbosity >= 3)
            {
                VPrintf2("      pass %d: size is %d, grp uses are ", iter + 1, totc / 8);

                for (t = 0; t < nGroups; t++)
                {
                    VPrintf1("%d ", fave[t]);
                }

                VPrintf0("\n");
            }

            /*--
              Recompute the tables based on the accumulated frequencies.
            --*/
            /* maxLen was changed from 20 to 17 in CompressionBZip2-1.0.3.  See
               comment in huffman.c for details. */
            for (t = 0; t < nGroups; t++)
                BZ2_hbMakeCodeLengths(&(s->len[t][0]), &(s->rfreq[t][0]), alphaSize, 17 /*20*/);
        }


        AssertH(nGroups < 8, 3002);
        AssertH(nSelectors < 32768 && nSelectors <= (2 + (900000 / BZ_G_SIZE)), 3003);


        /*--- Compute MTF values for the selectors. ---*/
        {
            UChar pos[BZ_N_GROUPS], ll_i, tmp2, tmp;

            for (i = 0; i < nGroups; i++)
            {
                pos[i] = i;
            }

            for (i = 0; i < nSelectors; i++)
            {
                ll_i = s->selector[i];
                j = 0;
                tmp = pos[j];

                while (ll_i != tmp)
                {
                    j++;
                    tmp2 = tmp;
                    tmp = pos[j];
                    pos[j] = tmp2;
                };

                pos[0] = tmp;

                s->selectorMtf[i] = j;
            }
        };

        /*--- Assign actual codes for the tables. --*/
        for (t = 0; t < nGroups; t++)
        {
            minLen = 32;
            maxLen = 0;

            for (i = 0; i < alphaSize; i++)
            {
                if (s->len[t][i] > maxLen)
                {
                    maxLen = s->len[t][i];
                }

                if (s->len[t][i] < minLen)
                {
                    minLen = s->len[t][i];
                }
            }

            AssertH(!(maxLen > 17 /*20*/), 3004);
            AssertH(!(minLen < 1), 3005);
            BZ2_hbAssignCodes(&(s->code[t][0]), &(s->len[t][0]), minLen, maxLen, alphaSize);
        }

        /*--- Transmit the mapping table. ---*/
        {
            Bool inUse16[16];

            for (i = 0; i < 16; i++)
            {
                inUse16[i] = BZ_False;

                for (j = 0; j < 16; j++)
                    if (s->inUse[i * 16 + j])
                    {
                        inUse16[i] = BZ_True;
                    }
            }

            nBytes = s->numZ;

            for (i = 0; i < 16; i++)
                if (inUse16[i])
                {
                    bsW(s, 1, 1);
                }
                else
                {
                    bsW(s, 1, 0);
                }

            for (i = 0; i < 16; i++)
                if (inUse16[i])
                    for (j = 0; j < 16; j++)
                    {
                        if (s->inUse[i * 16 + j])
                        {
                            bsW(s, 1, 1);
                        }
                        else
                        {
                            bsW(s, 1, 0);
                        }
                    }

            if (s->verbosity >= 3)
            {
                VPrintf1("      bytes: mapping %d, ", s->numZ - nBytes);
            }
        }

        /*--- Now the selectors. ---*/
        nBytes = s->numZ;
        bsW(s, 3, nGroups);
        bsW(s, 15, nSelectors);

        for (i = 0; i < nSelectors; i++)
        {
            for (j = 0; j < s->selectorMtf[i]; j++)
            {
                bsW(s, 1, 1);
            }

            bsW(s, 1, 0);
        }

        if (s->verbosity >= 3)
        {
            VPrintf1("selectors %d, ", s->numZ - nBytes);
        }

        /*--- Now the coding tables. ---*/
        nBytes = s->numZ;

        for (t = 0; t < nGroups; t++)
        {
            Int32 curr = s->len[t][0];
            bsW(s, 5, curr);

            for (i = 0; i < alphaSize; i++)
            {
                while (curr < s->len[t][i])
                {
                    bsW(s, 2, 2);
                    curr++; /* 10 */
                };

                while (curr > s->len[t][i])
                {
                    bsW(s, 2, 3);
                    curr--; /* 11 */
                };

                bsW(s, 1, 0);
            }
        }

        if (s->verbosity >= 3)
        {
            VPrintf1("code lengths %d, ", s->numZ - nBytes);
        }

        /*--- And finally, the block data proper ---*/
        nBytes = s->numZ;
        selCtr = 0;
        gs = 0;

        while (BZ_True)
        {
            if (gs >= s->nMTF)
            {
                break;
            }

            ge = gs + BZ_G_SIZE - 1;

            if (ge >= s->nMTF)
            {
                ge = s->nMTF - 1;
            }

            AssertH(s->selector[selCtr] < nGroups, 3006);

            if (nGroups == 6 && 50 == ge - gs + 1)
            {
                /*--- fast track the common case ---*/
                UInt16 mtfv_i;
                UChar* s_len_sel_selCtr = &(s->len[s->selector[selCtr]][0]);
                Int32* s_code_sel_selCtr = &(s->code[s->selector[selCtr]][0]);

#define BZ_ITAH(nn)                                                                                \
    mtfv_i = mtfv[gs + (nn)];                                                                      \
    bsW(s, s_len_sel_selCtr[mtfv_i], s_code_sel_selCtr[mtfv_i])

                BZ_ITAH(0);
                BZ_ITAH(1);
                BZ_ITAH(2);
                BZ_ITAH(3);
                BZ_ITAH(4);
                BZ_ITAH(5);
                BZ_ITAH(6);
                BZ_ITAH(7);
                BZ_ITAH(8);
                BZ_ITAH(9);
                BZ_ITAH(10);
                BZ_ITAH(11);
                BZ_ITAH(12);
                BZ_ITAH(13);
                BZ_ITAH(14);
                BZ_ITAH(15);
                BZ_ITAH(16);
                BZ_ITAH(17);
                BZ_ITAH(18);
                BZ_ITAH(19);
                BZ_ITAH(20);
                BZ_ITAH(21);
                BZ_ITAH(22);
                BZ_ITAH(23);
                BZ_ITAH(24);
                BZ_ITAH(25);
                BZ_ITAH(26);
                BZ_ITAH(27);
                BZ_ITAH(28);
                BZ_ITAH(29);
                BZ_ITAH(30);
                BZ_ITAH(31);
                BZ_ITAH(32);
                BZ_ITAH(33);
                BZ_ITAH(34);
                BZ_ITAH(35);
                BZ_ITAH(36);
                BZ_ITAH(37);
                BZ_ITAH(38);
                BZ_ITAH(39);
                BZ_ITAH(40);
                BZ_ITAH(41);
                BZ_ITAH(42);
                BZ_ITAH(43);
                BZ_ITAH(44);
                BZ_ITAH(45);
                BZ_ITAH(46);
                BZ_ITAH(47);
                BZ_ITAH(48);
                BZ_ITAH(49);

#undef BZ_ITAH
            }
            else
            {
                /*--- slow version which correctly handles all situations ---*/
                for (i = gs; i <= ge; i++)
                {
                    bsW(s,
                        s->len[s->selector[selCtr]][mtfv[i]],
                        s->code[s->selector[selCtr]][mtfv[i]]);
                }
            }


            gs = ge + 1;
            selCtr++;
        }

        AssertH(selCtr == nSelectors, 3007);

        if (s->verbosity >= 3)
        {
            VPrintf1("codes %d\n", s->numZ - nBytes);
        }
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_compressBlock(EState* s, Bool is_last_block)
    {
        if (s->nblock > 0)
        {

            BZ_FINALISE_CRC(s->blockCRC);
            s->combinedCRC = (s->combinedCRC << 1) | (s->combinedCRC >> 31);
            s->combinedCRC ^= s->blockCRC;

            if (s->blockNo > 1)
            {
                s->numZ = 0;
            }

            if (s->verbosity >= 2)
                VPrintf4("    block %d: crc = 0x%08x, "
                         "combined CRC = 0x%08x, size = %d\n",
                         s->blockNo,
                         s->blockCRC,
                         s->combinedCRC,
                         s->nblock);

            BZ2_blockSort(s);
        }

        s->zbits = (UChar*)(&((UChar*)s->arr2)[s->nblock]);

        /*-- If this is the first block, create the stream header. --*/
        if (s->blockNo == 1)
        {
            BZ2_bsInitWrite(s);
            bsPutUChar(s, BZ_HDR_B);
            bsPutUChar(s, BZ_HDR_Z);
            bsPutUChar(s, BZ_HDR_h);
            bsPutUChar(s, (UChar)(BZ_HDR_0 + s->blockSize100k));
        }

        if (s->nblock > 0)
        {

            bsPutUChar(s, 0x31);
            bsPutUChar(s, 0x41);
            bsPutUChar(s, 0x59);
            bsPutUChar(s, 0x26);
            bsPutUChar(s, 0x53);
            bsPutUChar(s, 0x59);

            /*-- Now the block's CRC, so it is in a known place. --*/
            bsPutUInt32(s, s->blockCRC);

            /*--
               Now a single bit indicating (non-)randomisation.
               As of version 0.9.5, we use a better sorting algorithm
               which makes randomisation unnecessary.  So always set
               the randomised bit to 'no'.  Of course, the decoder
               still needs to be able to handle randomised blocks
               so as to maintain backwards compatibility with
               older versions of CompressionBZip2.
            --*/
            bsW(s, 1, 0);

            bsW(s, 24, s->origPtr);
            generateMTFValues(s);
            sendMTFValues(s);
        }


        /*-- If this is the last block, add the stream trailer. --*/
        if (is_last_block)
        {

            bsPutUChar(s, 0x17);
            bsPutUChar(s, 0x72);
            bsPutUChar(s, 0x45);
            bsPutUChar(s, 0x38);
            bsPutUChar(s, 0x50);
            bsPutUChar(s, 0x90);
            bsPutUInt32(s, s->combinedCRC);

            if (s->verbosity >= 2)
            {
                VPrintf1("    final combined CRC = 0x%08x\n   ", s->combinedCRC);
            }

            bsFinishWrite(s);
        }
    }

    /*---------------------------------------------*/
    /* Pre:
          nblock > 0
          arr2 exists for [0 .. nblock-1 +N_OVERSHOOT]
          ((UChar*)arr2)  [0 .. nblock-1] holds block
          arr1 exists for [0 .. nblock-1]

       Post:
          ((UChar*)arr2) [0 .. nblock-1] holds block
          All other areas of block destroyed
          ftab [ 0 .. 65536 ] destroyed
          arr1 [0 .. nblock-1] holds sorted order
    */
    void
    CompressionBZip2::BZ2_blockSort(CompressionBZip2::EState* s)
    {
        UInt32* ptr = s->ptr;
        UChar* block = s->block;
        UInt32* ftab = s->ftab;
        Int32 nblock = s->nblock;
        Int32 verb = s->verbosity;
        Int32 wfact = s->workFactor;
        UInt16* quadrant;
        Int32 budget;
        Int32 budgetInit;
        Int32 i;

        if (nblock < 10000)
        {
            fallbackSort(s->arr1, s->arr2, ftab, nblock, verb);
        }
        else
        {
            /* Calculate the location for quadrant, remembering to get
               the alignment right.  Assumes that &(block[0]) is at least
               2-byte aligned -- this should be ok since block is really
               the first section of arr2.
            */
            i = nblock + BZ_N_OVERSHOOT;

            if (i & 1)
            {
                i++;
            }

            quadrant = (UInt16*)(&(block[i]));

            /* (wfact-1) / 3 puts the default-factor-30
               transition point at very roughly the same place as
               with v0.1 and v0.9.0.
               Not that it particularly matters any more, since the
               resulting compressed stream is now the same regardless
               of whether or not we use the main sort or fallback sort.
            */
            if (wfact < 1)
            {
                wfact = 1;
            }

            if (wfact > 100)
            {
                wfact = 100;
            }

            budgetInit = nblock * ((wfact - 1) / 3);
            budget = budgetInit;

            mainSort(ptr, block, quadrant, ftab, nblock, verb, &budget);

            if (verb >= 3)
                VPrintf3("      %d work, %d block, ratio %5.2f\n",
                         budgetInit - budget,
                         nblock,
                         (float)(budgetInit - budget) / (float)(nblock == 0 ? 1 : nblock));

            if (budget < 0)
            {
                if (verb >= 2)
                    VPrintf0("    too repetitive; using fallback"
                             " sorting algorithm\n");

                fallbackSort(s->arr1, s->arr2, ftab, nblock, verb);
            }
        }

        s->origPtr = -1;

        for (i = 0; i < s->nblock; i++)
            if (ptr[i] == 0)
            {
                s->origPtr = i;
                break;
            };

        AssertH(s->origPtr != -1, 1003);
    }

    /*---------------------------------------------*/
    /* Pre:
          nblock > N_OVERSHOOT
          block32 exists for [0 .. nblock-1 +N_OVERSHOOT]
          ((UChar*)block32) [0 .. nblock-1] holds block
          ptr exists for [0 .. nblock-1]

       Post:
          ((UChar*)block32) [0 .. nblock-1] holds block
          All other areas of block32 destroyed
          ftab [0 .. 65536 ] destroyed
          ptr [0 .. nblock-1] holds sorted order
          if (*budget < 0), sorting was abandoned
    */

#define BIGFREQ(b) (ftab[((b) + 1) << 8] - ftab[(b) << 8])
#define SETMASK (1 << 21)
#define CLEARMASK (~(SETMASK))

    void
    CompressionBZip2::mainSort(UInt32* ptr,
                               UChar* block,
                               UInt16* quadrant,
                               UInt32* ftab,
                               Int32 nblock,
                               Int32 verb,
                               Int32* budget)
    {
        Int32 i, j, k, ss, sb;
        Int32 runningOrder[256];
        Bool bigDone[256];
        Int32 copyStart[256];
        Int32 copyEnd[256];
        UChar c1;
        Int32 numQSorted;
        UInt16 s;

        if (verb >= 4)
        {
            VPrintf0("        main sort initialise ...\n");
        }

        /*-- set up the 2-byte frequency table --*/
        for (i = 65536; i >= 0; i--)
        {
            ftab[i] = 0;
        }

        j = block[0] << 8;
        i = nblock - 1;

        for (; i >= 3; i -= 4)
        {
            quadrant[i] = 0;
            j = (j >> 8) | (((UInt16)block[i]) << 8);
            ftab[j]++;
            quadrant[i - 1] = 0;
            j = (j >> 8) | (((UInt16)block[i - 1]) << 8);
            ftab[j]++;
            quadrant[i - 2] = 0;
            j = (j >> 8) | (((UInt16)block[i - 2]) << 8);
            ftab[j]++;
            quadrant[i - 3] = 0;
            j = (j >> 8) | (((UInt16)block[i - 3]) << 8);
            ftab[j]++;
        }

        for (; i >= 0; i--)
        {
            quadrant[i] = 0;
            j = (j >> 8) | (((UInt16)block[i]) << 8);
            ftab[j]++;
        }

        /*-- (emphasises close relationship of block & quadrant) --*/
        for (i = 0; i < BZ_N_OVERSHOOT; i++)
        {
            block[nblock + i] = block[i];
            quadrant[nblock + i] = 0;
        }

        if (verb >= 4)
        {
            VPrintf0("        bucket sorting ...\n");
        }

        /*-- Complete the initial radix sort --*/
        for (i = 1; i <= 65536; i++)
        {
            ftab[i] += ftab[i - 1];
        }

        s = block[0] << 8;
        i = nblock - 1;

        for (; i >= 3; i -= 4)
        {
            s = (s >> 8) | (block[i] << 8);
            j = ftab[s] - 1;
            ftab[s] = j;
            ptr[j] = i;
            s = (s >> 8) | (block[i - 1] << 8);
            j = ftab[s] - 1;
            ftab[s] = j;
            ptr[j] = i - 1;
            s = (s >> 8) | (block[i - 2] << 8);
            j = ftab[s] - 1;
            ftab[s] = j;
            ptr[j] = i - 2;
            s = (s >> 8) | (block[i - 3] << 8);
            j = ftab[s] - 1;
            ftab[s] = j;
            ptr[j] = i - 3;
        }

        for (; i >= 0; i--)
        {
            s = (s >> 8) | (block[i] << 8);
            j = ftab[s] - 1;
            ftab[s] = j;
            ptr[j] = i;
        }

        /*--
           Now ftab contains the first loc of every small bucket.
           Calculate the running order, from smallest to largest
           big bucket.
        --*/
        for (i = 0; i <= 255; i++)
        {
            bigDone[i] = BZ_False;
            runningOrder[i] = i;
        }

        {
            Int32 vv;
            Int32 h = 1;

            do
            {
                h = 3 * h + 1;
            } while (h <= 256);

            do
            {
                h = h / 3;

                for (i = h; i <= 255; i++)
                {
                    vv = runningOrder[i];
                    j = i;

                    while (BIGFREQ(runningOrder[j - h]) > BIGFREQ(vv))
                    {
                        runningOrder[j] = runningOrder[j - h];
                        j = j - h;

                        if (j <= (h - 1))
                        {
                            goto zero;
                        }
                    }

                zero:
                    runningOrder[j] = vv;
                }
            } while (h != 1);
        }

        /*--
           The main sorting loop.
        --*/

        numQSorted = 0;

        for (i = 0; i <= 255; i++)
        {

            /*--
               Process big buckets, starting with the least full.
               Basically this is a 3-step process in which we call
               mainQSort3 to sort the small buckets [ss, j], but
               also make a big effort to avoid the calls if we can.
            --*/
            ss = runningOrder[i];

            /*--
               Step 1:
               Complete the big bucket [ss] by quicksorting
               any unsorted small buckets [ss, j], for j != ss.
               Hopefully previous pointer-scanning phases have already
               completed many of the small buckets [ss, j], so
               we don't have to sort them at all.
            --*/
            for (j = 0; j <= 255; j++)
            {
                if (j != ss)
                {
                    sb = (ss << 8) + j;

                    if (!(ftab[sb] & SETMASK))
                    {
                        Int32 lo = ftab[sb] & CLEARMASK;
                        Int32 hi = (ftab[sb + 1] & CLEARMASK) - 1;

                        if (hi > lo)
                        {
                            if (verb >= 4)
                                VPrintf4("        qsort [0x%x, 0x%x]   "
                                         "done %d   this %d\n",
                                         ss,
                                         j,
                                         numQSorted,
                                         hi - lo + 1);

                            mainQSort3(ptr, block, quadrant, nblock, lo, hi, BZ_N_RADIX, budget);
                            numQSorted += (hi - lo + 1);

                            if (*budget < 0)
                            {
                                return;
                            }
                        }
                    }

                    ftab[sb] |= SETMASK;
                }
            }

            AssertH(!bigDone[ss], 1006);

            /*--
               Step 2:
               Now scan this big bucket [ss] so as to synthesise the
               sorted order for small buckets [t, ss] for all t,
               including, magically, the bucket [ss,ss] too.
               This will avoid doing Real Work in subsequent Step 1's.
            --*/
            {
                for (j = 0; j <= 255; j++)
                {
                    copyStart[j] = ftab[(j << 8) + ss] & CLEARMASK;
                    copyEnd[j] = (ftab[(j << 8) + ss + 1] & CLEARMASK) - 1;
                }

                for (j = ftab[ss << 8] & CLEARMASK; j < copyStart[ss]; j++)
                {
                    k = ptr[j] - 1;

                    if (k < 0)
                    {
                        k += nblock;
                    }

                    c1 = block[k];

                    if (!bigDone[c1])
                    {
                        ptr[copyStart[c1]++] = k;
                    }
                }

                for (j = (ftab[(ss + 1) << 8] & CLEARMASK) - 1; j > copyEnd[ss]; j--)
                {
                    k = ptr[j] - 1;

                    if (k < 0)
                    {
                        k += nblock;
                    }

                    c1 = block[k];

                    if (!bigDone[c1])
                    {
                        ptr[copyEnd[c1]--] = k;
                    }
                }
            }

            AssertH((copyStart[ss] - 1 == copyEnd[ss]) ||
                        /* Extremely rare case missing in bzip2-1.0.0 and 1.0.1.
                       Necessity for this case is demonstrated by compressing
                       a sequence of approximately 48.5 million of character
                       251; 1.0.0/1.0.1 will then die here. */
                        (copyStart[ss] == 0 && copyEnd[ss] == nblock - 1),
                    1007)

                for (j = 0; j <= 255; j++)
            {
                ftab[(j << 8) + ss] |= SETMASK;
            }

            /*--
               Step 3:
               The [ss] big bucket is now done.  Record this fact,
               and update the quadrant descriptors.  Remember to
               update quadrants in the overshoot area too, if
               necessary.  The "if (i < 255)" test merely skips
               this updating for the last bucket processed, since
               updating for the last bucket is pointless.

               The quadrant array provides a way to incrementally
               cache sort orderings, as they appear, so as to
               make subsequent comparisons in fullGtU() complete
               faster.  For repetitive blocks this makes a big
               difference (but not big enough to be able to avoid
               the fallback sorting mechanism, exponential radix sort).

               The precise meaning is: at all times:

                  for 0 <= i < nblock and 0 <= j <= nblock

                  if block[i] != block[j],

                     then the relative values of quadrant[i] and
                          quadrant[j] are meaningless.

                     else {
                        if quadrant[i] < quadrant[j]
                           then the string starting at i lexicographically
                           precedes the string starting at j

                        else if quadrant[i] > quadrant[j]
                           then the string starting at j lexicographically
                           precedes the string starting at i

                        else
                           the relative ordering of the strings starting
                           at i and j has not yet been determined.
                     }
            --*/
            bigDone[ss] = BZ_True;

            if (i < 255)
            {
                Int32 bbStart = ftab[ss << 8] & CLEARMASK;
                Int32 bbSize = (ftab[(ss + 1) << 8] & CLEARMASK) - bbStart;
                Int32 shifts = 0;

                while ((bbSize >> shifts) > 65534)
                {
                    shifts++;
                }

                for (j = bbSize - 1; j >= 0; j--)
                {
                    Int32 a2update = ptr[bbStart + j];
                    UInt16 qVal = (UInt16)(j >> shifts);
                    quadrant[a2update] = qVal;

                    if (a2update < BZ_N_OVERSHOOT)
                    {
                        quadrant[a2update + nblock] = qVal;
                    }
                }

                AssertH(((bbSize - 1) >> shifts) <= 65535, 1002);
            }
        }

        if (verb >= 4)
            VPrintf3("        %d pointers, %d sorted, %d scanned\n",
                     nblock,
                     numQSorted,
                     nblock - numQSorted);
    }

#undef BIGFREQ
#undef SETMASK
#undef CLEARMASK

    /*---------------------------------------------*/
    /*--- The main, O(N^2 log(N)) sorting       ---*/
    /*--- algorithm.  Faster for "normal"       ---*/
    /*--- non-repetitive blocks.                ---*/
    /*---------------------------------------------*/

    /*---------------------------------------------*/
    CompressionBZip2::Bool
    CompressionBZip2::mainGtU(UInt32 i1,
                              UInt32 i2,
                              UChar* block,
                              UInt16* quadrant,
                              UInt32 nblock,
                              Int32* budget)
    {
        Int32 k;
        UChar c1, c2;
        UInt16 s1, s2;

        AssertD(i1 != i2, "mainGtU");
        /* 1 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 2 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 3 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 4 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 5 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 6 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 7 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 8 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 9 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 10 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 11 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;
        /* 12 */
        c1 = block[i1];
        c2 = block[i2];

        if (c1 != c2)
        {
            return (c1 > c2);
        }

        i1++;
        i2++;

        k = nblock + 8;

        do
        {
            /* 1 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;
            /* 2 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;
            /* 3 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;
            /* 4 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;
            /* 5 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;
            /* 6 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;
            /* 7 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;
            /* 8 */
            c1 = block[i1];
            c2 = block[i2];

            if (c1 != c2)
            {
                return (c1 > c2);
            }

            s1 = quadrant[i1];
            s2 = quadrant[i2];

            if (s1 != s2)
            {
                return (s1 > s2);
            }

            i1++;
            i2++;

            if (i1 >= nblock)
            {
                i1 -= nblock;
            }

            if (i2 >= nblock)
            {
                i2 -= nblock;
            }

            k -= 8;
            (*budget)--;
        } while (k >= 0);

        return BZ_False;
    }

    /*---------------------------------------------*/
    /*--
       Knuth's increments seem to work better
       than Incerpi-Sedgewick here.  Possibly
       because the number of elems to sort is
       usually small, typically <= 20.
    --*/

    CompressionBZip2::Int32 CompressionBZip2::incs[14] =
        {1, 4, 13, 40, 121, 364, 1093, 3280, 9841, 29524, 88573, 265720, 797161, 2391484};

    void
    CompressionBZip2::mainSimpleSort(UInt32* ptr,
                                     UChar* block,
                                     UInt16* quadrant,
                                     Int32 nblock,
                                     Int32 lo,
                                     Int32 hi,
                                     Int32 d,
                                     Int32* budget)
    {
        Int32 i, j, h, bigN, hp;
        UInt32 v;

        bigN = hi - lo + 1;

        if (bigN < 2)
        {
            return;
        }

        hp = 0;

        while (incs[hp] < bigN)
        {
            hp++;
        }

        hp--;

        for (; hp >= 0; hp--)
        {
            h = incs[hp];

            i = lo + h;

            while (BZ_True)
            {

                /*-- copy 1 --*/
                if (i > hi)
                {
                    break;
                }

                v = ptr[i];
                j = i;

                while (mainGtU(ptr[j - h] + d, v + d, block, quadrant, nblock, budget))
                {
                    ptr[j] = ptr[j - h];
                    j = j - h;

                    if (j <= (lo + h - 1))
                    {
                        break;
                    }
                }

                ptr[j] = v;
                i++;

                /*-- copy 2 --*/
                if (i > hi)
                {
                    break;
                }

                v = ptr[i];
                j = i;

                while (mainGtU(ptr[j - h] + d, v + d, block, quadrant, nblock, budget))
                {
                    ptr[j] = ptr[j - h];
                    j = j - h;

                    if (j <= (lo + h - 1))
                    {
                        break;
                    }
                }

                ptr[j] = v;
                i++;

                /*-- copy 3 --*/
                if (i > hi)
                {
                    break;
                }

                v = ptr[i];
                j = i;

                while (mainGtU(ptr[j - h] + d, v + d, block, quadrant, nblock, budget))
                {
                    ptr[j] = ptr[j - h];
                    j = j - h;

                    if (j <= (lo + h - 1))
                    {
                        break;
                    }
                }

                ptr[j] = v;
                i++;

                if (*budget < 0)
                {
                    return;
                }
            }
        }
    }

    /*---------------------------------------------*/
    /*--
       The following is an implementation of
       an elegant 3-way quicksort for strings,
       described in a paper "Fast Algorithms for
       Sorting and Searching Strings", by Robert
       Sedgewick and Jon L. Bentley.
    --*/

#define mswap(zz1, zz2)                                                                            \
    {                                                                                              \
        Int32 zztmp = zz1;                                                                         \
        zz1 = zz2;                                                                                 \
        zz2 = zztmp;                                                                               \
    }

#define mvswap(zzp1, zzp2, zzn)                                                                    \
    {                                                                                              \
        Int32 yyp1 = (zzp1);                                                                       \
        Int32 yyp2 = (zzp2);                                                                       \
        Int32 yyn = (zzn);                                                                         \
        while (yyn > 0)                                                                            \
        {                                                                                          \
            mswap(ptr[yyp1], ptr[yyp2]);                                                           \
            yyp1++;                                                                                \
            yyp2++;                                                                                \
            yyn--;                                                                                 \
        }                                                                                          \
    }

    CompressionBZip2::UChar
    CompressionBZip2::mmed3(UChar a, UChar b, UChar c)
    {
        UChar t;

        if (a > b)
        {
            t = a;
            a = b;
            b = t;
        };

        if (b > c)
        {
            b = c;

            if (a > b)
            {
                b = a;
            }
        }

        return b;
    }

#define mmin(a, b) ((a) < (b)) ? (a) : (b)

#define mpush(lz, hz, dz)                                                                          \
    {                                                                                              \
        stackLo[sp] = lz;                                                                          \
        stackHi[sp] = hz;                                                                          \
        stackD[sp] = dz;                                                                           \
        sp++;                                                                                      \
    }

#define mpop(lz, hz, dz)                                                                           \
    {                                                                                              \
        sp--;                                                                                      \
        lz = stackLo[sp];                                                                          \
        hz = stackHi[sp];                                                                          \
        dz = stackD[sp];                                                                           \
    }


#define mnextsize(az) (nextHi[az] - nextLo[az])

#define mnextswap(az, bz)                                                                          \
    {                                                                                              \
        Int32 tz;                                                                                  \
        tz = nextLo[az];                                                                           \
        nextLo[az] = nextLo[bz];                                                                   \
        nextLo[bz] = tz;                                                                           \
        tz = nextHi[az];                                                                           \
        nextHi[az] = nextHi[bz];                                                                   \
        nextHi[bz] = tz;                                                                           \
        tz = nextD[az];                                                                            \
        nextD[az] = nextD[bz];                                                                     \
        nextD[bz] = tz;                                                                            \
    }


#define MAIN_QSORT_SMALL_THRESH 20
#define MAIN_QSORT_DEPTH_THRESH (BZ_N_RADIX + BZ_N_QSORT)
#define MAIN_QSORT_STACK_SIZE 100

    void
    CompressionBZip2::mainQSort3(UInt32* ptr,
                                 UChar* block,
                                 UInt16* quadrant,
                                 Int32 nblock,
                                 Int32 loSt,
                                 Int32 hiSt,
                                 Int32 dSt,
                                 Int32* budget)
    {
        Int32 unLo, unHi, ltLo, gtHi, n, m, med;
        Int32 sp, lo, hi, d;

        Int32 stackLo[MAIN_QSORT_STACK_SIZE];
        Int32 stackHi[MAIN_QSORT_STACK_SIZE];
        Int32 stackD[MAIN_QSORT_STACK_SIZE];

        Int32 nextLo[3];
        Int32 nextHi[3];
        Int32 nextD[3];

        sp = 0;
        mpush(loSt, hiSt, dSt);

        while (sp > 0)
        {

            AssertH(sp < MAIN_QSORT_STACK_SIZE - 2, 1001);

            mpop(lo, hi, d);

            if (hi - lo < MAIN_QSORT_SMALL_THRESH || d > MAIN_QSORT_DEPTH_THRESH)
            {
                mainSimpleSort(ptr, block, quadrant, nblock, lo, hi, d, budget);

                if (*budget < 0)
                {
                    return;
                }

                continue;
            }

            med = (Int32)mmed3(
                block[ptr[lo] + d], block[ptr[hi] + d], block[ptr[(lo + hi) >> 1] + d]);

            unLo = ltLo = lo;
            unHi = gtHi = hi;

            while (BZ_True)
            {
                while (BZ_True)
                {
                    if (unLo > unHi)
                    {
                        break;
                    }

                    n = ((Int32)block[ptr[unLo] + d]) - med;

                    if (n == 0)
                    {
                        mswap(ptr[unLo], ptr[ltLo]);
                        ltLo++;
                        unLo++;
                        continue;
                    };

                    if (n > 0)
                    {
                        break;
                    }

                    unLo++;
                }

                while (BZ_True)
                {
                    if (unLo > unHi)
                    {
                        break;
                    }

                    n = ((Int32)block[ptr[unHi] + d]) - med;

                    if (n == 0)
                    {
                        mswap(ptr[unHi], ptr[gtHi]);
                        gtHi--;
                        unHi--;
                        continue;
                    };

                    if (n < 0)
                    {
                        break;
                    }

                    unHi--;
                }

                if (unLo > unHi)
                {
                    break;
                }

                mswap(ptr[unLo], ptr[unHi]);
                unLo++;
                unHi--;
            }

            AssertD(unHi == unLo - 1, "mainQSort3(2)");

            if (gtHi < ltLo)
            {
                mpush(lo, hi, d + 1);
                continue;
            }

            n = mmin(ltLo - lo, unLo - ltLo);
            mvswap(lo, unLo - n, n);
            m = mmin(hi - gtHi, gtHi - unHi);
            mvswap(unLo, hi - m + 1, m);

            n = lo + unLo - ltLo - 1;
            m = hi - (gtHi - unHi) + 1;

            nextLo[0] = lo;
            nextHi[0] = n;
            nextD[0] = d;
            nextLo[1] = m;
            nextHi[1] = hi;
            nextD[1] = d;
            nextLo[2] = n + 1;
            nextHi[2] = m - 1;
            nextD[2] = d + 1;

            if (mnextsize(0) < mnextsize(1))
            {
                mnextswap(0, 1);
            }

            if (mnextsize(1) < mnextsize(2))
            {
                mnextswap(1, 2);
            }

            if (mnextsize(0) < mnextsize(1))
            {
                mnextswap(0, 1);
            }

            AssertD(mnextsize(0) >= mnextsize(1), "mainQSort3(8)");
            AssertD(mnextsize(1) >= mnextsize(2), "mainQSort3(9)");

            mpush(nextLo[0], nextHi[0], nextD[0]);
            mpush(nextLo[1], nextHi[1], nextD[1]);
            mpush(nextLo[2], nextHi[2], nextD[2]);
        }
    }

#undef mswap
#undef mvswap
#undef mpush
#undef mpop
#undef mmin
#undef mnextsize
#undef mnextswap
#undef MAIN_QSORT_SMALL_THRESH
#undef MAIN_QSORT_DEPTH_THRESH
#undef MAIN_QSORT_STACK_SIZE

    /*---------------------------------------------*/
    /* Pre:
          nblock > 0
          eclass exists for [0 .. nblock-1]
          ((UChar*)eclass) [0 .. nblock-1] holds block
          ptr exists for [0 .. nblock-1]

       Post:
          ((UChar*)eclass) [0 .. nblock-1] holds block
          All other areas of eclass destroyed
          fmap [0 .. nblock-1] holds sorted order
          bhtab [ 0 .. 2+(nblock/32) ] destroyed
    */

#define SET_BH(zz) bhtab[(zz) >> 5] |= (1 << ((zz) & 31))
#define CLEAR_BH(zz) bhtab[(zz) >> 5] &= ~(1 << ((zz) & 31))
#define ISSET_BH(zz) (bhtab[(zz) >> 5] & (1 << ((zz) & 31)))
#define WORD_BH(zz) bhtab[(zz) >> 5]
#define UNALIGNED_BH(zz) ((zz) & 0x01f)

    void
    CompressionBZip2::fallbackSort(UInt32* fmap,
                                   UInt32* eclass,
                                   UInt32* bhtab,
                                   Int32 nblock,
                                   Int32 verb)
    {
        Int32 ftab[257];
        Int32 ftabCopy[256];
        Int32 H, i, j, k, l, r, cc, cc1;
        Int32 nNotDone;
        Int32 nBhtab;
        UChar* eclass8 = (UChar*)eclass;

        /*--
           Initial 1-char radix sort to generate
           initial fmap and initial BH bits.
        --*/
        if (verb >= 4)
        {
            VPrintf0("        bucket sorting ...\n");
        }

        for (i = 0; i < 257; i++)
        {
            ftab[i] = 0;
        }

        for (i = 0; i < nblock; i++)
        {
            ftab[eclass8[i]]++;
        }

        for (i = 0; i < 256; i++)
        {
            ftabCopy[i] = ftab[i];
        }

        for (i = 1; i < 257; i++)
        {
            ftab[i] += ftab[i - 1];
        }

        for (i = 0; i < nblock; i++)
        {
            j = eclass8[i];
            k = ftab[j] - 1;
            ftab[j] = k;
            fmap[k] = i;
        }

        nBhtab = 2 + (nblock / 32);

        for (i = 0; i < nBhtab; i++)
        {
            bhtab[i] = 0;
        }

        for (i = 0; i < 256; i++)
        {
            SET_BH(ftab[i]);
        }

        /*--
           Inductively refine the buckets.  Kind-of an
           "exponential radix sort" (!), inspired by the
           Manber-Myers suffix array construction algorithm.
        --*/

        /*-- set sentinel bits for block-end detection --*/
        for (i = 0; i < 32; i++)
        {
            SET_BH(nblock + 2 * i);
            CLEAR_BH(nblock + 2 * i + 1);
        }

        /*-- the log(N) loop --*/
        H = 1;

        while (true)
        {

            if (verb >= 4)
            {
                VPrintf1("        depth %6d has ", H);
            }

            j = 0;

            for (i = 0; i < nblock; i++)
            {
                if (ISSET_BH(i))
                {
                    j = i;
                }

                k = fmap[i] - H;

                if (k < 0)
                {
                    k += nblock;
                }

                eclass[k] = j;
            }

            nNotDone = 0;
            r = -1;

            while (true)
            {

                /*-- find the next non-singleton bucket --*/
                k = r + 1;

                while (ISSET_BH(k) && UNALIGNED_BH(k))
                {
                    k++;
                }

                if (ISSET_BH(k))
                {
                    while (WORD_BH(k) == 0xffffffff)
                    {
                        k += 32;
                    }

                    while (ISSET_BH(k))
                    {
                        k++;
                    }
                }

                l = k - 1;

                if (l >= nblock)
                {
                    break;
                }

                while (!ISSET_BH(k) && UNALIGNED_BH(k))
                {
                    k++;
                }

                if (!ISSET_BH(k))
                {
                    while (WORD_BH(k) == 0x00000000)
                    {
                        k += 32;
                    }

                    while (!ISSET_BH(k))
                    {
                        k++;
                    }
                }

                r = k - 1;

                if (r >= nblock)
                {
                    break;
                }

                /*-- now [l, r] bracket current bucket --*/
                if (r > l)
                {
                    nNotDone += (r - l + 1);
                    fallbackQSort3(fmap, eclass, l, r);

                    /*-- scan bucket and generate header bits-- */
                    cc = -1;

                    for (i = l; i <= r; i++)
                    {
                        cc1 = eclass[fmap[i]];

                        if (cc != cc1)
                        {
                            SET_BH(i);
                            cc = cc1;
                        };
                    }
                }
            }

            if (verb >= 4)
            {
                VPrintf1("%6d unresolved strings\n", nNotDone);
            }

            H *= 2;

            if (H > nblock || nNotDone == 0)
            {
                break;
            }
        }

        /*--
           Reconstruct the original block in
           eclass8 [0 .. nblock-1], since the
           previous phase destroyed it.
        --*/
        if (verb >= 4)
        {
            VPrintf0("        reconstructing block ...\n");
        }

        j = 0;

        for (i = 0; i < nblock; i++)
        {
            while (ftabCopy[j] == 0)
            {
                j++;
            }

            ftabCopy[j]--;
            eclass8[fmap[i]] = (UChar)j;
        }

        AssertH(j < 256, 1005);
    }

#undef SET_BH
#undef CLEAR_BH
#undef ISSET_BH
#undef WORD_BH
#undef UNALIGNED_BH

    void
    CompressionBZip2::fallbackSimpleSort(UInt32* fmap, UInt32* eclass, Int32 lo, Int32 hi)
    {
        Int32 i, j, tmp;
        UInt32 ec_tmp;

        if (lo == hi)
        {
            return;
        }

        if (hi - lo > 3)
        {
            for (i = hi - 4; i >= lo; i--)
            {
                tmp = fmap[i];
                ec_tmp = eclass[tmp];

                for (j = i + 4; j <= hi && ec_tmp > eclass[fmap[j]]; j += 4)
                {
                    fmap[j - 4] = fmap[j];
                }

                fmap[j - 4] = tmp;
            }
        }

        for (i = hi - 1; i >= lo; i--)
        {
            tmp = fmap[i];
            ec_tmp = eclass[tmp];

            for (j = i + 1; j <= hi && ec_tmp > eclass[fmap[j]]; j++)
            {
                fmap[j - 1] = fmap[j];
            }

            fmap[j - 1] = tmp;
        }
    }

    /*---------------------------------------------*/
#define fswap(zz1, zz2)                                                                            \
    {                                                                                              \
        Int32 zztmp = zz1;                                                                         \
        zz1 = zz2;                                                                                 \
        zz2 = zztmp;                                                                               \
    }

#define fvswap(zzp1, zzp2, zzn)                                                                    \
    {                                                                                              \
        Int32 yyp1 = (zzp1);                                                                       \
        Int32 yyp2 = (zzp2);                                                                       \
        Int32 yyn = (zzn);                                                                         \
        while (yyn > 0)                                                                            \
        {                                                                                          \
            fswap(fmap[yyp1], fmap[yyp2]);                                                         \
            yyp1++;                                                                                \
            yyp2++;                                                                                \
            yyn--;                                                                                 \
        }                                                                                          \
    }


#define fmin(a, b) ((a) < (b)) ? (a) : (b)

#define fpush(lz, hz)                                                                              \
    {                                                                                              \
        stackLo[sp] = lz;                                                                          \
        stackHi[sp] = hz;                                                                          \
        sp++;                                                                                      \
    }

#define fpop(lz, hz)                                                                               \
    {                                                                                              \
        sp--;                                                                                      \
        lz = stackLo[sp];                                                                          \
        hz = stackHi[sp];                                                                          \
    }

#define FALLBACK_QSORT_SMALL_THRESH 10
#define FALLBACK_QSORT_STACK_SIZE 100

    void
    CompressionBZip2::fallbackQSort3(UInt32* fmap, UInt32* eclass, Int32 loSt, Int32 hiSt)
    {
        Int32 unLo, unHi, ltLo, gtHi, n, m;
        Int32 sp, lo, hi;
        UInt32 med, r, r3;
        Int32 stackLo[FALLBACK_QSORT_STACK_SIZE];
        Int32 stackHi[FALLBACK_QSORT_STACK_SIZE];

        r = 0;

        sp = 0;
        fpush(loSt, hiSt);

        while (sp > 0)
        {

            AssertH(sp < FALLBACK_QSORT_STACK_SIZE - 1, 1004);

            fpop(lo, hi);

            if (hi - lo < FALLBACK_QSORT_SMALL_THRESH)
            {
                fallbackSimpleSort(fmap, eclass, lo, hi);
                continue;
            }

            /* Random partitioning.  Median of 3 sometimes fails to
               avoid bad cases.  Median of 9 seems to help but
               looks rather expensive.  This too seems to work but
               is cheaper.  Guidance for the magic constants
               7621 and 32768 is taken from Sedgewick's algorithms
               book, chapter 35.
            */
            r = ((r * 7621) + 1) % 32768;
            r3 = r % 3;

            if (r3 == 0)
            {
                med = eclass[fmap[lo]];
            }
            else if (r3 == 1)
            {
                med = eclass[fmap[(lo + hi) >> 1]];
            }
            else
            {
                med = eclass[fmap[hi]];
            }

            unLo = ltLo = lo;
            unHi = gtHi = hi;

            while (true)
            {
                while (true)
                {
                    if (unLo > unHi)
                    {
                        break;
                    }

                    n = (Int32)eclass[fmap[unLo]] - (Int32)med;

                    if (n == 0)
                    {
                        fswap(fmap[unLo], fmap[ltLo]);
                        ltLo++;
                        unLo++;
                        continue;
                    };

                    if (n > 0)
                    {
                        break;
                    }

                    unLo++;
                }

                while (true)
                {
                    if (unLo > unHi)
                    {
                        break;
                    }

                    n = (Int32)eclass[fmap[unHi]] - (Int32)med;

                    if (n == 0)
                    {
                        fswap(fmap[unHi], fmap[gtHi]);
                        gtHi--;
                        unHi--;
                        continue;
                    };

                    if (n < 0)
                    {
                        break;
                    }

                    unHi--;
                }

                if (unLo > unHi)
                {
                    break;
                }

                fswap(fmap[unLo], fmap[unHi]);
                unLo++;
                unHi--;
            }

            AssertD(unHi == unLo - 1, "fallbackQSort3(2)");

            if (gtHi < ltLo)
            {
                continue;
            }

            n = fmin(ltLo - lo, unLo - ltLo);
            fvswap(lo, unLo - n, n);
            m = fmin(hi - gtHi, gtHi - unHi);
            fvswap(unLo, hi - m + 1, m);

            n = lo + unLo - ltLo - 1;
            m = hi - (gtHi - unHi) + 1;

            if (n - lo > hi - m)
            {
                fpush(lo, n);
                fpush(m, hi);
            }
            else
            {
                fpush(m, hi);
                fpush(lo, n);
            }
        }
    }

#undef fmin
#undef fpush
#undef fpop
#undef fswap
#undef fvswap
#undef FALLBACK_QSORT_SMALL_THRESH
#undef FALLBACK_QSORT_STACK_SIZE

    /*---------------------------------------------------*/
    void
    CompressionBZip2::makeMaps_d(DState* s)
    {
        Int32 i;
        s->nInUse = 0;

        for (i = 0; i < 256; i++)
            if (s->inUse[i])
            {
                s->seqToUnseq[s->nInUse] = i;
                s->nInUse++;
            }
    }

    /*---------------------------------------------------*/
#define RETURN(rrr)                                                                                \
    {                                                                                              \
        retVal = rrr;                                                                              \
        goto save_state_and_return;                                                                \
    };

#define GET_BITS(lll, vvv, nnn)                                                                    \
    case lll:                                                                                      \
        s->state = lll;                                                                            \
        while (BZ_True)                                                                            \
        {                                                                                          \
            if (s->bsLive >= nnn)                                                                  \
            {                                                                                      \
                UInt32 v;                                                                          \
                v = (s->bsBuff >> (s->bsLive - nnn)) & ((1 << nnn) - 1);                           \
                s->bsLive -= nnn;                                                                  \
                vvv = v;                                                                           \
                break;                                                                             \
            }                                                                                      \
            if (s->strm->avail_in == 0)                                                            \
                RETURN(BZ_OK);                                                                     \
            s->bsBuff = (s->bsBuff << 8) | ((UInt32)(*((UChar*)(s->strm->next_in))));              \
            s->bsLive += 8;                                                                        \
            s->strm->next_in++;                                                                    \
            s->strm->avail_in--;                                                                   \
            s->strm->total_in_lo32++;                                                              \
            if (s->strm->total_in_lo32 == 0)                                                       \
                s->strm->total_in_hi32++;                                                          \
        }

#define GET_UCHAR(lll, uuu) GET_BITS(lll, uuu, 8)

#define GET_BIT(lll, uuu) GET_BITS(lll, uuu, 1)

    /*---------------------------------------------------*/
#define GET_MTF_VAL(label1, label2, lval)                                                          \
    {                                                                                              \
        if (groupPos == 0)                                                                         \
        {                                                                                          \
            groupNo++;                                                                             \
            if (groupNo >= nSelectors)                                                             \
                RETURN(BZ_DATA_ERROR);                                                             \
            groupPos = BZ_G_SIZE;                                                                  \
            gSel = s->selector[groupNo];                                                           \
            gMinlen = s->minLens[gSel];                                                            \
            gLimit = &(s->limit[gSel][0]);                                                         \
            gPerm = &(s->perm[gSel][0]);                                                           \
            gBase = &(s->base[gSel][0]);                                                           \
        }                                                                                          \
        groupPos--;                                                                                \
        zn = gMinlen;                                                                              \
        GET_BITS(label1, zvec, zn);                                                                \
        while (1)                                                                                  \
        {                                                                                          \
            if (zn > 20 /* the longest code */)                                                    \
                RETURN(BZ_DATA_ERROR);                                                             \
            if (zvec <= gLimit[zn])                                                                \
                break;                                                                             \
            zn++;                                                                                  \
            GET_BIT(label2, zj);                                                                   \
            zvec = (zvec << 1) | zj;                                                               \
        };                                                                                         \
        if (zvec - gBase[zn] < 0 || zvec - gBase[zn] >= BZ_MAX_ALPHA_SIZE)                         \
            RETURN(BZ_DATA_ERROR);                                                                 \
        lval = gPerm[zvec - gBase[zn]];                                                            \
    }

    /*---------------------------------------------------*/
    CompressionBZip2::Int32
    CompressionBZip2::BZ2_decompress(DState* s)
    {
        UChar uc;
        Int32 retVal;
        Int32 minLen, maxLen;
        bz_stream* strm = s->strm;

        /* stuff that needs to be saved/restored */
        Int32 i;
        Int32 j;
        Int32 t;
        Int32 alphaSize;
        Int32 nGroups;
        Int32 nSelectors;
        Int32 EOB;
        Int32 groupNo;
        Int32 groupPos;
        Int32 nextSym;
        Int32 nblockMAX;
        Int32 nblock;
        Int32 es;
        Int32 N;
        Int32 curr;
        Int32 zt;
        Int32 zn;
        Int32 zvec;
        Int32 zj;
        Int32 gSel;
        Int32 gMinlen;
        Int32* gLimit;
        Int32* gBase;
        Int32* gPerm;

        if (s->state == BZ_X_MAGIC_1)
        {
            /*initialise the save area*/
            s->save_i = 0;
            s->save_j = 0;
            s->save_t = 0;
            s->save_alphaSize = 0;
            s->save_nGroups = 0;
            s->save_nSelectors = 0;
            s->save_EOB = 0;
            s->save_groupNo = 0;
            s->save_groupPos = 0;
            s->save_nextSym = 0;
            s->save_nblockMAX = 0;
            s->save_nblock = 0;
            s->save_es = 0;
            s->save_N = 0;
            s->save_curr = 0;
            s->save_zt = 0;
            s->save_zn = 0;
            s->save_zvec = 0;
            s->save_zj = 0;
            s->save_gSel = 0;
            s->save_gMinlen = 0;
            s->save_gLimit = nullptr;
            s->save_gBase = nullptr;
            s->save_gPerm = nullptr;
        }

        /*restore from the save area*/
        i = s->save_i;
        j = s->save_j;
        t = s->save_t;
        alphaSize = s->save_alphaSize;
        nGroups = s->save_nGroups;
        nSelectors = s->save_nSelectors;
        EOB = s->save_EOB;
        groupNo = s->save_groupNo;
        groupPos = s->save_groupPos;
        nextSym = s->save_nextSym;
        nblockMAX = s->save_nblockMAX;
        nblock = s->save_nblock;
        es = s->save_es;
        N = s->save_N;
        curr = s->save_curr;
        zt = s->save_zt;
        zn = s->save_zn;
        zvec = s->save_zvec;
        zj = s->save_zj;
        gSel = s->save_gSel;
        gMinlen = s->save_gMinlen;
        gLimit = s->save_gLimit;
        gBase = s->save_gBase;
        gPerm = s->save_gPerm;

        retVal = BZ_OK;

        switch (s->state)
        {

            GET_UCHAR(BZ_X_MAGIC_1, uc);

            if (uc != BZ_HDR_B)
            {
                RETURN(BZ_DATA_ERROR_MAGIC);
            }

            GET_UCHAR(BZ_X_MAGIC_2, uc);

            if (uc != BZ_HDR_Z)
            {
                RETURN(BZ_DATA_ERROR_MAGIC);
            }

            GET_UCHAR(BZ_X_MAGIC_3, uc)

            if (uc != BZ_HDR_h)
            {
                RETURN(BZ_DATA_ERROR_MAGIC);
            }

            GET_BITS(BZ_X_MAGIC_4, s->blockSize100k, 8)

            if (s->blockSize100k < (BZ_HDR_0 + 1) || s->blockSize100k > (BZ_HDR_0 + 9))
            {
                RETURN(BZ_DATA_ERROR_MAGIC);
            }

            s->blockSize100k -= BZ_HDR_0;

            if (s->smallDecompress)
            {
                s->ll16 = (UInt16*)BZALLOC(s->blockSize100k * 100000 * sizeof(UInt16));
                s->ll4 = (UChar*)BZALLOC(((1 + s->blockSize100k * 100000) >> 1) * sizeof(UChar));

                if (s->ll16 == nullptr || s->ll4 == nullptr)
                {
                    RETURN(BZ_MEM_ERROR);
                }
            }
            else
            {
                s->tt = (UInt32*)BZALLOC(s->blockSize100k * 100000 * sizeof(Int32));

                if (s->tt == nullptr)
                {
                    RETURN(BZ_MEM_ERROR);
                }
            }

            GET_UCHAR(BZ_X_BLKHDR_1, uc);

            if (uc == 0x17)
            {
                goto endhdr_2;
            }

            if (uc != 0x31)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_BLKHDR_2, uc);

            if (uc != 0x41)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_BLKHDR_3, uc);

            if (uc != 0x59)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_BLKHDR_4, uc);

            if (uc != 0x26)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_BLKHDR_5, uc);

            if (uc != 0x53)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_BLKHDR_6, uc);

            if (uc != 0x59)
            {
                RETURN(BZ_DATA_ERROR);
            }

            s->currBlockNo++;

            if (s->verbosity >= 2)
            {
                VPrintf1("\n    [%d: huff+mtf ", s->currBlockNo);
            }

            s->storedBlockCRC = 0;
            GET_UCHAR(BZ_X_BCRC_1, uc);
            s->storedBlockCRC = (s->storedBlockCRC << 8) | ((UInt32)uc);
            GET_UCHAR(BZ_X_BCRC_2, uc);
            s->storedBlockCRC = (s->storedBlockCRC << 8) | ((UInt32)uc);
            GET_UCHAR(BZ_X_BCRC_3, uc);
            s->storedBlockCRC = (s->storedBlockCRC << 8) | ((UInt32)uc);
            GET_UCHAR(BZ_X_BCRC_4, uc);
            s->storedBlockCRC = (s->storedBlockCRC << 8) | ((UInt32)uc);

            GET_BITS(BZ_X_RANDBIT, s->blockRandomised, 1);

            s->origPtr = 0;
            GET_UCHAR(BZ_X_ORIGPTR_1, uc);
            s->origPtr = (s->origPtr << 8) | ((Int32)uc);
            GET_UCHAR(BZ_X_ORIGPTR_2, uc);
            s->origPtr = (s->origPtr << 8) | ((Int32)uc);
            GET_UCHAR(BZ_X_ORIGPTR_3, uc);
            s->origPtr = (s->origPtr << 8) | ((Int32)uc);

            if (s->origPtr < 0)
            {
                RETURN(BZ_DATA_ERROR);
            }

            if (s->origPtr > 10 + 100000 * s->blockSize100k)
            {
                RETURN(BZ_DATA_ERROR);
            }

            /*--- Receive the mapping table ---*/
            for (i = 0; i < 16; i++)
            {
                GET_BIT(BZ_X_MAPPING_1, uc);

                if (uc == 1)
                {
                    s->inUse16[i] = BZ_True;
                }
                else
                {
                    s->inUse16[i] = BZ_False;
                }
            }

            for (i = 0; i < 256; i++)
            {
                s->inUse[i] = BZ_False;
            }

            for (i = 0; i < 16; i++)
                if (s->inUse16[i])
                    for (j = 0; j < 16; j++)
                    {
                        GET_BIT(BZ_X_MAPPING_2, uc);

                        if (uc == 1)
                        {
                            s->inUse[i * 16 + j] = BZ_True;
                        }
                    }

            makeMaps_d(s);

            if (s->nInUse == 0)
            {
                RETURN(BZ_DATA_ERROR);
            }

            alphaSize = s->nInUse + 2;

            /*--- Now the selectors ---*/
            GET_BITS(BZ_X_SELECTOR_1, nGroups, 3);

            if (nGroups < 2 || nGroups > 6)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_BITS(BZ_X_SELECTOR_2, nSelectors, 15);

            if (nSelectors < 1)
            {
                RETURN(BZ_DATA_ERROR);
            }

            for (i = 0; i < nSelectors; i++)
            {
                j = 0;

                while (BZ_True)
                {
                    GET_BIT(BZ_X_SELECTOR_3, uc);

                    if (uc == 0)
                    {
                        break;
                    }

                    j++;

                    if (j >= nGroups)
                    {
                        RETURN(BZ_DATA_ERROR);
                    }
                }

                s->selectorMtf[i] = j;
            }

            /*--- Undo the MTF values for the selectors. ---*/
            {
                UChar pos[BZ_N_GROUPS], tmp, v;

                for (v = 0; v < nGroups; v++)
                {
                    pos[v] = v;
                }

                for (i = 0; i < nSelectors; i++)
                {
                    v = s->selectorMtf[i];
                    tmp = pos[v];

                    while (v > 0)
                    {
                        pos[v] = pos[v - 1];
                        v--;
                    }

                    pos[0] = tmp;
                    s->selector[i] = tmp;
                }
            }

            /*--- Now the coding tables ---*/
            for (t = 0; t < nGroups; t++)
            {
                GET_BITS(BZ_X_CODING_1, curr, 5);

                for (i = 0; i < alphaSize; i++)
                {
                    while (BZ_True)
                    {
                        if (curr < 1 || curr > 20)
                        {
                            RETURN(BZ_DATA_ERROR);
                        }

                        GET_BIT(BZ_X_CODING_2, uc);

                        if (uc == 0)
                        {
                            break;
                        }

                        GET_BIT(BZ_X_CODING_3, uc);

                        if (uc == 0)
                        {
                            curr++;
                        }
                        else
                        {
                            curr--;
                        }
                    }

                    s->len[t][i] = curr;
                }
            }

            /*--- Create the Huffman decoding tables ---*/
            for (t = 0; t < nGroups; t++)
            {
                minLen = 32;
                maxLen = 0;

                for (i = 0; i < alphaSize; i++)
                {
                    if (s->len[t][i] > maxLen)
                    {
                        maxLen = s->len[t][i];
                    }

                    if (s->len[t][i] < minLen)
                    {
                        minLen = s->len[t][i];
                    }
                }

                BZ2_hbCreateDecodeTables(&(s->limit[t][0]),
                                         &(s->base[t][0]),
                                         &(s->perm[t][0]),
                                         &(s->len[t][0]),
                                         minLen,
                                         maxLen,
                                         alphaSize);
                s->minLens[t] = minLen;
            }

            /*--- Now the MTF values ---*/

            EOB = s->nInUse + 1;
            nblockMAX = 100000 * s->blockSize100k;
            groupNo = -1;
            groupPos = 0;

            for (i = 0; i <= 255; i++)
            {
                s->unzftab[i] = 0;
            }

            /*-- MTF init --*/
            {
                Int32 ii, jj, kk;
                kk = MTFA_SIZE - 1;

                for (ii = 256 / MTFL_SIZE - 1; ii >= 0; ii--)
                {
                    for (jj = MTFL_SIZE - 1; jj >= 0; jj--)
                    {
                        s->mtfa[kk] = (UChar)(ii * MTFL_SIZE + jj);
                        kk--;
                    }

                    s->mtfbase[ii] = kk + 1;
                }
            }
            /*-- end MTF init --*/

            nblock = 0;
            GET_MTF_VAL(BZ_X_MTF_1, BZ_X_MTF_2, nextSym);

            while (BZ_True)
            {

                if (nextSym == EOB)
                {
                    break;
                }

                if (nextSym == BZ_RUNA || nextSym == BZ_RUNB)
                {

                    es = -1;
                    N = 1;

                    do
                    {
                        /* Check that N doesn't get too big, so that es doesn't
                               go negative.  The maximum value that can be
                               RUNA/RUNB encoded is equal to the block size (post
                               the initial RLE), viz, 900k, so bounding N at 2
                               million should guard against overflow without
                               rejecting any legitimate inputs. */
                        if (N >= 2 * 1024 * 1024)
                        {
                            RETURN(BZ_DATA_ERROR);
                        }

                        if (nextSym == BZ_RUNA)
                        {
                            es = es + (0 + 1) * N;
                        }
                        else if (nextSym == BZ_RUNB)
                        {
                            es = es + (1 + 1) * N;
                        }

                        N = N * 2;
                        GET_MTF_VAL(BZ_X_MTF_3, BZ_X_MTF_4, nextSym);
                    } while (nextSym == BZ_RUNA || nextSym == BZ_RUNB);

                    es++;
                    uc = s->seqToUnseq[s->mtfa[s->mtfbase[0]]];
                    s->unzftab[uc] += es;

                    if (s->smallDecompress)
                        while (es > 0)
                        {
                            if (nblock >= nblockMAX)
                            {
                                RETURN(BZ_DATA_ERROR);
                            }

                            s->ll16[nblock] = (UInt16)uc;
                            nblock++;
                            es--;
                        }
                    else
                        while (es > 0)
                        {
                            if (nblock >= nblockMAX)
                            {
                                RETURN(BZ_DATA_ERROR);
                            }

                            s->tt[nblock] = (UInt32)uc;
                            nblock++;
                            es--;
                        };

                    continue;
                }
                else
                {

                    if (nblock >= nblockMAX)
                    {
                        RETURN(BZ_DATA_ERROR);
                    }

                    /*-- uc = MTF ( nextSym-1 ) --*/
                    {
                        Int32 ii, jj, kk, pp, lno, off;
                        UInt32 nn;
                        nn = (UInt32)(nextSym - 1);

                        if (nn < MTFL_SIZE)
                        {
                            /* avoid general-case expense */
                            pp = s->mtfbase[0];
                            uc = s->mtfa[pp + nn];

                            while (nn > 3)
                            {
                                Int32 z = pp + nn;
                                s->mtfa[(z)] = s->mtfa[(z)-1];
                                s->mtfa[(z)-1] = s->mtfa[(z)-2];
                                s->mtfa[(z)-2] = s->mtfa[(z)-3];
                                s->mtfa[(z)-3] = s->mtfa[(z)-4];
                                nn -= 4;
                            }

                            while (nn > 0)
                            {
                                s->mtfa[(pp + nn)] = s->mtfa[(pp + nn) - 1];
                                nn--;
                            };

                            s->mtfa[pp] = uc;
                        }
                        else
                        {
                            /* general case */
                            lno = nn / MTFL_SIZE;
                            off = nn % MTFL_SIZE;
                            pp = s->mtfbase[lno] + off;
                            uc = s->mtfa[pp];

                            while (pp > s->mtfbase[lno])
                            {
                                s->mtfa[pp] = s->mtfa[pp - 1];
                                pp--;
                            };

                            s->mtfbase[lno]++;

                            while (lno > 0)
                            {
                                s->mtfbase[lno]--;
                                s->mtfa[s->mtfbase[lno]] =
                                    s->mtfa[s->mtfbase[lno - 1] + MTFL_SIZE - 1];
                                lno--;
                            }

                            s->mtfbase[0]--;
                            s->mtfa[s->mtfbase[0]] = uc;

                            if (s->mtfbase[0] == 0)
                            {
                                kk = MTFA_SIZE - 1;

                                for (ii = 256 / MTFL_SIZE - 1; ii >= 0; ii--)
                                {
                                    for (jj = MTFL_SIZE - 1; jj >= 0; jj--)
                                    {
                                        s->mtfa[kk] = s->mtfa[s->mtfbase[ii] + jj];
                                        kk--;
                                    }

                                    s->mtfbase[ii] = kk + 1;
                                }
                            }
                        }
                    }
                    /*-- end uc = MTF ( nextSym-1 ) --*/

                    s->unzftab[s->seqToUnseq[uc]]++;

                    if (s->smallDecompress)
                    {
                        s->ll16[nblock] = (UInt16)(s->seqToUnseq[uc]);
                    }
                    else
                    {
                        s->tt[nblock] = (UInt32)(s->seqToUnseq[uc]);
                    }

                    nblock++;

                    GET_MTF_VAL(BZ_X_MTF_5, BZ_X_MTF_6, nextSym);
                    continue;
                }
            }

            /* Now we know what nblock is, we can do a better sanity
                   check on s->origPtr.
                */
            if (s->origPtr < 0 || s->origPtr >= nblock)
            {
                RETURN(BZ_DATA_ERROR);
            }

            /*-- Set up cftab to facilitate generation of T^(-1) --*/
            /* Check: unzftab entries in range. */
            for (i = 0; i <= 255; i++)
            {
                if (s->unzftab[i] < 0 || s->unzftab[i] > nblock)
                {
                    RETURN(BZ_DATA_ERROR);
                }
            }

            /* Actually generate cftab. */
            s->cftab[0] = 0;

            for (i = 1; i <= 256; i++)
            {
                s->cftab[i] = s->unzftab[i - 1];
            }

            for (i = 1; i <= 256; i++)
            {
                s->cftab[i] += s->cftab[i - 1];
            }

            /* Check: cftab entries in range. */
            for (i = 0; i <= 256; i++)
            {
                if (s->cftab[i] < 0 || s->cftab[i] > nblock)
                {
                    /* s->cftab[i] can legitimately be == nblock */
                    RETURN(BZ_DATA_ERROR);
                }
            }

            /* Check: cftab entries non-descending. */
            for (i = 1; i <= 256; i++)
            {
                if (s->cftab[i - 1] > s->cftab[i])
                {
                    RETURN(BZ_DATA_ERROR);
                }
            }

            s->state_out_len = 0;
            s->state_out_ch = 0;
            BZ_INITIALISE_CRC(s->calculatedBlockCRC);
            s->state = BZ_X_OUTPUT;

            if (s->verbosity >= 2)
            {
                VPrintf0("rt+rld");
            }

            if (s->smallDecompress)
            {

                /*-- Make a copy of cftab, used in generation of T --*/
                for (i = 0; i <= 256; i++)
                {
                    s->cftabCopy[i] = s->cftab[i];
                }

                /*-- compute the T vector --*/
                for (i = 0; i < nblock; i++)
                {
                    uc = (UChar)(s->ll16[i]);
                    SET_LL(i, s->cftabCopy[uc]);
                    s->cftabCopy[uc]++;
                }

                /*-- Compute T^(-1) by pointer reversal on T --*/
                i = s->origPtr;
                j = GET_LL(i);

                do
                {
                    Int32 tmp = GET_LL(j);
                    SET_LL(j, i);
                    i = j;
                    j = tmp;
                } while (i != s->origPtr);

                s->tPos = s->origPtr;
                s->nblock_used = 0;

                if (s->blockRandomised)
                {
                    BZ_RAND_INIT_MASK;
                    BZ_GET_SMALL(s->k0);
                    s->nblock_used++;
                    BZ_RAND_UPD_MASK;
                    s->k0 ^= BZ_RAND_MASK;
                }
                else
                {
                    BZ_GET_SMALL(s->k0);
                    s->nblock_used++;
                }
            }
            else
            {

                /*-- compute the T^(-1) vector --*/
                for (i = 0; i < nblock; i++)
                {
                    uc = (UChar)(s->tt[i] & 0xff);
                    s->tt[s->cftab[uc]] |= (i << 8);
                    s->cftab[uc]++;
                }

                s->tPos = s->tt[s->origPtr] >> 8;
                s->nblock_used = 0;

                if (s->blockRandomised)
                {
                    BZ_RAND_INIT_MASK;
                    BZ_GET_FAST(s->k0);
                    s->nblock_used++;
                    BZ_RAND_UPD_MASK;
                    s->k0 ^= BZ_RAND_MASK;
                }
                else
                {
                    BZ_GET_FAST(s->k0);
                    s->nblock_used++;
                }
            }

            RETURN(BZ_OK);


        endhdr_2:

            GET_UCHAR(BZ_X_ENDHDR_2, uc);

            if (uc != 0x72)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_ENDHDR_3, uc);

            if (uc != 0x45)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_ENDHDR_4, uc);

            if (uc != 0x38)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_ENDHDR_5, uc);

            if (uc != 0x50)
            {
                RETURN(BZ_DATA_ERROR);
            }

            GET_UCHAR(BZ_X_ENDHDR_6, uc);

            if (uc != 0x90)
            {
                RETURN(BZ_DATA_ERROR);
            }

            s->storedCombinedCRC = 0;
            GET_UCHAR(BZ_X_CCRC_1, uc);
            s->storedCombinedCRC = (s->storedCombinedCRC << 8) | ((UInt32)uc);
            GET_UCHAR(BZ_X_CCRC_2, uc);
            s->storedCombinedCRC = (s->storedCombinedCRC << 8) | ((UInt32)uc);
            GET_UCHAR(BZ_X_CCRC_3, uc);
            s->storedCombinedCRC = (s->storedCombinedCRC << 8) | ((UInt32)uc);
            GET_UCHAR(BZ_X_CCRC_4, uc);
            s->storedCombinedCRC = (s->storedCombinedCRC << 8) | ((UInt32)uc);

            s->state = BZ_X_IDLE;
            RETURN(BZ_STREAM_END);

            default:
                AssertH(BZ_False, 4001);
        }

        AssertH(BZ_False, 4002);

    save_state_and_return:

        s->save_i = i;
        s->save_j = j;
        s->save_t = t;
        s->save_alphaSize = alphaSize;
        s->save_nGroups = nGroups;
        s->save_nSelectors = nSelectors;
        s->save_EOB = EOB;
        s->save_groupNo = groupNo;
        s->save_groupPos = groupPos;
        s->save_nextSym = nextSym;
        s->save_nblockMAX = nblockMAX;
        s->save_nblock = nblock;
        s->save_es = es;
        s->save_N = N;
        s->save_curr = curr;
        s->save_zt = zt;
        s->save_zn = zn;
        s->save_zvec = zvec;
        s->save_zj = zj;
        s->save_gSel = gSel;
        s->save_gMinlen = gMinlen;
        s->save_gLimit = gLimit;
        s->save_gBase = gBase;
        s->save_gPerm = gPerm;

        return retVal;
    }

    CompressionBZip2::BZFILE*
    CompressionBZip2::BZ2_bzWriteOpen(int* bzerror,
                                      //FILE* f,
                                      std::ostream* f,
                                      int blockSize100k,
                                      int verbosity,
                                      int workFactor)
    {
        Int32 ret;
        bzFile* bzf = nullptr;

        BZ_SETERR(BZ_OK);

        if (f == nullptr || (blockSize100k < 1 || blockSize100k > 9) ||
            (workFactor < 0 || workFactor > 250) || (verbosity < 0 || verbosity > 4))
        {
            BZ_SETERR(BZ_PARAM_ERROR);
            return nullptr;
        };

        if (!f || f->bad())
        {
            BZ_SETERR(BZ_IO_ERROR);
            return nullptr;
        };

        bzf = (bzFile*)malloc(sizeof(bzFile));

        if (bzf == nullptr)
        {
            BZ_SETERR(BZ_MEM_ERROR);
            return nullptr;
        };

        BZ_SETERR(BZ_OK);

        bzf->initialisedOk = BZ_False;

        bzf->bufN = 0;

        bzf->handleIn = nullptr;

        bzf->handleOut = f;

        bzf->writing = BZ_True;

        bzf->strm.bzalloc = nullptr;

        bzf->strm.bzfree = nullptr;

        bzf->strm.opaque = nullptr;

        if (workFactor == 0)
        {
            workFactor = 30;
        }

        ret = BZ2_bzCompressInit(&(bzf->strm), blockSize100k, verbosity, workFactor);

        if (ret != BZ_OK)
        {
            BZ_SETERR(ret);
            free(bzf);
            return nullptr;
        };

        bzf->strm.avail_in = 0;

        bzf->initialisedOk = BZ_True;

        return bzf;
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::BZ2_bzCompressInit(bz_stream* strm,
                                         int blockSize100k,
                                         int verbosity,
                                         int workFactor)
    {
        Int32 n;
        EState* s;

        if (!bz_config_ok())
        {
            return BZ_CONFIG_ERROR;
        }

        if (strm == nullptr || blockSize100k < 1 || blockSize100k > 9 || workFactor < 0 ||
            workFactor > 250)
        {
            return BZ_PARAM_ERROR;
        }

        if (workFactor == 0)
        {
            workFactor = 30;
        }

        if (strm->bzalloc == nullptr)
        {
            strm->bzalloc = default_bzalloc;
        }

        if (strm->bzfree == nullptr)
        {
            strm->bzfree = default_bzfree;
        }

        s = (EState*)BZALLOC(sizeof(EState));

        if (s == nullptr)
        {
            return BZ_MEM_ERROR;
        }

        s->strm = strm;

        s->arr1 = nullptr;
        s->arr2 = nullptr;
        s->ftab = nullptr;

        n = 100000 * blockSize100k;
        s->arr1 = (UInt32*)BZALLOC(n * sizeof(UInt32));
        s->arr2 = (UInt32*)BZALLOC((n + BZ_N_OVERSHOOT) * sizeof(UInt32));
        s->ftab = (UInt32*)BZALLOC(65537 * sizeof(UInt32));

        if (s->arr1 == nullptr || s->arr2 == nullptr || s->ftab == nullptr)
        {
            if (s->arr1 != nullptr)
            {
                BZFREE(s->arr1);
            }

            if (s->arr2 != nullptr)
            {
                BZFREE(s->arr2);
            }

            if (s->ftab != nullptr)
            {
                BZFREE(s->ftab);
            }

            if (s != nullptr)
            {
                BZFREE(s);
            }

            return BZ_MEM_ERROR;
        }

        s->blockNo = 0;
        s->state = BZ_S_INPUT;
        s->mode = BZ_M_RUNNING;
        s->combinedCRC = 0;
        s->blockSize100k = blockSize100k;
        s->nblockMAX = 100000 * blockSize100k - 19;
        s->verbosity = verbosity;
        s->workFactor = workFactor;

        s->block = (UChar*)s->arr2;
        s->mtfv = (UInt16*)s->arr1;
        s->zbits = nullptr;
        s->ptr = (UInt32*)s->arr1;

        strm->state = s;
        strm->total_in_lo32 = 0;
        strm->total_in_hi32 = 0;
        strm->total_out_lo32 = 0;
        strm->total_out_hi32 = 0;
        init_RL(s);
        prepare_new_block(s);
        return BZ_OK;
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::bz_config_ok()
    {
        if (sizeof(int) != 4)
        {
            return 0;
        }

        if (sizeof(short) != 2)
        {
            return 0;
        }

        if (sizeof(char) != 1)
        {
            return 0;
        }

        return 1;
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::init_RL(CompressionBZip2::EState* s)
    {
        s->state_in_ch = 256;
        s->state_in_len = 0;
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::prepare_new_block(CompressionBZip2::EState* s)
    {
        Int32 i;
        s->nblock = 0;
        s->numZ = 0;
        s->state_out_pos = 0;
        BZ_INITIALISE_CRC(s->blockCRC);

        for (i = 0; i < 256; i++)
        {
            s->inUse[i] = BZ_False;
        }

        s->blockNo++;
    }

    CompressionBZip2::Bool
    CompressionBZip2::isempty_RL(CompressionBZip2::EState* s)
    {
        if (s->state_in_ch < 256 && s->state_in_len > 0)
        {
            return BZ_False;
        }
        else
        {
            return BZ_True;
        }
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_bzWrite(int* bzerror, BZFILE* b, void* buf, int len)
    {
        Int32 n, ret;
        bzFile* bzf = (bzFile*)b;

        BZ_SETERR(BZ_OK);

        if (bzf == nullptr || buf == nullptr || len < 0)
        {
            BZ_SETERR(BZ_PARAM_ERROR);
            return;
        };

        if (!(bzf->writing))
        {
            BZ_SETERR(BZ_SEQUENCE_ERROR);
            return;
        };

        if (!bzf->handleOut || bzf->handleOut->bad())
        {
            BZ_SETERR(BZ_IO_ERROR);
            return;
        };

        if (len == 0)
        {
            BZ_SETERR(BZ_OK);
            return;
        };

        bzf->strm.avail_in = len;

        bzf->strm.next_in = (char*)buf;

        while (BZ_True)
        {
            bzf->strm.avail_out = BZ_MAX_UNUSED;
            bzf->strm.next_out = bzf->buf;
            ret = BZ2_bzCompress(&(bzf->strm), BZ_RUN);

            if (ret != BZ_RUN_OK)
            {
                BZ_SETERR(ret);
                return;
            };

            if (bzf->strm.avail_out < BZ_MAX_UNUSED)
            {
                n = BZ_MAX_UNUSED - bzf->strm.avail_out;

                //n2 = fwrite ( (void*)(bzf->buf), sizeof(UChar),
                //              n, bzf->handle );
                bool ok = write(bzf->handleOut, (bzf->buf), n);

                if (!ok || bzf->handleOut->bad())
                {
                    BZ_SETERR(BZ_IO_ERROR);
                    return;
                };
            }

            if (bzf->strm.avail_in == 0)
            {
                BZ_SETERR(BZ_OK);
                return;
            };
        }
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_bzWriteClose(int* bzerror,
                                       BZFILE* b,
                                       int abandon,
                                       unsigned int* nbytes_in,
                                       unsigned int* nbytes_out)
    {
        BZ2_bzWriteClose64(bzerror, b, abandon, nbytes_in, nullptr, nbytes_out, nullptr);
    }

    void
    CompressionBZip2::BZ2_bzWriteClose64(int* bzerror,
                                         BZFILE* b,
                                         int abandon,
                                         unsigned int* nbytes_in_lo32,
                                         unsigned int* nbytes_in_hi32,
                                         unsigned int* nbytes_out_lo32,
                                         unsigned int* nbytes_out_hi32)
    {
        Int32 n, ret;
        bzFile* bzf = (bzFile*)b;

        if (bzf == nullptr)
        {
            BZ_SETERR(BZ_OK);
            return;
        };

        if (!(bzf->writing))
        {
            BZ_SETERR(BZ_SEQUENCE_ERROR);
            return;
        };

        // if (ferror(bzf->handle))
        if (!bzf->handleOut || bzf->handleOut->bad())
        {
            BZ_SETERR(BZ_IO_ERROR);
            return;
        };

        if (nbytes_in_lo32 != nullptr)
        {
            *nbytes_in_lo32 = 0;
        }

        if (nbytes_in_hi32 != nullptr)
        {
            *nbytes_in_hi32 = 0;
        }

        if (nbytes_out_lo32 != nullptr)
        {
            *nbytes_out_lo32 = 0;
        }

        if (nbytes_out_hi32 != nullptr)
        {
            *nbytes_out_hi32 = 0;
        }

        if ((!abandon) && bzf->lastErr == BZ_OK)
        {
            while (BZ_True)
            {
                bzf->strm.avail_out = BZ_MAX_UNUSED;
                bzf->strm.next_out = bzf->buf;
                ret = BZ2_bzCompress(&(bzf->strm), BZ_FINISH);

                if (ret != BZ_FINISH_OK && ret != BZ_STREAM_END)
                {
                    BZ_SETERR(ret);
                    return;
                };

                if (bzf->strm.avail_out < BZ_MAX_UNUSED)
                {
                    n = BZ_MAX_UNUSED - bzf->strm.avail_out;
                    bool ok = write(bzf->handleOut, (bzf->buf), n);

                    if (!ok || bzf->handleOut->bad())
                    {
                        BZ_SETERR(BZ_IO_ERROR);
                        return;
                    };
                }

                if (ret == BZ_STREAM_END)
                {
                    break;
                }
            }
        }

        if (!abandon && !bzf->handleOut->bad())
        {
            //fflush ( bzf->handle );
            bzf->handleOut->flush();

            if (bzf->handleOut->bad())
            {
                BZ_SETERR(BZ_IO_ERROR);
                return;
            };
        }

        if (nbytes_in_lo32 != nullptr)
        {
            *nbytes_in_lo32 = bzf->strm.total_in_lo32;
        }

        if (nbytes_in_hi32 != nullptr)
        {
            *nbytes_in_hi32 = bzf->strm.total_in_hi32;
        }

        if (nbytes_out_lo32 != nullptr)
        {
            *nbytes_out_lo32 = bzf->strm.total_out_lo32;
        }

        if (nbytes_out_hi32 != nullptr)
        {
            *nbytes_out_hi32 = bzf->strm.total_out_hi32;
        }

        BZ_SETERR(BZ_OK);
        BZ2_bzCompressEnd(&(bzf->strm));
        free(bzf);
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::BZ2_bzCompressEnd(CompressionBZip2::bz_stream* strm)
    {
        EState* s;

        if (strm == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        s = (EState*)strm->state;

        if (s == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        if (s->strm != strm)
        {
            return BZ_PARAM_ERROR;
        }

        if (s->arr1 != nullptr)
        {
            BZFREE(s->arr1);
        }

        if (s->arr2 != nullptr)
        {
            BZFREE(s->arr2);
        }

        if (s->ftab != nullptr)
        {
            BZFREE(s->ftab);
        }

        BZFREE(strm->state);

        strm->state = nullptr;

        return BZ_OK;
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::BZ2_bzDecompressEnd(CompressionBZip2::bz_stream* strm)
    {
        DState* s;

        if (strm == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        s = (DState*)strm->state;

        if (s == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        if (s->strm != strm)
        {
            return BZ_PARAM_ERROR;
        }

        if (s->tt != nullptr)
        {
            BZFREE(s->tt);
        }

        if (s->ll16 != nullptr)
        {
            BZFREE(s->ll16);
        }

        if (s->ll4 != nullptr)
        {
            BZFREE(s->ll4);
        }

        BZFREE(strm->state);
        strm->state = nullptr;

        return BZ_OK;
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::BZ2_bzCompress(CompressionBZip2::bz_stream* strm, int action)
    {
        Bool progress;
        EState* s;

        if (strm == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        s = (EState*)strm->state;

        if (s == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        if (s->strm != strm)
        {
            return BZ_PARAM_ERROR;
        }

    preswitch:

        switch (s->mode)
        {

            case BZ_M_IDLE:
                return BZ_SEQUENCE_ERROR;

            case BZ_M_RUNNING:
                if (action == BZ_RUN)
                {
                    progress = handle_compress(strm);
                    return progress ? BZ_RUN_OK : BZ_PARAM_ERROR;
                }
                else if (action == BZ_FLUSH)
                {
                    s->avail_in_expect = strm->avail_in;
                    s->mode = BZ_M_FLUSHING;
                    goto preswitch;
                }
                else if (action == BZ_FINISH)
                {
                    s->avail_in_expect = strm->avail_in;
                    s->mode = BZ_M_FINISHING;
                    goto preswitch;
                }
                else
                {
                    return BZ_PARAM_ERROR;
                }

            case BZ_M_FLUSHING:
                if (action != BZ_FLUSH)
                {
                    return BZ_SEQUENCE_ERROR;
                }

                if (s->avail_in_expect != s->strm->avail_in)
                {
                    return BZ_SEQUENCE_ERROR;
                }

                progress = handle_compress(strm);

                if (s->avail_in_expect > 0 || !isempty_RL(s) || s->state_out_pos < s->numZ)
                {
                    return BZ_FLUSH_OK;
                }

                s->mode = BZ_M_RUNNING;
                return BZ_RUN_OK;

            case BZ_M_FINISHING:
                if (action != BZ_FINISH)
                {
                    return BZ_SEQUENCE_ERROR;
                }

                if (s->avail_in_expect != s->strm->avail_in)
                {
                    return BZ_SEQUENCE_ERROR;
                }

                progress = handle_compress(strm);

                if (!progress)
                {
                    return BZ_SEQUENCE_ERROR;
                }

                if (s->avail_in_expect > 0 || !isempty_RL(s) || s->state_out_pos < s->numZ)
                {
                    return BZ_FINISH_OK;
                }

                s->mode = BZ_M_IDLE;
                return BZ_STREAM_END;
        }

        return BZ_OK; /*--not reached--*/
    }

    /*---------------------------------------------------*/
    CompressionBZip2::Bool
    CompressionBZip2::handle_compress(CompressionBZip2::bz_stream* strm)
    {
        Bool progress_in = BZ_False;
        Bool progress_out = BZ_False;
        EState* s = (EState*)strm->state;

        while (BZ_True)
        {

            if (s->state == BZ_S_OUTPUT)
            {
                progress_out |= copy_output_until_stop(s);

                if (s->state_out_pos < s->numZ)
                {
                    break;
                }

                if (s->mode == BZ_M_FINISHING && s->avail_in_expect == 0 && isempty_RL(s))
                {
                    break;
                }

                prepare_new_block(s);
                s->state = BZ_S_INPUT;

                if (s->mode == BZ_M_FLUSHING && s->avail_in_expect == 0 && isempty_RL(s))
                {
                    break;
                }
            }

            if (s->state == BZ_S_INPUT)
            {
                progress_in |= copy_input_until_stop(s);

                if (s->mode != BZ_M_RUNNING && s->avail_in_expect == 0)
                {
                    flush_RL(s);
                    BZ2_compressBlock(s, (Bool)(s->mode == BZ_M_FINISHING));
                    s->state = BZ_S_OUTPUT;
                }
                else if (s->nblock >= s->nblockMAX)
                {
                    BZ2_compressBlock(s, BZ_False);
                    s->state = BZ_S_OUTPUT;
                }
                else if (s->strm->avail_in == 0)
                {
                    break;
                }
            }
        }

        return progress_in || progress_out;
    }

    void
    CompressionBZip2::flush_RL(CompressionBZip2::EState* s)
    {
        if (s->state_in_ch < 256)
        {
            add_pair_to_block(s);
        }

        init_RL(s);
    }

    /*---------------------------------------------------*/
    CompressionBZip2::Bool
    CompressionBZip2::copy_input_until_stop(CompressionBZip2::EState* s)
    {
        Bool progress_in = BZ_False;

        if (s->mode == BZ_M_RUNNING)
        {

            /*-- fast track the common case --*/
            while (BZ_True)
            {
                /*-- block full? --*/
                if (s->nblock >= s->nblockMAX)
                {
                    break;
                }

                /*-- no input? --*/
                if (s->strm->avail_in == 0)
                {
                    break;
                }

                progress_in = BZ_True;
                ADD_CHAR_TO_BLOCK(s, (UInt32)(*((UChar*)(s->strm->next_in))));
                s->strm->next_in++;
                s->strm->avail_in--;
                s->strm->total_in_lo32++;

                if (s->strm->total_in_lo32 == 0)
                {
                    s->strm->total_in_hi32++;
                }
            }
        }
        else
        {

            /*-- general, uncommon case --*/
            while (BZ_True)
            {
                /*-- block full? --*/
                if (s->nblock >= s->nblockMAX)
                {
                    break;
                }

                /*-- no input? --*/
                if (s->strm->avail_in == 0)
                {
                    break;
                }

                /*-- flush/finish end? --*/
                if (s->avail_in_expect == 0)
                {
                    break;
                }

                progress_in = BZ_True;
                ADD_CHAR_TO_BLOCK(s, (UInt32)(*((UChar*)(s->strm->next_in))));
                s->strm->next_in++;
                s->strm->avail_in--;
                s->strm->total_in_lo32++;

                if (s->strm->total_in_lo32 == 0)
                {
                    s->strm->total_in_hi32++;
                }

                s->avail_in_expect--;
            }
        }

        return progress_in;
    }

    /*---------------------------------------------------*/
    CompressionBZip2::Bool
    CompressionBZip2::copy_output_until_stop(CompressionBZip2::EState* s)
    {
        Bool progress_out = BZ_False;

        while (BZ_True)
        {

            /*-- no output space? --*/
            if (s->strm->avail_out == 0)
            {
                break;
            }

            /*-- block done? --*/
            if (s->state_out_pos >= s->numZ)
            {
                break;
            }

            progress_out = BZ_True;
            *(s->strm->next_out) = s->zbits[s->state_out_pos];
            s->state_out_pos++;
            s->strm->avail_out--;
            s->strm->next_out++;
            s->strm->total_out_lo32++;

            if (s->strm->total_out_lo32 == 0)
            {
                s->strm->total_out_hi32++;
            }
        }

        return progress_out;
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::add_pair_to_block(CompressionBZip2::EState* s)
    {
        Int32 i;
        UChar ch = (UChar)(s->state_in_ch);

        for (i = 0; i < s->state_in_len; i++)
        {
            BZ_UPDATE_CRC(s->blockCRC, ch);
        }

        s->inUse[s->state_in_ch] = BZ_True;

        switch (s->state_in_len)
        {
            case 1:
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                break;

            case 2:
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                break;

            case 3:
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                break;

            default:
                s->inUse[s->state_in_len - 4] = BZ_True;
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                s->block[s->nblock] = (UChar)ch;
                s->nblock++;
                s->block[s->nblock] = ((UChar)(s->state_in_len - 4));
                s->nblock++;
                break;
        }
    }

    /*---------------------------------------------------*/
    CompressionBZip2::BZFILE*
    CompressionBZip2::BZ2_bzReadOpen(int* bzerror,
                                     std::istream* f,
                                     int verbosity,
                                     int smallValue,
                                     void* unused,
                                     int nUnused)
    {
        bzFile* bzf = nullptr;
        int ret;

        BZ_SETERR(BZ_OK);

        if (f == nullptr || (smallValue != 0 && smallValue != 1) ||
            (verbosity < 0 || verbosity > 4) || (unused == nullptr && nUnused != 0) ||
            (unused != nullptr && (nUnused < 0 || nUnused > BZ_MAX_UNUSED)))
        {
            BZ_SETERR(BZ_PARAM_ERROR);
            return nullptr;
        };

        // if (ferror(f))
        if (!f->good())
        {
            BZ_SETERR(BZ_IO_ERROR);
            return nullptr;
        };

        bzf = (CompressionBZip2::bzFile*)malloc(sizeof(bzFile));

        if (bzf == nullptr)
        {
            BZ_SETERR(BZ_MEM_ERROR);
            return nullptr;
        };

        BZ_SETERR(BZ_OK);

        bzf->initialisedOk = BZ_False;

        bzf->handleIn = f;

        bzf->handleOut = nullptr;

        bzf->bufN = 0;

        bzf->writing = BZ_False;

        bzf->strm.bzalloc = nullptr;

        bzf->strm.bzfree = nullptr;

        bzf->strm.opaque = nullptr;

        while (nUnused > 0)
        {
            bzf->buf[bzf->bufN] = *((UChar*)(unused));
            bzf->bufN++;
            unused = ((void*)(1 + ((UChar*)(unused))));
            nUnused--;
        }

        ret = BZ2_bzDecompressInit(&(bzf->strm), verbosity, smallValue);

        if (ret != BZ_OK)
        {
            BZ_SETERR(ret);
            free(bzf);
            return nullptr;
        };

        bzf->strm.avail_in = bzf->bufN;

        bzf->strm.next_in = bzf->buf;

        bzf->initialisedOk = BZ_True;

        return bzf;
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_bzReadClose(int* bzerror, BZFILE* b)
    {
        bzFile* bzf = (bzFile*)b;

        BZ_SETERR(BZ_OK);

        if (bzf == nullptr)
        {
            BZ_SETERR(BZ_OK);
            return;
        };

        if (bzf->writing)
        {
            BZ_SETERR(BZ_SEQUENCE_ERROR);
            return;
        };

        if (bzf->initialisedOk)
        {
            if (bzf->strm.avail_in)
            {
                if (bzf->handleIn)
                {
                    // set position in stream according to processed bytes (we have read more bytes than we processed)
                    /*std::streampos p =*/bzf->handleIn->tellg();
                    std::streamoff o(-(int)(bzf->strm.avail_in));
                    /*if (p>o)
                    {
                        std::cout << "Error, expecting position in stream > p?!" << std::endl;
                        BZ_SETERR(BZ_SEQUENCE_ERROR); return;
                    }*/
                    bzf->strm.avail_in = 0;
                    // unset eof and fail flags
                    bzf->handleIn->clear();
                    bzf->handleIn->seekg(o, std::ios_base::cur);
                }
                else
                {
                    std::cout << "internal error, expecting handleIn..." << std::endl;
                    BZ_SETERR(BZ_SEQUENCE_ERROR);
                    return;
                }
            }

            (void)BZ2_bzDecompressEnd(&(bzf->strm));
        }

        free(bzf);
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::BZ2_bzRead(int* bzerror, BZFILE* b, void* buf, int len)
    {
        Int32 n, ret;
        bzFile* bzf = (bzFile*)b;

        BZ_SETERR(BZ_OK);

        if (bzf == nullptr || buf == nullptr || len < 0)
        {
            BZ_SETERR(BZ_PARAM_ERROR);
            return 0;
        };

        if (bzf->writing)
        {
            BZ_SETERR(BZ_SEQUENCE_ERROR);
            return 0;
        };

        if (len == 0)
        {
            BZ_SETERR(BZ_OK);
            return 0;
        };

        bzf->strm.avail_out = len;

        bzf->strm.next_out = (char*)buf;

        while (BZ_True)
        {

            if (!bzf->handleIn || bzf->handleIn->bad())
            {
                BZ_SETERR(BZ_IO_ERROR);
                return 0;
            };

            if (bzf->strm.avail_in == 0 && !myfeof(bzf->handleIn))
            {

                /*n = fread ( bzf->buf, sizeof(UChar),
                            BZ_MAX_UNUSED, bzf->handle );
                        */
                n = read(bzf->buf, BZ_MAX_UNUSED, bzf->handleIn);

                if (bzf->handleIn->bad())
                {
                    BZ_SETERR(BZ_IO_ERROR);
                    return 0;
                };

                bzf->bufN = n;

                bzf->strm.avail_in = bzf->bufN;

                bzf->strm.next_in = bzf->buf;
            }

            ret = BZ2_bzDecompress(&(bzf->strm));

            if (ret != BZ_OK && ret != BZ_STREAM_END)
            {
                BZ_SETERR(ret);
                return 0;
            };

            if (ret == BZ_OK && myfeof(bzf->handleIn) && bzf->strm.avail_in == 0 &&
                bzf->strm.avail_out > 0)
            {
                BZ_SETERR(BZ_UNEXPECTED_EOF);
                return 0;
            };

            if (ret == BZ_STREAM_END)
            {
                BZ_SETERR(BZ_STREAM_END);
                return len - bzf->strm.avail_out;
            };

            if (bzf->strm.avail_out == 0)
            {
                BZ_SETERR(BZ_OK);
                return len;
            };
        }

        return 0; /*not reached*/
    }

    /*---------------------------------------------*/
    CompressionBZip2::Bool
    CompressionBZip2::myfeof(std::istream* f) //FILE* f )
    {
        if (!f)
        {
            return BZ_True;
        }

        char c;
        f->get(c);

        if (f->eof())
        {
            return BZ_True;
        }

        f->unget();
        return BZ_False;
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::BZ2_bzDecompress(CompressionBZip2::bz_stream* strm)
    {
        Bool corrupt;
        DState* s;

        if (strm == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        s = (DState*)strm->state;

        if (s == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        if (s->strm != strm)
        {
            return BZ_PARAM_ERROR;
        }

        while (BZ_True)
        {
            if (s->state == BZ_X_IDLE)
            {
                return BZ_SEQUENCE_ERROR;
            }

            if (s->state == BZ_X_OUTPUT)
            {
                if (s->smallDecompress)
                {
                    corrupt = unRLE_obuf_to_output_SMALL(s);
                }
                else
                {
                    corrupt = unRLE_obuf_to_output_FAST(s);
                }

                if (corrupt)
                {
                    return BZ_DATA_ERROR;
                }

                if (s->nblock_used == s->save_nblock + 1 && s->state_out_len == 0)
                {
                    BZ_FINALISE_CRC(s->calculatedBlockCRC);

                    if (s->verbosity >= 3)
                        VPrintf2(" {0x%08x, 0x%08x}", s->storedBlockCRC, s->calculatedBlockCRC);

                    if (s->verbosity >= 2)
                    {
                        VPrintf0("]");
                    }

                    if (s->calculatedBlockCRC != s->storedBlockCRC)
                    {
                        return BZ_DATA_ERROR;
                    }

                    s->calculatedCombinedCRC =
                        (s->calculatedCombinedCRC << 1) | (s->calculatedCombinedCRC >> 31);
                    s->calculatedCombinedCRC ^= s->calculatedBlockCRC;
                    s->state = BZ_X_BLKHDR_1;
                }
                else
                {
                    return BZ_OK;
                }
            }

            if (s->state >= BZ_X_MAGIC_1)
            {
                Int32 r = BZ2_decompress(s);

                if (r == BZ_STREAM_END)
                {
                    if (s->verbosity >= 3)
                        VPrintf2("\n    combined CRCs: stored = 0x%08x, computed = 0x%08x",
                                 s->storedCombinedCRC,
                                 s->calculatedCombinedCRC);

                    if (s->calculatedCombinedCRC != s->storedCombinedCRC)
                    {
                        return BZ_DATA_ERROR;
                    }

                    return r;
                }

                if (s->state != BZ_X_OUTPUT)
                {
                    return r;
                }
            }
        }

        AssertH(0, 6001);

        return 0; /*NOTREACHED*/
    }

    /*---------------------------------------------------*/
    /* Return  True iff data corruption is discovered.
       Returns False if there is no problem.
    */

    CompressionBZip2::Bool
    CompressionBZip2::unRLE_obuf_to_output_FAST(CompressionBZip2::DState* s)
    {
        UChar k1;

        if (s->blockRandomised)
        {

            while (BZ_True)
            {
                /* try to finish existing run */
                while (BZ_True)
                {
                    if (s->strm->avail_out == 0)
                    {
                        return BZ_False;
                    }

                    if (s->state_out_len == 0)
                    {
                        break;
                    }

                    *((UChar*)(s->strm->next_out)) = s->state_out_ch;
                    BZ_UPDATE_CRC(s->calculatedBlockCRC, s->state_out_ch);
                    s->state_out_len--;
                    s->strm->next_out++;
                    s->strm->avail_out--;
                    s->strm->total_out_lo32++;

                    if (s->strm->total_out_lo32 == 0)
                    {
                        s->strm->total_out_hi32++;
                    }
                }

                /* can a new run be started? */
                if (s->nblock_used == s->save_nblock + 1)
                {
                    return BZ_False;
                }

                /* Only caused by corrupt data stream? */
                if (s->nblock_used > s->save_nblock + 1)
                {
                    return BZ_True;
                }

                s->state_out_len = 1;
                s->state_out_ch = s->k0;
                BZ_GET_FAST(k1);
                BZ_RAND_UPD_MASK;
                k1 ^= BZ_RAND_MASK;
                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                s->state_out_len = 2;

                BZ_GET_FAST(k1);

                BZ_RAND_UPD_MASK;

                k1 ^= BZ_RAND_MASK;

                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                s->state_out_len = 3;

                BZ_GET_FAST(k1);

                BZ_RAND_UPD_MASK;

                k1 ^= BZ_RAND_MASK;

                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                BZ_GET_FAST(k1);

                BZ_RAND_UPD_MASK;

                k1 ^= BZ_RAND_MASK;

                s->nblock_used++;

                s->state_out_len = ((Int32)k1) + 4;

                BZ_GET_FAST(s->k0);

                BZ_RAND_UPD_MASK;

                s->k0 ^= BZ_RAND_MASK;

                s->nblock_used++;
            }
        }
        else
        {

            /* restore */
            UInt32 c_calculatedBlockCRC = s->calculatedBlockCRC;
            UChar c_state_out_ch = s->state_out_ch;
            Int32 c_state_out_len = s->state_out_len;
            Int32 c_nblock_used = s->nblock_used;
            Int32 c_k0 = s->k0;
            UInt32* c_tt = s->tt;
            UInt32 c_tPos = s->tPos;
            char* cs_next_out = s->strm->next_out;
            unsigned int cs_avail_out = s->strm->avail_out;
            Int32 ro_blockSize100k = s->blockSize100k;
            /* end restore */

            UInt32 avail_out_INIT = cs_avail_out;
            Int32 s_save_nblockPP = s->save_nblock + 1;
            unsigned int total_out_lo32_old;

            while (BZ_True)
            {

                /* try to finish existing run */
                if (c_state_out_len > 0)
                {
                    while (BZ_True)
                    {
                        if (cs_avail_out == 0)
                        {
                            goto return_notr;
                        }

                        if (c_state_out_len == 1)
                        {
                            break;
                        }

                        *((UChar*)(cs_next_out)) = c_state_out_ch;
                        BZ_UPDATE_CRC(c_calculatedBlockCRC, c_state_out_ch);
                        c_state_out_len--;
                        cs_next_out++;
                        cs_avail_out--;
                    }

                s_state_out_len_eq_one:
                {
                    if (cs_avail_out == 0)
                    {
                        c_state_out_len = 1;
                        goto return_notr;
                    };

                    *((UChar*)(cs_next_out)) = c_state_out_ch;

                    BZ_UPDATE_CRC(c_calculatedBlockCRC, c_state_out_ch);

                    cs_next_out++;

                    cs_avail_out--;
                }
                }

                /* Only caused by corrupt data stream? */
                if (c_nblock_used > s_save_nblockPP)
                {
                    return BZ_True;
                }

                /* can a new run be started? */
                if (c_nblock_used == s_save_nblockPP)
                {
                    c_state_out_len = 0;
                    goto return_notr;
                };

                c_state_out_ch = c_k0;

                BZ_GET_FAST_C(k1);

                c_nblock_used++;

                if (k1 != c_k0)
                {
                    c_k0 = k1;
                    goto s_state_out_len_eq_one;
                };

                if (c_nblock_used == s_save_nblockPP)
                {
                    goto s_state_out_len_eq_one;
                }

                c_state_out_len = 2;
                BZ_GET_FAST_C(k1);
                c_nblock_used++;

                if (c_nblock_used == s_save_nblockPP)
                {
                    continue;
                }

                if (k1 != c_k0)
                {
                    c_k0 = k1;
                    continue;
                };

                c_state_out_len = 3;

                BZ_GET_FAST_C(k1);

                c_nblock_used++;

                if (c_nblock_used == s_save_nblockPP)
                {
                    continue;
                }

                if (k1 != c_k0)
                {
                    c_k0 = k1;
                    continue;
                };

                BZ_GET_FAST_C(k1);

                c_nblock_used++;

                c_state_out_len = ((Int32)k1) + 4;

                BZ_GET_FAST_C(c_k0);

                c_nblock_used++;
            }

        return_notr:
            total_out_lo32_old = s->strm->total_out_lo32;
            s->strm->total_out_lo32 += (avail_out_INIT - cs_avail_out);

            if (s->strm->total_out_lo32 < total_out_lo32_old)
            {
                s->strm->total_out_hi32++;
            }

            /* save */
            s->calculatedBlockCRC = c_calculatedBlockCRC;
            s->state_out_ch = c_state_out_ch;
            s->state_out_len = c_state_out_len;
            s->nblock_used = c_nblock_used;
            s->k0 = c_k0;
            s->tt = c_tt;
            s->tPos = c_tPos;
            s->strm->next_out = cs_next_out;
            s->strm->avail_out = cs_avail_out;
            /* end save */
        }

        return BZ_False;
    }

    /*---------------------------------------------------*/
    /* Return  True iff data corruption is discovered.
       Returns False if there is no problem.
    */
    CompressionBZip2::Bool
    CompressionBZip2::unRLE_obuf_to_output_SMALL(CompressionBZip2::DState* s)
    {
        UChar k1;

        if (s->blockRandomised)
        {

            while (BZ_True)
            {
                /* try to finish existing run */
                while (BZ_True)
                {
                    if (s->strm->avail_out == 0)
                    {
                        return BZ_False;
                    }

                    if (s->state_out_len == 0)
                    {
                        break;
                    }

                    *((UChar*)(s->strm->next_out)) = s->state_out_ch;
                    BZ_UPDATE_CRC(s->calculatedBlockCRC, s->state_out_ch);
                    s->state_out_len--;
                    s->strm->next_out++;
                    s->strm->avail_out--;
                    s->strm->total_out_lo32++;

                    if (s->strm->total_out_lo32 == 0)
                    {
                        s->strm->total_out_hi32++;
                    }
                }

                /* can a new run be started? */
                if (s->nblock_used == s->save_nblock + 1)
                {
                    return BZ_False;
                }

                /* Only caused by corrupt data stream? */
                if (s->nblock_used > s->save_nblock + 1)
                {
                    return BZ_True;
                }

                s->state_out_len = 1;
                s->state_out_ch = s->k0;
                BZ_GET_SMALL(k1);
                BZ_RAND_UPD_MASK;
                k1 ^= BZ_RAND_MASK;
                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                s->state_out_len = 2;

                BZ_GET_SMALL(k1);

                BZ_RAND_UPD_MASK;

                k1 ^= BZ_RAND_MASK;

                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                s->state_out_len = 3;

                BZ_GET_SMALL(k1);

                BZ_RAND_UPD_MASK;

                k1 ^= BZ_RAND_MASK;

                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                BZ_GET_SMALL(k1);

                BZ_RAND_UPD_MASK;

                k1 ^= BZ_RAND_MASK;

                s->nblock_used++;

                s->state_out_len = ((Int32)k1) + 4;

                BZ_GET_SMALL(s->k0);

                BZ_RAND_UPD_MASK;

                s->k0 ^= BZ_RAND_MASK;

                s->nblock_used++;
            }
        }
        else
        {

            while (BZ_True)
            {
                /* try to finish existing run */
                while (BZ_True)
                {
                    if (s->strm->avail_out == 0)
                    {
                        return BZ_False;
                    }

                    if (s->state_out_len == 0)
                    {
                        break;
                    }

                    *((UChar*)(s->strm->next_out)) = s->state_out_ch;
                    BZ_UPDATE_CRC(s->calculatedBlockCRC, s->state_out_ch);
                    s->state_out_len--;
                    s->strm->next_out++;
                    s->strm->avail_out--;
                    s->strm->total_out_lo32++;

                    if (s->strm->total_out_lo32 == 0)
                    {
                        s->strm->total_out_hi32++;
                    }
                }

                /* can a new run be started? */
                if (s->nblock_used == s->save_nblock + 1)
                {
                    return BZ_False;
                }

                /* Only caused by corrupt data stream? */
                if (s->nblock_used > s->save_nblock + 1)
                {
                    return BZ_True;
                }

                s->state_out_len = 1;
                s->state_out_ch = s->k0;
                BZ_GET_SMALL(k1);
                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                s->state_out_len = 2;

                BZ_GET_SMALL(k1);

                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                s->state_out_len = 3;

                BZ_GET_SMALL(k1);

                s->nblock_used++;

                if (s->nblock_used == s->save_nblock + 1)
                {
                    continue;
                }

                if (k1 != s->k0)
                {
                    s->k0 = k1;
                    continue;
                };

                BZ_GET_SMALL(k1);

                s->nblock_used++;

                s->state_out_len = ((Int32)k1) + 4;

                BZ_GET_SMALL(s->k0);

                s->nblock_used++;
            }
        }
    }

    /*---------------------------------------------------*/
    int
    CompressionBZip2::BZ2_bzDecompressInit(bz_stream* strm, int verbosity, int smallValue)
    {
        DState* s;

        if (!bz_config_ok())
        {
            return BZ_CONFIG_ERROR;
        }

        if (strm == nullptr)
        {
            return BZ_PARAM_ERROR;
        }

        if (smallValue != 0 && smallValue != 1)
        {
            return BZ_PARAM_ERROR;
        }

        if (verbosity < 0 || verbosity > 4)
        {
            return BZ_PARAM_ERROR;
        }

        if (strm->bzalloc == nullptr)
        {
            strm->bzalloc = default_bzalloc;
        }

        if (strm->bzfree == nullptr)
        {
            strm->bzfree = default_bzfree;
        }

        s = (DState*)BZALLOC(sizeof(DState));

        if (s == nullptr)
        {
            return BZ_MEM_ERROR;
        }

        s->strm = strm;
        strm->state = s;
        s->state = BZ_X_MAGIC_1;
        s->bsLive = 0;
        s->bsBuff = 0;
        s->calculatedCombinedCRC = 0;
        strm->total_in_lo32 = 0;
        strm->total_in_hi32 = 0;
        strm->total_out_lo32 = 0;
        strm->total_out_hi32 = 0;
        s->smallDecompress = (Bool)smallValue;
        s->ll4 = nullptr;
        s->ll16 = nullptr;
        s->tt = nullptr;
        s->currBlockNo = 0;
        s->verbosity = verbosity;

        return BZ_OK;
    }

    bool
    CompressionBZip2::write(void* buf, int len)
    {
        if (!buf || len <= 0)
        {
            return false;
        }

        if (mode != eCompress)
        {
            VR_ERROR << "Not in compression mode?!" << std::endl;
            return false;
        }

        if (!bzFileData)
        {
            return false;
        }

        BZ2_bzWrite(&currentError, bzFileData, buf, len);

        if (currentError < 0) //== BZ_IO_ERROR) {
        {
            VR_ERROR << "Could not compress data..." << std::endl;
            close();
            return false;
        }

        return true;
    }

    void*
    CompressionBZip2::default_bzalloc(void* /*opaque*/,
                                      CompressionBZip2::Int32 items,
                                      CompressionBZip2::Int32 size)
    {
        void* v = malloc(items * size);
        return v;
    }

    void
    CompressionBZip2::default_bzfree(void* /*opaque*/, void* addr)
    {
        if (addr != nullptr)
        {
            free(addr);
        }
    }

    bool
    CompressionBZip2::read(void* buf, int maxLen, int& storeLengthRead)
    {
        if (!buf || maxLen <= 0)
        {
            return false;
        }

        if (mode != eUncompress)
        {
            VR_ERROR << "Not in uncompression mode?!" << std::endl;
            return false;
        }

        if (!bzFileData)
        {
            return false;
        }

        storeLengthRead = BZ2_bzRead(&currentError, bzFileData, buf, maxLen);

        if (currentError < 0) //== BZ_IO_ERROR) {
        {
            VR_ERROR << "Could not uncompress data..." << std::endl;
            close();
            return false;
        }

        return true;
    }

    CompressionBZip2::Int32
    CompressionBZip2::BZ2_indexIntoF(CompressionBZip2::Int32 indx, Int32* cftab)
    {
        Int32 nb, na, mid;
        nb = 0;
        na = 256;

        do
        {
            mid = (nb + na) >> 1;

            if (indx >= cftab[mid])
            {
                nb = mid;
            }
            else
            {
                na = mid;
            }
        } while (na - nb != 1);

        return nb;
    }

    /*---------------------------------------------------*/
#define WEIGHTOF(zz0) ((zz0) & 0xffffff00)
#define DEPTHOF(zz1) ((zz1) & 0x000000ff)
#define MYMAX(zz2, zz3) ((zz2) > (zz3) ? (zz2) : (zz3))

#define ADDWEIGHTS(zw1, zw2)                                                                       \
    (WEIGHTOF(zw1) + WEIGHTOF(zw2)) | (1 + MYMAX(DEPTHOF(zw1), DEPTHOF(zw2)))

#define UPHEAP(z)                                                                                  \
    {                                                                                              \
        Int32 zz, tmp;                                                                             \
        zz = z;                                                                                    \
        tmp = heap[zz];                                                                            \
        while (weight[tmp] < weight[heap[zz >> 1]])                                                \
        {                                                                                          \
            heap[zz] = heap[zz >> 1];                                                              \
            zz >>= 1;                                                                              \
        }                                                                                          \
        heap[zz] = tmp;                                                                            \
    }

#define DOWNHEAP(z)                                                                                \
    {                                                                                              \
        Int32 zz, yy, tmp;                                                                         \
        zz = z;                                                                                    \
        tmp = heap[zz];                                                                            \
        while (BZ_True)                                                                            \
        {                                                                                          \
            yy = zz << 1;                                                                          \
            if (yy > nHeap)                                                                        \
                break;                                                                             \
            if (yy < nHeap && weight[heap[yy + 1]] < weight[heap[yy]])                             \
                yy++;                                                                              \
            if (weight[tmp] < weight[heap[yy]])                                                    \
                break;                                                                             \
            heap[zz] = heap[yy];                                                                   \
            zz = yy;                                                                               \
        }                                                                                          \
        heap[zz] = tmp;                                                                            \
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_hbMakeCodeLengths(UChar* len, Int32* freq, Int32 alphaSize, Int32 maxLen)
    {
        /*--
           Nodes and heap entries run from 1.  Entry 0
           for both the heap and nodes is a sentinel.
        --*/
        Int32 nNodes, nHeap, n1, n2, i, j, k;
        Bool tooLong;

        Int32 heap[BZ_MAX_ALPHA_SIZE + 2];
        Int32 weight[BZ_MAX_ALPHA_SIZE * 2];
        Int32 parent[BZ_MAX_ALPHA_SIZE * 2];

        for (i = 0; i < alphaSize; i++)
        {
            weight[i + 1] = (freq[i] == 0 ? 1 : freq[i]) << 8;
        }

        while (BZ_True)
        {

            nNodes = alphaSize;
            nHeap = 0;

            heap[0] = 0;
            weight[0] = 0;
            parent[0] = -2;

            for (i = 1; i <= alphaSize; i++)
            {
                parent[i] = -1;
                nHeap++;
                heap[nHeap] = i;
                UPHEAP(nHeap);
            }

            AssertH(nHeap < (BZ_MAX_ALPHA_SIZE + 2), 2001);

            while (nHeap > 1)
            {
                n1 = heap[1];
                heap[1] = heap[nHeap];
                nHeap--;
                DOWNHEAP(1);
                n2 = heap[1];
                heap[1] = heap[nHeap];
                nHeap--;
                DOWNHEAP(1);
                nNodes++;
                parent[n1] = parent[n2] = nNodes;
                weight[nNodes] = ADDWEIGHTS(weight[n1], weight[n2]);
                parent[nNodes] = -1;
                nHeap++;
                heap[nHeap] = nNodes;
                UPHEAP(nHeap);
            }

            AssertH(nNodes < (BZ_MAX_ALPHA_SIZE * 2), 2002);

            tooLong = BZ_False;

            for (i = 1; i <= alphaSize; i++)
            {
                j = 0;
                k = i;

                while (parent[k] >= 0)
                {
                    k = parent[k];
                    j++;
                }

                len[i - 1] = j;

                if (j > maxLen)
                {
                    tooLong = BZ_True;
                }
            }

            if (!tooLong)
            {
                break;
            }

            /* 17 Oct 04: keep-going condition for the following loop used
               to be 'i < alphaSize', which missed the last element,
               theoretically leading to the possibility of the compressor
               looping.  However, this count-scaling step is only needed if
               one of the generated Huffman code words is longer than
               maxLen, which up to and including version 1.0.2 was 20 bits,
               which is extremely unlikely.  In version 1.0.3 maxLen was
               changed to 17 bits, which has minimal effect on compression
               ratio, but does mean this scaling step is used from time to
               time, enough to verify that it works.

               This means that bzip2-1.0.3 and later will only produce
               Huffman codes with a maximum length of 17 bits.  However, in
               order to preserve backwards compatibility with bitstreams
               produced by versions pre-1.0.3, the decompressor must still
               handle lengths of up to 20. */

            for (i = 1; i <= alphaSize; i++)
            {
                j = weight[i] >> 8;
                j = 1 + (j / 2);
                weight[i] = j << 8;
            }
        }
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_hbAssignCodes(Int32* code,
                                        UChar* length,
                                        Int32 minLen,
                                        Int32 maxLen,
                                        Int32 alphaSize)
    {
        Int32 n, vec, i;

        vec = 0;

        for (n = minLen; n <= maxLen; n++)
        {
            for (i = 0; i < alphaSize; i++)
                if (length[i] == n)
                {
                    code[i] = vec;
                    vec++;
                };

            vec <<= 1;
        }
    }

    /*---------------------------------------------------*/
    void
    CompressionBZip2::BZ2_hbCreateDecodeTables(Int32* limit,
                                               Int32* base,
                                               Int32* perm,
                                               UChar* length,
                                               Int32 minLen,
                                               Int32 maxLen,
                                               Int32 alphaSize)
    {
        Int32 pp, i, j, vec;

        pp = 0;

        for (i = minLen; i <= maxLen; i++)
            for (j = 0; j < alphaSize; j++)
                if (length[j] == i)
                {
                    perm[pp] = j;
                    pp++;
                };

        for (i = 0; i < BZ_MAX_CODE_LEN; i++)
        {
            base[i] = 0;
        }

        for (i = 0; i < alphaSize; i++)
        {
            base[length[i] + 1]++;
        }

        for (i = 1; i < BZ_MAX_CODE_LEN; i++)
        {
            base[i] += base[i - 1];
        }

        for (i = 0; i < BZ_MAX_CODE_LEN; i++)
        {
            limit[i] = 0;
        }

        vec = 0;

        for (i = minLen; i <= maxLen; i++)
        {
            vec += (base[i + 1] - base[i]);
            limit[i] = vec - 1;
            vec <<= 1;
        }

        for (i = minLen + 1; i <= maxLen; i++)
        {
            base[i] = ((limit[i - 1] + 1) << 1) - base[i];
        }
    }

    int
    CompressionBZip2::read(char* res, int num, std::istream* file)
    {
        if (!file)
        {
            return 0;
        }

        file->read((char*)res, num * sizeof(char));
        return (int)file->gcount();
    }

    bool
    CompressionBZip2::write(std::ostream* file, const char* value, int num)
    {
        if (!file)
        {
            return false;
        }

        file->write((char*)value, num * sizeof(char));
        return file->good();
    }

} // namespace VirtualRobot

#pragma GCC diagnostic pop

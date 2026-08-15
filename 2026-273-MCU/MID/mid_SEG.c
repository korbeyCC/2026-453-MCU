#include "mid_SEG.h"

static const uint8_t seg7code[] = {
    // 显示段码 数码管字跟
    0x3f, //    0
    0x06, //    1
    0x5b, //    2
    0x4f, //    3
    0x66, //    4
    0x6d, //    5
    0x7d, //    6
    0x07, //    7
    0x7f, //    8
    0x6f, //    9
    0x77, // A	10
    0x7c, // b	11
    0x39, // C	12
    0x5e, // d	13
    0x79, // E  14
    0x71, // F  15
    0x67, // 9  16
    0x73, // P	17
    0x40, // -	18
    0x00, //    19
    0xc0, // -. 20
    0x3e, // U  21
    0x38, // L  22
    0x78, // t  23
    0x5c, // o  24
    0x58, // c  25
    0x76, // H  26
    0x54, // n  27
    0x50, // r  28
};

/**
 * @brief 数码管显示
 *
 * @param seg_data 数码管显示缓存区指针
 * @param flag_data 按键显示缓冲区指针
 * @param SEG_quantity 数码管数量
 * @param OUT_data 输出缓冲区指针
 */
void MID_SEG_Display(uint8_t *seg_data, uint8_t *flag_data, uint8_t SEG_quantity, uint8_t *OUT_data)
{
    for (uint8_t i = 0; i < SEG_quantity; i++)
    {
        if (flag_data[i] == 0)
                OUT_data[i] = seg7code[seg_data[i]];
            else
                OUT_data[i] = seg7code[seg_data[i]] | 0x80;
    }
}

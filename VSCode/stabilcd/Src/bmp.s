//;  ******************************************************************************
//;  * @file    bmp.s
//;  * @author  eg
//;  * @brief   BMP-file inclusion assembly wrapper
//;  ******************************************************************************

//;    AREA    IMG_BMP, DATA, READONLY, PREINIT_ARRAY
//;    INCBIN  img.bmp
//;    END

    .section .rodata
include_bitmap_file: 
    .incbin "img.bmp"

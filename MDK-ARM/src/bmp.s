;  ******************************************************************************
;  * @file    bmp.s
;  * @author  eg
;  * @brief   BMP-file inclusion assembly wrapper
;  ******************************************************************************

    AREA    IMG_BMP, DATA, READONLY, PREINIT_ARRAY
    INCBIN  img2.bmp
    END

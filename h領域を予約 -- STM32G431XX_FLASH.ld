[1mdiff --git a/STM32G431XX_FLASH.ld b/STM32G431XX_FLASH.ld[m
[1mindex 8017b1c..8ef103c 100644[m
[1m--- a/STM32G431XX_FLASH.ld[m
[1m+++ b/STM32G431XX_FLASH.ld[m
[36m@@ -56,7 +56,7 @@[m [mENTRY(Reset_Handler)[m
 MEMORY[m
 {[m
 RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 32K[m
[31m-FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 128K[m
[32m+[m[32mFLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 126K[m
 }[m
 [m
 /* Highest address of the user mode stack */[m

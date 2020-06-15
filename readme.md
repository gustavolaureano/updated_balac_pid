Arduino code for the M5stack BalaC balancing robot:
https://m5stack.com/products/bala-c-esp32-development-mini-self-balancing-car

This code is a modified version of the updated Coppercele's code:
https://qiita.com/coppercele/items/527228e3f08c53597bd1

This code also requires the following modified M5StickC library:
https://github.com/gustavolaureano/M5StickC

Main changes:
 - Fixed the target angle (needs to be adjusted to your balaC balance angle)
 - Added minimum duty (for removing the DC motor dead-band)
 - Individual control of wheels (for compensating the different minimum duty of each DC motor)
 - Added the loop frequency input to the Mahony filter, for improved sensor fusion (uses the modified M5StickC lib for it)
 - Tuned PID gains (but it still have room for improvements)

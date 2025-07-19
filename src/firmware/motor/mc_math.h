
/**
  ******************************************************************************
  * @file    mc_math.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MC_Math
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_MATH_H
#define MC_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "mcsdk/mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_Math
  * @{
  */
#define SQRT_2  1.4142
#define SQRT_3  1.732

/**
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
  (((x) == 65535 ) ? 16 : \
   (((x) == (2*2*2*2*2*2*2*2*2*2*2*2*2*2*2)) ? 15 : \
    (((x) == (2*2*2*2*2*2*2*2*2*2*2*2*2*2)) ? 14 : \
     (((x) == (2*2*2*2*2*2*2*2*2*2*2*2*2)) ? 13 : \
      (((x) == (2*2*2*2*2*2*2*2*2*2*2*2)) ? 12 : \
       (((x) == (2*2*2*2*2*2*2*2*2*2*2)) ? 11 : \
        (((x) == (2*2*2*2*2*2*2*2*2*2)) ? 10 : \
         (((x) == (2*2*2*2*2*2*2*2*2)) ? 9 : \
          (((x) == (2*2*2*2*2*2*2*2)) ? 8 : \
           (((x) == (2*2*2*2*2*2*2)) ? 7 : \
            (((x) == (2*2*2*2*2*2)) ? 6 : \
             (((x) == (2*2*2*2*2)) ? 5 : \
              (((x) == (2*2*2*2)) ? 4 : \
               (((x) == (2*2*2)) ? 3 : \
                (((x) == (2*2)) ? 2 : \
                 (((x) == 2) ? 1 : \
                  (((x) == 1) ? 0 : -1)))))))))))))))))

/**
  * @brief  Trigonometrical functions type definition
  */
typedef struct
{
  int16_t hCos;
  int16_t hSin;
} Trig_Components;

/**
  * @brief  This function transforms stator currents Ia and qIb (which are
  *         directed along axes each displaced by 120 degrees) into currents
  *         Ialpha and Ibeta in a stationary qd reference frame.
  *                               Ialpha = Ia
  *                       Ibeta = -(2*Ib+Ia)/sqrt(3)
  * @param  Curr_Input: stator current Ia and Ib in ab_t format
  * @retval Stator current Ialpha and Ibeta in alphabeta_t format
  */
alphabeta_t MCM_Clarke(ab_t Input);

/**
  * @brief  This function transforms stator values alpha and beta, which
  *         belong to a stationary qd reference frame, to a rotor flux
  *         synchronous reference frame (properly oriented), so as Iq and Id.
  *                   Id= Ialpha *sin(theta)+qIbeta *cos(Theta)
  *                   Iq=qIalpha *cos(Theta)-qIbeta *sin(Theta)
  * @param  Curr_Input: stator values alpha and beta in alphabeta_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator current q and d in qd_t format
  */
qd_t MCM_Park(alphabeta_t Input, int16_t Theta);

/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Curr_Input: stator voltage Vq and Vd in qd_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator values alpha and beta in alphabeta_t format
  */
alphabeta_t MCM_Rev_Park(qd_t Input, int16_t Theta);

/**
  * @brief  This function returns cosine and sine functions of the angle fed in
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Trig_Components Cos(angle) and Sin(angle) in Trig_Components format
  */
Trig_Components MCM_Trig_Functions(int16_t hAngle);

/**
  * @brief  It calculates the square root of a non-negative s32. It returns 0
  *         for negative s32.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
int32_t MCM_Sqrt(int32_t wInput);

/**
  * @brief  Sqrt table used by Circle Limitation function
  *         used for STM32F0/STM32G0 series only
  */
#define SQRT_CIRCLE_LIMITATION {\
     0 , 1023 , 1448 , 1773 , 2047 , 2289 , 2508 , 2709,\
     2896 , 3071 , 3238 , 3396 , 3547 , 3691 , 3831 , 3965,\
     4095 , 4221 , 4344 , 4463 , 4579 , 4692 , 4802 , 4910,\
     5016 , 5119 , 5221 , 5320 , 5418 , 5514 , 5608 , 5701,\
     5792 , 5882 , 5970 , 6057 , 6143 , 6228 , 6312 , 6394,\
     6476 , 6556 , 6636 , 6714 , 6792 , 6868 , 6944 , 7019,\
     7094 , 7167 , 7240 , 7312 , 7383 , 7454 , 7524 , 7593,\
     7662 , 7730 , 7798 , 7865 , 7931 , 7997 , 8062 , 8127,\
     8191 , 8255 , 8318 , 8381 , 8443 , 8505 , 8567 , 8628,\
     8688 , 8748 , 8808 , 8867 , 8926 , 8985 , 9043 , 9101,\
     9158 , 9215 , 9272 , 9328 , 9384 , 9440 , 9495 , 9550,\
     9605 , 9660 , 9714 , 9768 , 9821 , 9874 , 9927 , 9980,\
     10032 , 10084 , 10136 , 10188 , 10239 , 10290 , 10341 , 10392,\
     10442 , 10492 , 10542 , 10592 , 10641 , 10690 , 10739 , 10788,\
     10836 , 10884 , 10932 , 10980 , 11028 , 11075 , 11123 , 11170,\
     11217 , 11263 , 11310 , 11356 , 11402 , 11448 , 11494 , 11539,\
     11584 , 11630 , 11675 , 11719 , 11764 , 11808 , 11853 , 11897,\
     11941 , 11985 , 12028 , 12072 , 12115 , 12158 , 12201 , 12244,\
     12287 , 12330 , 12372 , 12414 , 12457 , 12499 , 12541 , 12582,\
     12624 , 12665 , 12707 , 12748 , 12789 , 12830 , 12871 , 12911,\
     12952 , 12992 , 13032 , 13073 , 13113 , 13153 , 13192 , 13232,\
     13272 , 13311 , 13350 , 13390 , 13429 , 13468 , 13507 , 13545,\
     13584 , 13623 , 13661 , 13699 , 13737 , 13776 , 13814 , 13851,\
     13889 , 13927 , 13965 , 14002 , 14039 , 14077 , 14114 , 14151,\
     14188 , 14225 , 14262 , 14298 , 14335 , 14372 , 14408 , 14444,\
     14481 , 14517 , 14553 , 14589 , 14625 , 14661 , 14696 , 14732,\
     14767 , 14803 , 14838 , 14874 , 14909 , 14944 , 14979 , 15014,\
     15049 , 15084 , 15118 , 15153 , 15187 , 15222 , 15256 , 15291,\
     15325 , 15359 , 15393 , 15427 , 15461 , 15495 , 15529 , 15562,\
     15596 , 15630 , 15663 , 15697 , 15730 , 15763 , 15797 , 15830,\
     15863 , 15896 , 15929 , 15962 , 15994 , 16027 , 16060 , 16092,\
     16125 , 16157 , 16190 , 16222 , 16254 , 16287 , 16319 , 16351,\
     16383 , 16415 , 16447 , 16479 , 16510 , 16542 , 16574 , 16605,\
     16637 , 16669 , 16700 , 16731 , 16763 , 16794 , 16825 , 16856,\
     16887 , 16918 , 16949 , 16980 , 17011 , 17042 , 17072 , 17103,\
     17134 , 17164 , 17195 , 17225 , 17256 , 17286 , 17316 , 17347,\
     17377 , 17407 , 17437 , 17467 , 17497 , 17527 , 17557 , 17587,\
     17617 , 17646 , 17676 , 17706 , 17735 , 17765 , 17794 , 17824,\
     17853 , 17882 , 17912 , 17941 , 17970 , 17999 , 18028 , 18057,\
     18086 , 18115 , 18144 , 18173 , 18202 , 18231 , 18259 , 18288,\
     18317 , 18345 , 18374 , 18402 , 18431 , 18459 , 18488 , 18516,\
     18544 , 18573 , 18601 , 18629 , 18657 , 18685 , 18713 , 18741,\
     18769 , 18797 , 18825 , 18853 , 18881 , 18908 , 18936 , 18964,\
     18991 , 19019 , 19046 , 19074 , 19101 , 19129 , 19156 , 19184,\
     19211 , 19238 , 19265 , 19293 , 19320 , 19347 , 19374 , 19401,\
     19428 , 19455 , 19482 , 19509 , 19536 , 19562 , 19589 , 19616,\
     19643 , 19669 , 19696 , 19723 , 19749 , 19776 , 19802 , 19829,\
     19855 , 19881 , 19908 , 19934 , 19960 , 19987 , 20013 , 20039,\
     20065 , 20091 , 20117 , 20143 , 20169 , 20195 , 20221 , 20247,\
     20273 , 20299 , 20325 , 20350 , 20376 , 20402 , 20428 , 20453,\
     20479 , 20504 , 20530 , 20556 , 20581 , 20606 , 20632 , 20657,\
     20683 , 20708 , 20733 , 20759 , 20784 , 20809 , 20834 , 20859,\
     20884 , 20910 , 20935 , 20960 , 20985 , 21010 , 21035 , 21059,\
     21084 , 21109 , 21134 , 21159 , 21184 , 21208 , 21233 , 21258,\
     21282 , 21307 , 21331 , 21356 , 21381 , 21405 , 21430 , 21454,\
     21478 , 21503 , 21527 , 21552 , 21576 , 21600 , 21624 , 21649,\
     21673 , 21697 , 21721 , 21745 , 21769 , 21793 , 21817 , 21841,\
     21865 , 21889 , 21913 , 21937 , 21961 , 21985 , 22009 , 22033,\
     22056 , 22080 , 22104 , 22128 , 22151 , 22175 , 22199 , 22222,\
     22246 , 22269 , 22293 , 22316 , 22340 , 22363 , 22387 , 22410,\
     22434 , 22457 , 22480 , 22504 , 22527 , 22550 , 22573 , 22597,\
     22620 , 22643 , 22666 , 22689 , 22712 , 22735 , 22758 , 22781,\
     22804 , 22827 , 22850 , 22873 , 22896 , 22919 , 22942 , 22965,\
     22988 , 23010 , 23033 , 23056 , 23079 , 23101 , 23124 , 23147,\
     23169 , 23192 , 23214 , 23237 , 23260 , 23282 , 23305 , 23327,\
     23350 , 23372 , 23394 , 23417 , 23439 , 23462 , 23484 , 23506,\
     23529 , 23551 , 23573 , 23595 , 23617 , 23640 , 23662 , 23684,\
     23706 , 23728 , 23750 , 23772 , 23794 , 23816 , 23838 , 23860,\
     23882 , 23904 , 23926 , 23948 , 23970 , 23992 , 24014 , 24036,\
     24057 , 24079 , 24101 , 24123 , 24144 , 24166 , 24188 , 24209,\
     24231 , 24253 , 24274 , 24296 , 24317 , 24339 , 24360 , 24382,\
     24403 , 24425 , 24446 , 24468 , 24489 , 24511 , 24532 , 24553,\
     24575 , 24596 , 24617 , 24639 , 24660 , 24681 , 24702 , 24724,\
     24745 , 24766 , 24787 , 24808 , 24829 , 24851 , 24872 , 24893,\
     24914 , 24935 , 24956 , 24977 , 24998 , 25019 , 25040 , 25061,\
     25082 , 25102 , 25123 , 25144 , 25165 , 25186 , 25207 , 25227,\
     25248 , 25269 , 25290 , 25310 , 25331 , 25352 , 25372 , 25393,\
     25414 , 25434 , 25455 , 25476 , 25496 , 25517 , 25537 , 25558,\
     25578 , 25599 , 25619 , 25640 , 25660 , 25681 , 25701 , 25721,\
     25742 , 25762 , 25782 , 25803 , 25823 , 25843 , 25864 , 25884,\
     25904 , 25924 , 25945 , 25965 , 25985 , 26005 , 26025 , 26045,\
     26065 , 26086 , 26106 , 26126 , 26146 , 26166 , 26186 , 26206,\
     26226 , 26246 , 26266 , 26286 , 26306 , 26326 , 26346 , 26365,\
     26385 , 26405 , 26425 , 26445 , 26465 , 26484 , 26504 , 26524,\
     26544 , 26564 , 26583 , 26603 , 26623 , 26642 , 26662 , 26682,\
     26701 , 26721 , 26741 , 26760 , 26780 , 26799 , 26819 , 26838,\
     26858 , 26877 , 26897 , 26916 , 26936 , 26955 , 26975 , 26994,\
     27014 , 27033 , 27052 , 27072 , 27091 , 27111 , 27130 , 27149,\
     27168 , 27188 , 27207 , 27226 , 27246 , 27265 , 27284 , 27303,\
     27322 , 27342 , 27361 , 27380 , 27399 , 27418 , 27437 , 27456,\
     27475 , 27495 , 27514 , 27533 , 27552 , 27571 , 27590 , 27609,\
     27628 , 27647 , 27666 , 27685 , 27703 , 27722 , 27741 , 27760,\
     27779 , 27798 , 27817 , 27836 , 27854 , 27873 , 27892 , 27911,\
     27930 , 27948 , 27967 , 27986 , 28005 , 28023 , 28042 , 28061,\
     28079 , 28098 , 28117 , 28135 , 28154 , 28173 , 28191 , 28210,\
     28228 , 28247 , 28265 , 28284 , 28303 , 28321 , 28340 , 28358,\
     28377 , 28395 , 28413 , 28432 , 28450 , 28469 , 28487 , 28506,\
     28524 , 28542 , 28561 , 28579 , 28597 , 28616 , 28634 , 28652,\
     28671 , 28689 , 28707 , 28725 , 28744 , 28762 , 28780 , 28798,\
     28817 , 28835 , 28853 , 28871 , 28889 , 28907 , 28925 , 28944,\
     28962 , 28980 , 28998 , 29016 , 29034 , 29052 , 29070 , 29088,\
     29106 , 29124 , 29142 , 29160 , 29178 , 29196 , 29214 , 29232,\
     29250 , 29268 , 29286 , 29304 , 29322 , 29339 , 29357 , 29375,\
     29393 , 29411 , 29429 , 29446 , 29464 , 29482 , 29500 , 29518,\
     29535 , 29553 , 29571 , 29588 , 29606 , 29624 , 29642 , 29659,\
     29677 , 29695 , 29712 , 29730 , 29748 , 29765 , 29783 , 29800,\
     29818 , 29835 , 29853 , 29871 , 29888 , 29906 , 29923 , 29941,\
     29958 , 29976 , 29993 , 30011 , 30028 , 30046 , 30063 , 30080,\
     30098 , 30115 , 30133 , 30150 , 30168 , 30185 , 30202 , 30220,\
     30237 , 30254 , 30272 , 30289 , 30306 , 30324 , 30341 , 30358,\
     30375 , 30393 , 30410 , 30427 , 30444 , 30461 , 30479 , 30496,\
     30513 , 30530 , 30547 , 30565 , 30582 , 30599 , 30616 , 30633,\
     30650 , 30667 , 30684 , 30701 , 30719 , 30736 , 30753 , 30770,\
     30787 , 30804 , 30821 , 30838 , 30855 , 30872 , 30889 , 30906,\
     30923 , 30940 , 30957 , 30973 , 30990 , 31007 , 31024 , 31041,\
     31058 , 31075 , 31092 , 31109 , 31125 , 31142 , 31159 , 31176,\
     31193 , 31210 , 31226 , 31243 , 31260 , 31277 , 31293 , 31310,\
     31327 , 31344 , 31360 , 31377 , 31394 , 31410 , 31427 , 31444,\
     31461 , 31477 , 31494 , 31510 , 31527 , 31544 , 31560 , 31577,\
     31594 , 31610 , 31627 , 31643 , 31660 , 31676 , 31693 , 31709,\
     31726 , 31743 , 31759 , 31776 , 31792 , 31809 , 31825 , 31841,\
     31858 , 31874 , 31891 , 31907 , 31924 , 31940 , 31957 , 31973,\
     31989 , 32006 , 32022 , 32038 , 32055 , 32071 , 32087 , 32104,\
     32120 , 32136 , 32153 , 32169 , 32185 , 32202 , 32218 , 32234,\
     32250 , 32267 , 32283 , 32299 , 32315 , 32332 , 32348 , 32364,\
     32380 , 32396 , 32413 , 32429 , 32445 , 32461 , 32477 , 32493,\
     32509 , 32526 , 32542 , 32558 , 32574 , 32590 , 32606 , 32622,\
     32638 , 32654 , 32670 , 32686 , 32702 , 32718 , 32734 , 32750,\
     32767 }

#define ATAN1DIV1     (int16_t)8192
#define ATAN1DIV2     (int16_t)4836
#define ATAN1DIV4     (int16_t)2555
#define ATAN1DIV8     (int16_t)1297
#define ATAN1DIV16    (int16_t)651
#define ATAN1DIV32    (int16_t)326
#define ATAN1DIV64    (int16_t)163
#define ATAN1DIV128   (int16_t)81
#define ATAN1DIV256   (int16_t)41
#define ATAN1DIV512   (int16_t)20
#define ATAN1DIV1024  (int16_t)10
#define ATAN1DIV2048  (int16_t)5
#define ATAN1DIV4096  (int16_t)3
#define ATAN1DIV8192  (int16_t)1

/**
  * @brief  It executes Modulus algorithm
  * @param  alpha component
  *         beta component
  * @retval int16_t Modulus
  */
static inline int16_t MCM_Modulus( int16_t alpha, int16_t beta )
{

  int32_t wAux1;
  int32_t wAux2;

  wAux1 = ( int32_t )( alpha  * alpha );
  wAux2 = ( int32_t )( beta * beta );

  wAux1 += wAux2;
  wAux1 = MCM_Sqrt( wAux1 );

  if ( wAux1 > INT16_MAX )
  {
    wAux1 = ( int32_t ) INT16_MAX;
  }

  return ( ( int16_t )wAux1 );

}

/**
  * @brief  It executes CORDIC algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  wBemf_alfa_est estimated Bemf alpha on the stator reference frame
  *         wBemf_beta_est estimated Bemf beta on the stator reference frame
  * @retval int16_t rotor electrical angle (s16degrees)
  */
static inline int16_t MCM_PhaseComputation(int32_t wBemf_alfa_est, int32_t wBemf_beta_est)
{

  int16_t hAngle;
  int32_t wXi, wYi, wXold;

  /*Determining quadrant*/
  if (wBemf_alfa_est < 0)
  {
    if (wBemf_beta_est < 0)
    {
      /*Quadrant III, add 90 degrees so as to move to quadrant IV*/
      hAngle = 16384;
      wXi = - ( wBemf_beta_est / 2 );
      wYi = wBemf_alfa_est / 2;
    }
    else
    {
      /*Quadrant II, subtract 90 degrees so as to move to quadrant I*/
      hAngle = -16384;
      wXi = wBemf_beta_est / 2;
      wYi = - (wBemf_alfa_est / 2);
    }
  }
  else
  {
    /* Quadrant I or IV*/
    hAngle = 0;
    wXi = wBemf_alfa_est / 2;
    wYi = wBemf_beta_est / 2;
  }
  wXold = wXi;

  /*begin the successive approximation process*/
  /*iteration0*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV1;
    wXi = wXi - wYi;
    wYi = wXold + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV1;
    wXi = wXi + wYi;
    wYi = -wXold + wYi;
  }
  wXold = wXi;

  /*iteration1*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV2;
    wXi = wXi - (wYi / 2);
    wYi = (wXold / 2) + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV2;
    wXi = wXi + (wYi / 2);
    wYi = (-wXold / 2) + wYi;
  }
  wXold = wXi;

  /*iteration2*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV4;
    wXi = wXi - (wYi / 4);
    wYi = (wXold / 4) + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV4;
    wXi = wXi + (wYi / 4);
    wYi = (-wXold / 4) + wYi;
  }
  wXold = wXi;

  /*iteration3*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV8;
    wXi = wXi - (wYi / 8);
    wYi = (wXold / 8) + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV8;
    wXi = wXi + (wYi / 8);
    wYi = (-wXold / 8) + wYi;
  }
  wXold = wXi;

  /*iteration4*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV16;
    wXi = wXi - (wYi / 16);
    wYi = (wXold / 16) + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV16;
    wXi = wXi + (wYi / 16);
    wYi = (-wXold / 16) + wYi;
  }
  wXold = wXi;

  /*iteration5*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV32;
    wXi = wXi - (wYi / 32);
    wYi = (wXold / 32) + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV32;
    wXi = wXi + (wYi / 32);
    wYi = (-wXold / 32) + wYi;
  }
  wXold = wXi;

  /*iteration6*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV64;
    wXi = wXi - (wYi / 64);
    wYi = (wXold / 64) + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV64;
    wXi = wXi + (wYi / 64);
    wYi = (-wXold / 64) + wYi;
  }
  wXold = wXi;

  /*iteration7*/
  if (wYi < 0)
  {
    /*vector is in Quadrant IV*/
    hAngle += ATAN1DIV128;
    wXi = wXi - (wYi / 128);
    wYi = (wXold / 128) + wYi;
  }
  else
  {
    /*vector is in Quadrant I*/
    hAngle -= ATAN1DIV128;
    wXi = wXi + (wYi / 128);
    wYi = (-wXold / 128) + wYi;
  }

  return (-hAngle);

}

/**
  * @brief  This function codify a floting point number into the relative
  *         32bit integer.
  * @param  float Floting point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
uint32_t MCM_floatToIntBit(float x);

/**
  * @}
  */

/**
  * @}
  */
#endif /* MC_MATH_H*/
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

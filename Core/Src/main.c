/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include "usbd_cdc_if.h"
#include "as5600.h"
#include "bldc.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim1_ch3;

/* USER CODE BEGIN PV */
extern uint16_t focArray[(int)MOTOR_RESOLUTION];
//extern uint8_t encoderCorectionsArray[512];
extern struct BLDCMotorSt bldcMotor;
extern struct BLDCencoderSt bldcEncoder;
//ADC
uint32_t adc_raw_table[5];

// USB
uint8_t DataToSend[40];
uint8_t ReceivedData[40];
uint8_t ReceivedDataFlag = 0;
uint8_t MessageLength = 0;
uint8_t TXFlag = 0;

// PID
PID_TypeDef PowerPID;
double PowerPIDSetpoint = 0.0;
//PID_TypeDef AccelerationPID;

//inne
uint16_t testi = 0;
uint16_t test = 400;
extern uint8_t initPhase;
uint8_t licznik = 0;


//int16_t encoder_array[] = {1, 3, 10, 12, 14, 16, 18, 22, 23, 27, 28, 32, 32, 34, 36, 39, 40, 43, 46, 48, 49, 56, 57, 59, 76, 83, 83, 89, 92, 94, 103, 103, 112, 113, 122, 123, 133, 134, 144, 146, 150, 154, 155, 157, 159, 162, 164, 168, 170, 173, 173, 177, 179, 181, 181, 184, 186, 189, 190, 192, 195, 196, 197, 198, 216, 220, 223, 224, 227, 229, 231, 232, 235, 237, 240, 241, 245, 247, 250, 250, 252, 255, 258, 259, 262, 263, 267, 268, 273, 275, 278, 278, 282, 282, 286, 286, 290, 291, 294, 295, 298, 299, 301, 303, 304, 305, 309, 311, 313, 315, 317, 321, 322, 330, 331, 334, 336, 349, 351, 353, 355, 359, 359, 362, 364, 368, 368, 371, 373, 376, 377, 380, 381, 385, 386, 388, 391, 395, 397, 399, 401, 404, 406, 409, 411, 415, 416, 419, 420, 424, 425, 428, 431, 433, 434, 435, 438, 443, 445, 446, 454, 455, 458, 462, 464, 465, 468, 471, 489, 491, 494, 497, 498, 500, 504, 505, 507, 508, 512, 513, 517, 518, 520, 521, 522, 524, 525, 527, 528, 528, 531, 532, 533, 534, 536, 537, 538, 540, 543, 546, 414, 138, 553, 550, 557, 559, 558, 549, 563, 561, 567, 568, 570, 571, 573, 575, 576, 578, 579, 580, 581, 585, 586, 588, 589, 590, 595, 598, 599, 601, 606, 607, 609, 610, 631, 633, 635, 637, 641, 642, 644, 646, 651, 652, 655, 656, 661, 664, 664, 664, 670, 673, 674, 675, 680, 681, 683, 684, 690, 694, 695, 696, 699, 699, 701, 702, 703, 706, 708, 709, 711, 711, 715, 715, 718, 719, 720, 722, 723, 724, 725, 727, 728, 732, 734, 734, 735, 737, 741, 742, 745, 746, 752, 758, 765, 766, 770, 771, 772, 774, 775, 780, 781, 783, 785, 789, 790, 793, 794, 798, 800, 802, 804, 808, 809, 812, 813, 818, 819, 821, 823, 825, 828, 829, 831, 832, 835, 836, 837, 840, 840, 844, 844, 846, 848, 849, 853, 854, 856, 857, 861, 862, 864, 865, 866, 870, 871, 874, 875, 878, 880, 882, 887, 902, 903, 906, 907, 911, 911, 914, 916, 919, 921, 923, 924, 927, 929, 930, 932, 933, 936, 942, 943, 946, 948, 950, 952, 955, 957, 960, 961, 963, 964, 965, 967, 968, 970, 972, 972, 974, 975, 977, 978, 981, 983, 987, 988, 992, 992, 993, 996, 996, 1000, 1000, 1002, 1003, 1005, 1008, 1009, 1010, 1012, 1014, 1016, 1017, 1017, 1035, 1037, 1041, 1042, 1043, 1046, 1046, 1049, 1050, 1050, 1051, 1053, 1054, 1058, 1058, 1058, 1061, 1062, 1063, 1064, 1065, 1067, 1069, 1070, 1072, 1073, 1074, 1075, 1077, 1078, 1080, 1081, 1082, 1082, 1085, 1085, 1093, 1089, 1092, 1091, 1095, 1097, 1096, 1098, 1101, 1102, 1099, 1105, 1106, 1108, 1109, 1109, 1110, 1113, 1115, 1117, 1119, 1120, 1121, 1123, 1124, 1125, 1128, 1128, 1131, 1132, 1133, 1133, 1136, 1137, 1140, 1141, 1143, 1144, 1145, 1148, 1148, 1150, 1151, 1154, 1155, 1163, 1170, 1171, 1172, 1174, 1176, 1177, 1178, 1179, 1181, 1182, 1184, 1185, 1187, 1189, 1190, 1194, 1194, 1195, 1198, 1199, 1199, 1200, 1202, 1203, 1206, 1206, 1207, 1208, 1209, 1210, 1212, 1214, 1214, 1216, 1218, 1219, 1222, 1222, 1223, 1224, 1225, 1226, 1227, 1227, 1228, 1230, 1231, 1232, 1233, 1234, 1235, 1236, 1238, 1239, 1241, 1242, 1243, 1244, 1245, 1248, 1248, 1249, 1250, 1251, 1252, 1254, 1256, 1257, 1258, 1259, 1262, 1264, 1265, 1266, 1267, 1268, 1269, 1271, 1272, 1273, 1273, 1274, 1275, 1276, 1279, 1280, 1280, 1282, 1284, 1286, 1288, 1289, 1296, 1304, 1307, 1308, 1310, 1311, 1314, 1315, 1317, 1320, 1320, 1321, 1323, 1325, 1326, 1327, 1330, 1333, 1337, 1338, 1340, 1342, 1343, 1346, 1347, 1350, 1351, 1353, 1356, 1358, 1363, 1369, 1367, 1373, 1375, 1375, 1378, 1382, 1382, 1384, 1386, 1387, 1392, 1394, 1396, 1397, 1401, 1402, 1403, 1406, 1407, 1410, 1411, 1412, 1414, 1415, 1418, 1420, 1421, 1422, 1424, 1425, 1429, 1430, 1438, 1449, 1450, 1451, 1453, 1454, 1456, 1456, 1458, 1459, 1464, 1464, 1465, 1468, 1469, 1472, 1474, 1475, 1476, 1478, 1480, 1481, 1482, 1483, 1484, 1487, 1488, 1490, 1491, 1491, 1494, 1495, 1496, 1498, 1500, 1502, 1503, 1506, 1507, 1510, 1511, 1512, 1513, 1515, 1516, 1519, 1521, 1522, 1525, 1526, 1528, 1530, 1531, 1534, 1535, 1537, 1539, 1540, 1542, 1542, 1546, 1547, 1547, 1549, 1551, 1552, 1554, 1555, 1557, 1559, 1560, 1562, 1563, 1564, 1565, 1568, 1568, 1577, 1582, 1584, 1585, 1586, 1588, 1590, 1592, 1593, 1594, 1595, 1598, 1599, 1600, 1601, 1603, 1604, 1605, 1607, 1608, 1609, 1611, 1613, 1614, 1616, 1616, 1617, 1620, 1621, 1623, 1624, 1625, 1627, 1628, 1629, 1632, 1634, 1635, 1637, 1638, 1641, 1642, 1643, 1643, 1647, 1648, 1651, 1651, 1653, 1654, 1655, 1657, 1659, 1660, 1663, 1664, 1668, 1669, 1670, 1671, 1672, 1673, 1676, 1677, 1678, 1679, 1680, 1682, 1683, 1686, 1687, 1689, 1690, 1693, 1694, 1694, 1696, 1698, 1700, 1709, 1720, 1722, 1724, 1725, 1727, 1728, 1729, 1730, 1733, 1734, 1734, 1735, 1736, 1737, 1739, 1740, 1741, 1742, 1744, 1744, 1746, 1748, 1750, 1751, 1753, 1754, 1757, 1759, 1760, 1761, 1763, 1764, 1767, 1768, 1768, 1772, 1772, 1775, 1777, 1778, 1780, 1781, 1784, 1785, 1786, 1787, 1790, 1791, 1794, 1795, 1796, 1798, 1799, 1799, 1802, 1803, 1804, 1805, 1807, 1807, 1811, 1812, 1814, 1816, 1819, 1821, 1823, 1826, 1827, 1829, 1831, 1833, 1834, 1836, 1841, 1854, 1857, 1858, 1860, 1861, 1863, 1864, 1865, 1866, 1867, 1869, 1869, 1870, 1871, 1873, 1874, 1875, 1877, 1878, 1879, 1880, 1880, 1882, 1883, 1885, 1886, 1887, 1888, 1890, 1891, 1893, 1894, 1896, 1897, 1898, 1901, 1901, 1903, 1905, 1906, 1909, 1911, 1912, 1913, 1916, 1437, 959, 1439, 1920, 1921, 1918, 1921, 1925, 1925, 1928, 1929, 1929, 1931, 1933, 1934, 1935, 1937, 1938, 1940, 1941, 1942, 1943, 1944, 1946, 1948, 1949, 1951, 1951, 1952, 1953, 1956, 1956, 1958, 1959, 1960, 1963, 1964, 1966, 1967, 1968, 1970, 1971, 1971, 1973, 1975, 1988, 1991, 1991, 1992, 1994, 1995, 1996, 1997, 1998, 1999, 2001, 2002, 2002, 2003, 2004, 2005, 2006, 2007, 2009, 2010, 2011, 2012, 2013, 2014, 2017, 2018, 2020, 2021, 2023, 2024, 2025, 2026, 2027, 2029, 2029, 2031, 2032, 2032, 2033, 2034, 2035, 2037, 2037, 2038, 2039, 2041, 2041, 2042, 2043, 2044, 2045, 2046, 2047, 2048, 2049, 2049, 2050, 2052, 2054, 2056, 2057, 2059, 2061, 2062, 2062, 2063, 2064, 2065, 2066, 2067, 2068, 2070, 2072, 2072, 2073, 2076, 2077, 2078, 2080, 2080, 2081, 2083, 2084, 2085, 2087, 2088, 2088, 2091, 2092, 2092, 2093, 2095, 2096, 2096, 2098, 2099, 2100, 2101, 2102, 2102, 2103, 2105, 2106, 2107, 2114, 2120, 2125, 2129, 2130, 2133, 2134, 2135, 2138, 2139, 2144, 2146, 2148, 2153, 2154, 2155, 2156, 2162, 2162, 2164, 2166, 2171, 2173, 2174, 2177, 2182, 2189, 2186, 1640, 2190, 2190, 2190, 2188, 2201, 2199, 2203, 2204, 2209, 2210, 2212, 2213, 2215, 2216, 2216, 2217, 2220, 2222, 2224, 2225, 2228, 2228, 2229, 2232, 2233, 2233, 2237, 2239, 2244, 2244, 2245, 2247, 2268, 2269, 2270, 2271, 2276, 2277, 2279, 2280, 2284, 2285, 2288, 2289, 2294, 2294, 2297, 2298, 2302, 2303, 2304, 2306, 2307, 2310, 2311, 2314, 2315, 2316, 2319, 2320, 2321, 2323, 2324, 2326, 2327, 2329, 2330, 2331, 2332, 2335, 2336, 2337, 2338, 2340, 2340, 2341, 2344, 2345, 2347, 2348, 2349, 2351, 2352, 2353, 2354, 2355, 2357, 2358, 2359, 2360, 2360, 2361, 2362, 2363, 2365, 2366, 2368, 2371, 2372, 2374, 2375, 2376, 2377, 2378, 2380, 2381, 2382, 2384, 2384, 2386, 2387, 2390, 2392, 2402, 2403, 2404, 2405, 2408, 2410, 2411, 2412, 2413, 2415, 2416, 2418, 2419, 2421, 2421, 2424, 2425, 2426, 2428, 2430, 2432, 2435, 2437, 2438, 2441, 2442, 2443, 2444, 2446, 2447, 2450, 2451, 2451, 2453, 2454, 2456, 2459, 2461, 2462, 2462, 2463, 2464, 2466, 2467, 2468, 2469, 2471, 2472, 2474, 2475, 2477, 2480, 2481, 2483, 2485, 2486, 2488, 2490, 2492, 2493, 2494, 2494, 2495, 2496, 2498, 2501, 2502, 2504, 2506, 2506, 2509, 2510, 2510, 2512, 2513, 2514, 2516, 2517, 2519, 2521, 2532, 2538, 2540, 2541, 2543, 2545, 2547, 2548, 2551, 2552, 2553, 2554, 2557, 2558, 2559, 2561, 2562, 2564, 2566, 2567, 2569, 2571, 2573, 2575, 2576, 2578, 2580, 2581, 2582, 2584, 2587, 2588, 2590, 2591, 2592, 2593, 2595, 2596, 2599, 2600, 2602, 2604, 2605, 2607, 2609, 2611, 2613, 2614, 2615, 2617, 2617, 2618, 2619, 2621, 2622, 2622, 2623, 2625, 2626, 2627, 2628, 2630, 2631, 2632, 2633, 2634, 2634, 2636, 2638, 2638, 2639, 2642, 2644, 2645, 2645, 2647, 2648, 2650, 2652, 2653, 2673, 2676, 2678, 2679, 2685, 2685, 2686, 2687, 2693, 2693, 2696, 2697, 2703, 2704, 2705, 2711, 2712, 2714, 2716, 2718, 2719, 2720, 2721, 2722, 2723, 2725, 2728, 2730, 2732, 2735, 2735, 2735, 2735, 2738, 2740, 2747, 2749, 2750, 2755, 2755, 2757, 2759, 2764, 2766, 2767, 2773, 2774, 2774, 2775, 2782, 2782, 2784, 2785, 2790, 2792, 2793, 2797, 2810, 2813, 2814, 2819, 2821, 2822, 2824, 2829, 2832, 2833, 2839, 2839, 2841, 2842, 2848, 2849, 2851, 2852, 2854, 2855, 2856, 2859, 2860, 2860, 2864, 2865, 2866, 2869, 2870, 2874, 2875, 2877, 2878, 2879, 2884, 2888, 2890, 2895, 2897, 2898, 2903, 2905, 2907, 2907, 2910, 2910, 2914, 2915, 2916, 2921, 2922, 2924, 2925, 2942, 2945, 2947, 2948, 2953, 2954, 2956, 2958, 2961, 2963, 2965, 2965, 2969, 2970, 2970, 2973, 2975, 2978, 2980, 2984, 2985, 2986, 2988, 2989, 2990, 2992, 2993, 2994, 2995, 2996, 3000, 3005, 3007, 2256, 1504, 752, 3009, 3012, 3007, 3013, 3012, 3016, 3015, 3017, 3020, 3021, 3024, 3026, 3027, 3028, 3031, 3032, 3035, 3036, 3036, 3040, 3041, 3043, 3043, 3044, 3045, 3048, 3049, 3050, 3051, 3053, 3055, 3058, 3060, 3062, 3063, 3066, 3074, 3081, 3088, 3089, 3090, 3091, 3093, 3094, 3095, 3097, 3100, 3102, 3103, 3104, 3105, 3108, 3109, 3111, 3113, 3114, 3115, 3117, 3117, 3120, 3122, 3123, 3124, 3125, 3127, 3128, 3130, 3132, 3134, 3137, 3139, 3140, 3143, 3144, 3146, 3149, 3150, 3152, 3153, 3155, 3156, 3158, 3159, 3163, 3164, 3167, 3168, 3172, 3172, 3176, 3177, 3178, 3180, 3182, 3184, 3185, 3186, 3188, 3189, 3192, 3193, 3193, 3194, 3196, 3197, 3198, 3200, 3202, 3203, 3206, 3207, 3208, 3216, 3220, 3222, 3223, 3224, 3226, 3228, 3229, 3230, 3232, 3234, 3235, 3237, 3239, 3241, 3243, 3244, 3246, 3247, 3249, 3250, 3251, 3252, 3254, 3255, 3256, 3258, 3259, 3261, 3262, 3263, 3264, 3265, 3267, 3268, 3271, 3272, 3274, 3274, 3277, 3278, 3279, 3281, 3287, 3288, 3289, 3290, 3296, 3297, 3297, 3298, 3304, 3304, 3307, 3308, 3313, 3314, 3316, 3317, 3323, 3324, 3326, 3327, 3332, 3334, 3335, 3336, 3357, 3359, 3359, 3360, 3365, 3366, 3368, 3369, 3375, 3375, 3377, 3378, 3381, 3382, 3383, 3386, 3387, 3390, 3390, 3391, 3392, 3394, 3395, 3396, 3399, 3400, 3402, 3404, 3406, 3410, 3411, 3415, 3415, 3422, 3423, 3424, 3426, 3432, 3436, 3437, 3443, 3445, 3446, 3453, 3453, 3455, 3457, 3464, 3466, 3467, 3473, 3480, 3493, 3500, 3501, 3502, 3503, 3509, 3510, 3513, 3514, 3520, 3521, 3523, 3524, 3530, 3531, 3532, 3534, 3535, 3550, 3542, 3542, 3543, 3546, 3548, 3554, 3554, 3558, 3560, 3563, 3566, 3571, 3573, 3573, 3575, 3581, 3583, 3586, 3591, 3592, 3593, 3594, 3599, 3600, 3602, 3602, 3608, 3609, 3612, 3616, 3629, 3631, 3632, 3632, 3637, 3637, 3639, 3640, 3641, 3646, 3647, 3649, 3651, 3657, 3658, 3659, 3660, 3667, 3667, 3669, 3671, 3672, 3673, 3674, 3675, 3676, 3678, 3679, 3683, 3684, 3687, 3688, 3694, 3695, 3697, 3698, 3705, 3706, 3707, 3708, 3714, 3715, 3718, 3718, 3723, 3724, 3726, 3728, 3732, 3733, 3734, 3736, 3737, 3740, 3741, 3742, 3744, 3745, 3763, 3766, 3768, 3769, 3769, 3773, 3773, 3773, 3776, 3777, 3777, 3780, 3781, 3783, 3784, 3785, 3786, 3789, 3790, 3792, 3793, 3794, 3797, 3799, 3801, 3803, 3805, 3807, 3808, 3811, 3818, 3814, 3817, 3824, 3822, 3827, 2871, 2392, 3828, 3831, 3830, 3832, 3833, 3834, 3836, 3837, 3838, 3841, 3842, 3846, 3847, 3847, 3849, 3850, 3852, 3855, 3856, 3857, 3859, 3859, 3862, 3863, 3866, 3867, 3869, 3870, 3871, 3874, 3874, 3874, 3875, 3878, 3879, 3881, 3881, 3882, 3884, 3885, 3886, 3894, 3904, 3907, 3908, 3909, 3910, 3911, 3912, 3913, 3914, 3915, 3916, 3918, 3919, 3921, 3923, 3924, 3925, 3927, 3928, 3930, 3931, 3932, 3934, 3935, 3936, 3938, 3940, 3940, 3941, 3944, 3944, 3946, 3948, 3948, 3950, 3952, 3952, 3955, 3956, 3956, 3957, 3959, 3960, 3967, 3967, 3969, 3971, 3976, 3978, 3979, 3984, 3985, 3987, 3988, 3993, 3994, 3995, 3996, 4003, 4003, 4004, 4005, 4010, 4011, 4013, 4014, 4019, 4020, 4021, 4022, 4028, 4034, 4039, 4041, 4046, 4046, 4047, 4048, 4050, 4054, 4055, 4057, 4057, 4062, 4065, 4065, 4067, 4068, 4069, 4070, 4072, 4072, 4073, 4075, 4077, 4079, 4080, 4081, 4081, 4086, 4087, 4089, 4090, 4092, 4094, 4095, 4095, 0, 0};
uint16_t test_array[4095];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if(initPhase == 0){
			bldcSetNewPosition(15, (uint16_t)bldcMotor.actualAcceleration);
		}
	}

	if (htim->Instance == TIM3) {
		bldcEncoder.status = as5600_ReadPosition(&hi2c2, &bldcEncoder.angle);
		if(initPhase == 0){
			PID_Compute(&PowerPID);
			bldcSyncWithEncoder(2);
			bldcCalc(&htim2);
		}
	}

	if (htim->Instance == TIM4) {
		MessageLength = sprintf((char*) DataToSend, "spd: %i\n\r",1);// (uint16_t)bldcMotor.speed);
		CDC_Transmit_FS(DataToSend, MessageLength);
		HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);
		testi++;
		if(testi > 10){
			testi = 0;
			bldcMotor.expectedPosition += 500;
			if(bldcMotor.expectedPosition > 4095){
				bldcMotor.expectedPosition = bldcMotor.expectedPosition - 4095;
			}
		}
	}
}


void encoderTest(uint8_t power) {
	int diff;
	int maxDiff = 0;

	for (bldcMotor.fieldPosition = 0; bldcMotor.fieldPosition < 4095; bldcMotor.fieldPosition++) {
			setMotorPosition(bldcMotor.fieldPosition, power);
			HAL_Delay(10);
			diff = bldcMotor.fieldPosition - bldcEncoder.calculatedAngle;

			if(abs(diff) > 2000){
				diff = 4095 + (bldcMotor.fieldPosition - bldcEncoder.calculatedAngle);
			}
			if(diff > 255){diff = 255;}
			if(diff < -255){diff = 255;}

			test_array[bldcEncoder.calculatedAngle] = bldcMotor.fieldPosition;

			if(abs(diff) > maxDiff && abs(diff) < 2000){maxDiff = abs(diff);}
			//encoder_array[bldcMotor.fieldPosition] = diff;
		}
	for(uint16_t i=0; i<4095; i++){
		if(i < 4090){
			if(test_array[i] == 0 && test_array[i+1] == 0 && test_array[i+2] == 0){
				test_array[i+1] = ((test_array[i+3] - test_array[i-1]) / 2) + test_array[i-1];
				test_array[i] = ((test_array[i+1] - test_array[i-1]) / 2) + test_array[i-1];
				test_array[i+2] = ((test_array[i+3] - test_array[i+1]) / 2) + test_array[i+1];
			} else if(test_array[i] == 0 && test_array[i+1] == 0){
				test_array[i] = ((test_array[i+2] - test_array[i-1]) / 3) + test_array[i-1];
				test_array[i+1] = (((test_array[i+2] - test_array[i-1]) / 3)*2) + test_array[i-1];
			} else if(test_array[i] == 0){
				test_array[i] = ((test_array[i+1] - test_array[i-1]) / 2) + test_array[i-1];
			}
		}
		MessageLength = sprintf((char*) DataToSend, "%i, ", test_array[i]);
		CDC_Transmit_FS(DataToSend, MessageLength);
	}
	MessageLength = sprintf((char*) DataToSend, "\n\r->  Maximum diff: %i \n\r", maxDiff);
	CDC_Transmit_FS(DataToSend, MessageLength);
	HAL_Delay(100);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //bldcEncoder.angle = 0;
  //bldcEncoder.calculatedAngle = 0;
  bldcMotor.pwmU = 0;
  bldcMotor.pwmV = 0;
  bldcMotor.pwmW = 0;
  bldcMotor.expectedPosition = 1500;
  bldcMotor.fieldPosition = 0;
  bldcMotor.fieldLastPosition = 0;
  bldcMotor.direction = 0;
  bldcMotor.distance = 0;
  //bldcMotor.actualPower = 0.0;
  bldcMotor.expectedPower = 30.0;
  //bldcMotor.expectedAcceleration = 900.0;
  bldcMotor.actualAcceleration = 5.0;
  bldcMotor.actualSpeedARRReg = 1000;
  bldcMotor.expectedSpeed = 15.0;
  bldcMotor.actualSpeed = 0.0;
  //bldcMotor.accelerate = 30.0;
  //bldcMotor.expectedDeadZone = 20; //12;

  __HAL_TIM_SET_AUTORELOAD(&htim2, bldcMotor.actualSpeedARRReg);

  HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, 1);
  HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, 0);

  createFocArray(&focArray[0]);

  if (as5600_Init(&hi2c2)) {
	  HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, 1);
	  //error
  }
  HAL_Delay(500);

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  if(bldcInit(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3)){
	  HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, 1);
	  //error
  }
  HAL_ADC_Start_DMA(&hadc1, adc_raw_table, 5);

  /*testFlag = 0;
  HAL_TIM_Base_Stop_IT(&htim2);
  encoderTest(90);
  HAL_TIM_Base_Start_IT(&htim2);
*/
  HAL_TIM_Base_Start_IT(&htim4);

  //  PID for power
      	PowerPIDSetpoint = 0.0;
    	PID(&PowerPID, &bldcMotor.distance, &bldcMotor.actualPower, &PowerPIDSetpoint, 0.8, 0.01, 0.01, _PID_P_ON_E, _PID_CD_REVERSE);
    	PID_SetMode(&PowerPID, _PID_MODE_AUTOMATIC);
    	PID_SetSampleTime(&PowerPID, 1); //1ms refresh time
    	PID_SetOutputLimits(&PowerPID, 11, bldcMotor.expectedPower);

  /*
   *
   *
   *   !!!!!!!!!!!!!! ZAMIENIC INICJALIZACJE DMA PRZED TIMERAMI !!!!!!!!!!!!!!!!!!!!!!
   *
   *
   * */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1024;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 100;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 70 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 33333 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LEDR_Pin|LEDG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDR_Pin LEDG_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin|LEDG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

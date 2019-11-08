//! Algorithms that deal with low-discrepancy point sets.

use hexf::*;

// pbrt
use crate::core::geometry::{Point2f, Point2i};
use crate::core::pbrt::Float;
use crate::core::rng::Rng;
use crate::core::rng::ONE_MINUS_EPSILON;
use crate::core::sampling::shuffle;
use crate::core::sobolmatrices::{
    NUM_SOBOL_DIMENSIONS, SOBOL_MATRICES_32, SOBOL_MATRIX_SIZE, VD_C_SOBOL_MATRICES,
    VD_C_SOBOL_MATRICES_INV,
};

// see lowdiscrepancy.h

pub const PRIME_TABLE_SIZE: u16 = 1000_u16;

pub const PRIMES: [u32; PRIME_TABLE_SIZE as usize] = [
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97,
    101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193,
    197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281, 283, 293, 307,
    311, 313, 317, 331, 337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409, 419, 421,
    431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509, 521, 523, 541, 547,
    557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619, 631, 641, 643, 647, 653, 659,
    661, 673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743, 751, 757, 761, 769, 773, 787, 797,
    809, 811, 821, 823, 827, 829, 839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929,
    937, 941, 947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013, 1019, 1021, 1031, 1033, 1039,
    1049, 1051, 1061, 1063, 1069, 1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153,
    1163, 1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231, 1237, 1249, 1259, 1277, 1279,
    1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321, 1327, 1361, 1367, 1373, 1381, 1399, 1409,
    1423, 1427, 1429, 1433, 1439, 1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499,
    1511, 1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601, 1607, 1609, 1613,
    1619, 1621, 1627, 1637, 1657, 1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733, 1741,
    1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 1823, 1831, 1847, 1861, 1867, 1871, 1873,
    1877, 1879, 1889, 1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997, 1999,
    2003, 2011, 2017, 2027, 2029, 2039, 2053, 2063, 2069, 2081, 2083, 2087, 2089, 2099, 2111, 2113,
    2129, 2131, 2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213, 2221, 2237, 2239, 2243, 2251,
    2267, 2269, 2273, 2281, 2287, 2293, 2297, 2309, 2311, 2333, 2339, 2341, 2347, 2351, 2357, 2371,
    2377, 2381, 2383, 2389, 2393, 2399, 2411, 2417, 2423, 2437, 2441, 2447, 2459, 2467, 2473, 2477,
    2503, 2521, 2531, 2539, 2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617, 2621, 2633, 2647,
    2657, 2659, 2663, 2671, 2677, 2683, 2687, 2689, 2693, 2699, 2707, 2711, 2713, 2719, 2729, 2731,
    2741, 2749, 2753, 2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819, 2833, 2837, 2843, 2851, 2857,
    2861, 2879, 2887, 2897, 2903, 2909, 2917, 2927, 2939, 2953, 2957, 2963, 2969, 2971, 2999, 3001,
    3011, 3019, 3023, 3037, 3041, 3049, 3061, 3067, 3079, 3083, 3089, 3109, 3119, 3121, 3137, 3163,
    3167, 3169, 3181, 3187, 3191, 3203, 3209, 3217, 3221, 3229, 3251, 3253, 3257, 3259, 3271, 3299,
    3301, 3307, 3313, 3319, 3323, 3329, 3331, 3343, 3347, 3359, 3361, 3371, 3373, 3389, 3391, 3407,
    3413, 3433, 3449, 3457, 3461, 3463, 3467, 3469, 3491, 3499, 3511, 3517, 3527, 3529, 3533, 3539,
    3541, 3547, 3557, 3559, 3571, 3581, 3583, 3593, 3607, 3613, 3617, 3623, 3631, 3637, 3643, 3659,
    3671, 3673, 3677, 3691, 3697, 3701, 3709, 3719, 3727, 3733, 3739, 3761, 3767, 3769, 3779, 3793,
    3797, 3803, 3821, 3823, 3833, 3847, 3851, 3853, 3863, 3877, 3881, 3889, 3907, 3911, 3917, 3919,
    3923, 3929, 3931, 3943, 3947, 3967, 3989, 4001, 4003, 4007, 4013, 4019, 4021, 4027, 4049, 4051,
    4057, 4073, 4079, 4091, 4093, 4099, 4111, 4127, 4129, 4133, 4139, 4153, 4157, 4159, 4177, 4201,
    4211, 4217, 4219, 4229, 4231, 4241, 4243, 4253, 4259, 4261, 4271, 4273, 4283, 4289, 4297, 4327,
    4337, 4339, 4349, 4357, 4363, 4373, 4391, 4397, 4409, 4421, 4423, 4441, 4447, 4451, 4457, 4463,
    4481, 4483, 4493, 4507, 4513, 4517, 4519, 4523, 4547, 4549, 4561, 4567, 4583, 4591, 4597, 4603,
    4621, 4637, 4639, 4643, 4649, 4651, 4657, 4663, 4673, 4679, 4691, 4703, 4721, 4723, 4729, 4733,
    4751, 4759, 4783, 4787, 4789, 4793, 4799, 4801, 4813, 4817, 4831, 4861, 4871, 4877, 4889, 4903,
    4909, 4919, 4931, 4933, 4937, 4943, 4951, 4957, 4967, 4969, 4973, 4987, 4993, 4999, 5003, 5009,
    5011, 5021, 5023, 5039, 5051, 5059, 5077, 5081, 5087, 5099, 5101, 5107, 5113, 5119, 5147, 5153,
    5167, 5171, 5179, 5189, 5197, 5209, 5227, 5231, 5233, 5237, 5261, 5273, 5279, 5281, 5297, 5303,
    5309, 5323, 5333, 5347, 5351, 5381, 5387, 5393, 5399, 5407, 5413, 5417, 5419, 5431, 5437, 5441,
    5443, 5449, 5471, 5477, 5479, 5483, 5501, 5503, 5507, 5519, 5521, 5527, 5531, 5557, 5563, 5569,
    5573, 5581, 5591, 5623, 5639, 5641, 5647, 5651, 5653, 5657, 5659, 5669, 5683, 5689, 5693, 5701,
    5711, 5717, 5737, 5741, 5743, 5749, 5779, 5783, 5791, 5801, 5807, 5813, 5821, 5827, 5839, 5843,
    5849, 5851, 5857, 5861, 5867, 5869, 5879, 5881, 5897, 5903, 5923, 5927, 5939, 5953, 5981, 5987,
    6007, 6011, 6029, 6037, 6043, 6047, 6053, 6067, 6073, 6079, 6089, 6091, 6101, 6113, 6121, 6131,
    6133, 6143, 6151, 6163, 6173, 6197, 6199, 6203, 6211, 6217, 6221, 6229, 6247, 6257, 6263, 6269,
    6271, 6277, 6287, 6299, 6301, 6311, 6317, 6323, 6329, 6337, 6343, 6353, 6359, 6361, 6367, 6373,
    6379, 6389, 6397, 6421, 6427, 6449, 6451, 6469, 6473, 6481, 6491, 6521, 6529, 6547, 6551, 6553,
    6563, 6569, 6571, 6577, 6581, 6599, 6607, 6619, 6637, 6653, 6659, 6661, 6673, 6679, 6689, 6691,
    6701, 6703, 6709, 6719, 6733, 6737, 6761, 6763, 6779, 6781, 6791, 6793, 6803, 6823, 6827, 6829,
    6833, 6841, 6857, 6863, 6869, 6871, 6883, 6899, 6907, 6911, 6917, 6947, 6949, 6959, 6961, 6967,
    6971, 6977, 6983, 6991, 6997, 7001, 7013, 7019, 7027, 7039, 7043, 7057, 7069, 7079, 7103, 7109,
    7121, 7127, 7129, 7151, 7159, 7177, 7187, 7193, 7207, 7211, 7213, 7219, 7229, 7237, 7243, 7247,
    7253, 7283, 7297, 7307, 7309, 7321, 7331, 7333, 7349, 7351, 7369, 7393, 7411, 7417, 7433, 7451,
    7457, 7459, 7477, 7481, 7487, 7489, 7499, 7507, 7517, 7523, 7529, 7537, 7541, 7547, 7549, 7559,
    7561, 7573, 7577, 7583, 7589, 7591, 7603, 7607, 7621, 7639, 7643, 7649, 7669, 7673, 7681, 7687,
    7691, 7699, 7703, 7717, 7723, 7727, 7741, 7753, 7757, 7759, 7789, 7793, 7817, 7823, 7829, 7841,
    7853, 7867, 7873, 7877, 7879, 7883, 7901, 7907, 7919,
];

pub const PRIME_SUMS: [u32; PRIME_TABLE_SIZE as usize] = [
    0, 2, 5, 10, 17, 28, 41, 58, 77, 100, 129, 160, 197, 238, 281, 328, 381, 440, 501, 568, 639,
    712, 791, 874, 963, 1060, 1161, 1264, 1371, 1480, 1593, 1720, 1851, 1988, 2127, 2276, 2427,
    2584, 2747, 2914, 3087, 3266, 3447, 3638, 3831, 4028, 4227, 4438, 4661, 4888, 5117, 5350, 5589,
    5830, 6081, 6338, 6601, 6870, 7141, 7418, 7699, 7982, 8275, 8582, 8893, 9206, 9523, 9854,
    10191, 10538, 10887, 11240, 11599, 11966, 12339, 12718, 13101, 13490, 13887, 14288, 14697,
    15116, 15537, 15968, 16401, 16840, 17283, 17732, 18189, 18650, 19113, 19580, 20059, 20546,
    21037, 21536, 22039, 22548, 23069, 23592, 24133, 24680, 25237, 25800, 26369, 26940, 27517,
    28104, 28697, 29296, 29897, 30504, 31117, 31734, 32353, 32984, 33625, 34268, 34915, 35568,
    36227, 36888, 37561, 38238, 38921, 39612, 40313, 41022, 41741, 42468, 43201, 43940, 44683,
    45434, 46191, 46952, 47721, 48494, 49281, 50078, 50887, 51698, 52519, 53342, 54169, 54998,
    55837, 56690, 57547, 58406, 59269, 60146, 61027, 61910, 62797, 63704, 64615, 65534, 66463,
    67400, 68341, 69288, 70241, 71208, 72179, 73156, 74139, 75130, 76127, 77136, 78149, 79168,
    80189, 81220, 82253, 83292, 84341, 85392, 86453, 87516, 88585, 89672, 90763, 91856, 92953,
    94056, 95165, 96282, 97405, 98534, 99685, 100838, 102001, 103172, 104353, 105540, 106733,
    107934, 109147, 110364, 111587, 112816, 114047, 115284, 116533, 117792, 119069, 120348, 121631,
    122920, 124211, 125508, 126809, 128112, 129419, 130738, 132059, 133386, 134747, 136114, 137487,
    138868, 140267, 141676, 143099, 144526, 145955, 147388, 148827, 150274, 151725, 153178, 154637,
    156108, 157589, 159072, 160559, 162048, 163541, 165040, 166551, 168074, 169605, 171148, 172697,
    174250, 175809, 177376, 178947, 180526, 182109, 183706, 185307, 186914, 188523, 190136, 191755,
    193376, 195003, 196640, 198297, 199960, 201627, 203296, 204989, 206686, 208385, 210094, 211815,
    213538, 215271, 217012, 218759, 220512, 222271, 224048, 225831, 227618, 229407, 231208, 233019,
    234842, 236673, 238520, 240381, 242248, 244119, 245992, 247869, 249748, 251637, 253538, 255445,
    257358, 259289, 261222, 263171, 265122, 267095, 269074, 271061, 273054, 275051, 277050, 279053,
    281064, 283081, 285108, 287137, 289176, 291229, 293292, 295361, 297442, 299525, 301612, 303701,
    305800, 307911, 310024, 312153, 314284, 316421, 318562, 320705, 322858, 325019, 327198, 329401,
    331608, 333821, 336042, 338279, 340518, 342761, 345012, 347279, 349548, 351821, 354102, 356389,
    358682, 360979, 363288, 365599, 367932, 370271, 372612, 374959, 377310, 379667, 382038, 384415,
    386796, 389179, 391568, 393961, 396360, 398771, 401188, 403611, 406048, 408489, 410936, 413395,
    415862, 418335, 420812, 423315, 425836, 428367, 430906, 433449, 435998, 438549, 441106, 443685,
    446276, 448869, 451478, 454095, 456716, 459349, 461996, 464653, 467312, 469975, 472646, 475323,
    478006, 480693, 483382, 486075, 488774, 491481, 494192, 496905, 499624, 502353, 505084, 507825,
    510574, 513327, 516094, 518871, 521660, 524451, 527248, 530049, 532852, 535671, 538504, 541341,
    544184, 547035, 549892, 552753, 555632, 558519, 561416, 564319, 567228, 570145, 573072, 576011,
    578964, 581921, 584884, 587853, 590824, 593823, 596824, 599835, 602854, 605877, 608914, 611955,
    615004, 618065, 621132, 624211, 627294, 630383, 633492, 636611, 639732, 642869, 646032, 649199,
    652368, 655549, 658736, 661927, 665130, 668339, 671556, 674777, 678006, 681257, 684510, 687767,
    691026, 694297, 697596, 700897, 704204, 707517, 710836, 714159, 717488, 720819, 724162, 727509,
    730868, 734229, 737600, 740973, 744362, 747753, 751160, 754573, 758006, 761455, 764912, 768373,
    771836, 775303, 778772, 782263, 785762, 789273, 792790, 796317, 799846, 803379, 806918, 810459,
    814006, 817563, 821122, 824693, 828274, 831857, 835450, 839057, 842670, 846287, 849910, 853541,
    857178, 860821, 864480, 868151, 871824, 875501, 879192, 882889, 886590, 890299, 894018, 897745,
    901478, 905217, 908978, 912745, 916514, 920293, 924086, 927883, 931686, 935507, 939330, 943163,
    947010, 950861, 954714, 958577, 962454, 966335, 970224, 974131, 978042, 981959, 985878, 989801,
    993730, 997661, 1001604, 1005551, 1009518, 1013507, 1017508, 1021511, 1025518, 1029531,
    1033550, 1037571, 1041598, 1045647, 1049698, 1053755, 1057828, 1061907, 1065998, 1070091,
    1074190, 1078301, 1082428, 1086557, 1090690, 1094829, 1098982, 1103139, 1107298, 1111475,
    1115676, 1119887, 1124104, 1128323, 1132552, 1136783, 1141024, 1145267, 1149520, 1153779,
    1158040, 1162311, 1166584, 1170867, 1175156, 1179453, 1183780, 1188117, 1192456, 1196805,
    1201162, 1205525, 1209898, 1214289, 1218686, 1223095, 1227516, 1231939, 1236380, 1240827,
    1245278, 1249735, 1254198, 1258679, 1263162, 1267655, 1272162, 1276675, 1281192, 1285711,
    1290234, 1294781, 1299330, 1303891, 1308458, 1313041, 1317632, 1322229, 1326832, 1331453,
    1336090, 1340729, 1345372, 1350021, 1354672, 1359329, 1363992, 1368665, 1373344, 1378035,
    1382738, 1387459, 1392182, 1396911, 1401644, 1406395, 1411154, 1415937, 1420724, 1425513,
    1430306, 1435105, 1439906, 1444719, 1449536, 1454367, 1459228, 1464099, 1468976, 1473865,
    1478768, 1483677, 1488596, 1493527, 1498460, 1503397, 1508340, 1513291, 1518248, 1523215,
    1528184, 1533157, 1538144, 1543137, 1548136, 1553139, 1558148, 1563159, 1568180, 1573203,
    1578242, 1583293, 1588352, 1593429, 1598510, 1603597, 1608696, 1613797, 1618904, 1624017,
    1629136, 1634283, 1639436, 1644603, 1649774, 1654953, 1660142, 1665339, 1670548, 1675775,
    1681006, 1686239, 1691476, 1696737, 1702010, 1707289, 1712570, 1717867, 1723170, 1728479,
    1733802, 1739135, 1744482, 1749833, 1755214, 1760601, 1765994, 1771393, 1776800, 1782213,
    1787630, 1793049, 1798480, 1803917, 1809358, 1814801, 1820250, 1825721, 1831198, 1836677,
    1842160, 1847661, 1853164, 1858671, 1864190, 1869711, 1875238, 1880769, 1886326, 1891889,
    1897458, 1903031, 1908612, 1914203, 1919826, 1925465, 1931106, 1936753, 1942404, 1948057,
    1953714, 1959373, 1965042, 1970725, 1976414, 1982107, 1987808, 1993519, 1999236, 2004973,
    2010714, 2016457, 2022206, 2027985, 2033768, 2039559, 2045360, 2051167, 2056980, 2062801,
    2068628, 2074467, 2080310, 2086159, 2092010, 2097867, 2103728, 2109595, 2115464, 2121343,
    2127224, 2133121, 2139024, 2144947, 2150874, 2156813, 2162766, 2168747, 2174734, 2180741,
    2186752, 2192781, 2198818, 2204861, 2210908, 2216961, 2223028, 2229101, 2235180, 2241269,
    2247360, 2253461, 2259574, 2265695, 2271826, 2277959, 2284102, 2290253, 2296416, 2302589,
    2308786, 2314985, 2321188, 2327399, 2333616, 2339837, 2346066, 2352313, 2358570, 2364833,
    2371102, 2377373, 2383650, 2389937, 2396236, 2402537, 2408848, 2415165, 2421488, 2427817,
    2434154, 2440497, 2446850, 2453209, 2459570, 2465937, 2472310, 2478689, 2485078, 2491475,
    2497896, 2504323, 2510772, 2517223, 2523692, 2530165, 2536646, 2543137, 2549658, 2556187,
    2562734, 2569285, 2575838, 2582401, 2588970, 2595541, 2602118, 2608699, 2615298, 2621905,
    2628524, 2635161, 2641814, 2648473, 2655134, 2661807, 2668486, 2675175, 2681866, 2688567,
    2695270, 2701979, 2708698, 2715431, 2722168, 2728929, 2735692, 2742471, 2749252, 2756043,
    2762836, 2769639, 2776462, 2783289, 2790118, 2796951, 2803792, 2810649, 2817512, 2824381,
    2831252, 2838135, 2845034, 2851941, 2858852, 2865769, 2872716, 2879665, 2886624, 2893585,
    2900552, 2907523, 2914500, 2921483, 2928474, 2935471, 2942472, 2949485, 2956504, 2963531,
    2970570, 2977613, 2984670, 2991739, 2998818, 3005921, 3013030, 3020151, 3027278, 3034407,
    3041558, 3048717, 3055894, 3063081, 3070274, 3077481, 3084692, 3091905, 3099124, 3106353,
    3113590, 3120833, 3128080, 3135333, 3142616, 3149913, 3157220, 3164529, 3171850, 3179181,
    3186514, 3193863, 3201214, 3208583, 3215976, 3223387, 3230804, 3238237, 3245688, 3253145,
    3260604, 3268081, 3275562, 3283049, 3290538, 3298037, 3305544, 3313061, 3320584, 3328113,
    3335650, 3343191, 3350738, 3358287, 3365846, 3373407, 3380980, 3388557, 3396140, 3403729,
    3411320, 3418923, 3426530, 3434151, 3441790, 3449433, 3457082, 3464751, 3472424, 3480105,
    3487792, 3495483, 3503182, 3510885, 3518602, 3526325, 3534052, 3541793, 3549546, 3557303,
    3565062, 3572851, 3580644, 3588461, 3596284, 3604113, 3611954, 3619807, 3627674, 3635547,
    3643424, 3651303, 3659186, 3667087, 3674994,
];

/// The bits of an integer quantity can be efficiently reversed with a
/// series of logical bit operations.
pub fn reverse_bits_32(n: u32) -> u32 {
    let mut n = (n << 16) | (n >> 16);
    n = ((n & 0x00ff00ff) << 8) | ((n & 0xff00ff00) >> 8);
    n = ((n & 0x0f0f0f0f) << 4) | ((n & 0xf0f0f0f0) >> 4);
    n = ((n & 0x33333333) << 2) | ((n & 0xcccccccc) >> 2);
    n = ((n & 0x55555555) << 1) | ((n & 0xaaaaaaaa) >> 1);
    n
}

/// The bits of a 64-bit value can be reversed by reversing the two
/// 32-bit components individually and then interchanging them.
pub fn reverse_bits_64(n: u64) -> u64 {
    let n0: u64 = reverse_bits_32(n as u32) as u64;
    let n1: u64 = reverse_bits_32((n >> 32) as u32) as u64;
    (n0 << 32) | n1
}

/// Compute the inverse of the radical inverse function.
pub fn inverse_radical_inverse(base: u8, inverse: u64, n_digits: u64) -> u64 {
    let mut inverse: u64 = inverse;
    let mut index: u64 = 0;
    for _i in 0..n_digits {
        let digit: u64 = inverse % base as u64;
        inverse /= base as u64;
        index = index * base as u64 + digit;
    }
    index
}

/// Takes a generator matrix *c*, a number of 1D samples to generate
/// *n*, and stores the corresponding samples in memory at the
/// location pointed to by *p*.
pub fn gray_code_sample_1d(c: [u32; 32], n: u32, scramble: u32, p: &mut [Float]) {
    let mut v: u32 = scramble;
    for i in 0..n as usize {
        // 1/2^32
        //#ifndef PBRT_HAVE_HEX_FP_CONSTANTS
        // p[i] = (v as Float * 2.3283064365386963e-10 as Float).min(ONE_MINUS_EPSILON);
        //#else
        p[i] = (v as Float * hexf32!("0x1.0p-32") as Float).min(ONE_MINUS_EPSILON);
        //#endif
        v ^= c[(i + 1).trailing_zeros() as usize];
    }
}

/// Takes two generator matrices *c0* and *c1*, a number of 2D samples
/// to generate *n*, and stores the corresponding samples in memory at
/// the location pointed to by *p*.
pub fn gray_code_sample_2d(c0: &[u32], c1: &[u32], n: u32, scramble: &Point2i, p: &mut [Point2f]) {
    let mut v: [u32; 2] = [scramble.x as u32, scramble.y as u32];
    for i in 0..n as usize {
        // #ifndef PBRT_HAVE_HEX_FP_CONSTANTS
        // p[i].x = (v[0] as Float * 2.3283064365386963e-10 as Float).min(ONE_MINUS_EPSILON);
        // p[i].y = (v[1] as Float * 2.3283064365386963e-10 as Float).min(ONE_MINUS_EPSILON);
        // #else
        p[i].x = (v[0] as Float * hexf32!("0x1.0p-32") as Float).min(ONE_MINUS_EPSILON);
        p[i].y = (v[1] as Float * hexf32!("0x1.0p-32") as Float).min(ONE_MINUS_EPSILON);
        // #endif
        v[0] ^= c0[(i + 1).trailing_zeros() as usize];
        v[1] ^= c1[(i + 1).trailing_zeros() as usize];
    }
}

/// Generates a number of scrambled 1D sample values using the Gray
/// code-based sampling machinery.
pub fn van_der_corput(
    n_samples_per_pixel_sample: i32,
    n_pixel_samples: i32,
    samples: &mut [Float],
    rng: &mut Rng,
) {
    let scramble: u32 = rng.uniform_uint32();
    let c_van_der_corput: [u32; 32] = [
        0x80000000, 0x40000000, 0x20000000, 0x10000000, 0x8000000, 0x4000000, 0x2000000, 0x1000000,
        0x800000, 0x400000, 0x200000, 0x100000, 0x80000, 0x40000, 0x20000, 0x10000, 0x8000, 0x4000,
        0x2000, 0x1000, 0x800, 0x400, 0x200, 0x100, 0x80, 0x40, 0x20, 0x10, 0x8, 0x4, 0x2, 0x1,
    ];
    let total_samples: i32 = n_samples_per_pixel_sample * n_pixel_samples;
    gray_code_sample_1d(c_van_der_corput, total_samples as u32, scramble, samples);
    // randomly shuffle 1D sample points
    for i in 0..n_pixel_samples as usize {
        shuffle(
            &mut samples[(i * n_samples_per_pixel_sample as usize)..],
            n_samples_per_pixel_sample as u32,
            1,
            rng,
        );
    }
    shuffle(
        &mut samples[..],
        n_pixel_samples as u32,
        n_samples_per_pixel_sample as u32,
        rng,
    );
}

/// Similar to *van_der_corput()*, but uses two generator matrices to
/// generate the first two dimensions of Sobol' points.
pub fn sobol_2d(
    n_samples_per_pixel_sample: i32,
    n_pixel_samples: i32,
    samples: &mut [Point2f],
    rng: &mut Rng,
) {
    let x: i32 = rng.uniform_uint32() as i32;
    let y: i32 = rng.uniform_uint32() as i32;
    let scramble: Point2i = Point2i { x, y };
    // define 2D Sobol$'$ generator matrices _c_sobol[2]_
    let c_sobol: [[u32; 32]; 2] = [
        [
            0x80000000_u32,
            0x40000000,
            0x20000000,
            0x10000000,
            0x8000000,
            0x4000000,
            0x2000000,
            0x1000000,
            0x800000,
            0x400000,
            0x200000,
            0x100000,
            0x80000,
            0x40000,
            0x20000,
            0x10000,
            0x8000,
            0x4000,
            0x2000,
            0x1000,
            0x800,
            0x400,
            0x200,
            0x100,
            0x80,
            0x40,
            0x20,
            0x10,
            0x8,
            0x4,
            0x2,
            0x1,
        ],
        [
            0x80000000, 0xc0000000, 0xa0000000, 0xf0000000, 0x88000000, 0xcc000000, 0xaa000000,
            0xff000000, 0x80800000, 0xc0c00000, 0xa0a00000, 0xf0f00000, 0x88880000, 0xcccc0000,
            0xaaaa0000, 0xffff0000, 0x80008000, 0xc000c000, 0xa000a000, 0xf000f000, 0x88008800,
            0xcc00cc00, 0xaa00aa00, 0xff00ff00, 0x80808080, 0xc0c0c0c0, 0xa0a0a0a0, 0xf0f0f0f0,
            0x88888888, 0xcccccccc, 0xaaaaaaaa, 0xffffffff,
        ],
    ];
    gray_code_sample_2d(
        &c_sobol[0],
        &c_sobol[1],
        (n_samples_per_pixel_sample * n_pixel_samples) as u32,
        &scramble,
        &mut samples[..],
    );
    for i in 0..n_pixel_samples as usize {
        shuffle(
            &mut samples[(i * n_samples_per_pixel_sample as usize)..],
            n_samples_per_pixel_sample as u32,
            1,
            rng,
        );
    }
    shuffle(
        &mut samples[..],
        n_pixel_samples as u32,
        n_samples_per_pixel_sample as u32,
        rng,
    );
}

/// Returns the index of the _frame_th sample in the pixel p, if the
/// sampling domain has be scaled to cover the pixel sampling area.
pub fn sobol_interval_to_index(m: u32, frame: u64, p: &Point2i) -> u64 {
    if m == 0_u32 {
        return 0_u64;
    }
    let m2: u32 = m << 1;
    let mut index: u64 = frame << m2;
    let mut delta: u64 = 0;
    let mut c: i32 = 0;
    let mut frame: u64 = frame;
    while frame > 0_u64 {
        if frame & 1 > 0_u64 {
            // add flipped column m + c + 1.
            delta ^= VD_C_SOBOL_MATRICES[(m - 1) as usize][c as usize];
        }
        frame = frame >> 1;
        c += 1_i32;
    }
    // flipped b
    let mut b: u64 = (((p.x as u32) << m) as u64 | (p.y as u64)) ^ delta;
    c = 0;
    while b > 0_u64 {
        if b & 1 > 0_u64 {
            // add column 2 * m - c.
            index ^= VD_C_SOBOL_MATRICES_INV[(m - 1) as usize][c as usize];
        }
        b = b >> 1;
        c += 1_i32;
    }
    return index;
}

/// Takes different paths for 32- and 64-bit floating point values.
pub fn sobol_sample(index: i64, dimension: i32, scramble: u64) -> Float {
    // #ifdef PBRT_FLOAT_AS_DOUBLE
    //     return SobolSampleDouble(index, dimension, scramble);
    sobol_sample_float(index, dimension, scramble as u32)
}

/// Takes a 64 bit index and 32x52 matrices to calculate sample values.
pub fn sobol_sample_float(a: i64, dimension: i32, scramble: u32) -> Float {
    assert!(
        dimension < NUM_SOBOL_DIMENSIONS as i32,
        "Integrator has consumed too many Sobol' dimensions; \
         you may want to use a Sampler without a dimension limit like \"02sequence.\""
    );
    let mut a: i64 = a;
    let mut v: u32 = scramble;
    // for (int i = dimension * SobolMatrixSize; a != 0; a >>= 1, i++)
    let mut i: usize = dimension as usize * SOBOL_MATRIX_SIZE as usize;
    while a != 0 {
        if a & 1 > 0 {
            v ^= SOBOL_MATRICES_32[i];
        }
        a = a >> 1;
        i += 1_usize;
    }
    //#ifndef PBRT_HAVE_HEX_FP_CONSTANTS
    // let x = (2.0 as f32).powi(-32 as i32); // 0x1p-32f: 1/2^32
    // (v as Float * x).min(ONE_MINUS_EPSILON)
    //#else
    (v as Float * hexf32!("0x1.0p-32") as Float).min(ONE_MINUS_EPSILON)
    //#endif
}

// see lowdiscrepancy.cpp

/// Once we have an appropriate prime number, use it to compute the
/// radical inverse.
pub fn radical_inverse_specialized(base: u16, a: u64) -> Float {
    let inv_base: Float = 1.0 as Float / base as Float;
    let mut reversed_digits: u64 = 0_u64;
    let mut inv_base_n: Float = 1.0 as Float;
    let mut a: u64 = a; // shadowing input parameter
    while a != 0_u64 {
        let next: u64 = a / base as u64;
        let digit: u64 = a - next * base as u64;
        reversed_digits = reversed_digits * base as u64 + digit;
        inv_base_n *= inv_base;
        a = next;
    }
    assert!(reversed_digits as Float * inv_base_n < 1.00001 as Float);
    (reversed_digits as Float * inv_base_n).min(ONE_MINUS_EPSILON)
}

/// Compute the radical inverse, but put each pixel through the
/// permutation table for the given base. Deal with a special case
/// that can arise.
pub fn scrambled_radical_inverse_specialized(base: u16, perm: &[u16], a: u64) -> Float {
    let inv_base: Float = 1.0 as Float / base as Float;
    let mut reversed_digits: u64 = 0_u64;
    let mut inv_base_n: Float = 1.0 as Float;
    let mut a: u64 = a; // shadowing input parameter
    while a != 0_u64 {
        let next: u64 = a / base as u64;
        let digit: u64 = a - next * base as u64;
        assert!(perm[digit as usize] < base);
        reversed_digits = reversed_digits * base as u64 + perm[digit as usize] as u64;
        inv_base_n *= inv_base;
        a = next;
    }
    assert!(
        (inv_base_n
            * (reversed_digits as Float + inv_base * perm[0] as Float / (1.0 as Float - inv_base)))
            < 1.00001 as Float
    );
    (inv_base_n
        * (reversed_digits as Float + inv_base * perm[0] as Float / (1.0 as Float - inv_base)))
        .min(ONE_MINUS_EPSILON)
}

/// Map to an appropriate prime number and delegate to another
/// function to compute the radical inverse.
pub fn radical_inverse(base_index: u16, a: u64) -> Float {
    match base_index {
        0 => {
            //#ifndef PBRT_HAVE_HEX_FP_CONSTANTS
            //reverse_bits_64(a) as Float * (2.0 as Float).powi(-64 as i32)
            //#else
            reverse_bits_64(a) as Float * hexf32!("0x1.0p-64") as Float
            //#endif
        }
        1 => radical_inverse_specialized(3_u16, a),
        2 => radical_inverse_specialized(5_u16, a),
        3 => radical_inverse_specialized(7_u16, a),
        4 => radical_inverse_specialized(11_u16, a),
        5 => radical_inverse_specialized(13_u16, a),
        6 => radical_inverse_specialized(17_u16, a),
        7 => radical_inverse_specialized(19_u16, a),
        8 => radical_inverse_specialized(23_u16, a),
        9 => radical_inverse_specialized(29_u16, a),
        10 => radical_inverse_specialized(31_u16, a),
        11 => radical_inverse_specialized(37_u16, a),
        12 => radical_inverse_specialized(41_u16, a),
        13 => radical_inverse_specialized(43_u16, a),
        14 => radical_inverse_specialized(47_u16, a),
        15 => radical_inverse_specialized(53_u16, a),
        16 => radical_inverse_specialized(59_u16, a),
        17 => radical_inverse_specialized(61_u16, a),
        18 => radical_inverse_specialized(67_u16, a),
        19 => radical_inverse_specialized(71_u16, a),
        20 => radical_inverse_specialized(73_u16, a),
        21 => radical_inverse_specialized(79_u16, a),
        22 => radical_inverse_specialized(83_u16, a),
        23 => radical_inverse_specialized(89_u16, a),
        24 => radical_inverse_specialized(97_u16, a),
        25 => radical_inverse_specialized(101_u16, a),
        26 => radical_inverse_specialized(103_u16, a),
        27 => radical_inverse_specialized(107_u16, a),
        28 => radical_inverse_specialized(109_u16, a),
        29 => radical_inverse_specialized(113_u16, a),
        30 => radical_inverse_specialized(127_u16, a),
        31 => radical_inverse_specialized(131_u16, a),
        32 => radical_inverse_specialized(137_u16, a),
        33 => radical_inverse_specialized(139_u16, a),
        34 => radical_inverse_specialized(149_u16, a),
        35 => radical_inverse_specialized(151_u16, a),
        36 => radical_inverse_specialized(157_u16, a),
        37 => radical_inverse_specialized(163_u16, a),
        38 => radical_inverse_specialized(167_u16, a),
        39 => radical_inverse_specialized(173_u16, a),
        40 => radical_inverse_specialized(179_u16, a),
        41 => radical_inverse_specialized(181_u16, a),
        42 => radical_inverse_specialized(191_u16, a),
        43 => radical_inverse_specialized(193_u16, a),
        44 => radical_inverse_specialized(197_u16, a),
        45 => radical_inverse_specialized(199_u16, a),
        46 => radical_inverse_specialized(211_u16, a),
        47 => radical_inverse_specialized(223_u16, a),
        48 => radical_inverse_specialized(227_u16, a),
        49 => radical_inverse_specialized(229_u16, a),
        50 => radical_inverse_specialized(233_u16, a),
        51 => radical_inverse_specialized(239_u16, a),
        52 => radical_inverse_specialized(241_u16, a),
        53 => radical_inverse_specialized(251_u16, a),
        54 => radical_inverse_specialized(257_u16, a),
        55 => radical_inverse_specialized(263_u16, a),
        56 => radical_inverse_specialized(269_u16, a),
        57 => radical_inverse_specialized(271_u16, a),
        58 => radical_inverse_specialized(277_u16, a),
        59 => radical_inverse_specialized(281_u16, a),
        60 => radical_inverse_specialized(283_u16, a),
        61 => radical_inverse_specialized(293_u16, a),
        62 => radical_inverse_specialized(307_u16, a),
        63 => radical_inverse_specialized(311_u16, a),
        64 => radical_inverse_specialized(313_u16, a),
        65 => radical_inverse_specialized(317_u16, a),
        66 => radical_inverse_specialized(331_u16, a),
        67 => radical_inverse_specialized(337_u16, a),
        68 => radical_inverse_specialized(347_u16, a),
        69 => radical_inverse_specialized(349_u16, a),
        70 => radical_inverse_specialized(353_u16, a),
        71 => radical_inverse_specialized(359_u16, a),
        72 => radical_inverse_specialized(367_u16, a),
        73 => radical_inverse_specialized(373_u16, a),
        74 => radical_inverse_specialized(379_u16, a),
        75 => radical_inverse_specialized(383_u16, a),
        76 => radical_inverse_specialized(389_u16, a),
        77 => radical_inverse_specialized(397_u16, a),
        78 => radical_inverse_specialized(401_u16, a),
        79 => radical_inverse_specialized(409_u16, a),
        80 => radical_inverse_specialized(419_u16, a),
        81 => radical_inverse_specialized(421_u16, a),
        82 => radical_inverse_specialized(431_u16, a),
        83 => radical_inverse_specialized(433_u16, a),
        84 => radical_inverse_specialized(439_u16, a),
        85 => radical_inverse_specialized(443_u16, a),
        86 => radical_inverse_specialized(449_u16, a),
        87 => radical_inverse_specialized(457_u16, a),
        88 => radical_inverse_specialized(461_u16, a),
        89 => radical_inverse_specialized(463_u16, a),
        90 => radical_inverse_specialized(467_u16, a),
        91 => radical_inverse_specialized(479_u16, a),
        92 => radical_inverse_specialized(487_u16, a),
        93 => radical_inverse_specialized(491_u16, a),
        94 => radical_inverse_specialized(499_u16, a),
        95 => radical_inverse_specialized(503_u16, a),
        96 => radical_inverse_specialized(509_u16, a),
        97 => radical_inverse_specialized(521_u16, a),
        98 => radical_inverse_specialized(523_u16, a),
        99 => radical_inverse_specialized(541_u16, a),
        100 => radical_inverse_specialized(547_u16, a),
        101 => radical_inverse_specialized(557_u16, a),
        102 => radical_inverse_specialized(563_u16, a),
        103 => radical_inverse_specialized(569_u16, a),
        104 => radical_inverse_specialized(571_u16, a),
        105 => radical_inverse_specialized(577_u16, a),
        106 => radical_inverse_specialized(587_u16, a),
        107 => radical_inverse_specialized(593_u16, a),
        108 => radical_inverse_specialized(599_u16, a),
        109 => radical_inverse_specialized(601_u16, a),
        110 => radical_inverse_specialized(607_u16, a),
        111 => radical_inverse_specialized(613_u16, a),
        112 => radical_inverse_specialized(617_u16, a),
        113 => radical_inverse_specialized(619_u16, a),
        114 => radical_inverse_specialized(631_u16, a),
        115 => radical_inverse_specialized(641_u16, a),
        116 => radical_inverse_specialized(643_u16, a),
        117 => radical_inverse_specialized(647_u16, a),
        118 => radical_inverse_specialized(653_u16, a),
        119 => radical_inverse_specialized(659_u16, a),
        120 => radical_inverse_specialized(661_u16, a),
        121 => radical_inverse_specialized(673_u16, a),
        122 => radical_inverse_specialized(677_u16, a),
        123 => radical_inverse_specialized(683_u16, a),
        124 => radical_inverse_specialized(691_u16, a),
        125 => radical_inverse_specialized(701_u16, a),
        126 => radical_inverse_specialized(709_u16, a),
        127 => radical_inverse_specialized(719_u16, a),
        128 => radical_inverse_specialized(727_u16, a),
        129 => radical_inverse_specialized(733_u16, a),
        130 => radical_inverse_specialized(739_u16, a),
        131 => radical_inverse_specialized(743_u16, a),
        132 => radical_inverse_specialized(751_u16, a),
        133 => radical_inverse_specialized(757_u16, a),
        134 => radical_inverse_specialized(761_u16, a),
        135 => radical_inverse_specialized(769_u16, a),
        136 => radical_inverse_specialized(773_u16, a),
        137 => radical_inverse_specialized(787_u16, a),
        138 => radical_inverse_specialized(797_u16, a),
        139 => radical_inverse_specialized(809_u16, a),
        140 => radical_inverse_specialized(811_u16, a),
        141 => radical_inverse_specialized(821_u16, a),
        142 => radical_inverse_specialized(823_u16, a),
        143 => radical_inverse_specialized(827_u16, a),
        144 => radical_inverse_specialized(829_u16, a),
        145 => radical_inverse_specialized(839_u16, a),
        146 => radical_inverse_specialized(853_u16, a),
        147 => radical_inverse_specialized(857_u16, a),
        148 => radical_inverse_specialized(859_u16, a),
        149 => radical_inverse_specialized(863_u16, a),
        150 => radical_inverse_specialized(877_u16, a),
        151 => radical_inverse_specialized(881_u16, a),
        152 => radical_inverse_specialized(883_u16, a),
        153 => radical_inverse_specialized(887_u16, a),
        154 => radical_inverse_specialized(907_u16, a),
        155 => radical_inverse_specialized(911_u16, a),
        156 => radical_inverse_specialized(919_u16, a),
        157 => radical_inverse_specialized(929_u16, a),
        158 => radical_inverse_specialized(937_u16, a),
        159 => radical_inverse_specialized(941_u16, a),
        160 => radical_inverse_specialized(947_u16, a),
        161 => radical_inverse_specialized(953_u16, a),
        162 => radical_inverse_specialized(967_u16, a),
        163 => radical_inverse_specialized(971_u16, a),
        164 => radical_inverse_specialized(977_u16, a),
        165 => radical_inverse_specialized(983_u16, a),
        166 => radical_inverse_specialized(991_u16, a),
        167 => radical_inverse_specialized(997_u16, a),
        168 => radical_inverse_specialized(1009_u16, a),
        169 => radical_inverse_specialized(1013_u16, a),
        170 => radical_inverse_specialized(1019_u16, a),
        171 => radical_inverse_specialized(1021_u16, a),
        172 => radical_inverse_specialized(1031_u16, a),
        173 => radical_inverse_specialized(1033_u16, a),
        174 => radical_inverse_specialized(1039_u16, a),
        175 => radical_inverse_specialized(1049_u16, a),
        176 => radical_inverse_specialized(1051_u16, a),
        177 => radical_inverse_specialized(1061_u16, a),
        178 => radical_inverse_specialized(1063_u16, a),
        179 => radical_inverse_specialized(1069_u16, a),
        180 => radical_inverse_specialized(1087_u16, a),
        181 => radical_inverse_specialized(1091_u16, a),
        182 => radical_inverse_specialized(1093_u16, a),
        183 => radical_inverse_specialized(1097_u16, a),
        184 => radical_inverse_specialized(1103_u16, a),
        185 => radical_inverse_specialized(1109_u16, a),
        186 => radical_inverse_specialized(1117_u16, a),
        187 => radical_inverse_specialized(1123_u16, a),
        188 => radical_inverse_specialized(1129_u16, a),
        189 => radical_inverse_specialized(1151_u16, a),
        190 => radical_inverse_specialized(1153_u16, a),
        191 => radical_inverse_specialized(1163_u16, a),
        192 => radical_inverse_specialized(1171_u16, a),
        193 => radical_inverse_specialized(1181_u16, a),
        194 => radical_inverse_specialized(1187_u16, a),
        195 => radical_inverse_specialized(1193_u16, a),
        196 => radical_inverse_specialized(1201_u16, a),
        197 => radical_inverse_specialized(1213_u16, a),
        198 => radical_inverse_specialized(1217_u16, a),
        199 => radical_inverse_specialized(1223_u16, a),
        200 => radical_inverse_specialized(1229_u16, a),
        201 => radical_inverse_specialized(1231_u16, a),
        202 => radical_inverse_specialized(1237_u16, a),
        203 => radical_inverse_specialized(1249_u16, a),
        204 => radical_inverse_specialized(1259_u16, a),
        205 => radical_inverse_specialized(1277_u16, a),
        206 => radical_inverse_specialized(1279_u16, a),
        207 => radical_inverse_specialized(1283_u16, a),
        208 => radical_inverse_specialized(1289_u16, a),
        209 => radical_inverse_specialized(1291_u16, a),
        210 => radical_inverse_specialized(1297_u16, a),
        211 => radical_inverse_specialized(1301_u16, a),
        212 => radical_inverse_specialized(1303_u16, a),
        213 => radical_inverse_specialized(1307_u16, a),
        214 => radical_inverse_specialized(1319_u16, a),
        215 => radical_inverse_specialized(1321_u16, a),
        216 => radical_inverse_specialized(1327_u16, a),
        217 => radical_inverse_specialized(1361_u16, a),
        218 => radical_inverse_specialized(1367_u16, a),
        219 => radical_inverse_specialized(1373_u16, a),
        220 => radical_inverse_specialized(1381_u16, a),
        221 => radical_inverse_specialized(1399_u16, a),
        222 => radical_inverse_specialized(1409_u16, a),
        223 => radical_inverse_specialized(1423_u16, a),
        224 => radical_inverse_specialized(1427_u16, a),
        225 => radical_inverse_specialized(1429_u16, a),
        226 => radical_inverse_specialized(1433_u16, a),
        227 => radical_inverse_specialized(1439_u16, a),
        228 => radical_inverse_specialized(1447_u16, a),
        229 => radical_inverse_specialized(1451_u16, a),
        230 => radical_inverse_specialized(1453_u16, a),
        231 => radical_inverse_specialized(1459_u16, a),
        232 => radical_inverse_specialized(1471_u16, a),
        233 => radical_inverse_specialized(1481_u16, a),
        234 => radical_inverse_specialized(1483_u16, a),
        235 => radical_inverse_specialized(1487_u16, a),
        236 => radical_inverse_specialized(1489_u16, a),
        237 => radical_inverse_specialized(1493_u16, a),
        238 => radical_inverse_specialized(1499_u16, a),
        239 => radical_inverse_specialized(1511_u16, a),
        240 => radical_inverse_specialized(1523_u16, a),
        241 => radical_inverse_specialized(1531_u16, a),
        242 => radical_inverse_specialized(1543_u16, a),
        243 => radical_inverse_specialized(1549_u16, a),
        244 => radical_inverse_specialized(1553_u16, a),
        245 => radical_inverse_specialized(1559_u16, a),
        246 => radical_inverse_specialized(1567_u16, a),
        247 => radical_inverse_specialized(1571_u16, a),
        248 => radical_inverse_specialized(1579_u16, a),
        249 => radical_inverse_specialized(1583_u16, a),
        250 => radical_inverse_specialized(1597_u16, a),
        251 => radical_inverse_specialized(1601_u16, a),
        252 => radical_inverse_specialized(1607_u16, a),
        253 => radical_inverse_specialized(1609_u16, a),
        254 => radical_inverse_specialized(1613_u16, a),
        255 => radical_inverse_specialized(1619_u16, a),
        256 => radical_inverse_specialized(1621_u16, a),
        257 => radical_inverse_specialized(1627_u16, a),
        258 => radical_inverse_specialized(1637_u16, a),
        259 => radical_inverse_specialized(1657_u16, a),
        260 => radical_inverse_specialized(1663_u16, a),
        261 => radical_inverse_specialized(1667_u16, a),
        262 => radical_inverse_specialized(1669_u16, a),
        263 => radical_inverse_specialized(1693_u16, a),
        264 => radical_inverse_specialized(1697_u16, a),
        265 => radical_inverse_specialized(1699_u16, a),
        266 => radical_inverse_specialized(1709_u16, a),
        267 => radical_inverse_specialized(1721_u16, a),
        268 => radical_inverse_specialized(1723_u16, a),
        269 => radical_inverse_specialized(1733_u16, a),
        270 => radical_inverse_specialized(1741_u16, a),
        271 => radical_inverse_specialized(1747_u16, a),
        272 => radical_inverse_specialized(1753_u16, a),
        273 => radical_inverse_specialized(1759_u16, a),
        274 => radical_inverse_specialized(1777_u16, a),
        275 => radical_inverse_specialized(1783_u16, a),
        276 => radical_inverse_specialized(1787_u16, a),
        277 => radical_inverse_specialized(1789_u16, a),
        278 => radical_inverse_specialized(1801_u16, a),
        279 => radical_inverse_specialized(1811_u16, a),
        280 => radical_inverse_specialized(1823_u16, a),
        281 => radical_inverse_specialized(1831_u16, a),
        282 => radical_inverse_specialized(1847_u16, a),
        283 => radical_inverse_specialized(1861_u16, a),
        284 => radical_inverse_specialized(1867_u16, a),
        285 => radical_inverse_specialized(1871_u16, a),
        286 => radical_inverse_specialized(1873_u16, a),
        287 => radical_inverse_specialized(1877_u16, a),
        288 => radical_inverse_specialized(1879_u16, a),
        289 => radical_inverse_specialized(1889_u16, a),
        290 => radical_inverse_specialized(1901_u16, a),
        291 => radical_inverse_specialized(1907_u16, a),
        292 => radical_inverse_specialized(1913_u16, a),
        293 => radical_inverse_specialized(1931_u16, a),
        294 => radical_inverse_specialized(1933_u16, a),
        295 => radical_inverse_specialized(1949_u16, a),
        296 => radical_inverse_specialized(1951_u16, a),
        297 => radical_inverse_specialized(1973_u16, a),
        298 => radical_inverse_specialized(1979_u16, a),
        299 => radical_inverse_specialized(1987_u16, a),
        300 => radical_inverse_specialized(1993_u16, a),
        301 => radical_inverse_specialized(1997_u16, a),
        302 => radical_inverse_specialized(1999_u16, a),
        303 => radical_inverse_specialized(2003_u16, a),
        304 => radical_inverse_specialized(2011_u16, a),
        305 => radical_inverse_specialized(2017_u16, a),
        306 => radical_inverse_specialized(2027_u16, a),
        307 => radical_inverse_specialized(2029_u16, a),
        308 => radical_inverse_specialized(2039_u16, a),
        309 => radical_inverse_specialized(2053_u16, a),
        310 => radical_inverse_specialized(2063_u16, a),
        311 => radical_inverse_specialized(2069_u16, a),
        312 => radical_inverse_specialized(2081_u16, a),
        313 => radical_inverse_specialized(2083_u16, a),
        314 => radical_inverse_specialized(2087_u16, a),
        315 => radical_inverse_specialized(2089_u16, a),
        316 => radical_inverse_specialized(2099_u16, a),
        317 => radical_inverse_specialized(2111_u16, a),
        318 => radical_inverse_specialized(2113_u16, a),
        319 => radical_inverse_specialized(2129_u16, a),
        320 => radical_inverse_specialized(2131_u16, a),
        321 => radical_inverse_specialized(2137_u16, a),
        322 => radical_inverse_specialized(2141_u16, a),
        323 => radical_inverse_specialized(2143_u16, a),
        324 => radical_inverse_specialized(2153_u16, a),
        325 => radical_inverse_specialized(2161_u16, a),
        326 => radical_inverse_specialized(2179_u16, a),
        327 => radical_inverse_specialized(2203_u16, a),
        328 => radical_inverse_specialized(2207_u16, a),
        329 => radical_inverse_specialized(2213_u16, a),
        330 => radical_inverse_specialized(2221_u16, a),
        331 => radical_inverse_specialized(2237_u16, a),
        332 => radical_inverse_specialized(2239_u16, a),
        333 => radical_inverse_specialized(2243_u16, a),
        334 => radical_inverse_specialized(2251_u16, a),
        335 => radical_inverse_specialized(2267_u16, a),
        336 => radical_inverse_specialized(2269_u16, a),
        337 => radical_inverse_specialized(2273_u16, a),
        338 => radical_inverse_specialized(2281_u16, a),
        339 => radical_inverse_specialized(2287_u16, a),
        340 => radical_inverse_specialized(2293_u16, a),
        341 => radical_inverse_specialized(2297_u16, a),
        342 => radical_inverse_specialized(2309_u16, a),
        343 => radical_inverse_specialized(2311_u16, a),
        344 => radical_inverse_specialized(2333_u16, a),
        345 => radical_inverse_specialized(2339_u16, a),
        346 => radical_inverse_specialized(2341_u16, a),
        347 => radical_inverse_specialized(2347_u16, a),
        348 => radical_inverse_specialized(2351_u16, a),
        349 => radical_inverse_specialized(2357_u16, a),
        350 => radical_inverse_specialized(2371_u16, a),
        351 => radical_inverse_specialized(2377_u16, a),
        352 => radical_inverse_specialized(2381_u16, a),
        353 => radical_inverse_specialized(2383_u16, a),
        354 => radical_inverse_specialized(2389_u16, a),
        355 => radical_inverse_specialized(2393_u16, a),
        356 => radical_inverse_specialized(2399_u16, a),
        357 => radical_inverse_specialized(2411_u16, a),
        358 => radical_inverse_specialized(2417_u16, a),
        359 => radical_inverse_specialized(2423_u16, a),
        360 => radical_inverse_specialized(2437_u16, a),
        361 => radical_inverse_specialized(2441_u16, a),
        362 => radical_inverse_specialized(2447_u16, a),
        363 => radical_inverse_specialized(2459_u16, a),
        364 => radical_inverse_specialized(2467_u16, a),
        365 => radical_inverse_specialized(2473_u16, a),
        366 => radical_inverse_specialized(2477_u16, a),
        367 => radical_inverse_specialized(2503_u16, a),
        368 => radical_inverse_specialized(2521_u16, a),
        369 => radical_inverse_specialized(2531_u16, a),
        370 => radical_inverse_specialized(2539_u16, a),
        371 => radical_inverse_specialized(2543_u16, a),
        372 => radical_inverse_specialized(2549_u16, a),
        373 => radical_inverse_specialized(2551_u16, a),
        374 => radical_inverse_specialized(2557_u16, a),
        375 => radical_inverse_specialized(2579_u16, a),
        376 => radical_inverse_specialized(2591_u16, a),
        377 => radical_inverse_specialized(2593_u16, a),
        378 => radical_inverse_specialized(2609_u16, a),
        379 => radical_inverse_specialized(2617_u16, a),
        380 => radical_inverse_specialized(2621_u16, a),
        381 => radical_inverse_specialized(2633_u16, a),
        382 => radical_inverse_specialized(2647_u16, a),
        383 => radical_inverse_specialized(2657_u16, a),
        384 => radical_inverse_specialized(2659_u16, a),
        385 => radical_inverse_specialized(2663_u16, a),
        386 => radical_inverse_specialized(2671_u16, a),
        387 => radical_inverse_specialized(2677_u16, a),
        388 => radical_inverse_specialized(2683_u16, a),
        389 => radical_inverse_specialized(2687_u16, a),
        390 => radical_inverse_specialized(2689_u16, a),
        391 => radical_inverse_specialized(2693_u16, a),
        392 => radical_inverse_specialized(2699_u16, a),
        393 => radical_inverse_specialized(2707_u16, a),
        394 => radical_inverse_specialized(2711_u16, a),
        395 => radical_inverse_specialized(2713_u16, a),
        396 => radical_inverse_specialized(2719_u16, a),
        397 => radical_inverse_specialized(2729_u16, a),
        398 => radical_inverse_specialized(2731_u16, a),
        399 => radical_inverse_specialized(2741_u16, a),
        400 => radical_inverse_specialized(2749_u16, a),
        401 => radical_inverse_specialized(2753_u16, a),
        402 => radical_inverse_specialized(2767_u16, a),
        403 => radical_inverse_specialized(2777_u16, a),
        404 => radical_inverse_specialized(2789_u16, a),
        405 => radical_inverse_specialized(2791_u16, a),
        406 => radical_inverse_specialized(2797_u16, a),
        407 => radical_inverse_specialized(2801_u16, a),
        408 => radical_inverse_specialized(2803_u16, a),
        409 => radical_inverse_specialized(2819_u16, a),
        410 => radical_inverse_specialized(2833_u16, a),
        411 => radical_inverse_specialized(2837_u16, a),
        412 => radical_inverse_specialized(2843_u16, a),
        413 => radical_inverse_specialized(2851_u16, a),
        414 => radical_inverse_specialized(2857_u16, a),
        415 => radical_inverse_specialized(2861_u16, a),
        416 => radical_inverse_specialized(2879_u16, a),
        417 => radical_inverse_specialized(2887_u16, a),
        418 => radical_inverse_specialized(2897_u16, a),
        419 => radical_inverse_specialized(2903_u16, a),
        420 => radical_inverse_specialized(2909_u16, a),
        421 => radical_inverse_specialized(2917_u16, a),
        422 => radical_inverse_specialized(2927_u16, a),
        423 => radical_inverse_specialized(2939_u16, a),
        424 => radical_inverse_specialized(2953_u16, a),
        425 => radical_inverse_specialized(2957_u16, a),
        426 => radical_inverse_specialized(2963_u16, a),
        427 => radical_inverse_specialized(2969_u16, a),
        428 => radical_inverse_specialized(2971_u16, a),
        429 => radical_inverse_specialized(2999_u16, a),
        430 => radical_inverse_specialized(3001_u16, a),
        431 => radical_inverse_specialized(3011_u16, a),
        432 => radical_inverse_specialized(3019_u16, a),
        433 => radical_inverse_specialized(3023_u16, a),
        434 => radical_inverse_specialized(3037_u16, a),
        435 => radical_inverse_specialized(3041_u16, a),
        436 => radical_inverse_specialized(3049_u16, a),
        437 => radical_inverse_specialized(3061_u16, a),
        438 => radical_inverse_specialized(3067_u16, a),
        439 => radical_inverse_specialized(3079_u16, a),
        440 => radical_inverse_specialized(3083_u16, a),
        441 => radical_inverse_specialized(3089_u16, a),
        442 => radical_inverse_specialized(3109_u16, a),
        443 => radical_inverse_specialized(3119_u16, a),
        444 => radical_inverse_specialized(3121_u16, a),
        445 => radical_inverse_specialized(3137_u16, a),
        446 => radical_inverse_specialized(3163_u16, a),
        447 => radical_inverse_specialized(3167_u16, a),
        448 => radical_inverse_specialized(3169_u16, a),
        449 => radical_inverse_specialized(3181_u16, a),
        450 => radical_inverse_specialized(3187_u16, a),
        451 => radical_inverse_specialized(3191_u16, a),
        452 => radical_inverse_specialized(3203_u16, a),
        453 => radical_inverse_specialized(3209_u16, a),
        454 => radical_inverse_specialized(3217_u16, a),
        455 => radical_inverse_specialized(3221_u16, a),
        456 => radical_inverse_specialized(3229_u16, a),
        457 => radical_inverse_specialized(3251_u16, a),
        458 => radical_inverse_specialized(3253_u16, a),
        459 => radical_inverse_specialized(3257_u16, a),
        460 => radical_inverse_specialized(3259_u16, a),
        461 => radical_inverse_specialized(3271_u16, a),
        462 => radical_inverse_specialized(3299_u16, a),
        463 => radical_inverse_specialized(3301_u16, a),
        464 => radical_inverse_specialized(3307_u16, a),
        465 => radical_inverse_specialized(3313_u16, a),
        466 => radical_inverse_specialized(3319_u16, a),
        467 => radical_inverse_specialized(3323_u16, a),
        468 => radical_inverse_specialized(3329_u16, a),
        469 => radical_inverse_specialized(3331_u16, a),
        470 => radical_inverse_specialized(3343_u16, a),
        471 => radical_inverse_specialized(3347_u16, a),
        472 => radical_inverse_specialized(3359_u16, a),
        473 => radical_inverse_specialized(3361_u16, a),
        474 => radical_inverse_specialized(3371_u16, a),
        475 => radical_inverse_specialized(3373_u16, a),
        476 => radical_inverse_specialized(3389_u16, a),
        477 => radical_inverse_specialized(3391_u16, a),
        478 => radical_inverse_specialized(3407_u16, a),
        479 => radical_inverse_specialized(3413_u16, a),
        480 => radical_inverse_specialized(3433_u16, a),
        481 => radical_inverse_specialized(3449_u16, a),
        482 => radical_inverse_specialized(3457_u16, a),
        483 => radical_inverse_specialized(3461_u16, a),
        484 => radical_inverse_specialized(3463_u16, a),
        485 => radical_inverse_specialized(3467_u16, a),
        486 => radical_inverse_specialized(3469_u16, a),
        487 => radical_inverse_specialized(3491_u16, a),
        488 => radical_inverse_specialized(3499_u16, a),
        489 => radical_inverse_specialized(3511_u16, a),
        490 => radical_inverse_specialized(3517_u16, a),
        491 => radical_inverse_specialized(3527_u16, a),
        492 => radical_inverse_specialized(3529_u16, a),
        493 => radical_inverse_specialized(3533_u16, a),
        494 => radical_inverse_specialized(3539_u16, a),
        495 => radical_inverse_specialized(3541_u16, a),
        496 => radical_inverse_specialized(3547_u16, a),
        497 => radical_inverse_specialized(3557_u16, a),
        498 => radical_inverse_specialized(3559_u16, a),
        499 => radical_inverse_specialized(3571_u16, a),
        500 => radical_inverse_specialized(3581_u16, a),
        501 => radical_inverse_specialized(3583_u16, a),
        502 => radical_inverse_specialized(3593_u16, a),
        503 => radical_inverse_specialized(3607_u16, a),
        504 => radical_inverse_specialized(3613_u16, a),
        505 => radical_inverse_specialized(3617_u16, a),
        506 => radical_inverse_specialized(3623_u16, a),
        507 => radical_inverse_specialized(3631_u16, a),
        508 => radical_inverse_specialized(3637_u16, a),
        509 => radical_inverse_specialized(3643_u16, a),
        510 => radical_inverse_specialized(3659_u16, a),
        511 => radical_inverse_specialized(3671_u16, a),
        512 => radical_inverse_specialized(3673_u16, a),
        513 => radical_inverse_specialized(3677_u16, a),
        514 => radical_inverse_specialized(3691_u16, a),
        515 => radical_inverse_specialized(3697_u16, a),
        516 => radical_inverse_specialized(3701_u16, a),
        517 => radical_inverse_specialized(3709_u16, a),
        518 => radical_inverse_specialized(3719_u16, a),
        519 => radical_inverse_specialized(3727_u16, a),
        520 => radical_inverse_specialized(3733_u16, a),
        521 => radical_inverse_specialized(3739_u16, a),
        522 => radical_inverse_specialized(3761_u16, a),
        523 => radical_inverse_specialized(3767_u16, a),
        524 => radical_inverse_specialized(3769_u16, a),
        525 => radical_inverse_specialized(3779_u16, a),
        526 => radical_inverse_specialized(3793_u16, a),
        527 => radical_inverse_specialized(3797_u16, a),
        528 => radical_inverse_specialized(3803_u16, a),
        529 => radical_inverse_specialized(3821_u16, a),
        530 => radical_inverse_specialized(3823_u16, a),
        531 => radical_inverse_specialized(3833_u16, a),
        532 => radical_inverse_specialized(3847_u16, a),
        533 => radical_inverse_specialized(3851_u16, a),
        534 => radical_inverse_specialized(3853_u16, a),
        535 => radical_inverse_specialized(3863_u16, a),
        536 => radical_inverse_specialized(3877_u16, a),
        537 => radical_inverse_specialized(3881_u16, a),
        538 => radical_inverse_specialized(3889_u16, a),
        539 => radical_inverse_specialized(3907_u16, a),
        540 => radical_inverse_specialized(3911_u16, a),
        541 => radical_inverse_specialized(3917_u16, a),
        542 => radical_inverse_specialized(3919_u16, a),
        543 => radical_inverse_specialized(3923_u16, a),
        544 => radical_inverse_specialized(3929_u16, a),
        545 => radical_inverse_specialized(3931_u16, a),
        546 => radical_inverse_specialized(3943_u16, a),
        547 => radical_inverse_specialized(3947_u16, a),
        548 => radical_inverse_specialized(3967_u16, a),
        549 => radical_inverse_specialized(3989_u16, a),
        550 => radical_inverse_specialized(4001_u16, a),
        551 => radical_inverse_specialized(4003_u16, a),
        552 => radical_inverse_specialized(4007_u16, a),
        553 => radical_inverse_specialized(4013_u16, a),
        554 => radical_inverse_specialized(4019_u16, a),
        555 => radical_inverse_specialized(4021_u16, a),
        556 => radical_inverse_specialized(4027_u16, a),
        557 => radical_inverse_specialized(4049_u16, a),
        558 => radical_inverse_specialized(4051_u16, a),
        559 => radical_inverse_specialized(4057_u16, a),
        560 => radical_inverse_specialized(4073_u16, a),
        561 => radical_inverse_specialized(4079_u16, a),
        562 => radical_inverse_specialized(4091_u16, a),
        563 => radical_inverse_specialized(4093_u16, a),
        564 => radical_inverse_specialized(4099_u16, a),
        565 => radical_inverse_specialized(4111_u16, a),
        566 => radical_inverse_specialized(4127_u16, a),
        567 => radical_inverse_specialized(4129_u16, a),
        568 => radical_inverse_specialized(4133_u16, a),
        569 => radical_inverse_specialized(4139_u16, a),
        570 => radical_inverse_specialized(4153_u16, a),
        571 => radical_inverse_specialized(4157_u16, a),
        572 => radical_inverse_specialized(4159_u16, a),
        573 => radical_inverse_specialized(4177_u16, a),
        574 => radical_inverse_specialized(4201_u16, a),
        575 => radical_inverse_specialized(4211_u16, a),
        576 => radical_inverse_specialized(4217_u16, a),
        577 => radical_inverse_specialized(4219_u16, a),
        578 => radical_inverse_specialized(4229_u16, a),
        579 => radical_inverse_specialized(4231_u16, a),
        580 => radical_inverse_specialized(4241_u16, a),
        581 => radical_inverse_specialized(4243_u16, a),
        582 => radical_inverse_specialized(4253_u16, a),
        583 => radical_inverse_specialized(4259_u16, a),
        584 => radical_inverse_specialized(4261_u16, a),
        585 => radical_inverse_specialized(4271_u16, a),
        586 => radical_inverse_specialized(4273_u16, a),
        587 => radical_inverse_specialized(4283_u16, a),
        588 => radical_inverse_specialized(4289_u16, a),
        589 => radical_inverse_specialized(4297_u16, a),
        590 => radical_inverse_specialized(4327_u16, a),
        591 => radical_inverse_specialized(4337_u16, a),
        592 => radical_inverse_specialized(4339_u16, a),
        593 => radical_inverse_specialized(4349_u16, a),
        594 => radical_inverse_specialized(4357_u16, a),
        595 => radical_inverse_specialized(4363_u16, a),
        596 => radical_inverse_specialized(4373_u16, a),
        597 => radical_inverse_specialized(4391_u16, a),
        598 => radical_inverse_specialized(4397_u16, a),
        599 => radical_inverse_specialized(4409_u16, a),
        600 => radical_inverse_specialized(4421_u16, a),
        601 => radical_inverse_specialized(4423_u16, a),
        602 => radical_inverse_specialized(4441_u16, a),
        603 => radical_inverse_specialized(4447_u16, a),
        604 => radical_inverse_specialized(4451_u16, a),
        605 => radical_inverse_specialized(4457_u16, a),
        606 => radical_inverse_specialized(4463_u16, a),
        607 => radical_inverse_specialized(4481_u16, a),
        608 => radical_inverse_specialized(4483_u16, a),
        609 => radical_inverse_specialized(4493_u16, a),
        610 => radical_inverse_specialized(4507_u16, a),
        611 => radical_inverse_specialized(4513_u16, a),
        612 => radical_inverse_specialized(4517_u16, a),
        613 => radical_inverse_specialized(4519_u16, a),
        614 => radical_inverse_specialized(4523_u16, a),
        615 => radical_inverse_specialized(4547_u16, a),
        616 => radical_inverse_specialized(4549_u16, a),
        617 => radical_inverse_specialized(4561_u16, a),
        618 => radical_inverse_specialized(4567_u16, a),
        619 => radical_inverse_specialized(4583_u16, a),
        620 => radical_inverse_specialized(4591_u16, a),
        621 => radical_inverse_specialized(4597_u16, a),
        622 => radical_inverse_specialized(4603_u16, a),
        623 => radical_inverse_specialized(4621_u16, a),
        624 => radical_inverse_specialized(4637_u16, a),
        625 => radical_inverse_specialized(4639_u16, a),
        626 => radical_inverse_specialized(4643_u16, a),
        627 => radical_inverse_specialized(4649_u16, a),
        628 => radical_inverse_specialized(4651_u16, a),
        629 => radical_inverse_specialized(4657_u16, a),
        630 => radical_inverse_specialized(4663_u16, a),
        631 => radical_inverse_specialized(4673_u16, a),
        632 => radical_inverse_specialized(4679_u16, a),
        633 => radical_inverse_specialized(4691_u16, a),
        634 => radical_inverse_specialized(4703_u16, a),
        635 => radical_inverse_specialized(4721_u16, a),
        636 => radical_inverse_specialized(4723_u16, a),
        637 => radical_inverse_specialized(4729_u16, a),
        638 => radical_inverse_specialized(4733_u16, a),
        639 => radical_inverse_specialized(4751_u16, a),
        640 => radical_inverse_specialized(4759_u16, a),
        641 => radical_inverse_specialized(4783_u16, a),
        642 => radical_inverse_specialized(4787_u16, a),
        643 => radical_inverse_specialized(4789_u16, a),
        644 => radical_inverse_specialized(4793_u16, a),
        645 => radical_inverse_specialized(4799_u16, a),
        646 => radical_inverse_specialized(4801_u16, a),
        647 => radical_inverse_specialized(4813_u16, a),
        648 => radical_inverse_specialized(4817_u16, a),
        649 => radical_inverse_specialized(4831_u16, a),
        650 => radical_inverse_specialized(4861_u16, a),
        651 => radical_inverse_specialized(4871_u16, a),
        652 => radical_inverse_specialized(4877_u16, a),
        653 => radical_inverse_specialized(4889_u16, a),
        654 => radical_inverse_specialized(4903_u16, a),
        655 => radical_inverse_specialized(4909_u16, a),
        656 => radical_inverse_specialized(4919_u16, a),
        657 => radical_inverse_specialized(4931_u16, a),
        658 => radical_inverse_specialized(4933_u16, a),
        659 => radical_inverse_specialized(4937_u16, a),
        660 => radical_inverse_specialized(4943_u16, a),
        661 => radical_inverse_specialized(4951_u16, a),
        662 => radical_inverse_specialized(4957_u16, a),
        663 => radical_inverse_specialized(4967_u16, a),
        664 => radical_inverse_specialized(4969_u16, a),
        665 => radical_inverse_specialized(4973_u16, a),
        666 => radical_inverse_specialized(4987_u16, a),
        667 => radical_inverse_specialized(4993_u16, a),
        668 => radical_inverse_specialized(4999_u16, a),
        669 => radical_inverse_specialized(5003_u16, a),
        670 => radical_inverse_specialized(5009_u16, a),
        671 => radical_inverse_specialized(5011_u16, a),
        672 => radical_inverse_specialized(5021_u16, a),
        673 => radical_inverse_specialized(5023_u16, a),
        674 => radical_inverse_specialized(5039_u16, a),
        675 => radical_inverse_specialized(5051_u16, a),
        676 => radical_inverse_specialized(5059_u16, a),
        677 => radical_inverse_specialized(5077_u16, a),
        678 => radical_inverse_specialized(5081_u16, a),
        679 => radical_inverse_specialized(5087_u16, a),
        680 => radical_inverse_specialized(5099_u16, a),
        681 => radical_inverse_specialized(5101_u16, a),
        682 => radical_inverse_specialized(5107_u16, a),
        683 => radical_inverse_specialized(5113_u16, a),
        684 => radical_inverse_specialized(5119_u16, a),
        685 => radical_inverse_specialized(5147_u16, a),
        686 => radical_inverse_specialized(5153_u16, a),
        687 => radical_inverse_specialized(5167_u16, a),
        688 => radical_inverse_specialized(5171_u16, a),
        689 => radical_inverse_specialized(5179_u16, a),
        690 => radical_inverse_specialized(5189_u16, a),
        691 => radical_inverse_specialized(5197_u16, a),
        692 => radical_inverse_specialized(5209_u16, a),
        693 => radical_inverse_specialized(5227_u16, a),
        694 => radical_inverse_specialized(5231_u16, a),
        695 => radical_inverse_specialized(5233_u16, a),
        696 => radical_inverse_specialized(5237_u16, a),
        697 => radical_inverse_specialized(5261_u16, a),
        698 => radical_inverse_specialized(5273_u16, a),
        699 => radical_inverse_specialized(5279_u16, a),
        700 => radical_inverse_specialized(5281_u16, a),
        701 => radical_inverse_specialized(5297_u16, a),
        702 => radical_inverse_specialized(5303_u16, a),
        703 => radical_inverse_specialized(5309_u16, a),
        704 => radical_inverse_specialized(5323_u16, a),
        705 => radical_inverse_specialized(5333_u16, a),
        706 => radical_inverse_specialized(5347_u16, a),
        707 => radical_inverse_specialized(5351_u16, a),
        708 => radical_inverse_specialized(5381_u16, a),
        709 => radical_inverse_specialized(5387_u16, a),
        710 => radical_inverse_specialized(5393_u16, a),
        711 => radical_inverse_specialized(5399_u16, a),
        712 => radical_inverse_specialized(5407_u16, a),
        713 => radical_inverse_specialized(5413_u16, a),
        714 => radical_inverse_specialized(5417_u16, a),
        715 => radical_inverse_specialized(5419_u16, a),
        716 => radical_inverse_specialized(5431_u16, a),
        717 => radical_inverse_specialized(5437_u16, a),
        718 => radical_inverse_specialized(5441_u16, a),
        719 => radical_inverse_specialized(5443_u16, a),
        720 => radical_inverse_specialized(5449_u16, a),
        721 => radical_inverse_specialized(5471_u16, a),
        722 => radical_inverse_specialized(5477_u16, a),
        723 => radical_inverse_specialized(5479_u16, a),
        724 => radical_inverse_specialized(5483_u16, a),
        725 => radical_inverse_specialized(5501_u16, a),
        726 => radical_inverse_specialized(5503_u16, a),
        727 => radical_inverse_specialized(5507_u16, a),
        728 => radical_inverse_specialized(5519_u16, a),
        729 => radical_inverse_specialized(5521_u16, a),
        730 => radical_inverse_specialized(5527_u16, a),
        731 => radical_inverse_specialized(5531_u16, a),
        732 => radical_inverse_specialized(5557_u16, a),
        733 => radical_inverse_specialized(5563_u16, a),
        734 => radical_inverse_specialized(5569_u16, a),
        735 => radical_inverse_specialized(5573_u16, a),
        736 => radical_inverse_specialized(5581_u16, a),
        737 => radical_inverse_specialized(5591_u16, a),
        738 => radical_inverse_specialized(5623_u16, a),
        739 => radical_inverse_specialized(5639_u16, a),
        740 => radical_inverse_specialized(5641_u16, a),
        741 => radical_inverse_specialized(5647_u16, a),
        742 => radical_inverse_specialized(5651_u16, a),
        743 => radical_inverse_specialized(5653_u16, a),
        744 => radical_inverse_specialized(5657_u16, a),
        745 => radical_inverse_specialized(5659_u16, a),
        746 => radical_inverse_specialized(5669_u16, a),
        747 => radical_inverse_specialized(5683_u16, a),
        748 => radical_inverse_specialized(5689_u16, a),
        749 => radical_inverse_specialized(5693_u16, a),
        750 => radical_inverse_specialized(5701_u16, a),
        751 => radical_inverse_specialized(5711_u16, a),
        752 => radical_inverse_specialized(5717_u16, a),
        753 => radical_inverse_specialized(5737_u16, a),
        754 => radical_inverse_specialized(5741_u16, a),
        755 => radical_inverse_specialized(5743_u16, a),
        756 => radical_inverse_specialized(5749_u16, a),
        757 => radical_inverse_specialized(5779_u16, a),
        758 => radical_inverse_specialized(5783_u16, a),
        759 => radical_inverse_specialized(5791_u16, a),
        760 => radical_inverse_specialized(5801_u16, a),
        761 => radical_inverse_specialized(5807_u16, a),
        762 => radical_inverse_specialized(5813_u16, a),
        763 => radical_inverse_specialized(5821_u16, a),
        764 => radical_inverse_specialized(5827_u16, a),
        765 => radical_inverse_specialized(5839_u16, a),
        766 => radical_inverse_specialized(5843_u16, a),
        767 => radical_inverse_specialized(5849_u16, a),
        768 => radical_inverse_specialized(5851_u16, a),
        769 => radical_inverse_specialized(5857_u16, a),
        770 => radical_inverse_specialized(5861_u16, a),
        771 => radical_inverse_specialized(5867_u16, a),
        772 => radical_inverse_specialized(5869_u16, a),
        773 => radical_inverse_specialized(5879_u16, a),
        774 => radical_inverse_specialized(5881_u16, a),
        775 => radical_inverse_specialized(5897_u16, a),
        776 => radical_inverse_specialized(5903_u16, a),
        777 => radical_inverse_specialized(5923_u16, a),
        778 => radical_inverse_specialized(5927_u16, a),
        779 => radical_inverse_specialized(5939_u16, a),
        780 => radical_inverse_specialized(5953_u16, a),
        781 => radical_inverse_specialized(5981_u16, a),
        782 => radical_inverse_specialized(5987_u16, a),
        783 => radical_inverse_specialized(6007_u16, a),
        784 => radical_inverse_specialized(6011_u16, a),
        785 => radical_inverse_specialized(6029_u16, a),
        786 => radical_inverse_specialized(6037_u16, a),
        787 => radical_inverse_specialized(6043_u16, a),
        788 => radical_inverse_specialized(6047_u16, a),
        789 => radical_inverse_specialized(6053_u16, a),
        790 => radical_inverse_specialized(6067_u16, a),
        791 => radical_inverse_specialized(6073_u16, a),
        792 => radical_inverse_specialized(6079_u16, a),
        793 => radical_inverse_specialized(6089_u16, a),
        794 => radical_inverse_specialized(6091_u16, a),
        795 => radical_inverse_specialized(6101_u16, a),
        796 => radical_inverse_specialized(6113_u16, a),
        797 => radical_inverse_specialized(6121_u16, a),
        798 => radical_inverse_specialized(6131_u16, a),
        799 => radical_inverse_specialized(6133_u16, a),
        800 => radical_inverse_specialized(6143_u16, a),
        801 => radical_inverse_specialized(6151_u16, a),
        802 => radical_inverse_specialized(6163_u16, a),
        803 => radical_inverse_specialized(6173_u16, a),
        804 => radical_inverse_specialized(6197_u16, a),
        805 => radical_inverse_specialized(6199_u16, a),
        806 => radical_inverse_specialized(6203_u16, a),
        807 => radical_inverse_specialized(6211_u16, a),
        808 => radical_inverse_specialized(6217_u16, a),
        809 => radical_inverse_specialized(6221_u16, a),
        810 => radical_inverse_specialized(6229_u16, a),
        811 => radical_inverse_specialized(6247_u16, a),
        812 => radical_inverse_specialized(6257_u16, a),
        813 => radical_inverse_specialized(6263_u16, a),
        814 => radical_inverse_specialized(6269_u16, a),
        815 => radical_inverse_specialized(6271_u16, a),
        816 => radical_inverse_specialized(6277_u16, a),
        817 => radical_inverse_specialized(6287_u16, a),
        818 => radical_inverse_specialized(6299_u16, a),
        819 => radical_inverse_specialized(6301_u16, a),
        820 => radical_inverse_specialized(6311_u16, a),
        821 => radical_inverse_specialized(6317_u16, a),
        822 => radical_inverse_specialized(6323_u16, a),
        823 => radical_inverse_specialized(6329_u16, a),
        824 => radical_inverse_specialized(6337_u16, a),
        825 => radical_inverse_specialized(6343_u16, a),
        826 => radical_inverse_specialized(6353_u16, a),
        827 => radical_inverse_specialized(6359_u16, a),
        828 => radical_inverse_specialized(6361_u16, a),
        829 => radical_inverse_specialized(6367_u16, a),
        830 => radical_inverse_specialized(6373_u16, a),
        831 => radical_inverse_specialized(6379_u16, a),
        832 => radical_inverse_specialized(6389_u16, a),
        833 => radical_inverse_specialized(6397_u16, a),
        834 => radical_inverse_specialized(6421_u16, a),
        835 => radical_inverse_specialized(6427_u16, a),
        836 => radical_inverse_specialized(6449_u16, a),
        837 => radical_inverse_specialized(6451_u16, a),
        838 => radical_inverse_specialized(6469_u16, a),
        839 => radical_inverse_specialized(6473_u16, a),
        840 => radical_inverse_specialized(6481_u16, a),
        841 => radical_inverse_specialized(6491_u16, a),
        842 => radical_inverse_specialized(6521_u16, a),
        843 => radical_inverse_specialized(6529_u16, a),
        844 => radical_inverse_specialized(6547_u16, a),
        845 => radical_inverse_specialized(6551_u16, a),
        846 => radical_inverse_specialized(6553_u16, a),
        847 => radical_inverse_specialized(6563_u16, a),
        848 => radical_inverse_specialized(6569_u16, a),
        849 => radical_inverse_specialized(6571_u16, a),
        850 => radical_inverse_specialized(6577_u16, a),
        851 => radical_inverse_specialized(6581_u16, a),
        852 => radical_inverse_specialized(6599_u16, a),
        853 => radical_inverse_specialized(6607_u16, a),
        854 => radical_inverse_specialized(6619_u16, a),
        855 => radical_inverse_specialized(6637_u16, a),
        856 => radical_inverse_specialized(6653_u16, a),
        857 => radical_inverse_specialized(6659_u16, a),
        858 => radical_inverse_specialized(6661_u16, a),
        859 => radical_inverse_specialized(6673_u16, a),
        860 => radical_inverse_specialized(6679_u16, a),
        861 => radical_inverse_specialized(6689_u16, a),
        862 => radical_inverse_specialized(6691_u16, a),
        863 => radical_inverse_specialized(6701_u16, a),
        864 => radical_inverse_specialized(6703_u16, a),
        865 => radical_inverse_specialized(6709_u16, a),
        866 => radical_inverse_specialized(6719_u16, a),
        867 => radical_inverse_specialized(6733_u16, a),
        868 => radical_inverse_specialized(6737_u16, a),
        869 => radical_inverse_specialized(6761_u16, a),
        870 => radical_inverse_specialized(6763_u16, a),
        871 => radical_inverse_specialized(6779_u16, a),
        872 => radical_inverse_specialized(6781_u16, a),
        873 => radical_inverse_specialized(6791_u16, a),
        874 => radical_inverse_specialized(6793_u16, a),
        875 => radical_inverse_specialized(6803_u16, a),
        876 => radical_inverse_specialized(6823_u16, a),
        877 => radical_inverse_specialized(6827_u16, a),
        878 => radical_inverse_specialized(6829_u16, a),
        879 => radical_inverse_specialized(6833_u16, a),
        880 => radical_inverse_specialized(6841_u16, a),
        881 => radical_inverse_specialized(6857_u16, a),
        882 => radical_inverse_specialized(6863_u16, a),
        883 => radical_inverse_specialized(6869_u16, a),
        884 => radical_inverse_specialized(6871_u16, a),
        885 => radical_inverse_specialized(6883_u16, a),
        886 => radical_inverse_specialized(6899_u16, a),
        887 => radical_inverse_specialized(6907_u16, a),
        888 => radical_inverse_specialized(6911_u16, a),
        889 => radical_inverse_specialized(6917_u16, a),
        890 => radical_inverse_specialized(6947_u16, a),
        891 => radical_inverse_specialized(6949_u16, a),
        892 => radical_inverse_specialized(6959_u16, a),
        893 => radical_inverse_specialized(6961_u16, a),
        894 => radical_inverse_specialized(6967_u16, a),
        895 => radical_inverse_specialized(6971_u16, a),
        896 => radical_inverse_specialized(6977_u16, a),
        897 => radical_inverse_specialized(6983_u16, a),
        898 => radical_inverse_specialized(6991_u16, a),
        899 => radical_inverse_specialized(6997_u16, a),
        900 => radical_inverse_specialized(7001_u16, a),
        901 => radical_inverse_specialized(7013_u16, a),
        902 => radical_inverse_specialized(7019_u16, a),
        903 => radical_inverse_specialized(7027_u16, a),
        904 => radical_inverse_specialized(7039_u16, a),
        905 => radical_inverse_specialized(7043_u16, a),
        906 => radical_inverse_specialized(7057_u16, a),
        907 => radical_inverse_specialized(7069_u16, a),
        908 => radical_inverse_specialized(7079_u16, a),
        909 => radical_inverse_specialized(7103_u16, a),
        910 => radical_inverse_specialized(7109_u16, a),
        911 => radical_inverse_specialized(7121_u16, a),
        912 => radical_inverse_specialized(7127_u16, a),
        913 => radical_inverse_specialized(7129_u16, a),
        914 => radical_inverse_specialized(7151_u16, a),
        915 => radical_inverse_specialized(7159_u16, a),
        916 => radical_inverse_specialized(7177_u16, a),
        917 => radical_inverse_specialized(7187_u16, a),
        918 => radical_inverse_specialized(7193_u16, a),
        919 => radical_inverse_specialized(7207_u16, a),
        920 => radical_inverse_specialized(7211_u16, a),
        921 => radical_inverse_specialized(7213_u16, a),
        922 => radical_inverse_specialized(7219_u16, a),
        923 => radical_inverse_specialized(7229_u16, a),
        924 => radical_inverse_specialized(7237_u16, a),
        925 => radical_inverse_specialized(7243_u16, a),
        926 => radical_inverse_specialized(7247_u16, a),
        927 => radical_inverse_specialized(7253_u16, a),
        928 => radical_inverse_specialized(7283_u16, a),
        929 => radical_inverse_specialized(7297_u16, a),
        930 => radical_inverse_specialized(7307_u16, a),
        931 => radical_inverse_specialized(7309_u16, a),
        932 => radical_inverse_specialized(7321_u16, a),
        933 => radical_inverse_specialized(7331_u16, a),
        934 => radical_inverse_specialized(7333_u16, a),
        935 => radical_inverse_specialized(7349_u16, a),
        936 => radical_inverse_specialized(7351_u16, a),
        937 => radical_inverse_specialized(7369_u16, a),
        938 => radical_inverse_specialized(7393_u16, a),
        939 => radical_inverse_specialized(7411_u16, a),
        940 => radical_inverse_specialized(7417_u16, a),
        941 => radical_inverse_specialized(7433_u16, a),
        942 => radical_inverse_specialized(7451_u16, a),
        943 => radical_inverse_specialized(7457_u16, a),
        944 => radical_inverse_specialized(7459_u16, a),
        945 => radical_inverse_specialized(7477_u16, a),
        946 => radical_inverse_specialized(7481_u16, a),
        947 => radical_inverse_specialized(7487_u16, a),
        948 => radical_inverse_specialized(7489_u16, a),
        949 => radical_inverse_specialized(7499_u16, a),
        950 => radical_inverse_specialized(7507_u16, a),
        951 => radical_inverse_specialized(7517_u16, a),
        952 => radical_inverse_specialized(7523_u16, a),
        953 => radical_inverse_specialized(7529_u16, a),
        954 => radical_inverse_specialized(7537_u16, a),
        955 => radical_inverse_specialized(7541_u16, a),
        956 => radical_inverse_specialized(7547_u16, a),
        957 => radical_inverse_specialized(7549_u16, a),
        958 => radical_inverse_specialized(7559_u16, a),
        959 => radical_inverse_specialized(7561_u16, a),
        960 => radical_inverse_specialized(7573_u16, a),
        961 => radical_inverse_specialized(7577_u16, a),
        962 => radical_inverse_specialized(7583_u16, a),
        963 => radical_inverse_specialized(7589_u16, a),
        964 => radical_inverse_specialized(7591_u16, a),
        965 => radical_inverse_specialized(7603_u16, a),
        966 => radical_inverse_specialized(7607_u16, a),
        967 => radical_inverse_specialized(7621_u16, a),
        968 => radical_inverse_specialized(7639_u16, a),
        969 => radical_inverse_specialized(7643_u16, a),
        970 => radical_inverse_specialized(7649_u16, a),
        971 => radical_inverse_specialized(7669_u16, a),
        972 => radical_inverse_specialized(7673_u16, a),
        973 => radical_inverse_specialized(7681_u16, a),
        974 => radical_inverse_specialized(7687_u16, a),
        975 => radical_inverse_specialized(7691_u16, a),
        976 => radical_inverse_specialized(7699_u16, a),
        977 => radical_inverse_specialized(7703_u16, a),
        978 => radical_inverse_specialized(7717_u16, a),
        979 => radical_inverse_specialized(7723_u16, a),
        980 => radical_inverse_specialized(7727_u16, a),
        981 => radical_inverse_specialized(7741_u16, a),
        982 => radical_inverse_specialized(7753_u16, a),
        983 => radical_inverse_specialized(7757_u16, a),
        984 => radical_inverse_specialized(7759_u16, a),
        985 => radical_inverse_specialized(7789_u16, a),
        986 => radical_inverse_specialized(7793_u16, a),
        987 => radical_inverse_specialized(7817_u16, a),
        988 => radical_inverse_specialized(7823_u16, a),
        989 => radical_inverse_specialized(7829_u16, a),
        990 => radical_inverse_specialized(7841_u16, a),
        991 => radical_inverse_specialized(7853_u16, a),
        992 => radical_inverse_specialized(7867_u16, a),
        993 => radical_inverse_specialized(7873_u16, a),
        994 => radical_inverse_specialized(7877_u16, a),
        995 => radical_inverse_specialized(7879_u16, a),
        996 => radical_inverse_specialized(7883_u16, a),
        997 => radical_inverse_specialized(7901_u16, a),
        998 => radical_inverse_specialized(7907_u16, a),
        999 => radical_inverse_specialized(7919_u16, a),
        1000 => radical_inverse_specialized(7927_u16, a),
        1001 => radical_inverse_specialized(7933_u16, a),
        1002 => radical_inverse_specialized(7937_u16, a),
        1003 => radical_inverse_specialized(7949_u16, a),
        1004 => radical_inverse_specialized(7951_u16, a),
        1005 => radical_inverse_specialized(7963_u16, a),
        1006 => radical_inverse_specialized(7993_u16, a),
        1007 => radical_inverse_specialized(8009_u16, a),
        1008 => radical_inverse_specialized(8011_u16, a),
        1009 => radical_inverse_specialized(8017_u16, a),
        1010 => radical_inverse_specialized(8039_u16, a),
        1011 => radical_inverse_specialized(8053_u16, a),
        1012 => radical_inverse_specialized(8059_u16, a),
        1013 => radical_inverse_specialized(8069_u16, a),
        1014 => radical_inverse_specialized(8081_u16, a),
        1015 => radical_inverse_specialized(8087_u16, a),
        1016 => radical_inverse_specialized(8089_u16, a),
        1017 => radical_inverse_specialized(8093_u16, a),
        1018 => radical_inverse_specialized(8101_u16, a),
        1019 => radical_inverse_specialized(8111_u16, a),
        1020 => radical_inverse_specialized(8117_u16, a),
        1021 => radical_inverse_specialized(8123_u16, a),
        1022 => radical_inverse_specialized(8147_u16, a),
        1023 => radical_inverse_specialized(8161_u16, a),
        _ => {
            panic!("TODO: radical_inverse({:?}, {:?})", base_index, a);
        }
    }
}

/// Computes random permutation tables.
pub fn compute_radical_inverse_permutations(mut rng: &mut Rng) -> Vec<u16> {
    // allocate space in _perms_ for radical inverse permutations
    let mut perm_array_size: usize = 0_usize;
    for i in 0..PRIME_TABLE_SIZE {
        perm_array_size += PRIMES[i as usize] as usize;
    }
    let mut perms: Vec<u16> = vec![0_u16; perm_array_size];
    let mut p: usize = 0;
    for i in 0..PRIME_TABLE_SIZE {
        // generate random permutation for $i$th prime base
        for j in 0..PRIMES[i as usize] {
            perms[p + j as usize] = j as u16;
        }
        shuffle(
            &mut perms[p..(p + PRIMES[i as usize] as usize)],
            PRIMES[i as usize],
            1,
            &mut rng,
        );
        p += PRIMES[i as usize] as usize;
    }
    perms
}

/// Compute the radical inverse, but put each pixel through the
/// permutation table for the given base.
pub fn scrambled_radical_inverse(base_index: u16, a: u64, perm: &[u16]) -> Float {
    match base_index {
        0 => scrambled_radical_inverse_specialized(2_u16, perm, a),
        1 => scrambled_radical_inverse_specialized(3_u16, perm, a),
        2 => scrambled_radical_inverse_specialized(5_u16, perm, a),
        3 => scrambled_radical_inverse_specialized(7_u16, perm, a),
        4 => scrambled_radical_inverse_specialized(11_u16, perm, a),
        5 => scrambled_radical_inverse_specialized(13_u16, perm, a),
        6 => scrambled_radical_inverse_specialized(17_u16, perm, a),
        7 => scrambled_radical_inverse_specialized(19_u16, perm, a),
        8 => scrambled_radical_inverse_specialized(23_u16, perm, a),
        9 => scrambled_radical_inverse_specialized(29_u16, perm, a),
        10 => scrambled_radical_inverse_specialized(31_u16, perm, a),
        11 => scrambled_radical_inverse_specialized(37_u16, perm, a),

        12 => scrambled_radical_inverse_specialized(41_u16, perm, a),

        13 => scrambled_radical_inverse_specialized(43_u16, perm, a),

        14 => scrambled_radical_inverse_specialized(47_u16, perm, a),

        15 => scrambled_radical_inverse_specialized(53_u16, perm, a),

        16 => scrambled_radical_inverse_specialized(59_u16, perm, a),

        17 => scrambled_radical_inverse_specialized(61_u16, perm, a),

        18 => scrambled_radical_inverse_specialized(67_u16, perm, a),

        19 => scrambled_radical_inverse_specialized(71_u16, perm, a),

        20 => scrambled_radical_inverse_specialized(73_u16, perm, a),

        21 => scrambled_radical_inverse_specialized(79_u16, perm, a),

        22 => scrambled_radical_inverse_specialized(83_u16, perm, a),

        23 => scrambled_radical_inverse_specialized(89_u16, perm, a),

        24 => scrambled_radical_inverse_specialized(97_u16, perm, a),

        25 => scrambled_radical_inverse_specialized(101_u16, perm, a),

        26 => scrambled_radical_inverse_specialized(103_u16, perm, a),

        27 => scrambled_radical_inverse_specialized(107_u16, perm, a),

        28 => scrambled_radical_inverse_specialized(109_u16, perm, a),

        29 => scrambled_radical_inverse_specialized(113_u16, perm, a),

        30 => scrambled_radical_inverse_specialized(127_u16, perm, a),

        31 => scrambled_radical_inverse_specialized(131_u16, perm, a),

        32 => scrambled_radical_inverse_specialized(137_u16, perm, a),

        33 => scrambled_radical_inverse_specialized(139_u16, perm, a),

        34 => scrambled_radical_inverse_specialized(149_u16, perm, a),

        35 => scrambled_radical_inverse_specialized(151_u16, perm, a),

        36 => scrambled_radical_inverse_specialized(157_u16, perm, a),

        37 => scrambled_radical_inverse_specialized(163_u16, perm, a),

        38 => scrambled_radical_inverse_specialized(167_u16, perm, a),

        39 => scrambled_radical_inverse_specialized(173_u16, perm, a),

        40 => scrambled_radical_inverse_specialized(179_u16, perm, a),

        41 => scrambled_radical_inverse_specialized(181_u16, perm, a),

        42 => scrambled_radical_inverse_specialized(191_u16, perm, a),

        43 => scrambled_radical_inverse_specialized(193_u16, perm, a),

        44 => scrambled_radical_inverse_specialized(197_u16, perm, a),

        45 => scrambled_radical_inverse_specialized(199_u16, perm, a),

        46 => scrambled_radical_inverse_specialized(211_u16, perm, a),

        47 => scrambled_radical_inverse_specialized(223_u16, perm, a),

        48 => scrambled_radical_inverse_specialized(227_u16, perm, a),

        49 => scrambled_radical_inverse_specialized(229_u16, perm, a),

        50 => scrambled_radical_inverse_specialized(233_u16, perm, a),

        51 => scrambled_radical_inverse_specialized(239_u16, perm, a),

        52 => scrambled_radical_inverse_specialized(241_u16, perm, a),

        53 => scrambled_radical_inverse_specialized(251_u16, perm, a),

        54 => scrambled_radical_inverse_specialized(257_u16, perm, a),

        55 => scrambled_radical_inverse_specialized(263_u16, perm, a),

        56 => scrambled_radical_inverse_specialized(269_u16, perm, a),

        57 => scrambled_radical_inverse_specialized(271_u16, perm, a),

        58 => scrambled_radical_inverse_specialized(277_u16, perm, a),

        59 => scrambled_radical_inverse_specialized(281_u16, perm, a),

        60 => scrambled_radical_inverse_specialized(283_u16, perm, a),

        61 => scrambled_radical_inverse_specialized(293_u16, perm, a),

        62 => scrambled_radical_inverse_specialized(307_u16, perm, a),

        63 => scrambled_radical_inverse_specialized(311_u16, perm, a),

        64 => scrambled_radical_inverse_specialized(313_u16, perm, a),

        65 => scrambled_radical_inverse_specialized(317_u16, perm, a),

        66 => scrambled_radical_inverse_specialized(331_u16, perm, a),

        67 => scrambled_radical_inverse_specialized(337_u16, perm, a),

        68 => scrambled_radical_inverse_specialized(347_u16, perm, a),

        69 => scrambled_radical_inverse_specialized(349_u16, perm, a),

        70 => scrambled_radical_inverse_specialized(353_u16, perm, a),

        71 => scrambled_radical_inverse_specialized(359_u16, perm, a),

        72 => scrambled_radical_inverse_specialized(367_u16, perm, a),

        73 => scrambled_radical_inverse_specialized(373_u16, perm, a),

        74 => scrambled_radical_inverse_specialized(379_u16, perm, a),

        75 => scrambled_radical_inverse_specialized(383_u16, perm, a),

        76 => scrambled_radical_inverse_specialized(389_u16, perm, a),

        77 => scrambled_radical_inverse_specialized(397_u16, perm, a),

        78 => scrambled_radical_inverse_specialized(401_u16, perm, a),

        79 => scrambled_radical_inverse_specialized(409_u16, perm, a),

        80 => scrambled_radical_inverse_specialized(419_u16, perm, a),

        81 => scrambled_radical_inverse_specialized(421_u16, perm, a),

        82 => scrambled_radical_inverse_specialized(431_u16, perm, a),

        83 => scrambled_radical_inverse_specialized(433_u16, perm, a),

        84 => scrambled_radical_inverse_specialized(439_u16, perm, a),

        85 => scrambled_radical_inverse_specialized(443_u16, perm, a),

        86 => scrambled_radical_inverse_specialized(449_u16, perm, a),

        87 => scrambled_radical_inverse_specialized(457_u16, perm, a),

        88 => scrambled_radical_inverse_specialized(461_u16, perm, a),

        89 => scrambled_radical_inverse_specialized(463_u16, perm, a),

        90 => scrambled_radical_inverse_specialized(467_u16, perm, a),

        91 => scrambled_radical_inverse_specialized(479_u16, perm, a),

        92 => scrambled_radical_inverse_specialized(487_u16, perm, a),

        93 => scrambled_radical_inverse_specialized(491_u16, perm, a),

        94 => scrambled_radical_inverse_specialized(499_u16, perm, a),

        95 => scrambled_radical_inverse_specialized(503_u16, perm, a),

        96 => scrambled_radical_inverse_specialized(509_u16, perm, a),

        97 => scrambled_radical_inverse_specialized(521_u16, perm, a),

        98 => scrambled_radical_inverse_specialized(523_u16, perm, a),

        99 => scrambled_radical_inverse_specialized(541_u16, perm, a),

        100 => scrambled_radical_inverse_specialized(547_u16, perm, a),

        101 => scrambled_radical_inverse_specialized(557_u16, perm, a),

        102 => scrambled_radical_inverse_specialized(563_u16, perm, a),

        103 => scrambled_radical_inverse_specialized(569_u16, perm, a),

        104 => scrambled_radical_inverse_specialized(571_u16, perm, a),

        105 => scrambled_radical_inverse_specialized(577_u16, perm, a),

        106 => scrambled_radical_inverse_specialized(587_u16, perm, a),

        107 => scrambled_radical_inverse_specialized(593_u16, perm, a),

        108 => scrambled_radical_inverse_specialized(599_u16, perm, a),

        109 => scrambled_radical_inverse_specialized(601_u16, perm, a),

        110 => scrambled_radical_inverse_specialized(607_u16, perm, a),

        111 => scrambled_radical_inverse_specialized(613_u16, perm, a),

        112 => scrambled_radical_inverse_specialized(617_u16, perm, a),

        113 => scrambled_radical_inverse_specialized(619_u16, perm, a),

        114 => scrambled_radical_inverse_specialized(631_u16, perm, a),

        115 => scrambled_radical_inverse_specialized(641_u16, perm, a),

        116 => scrambled_radical_inverse_specialized(643_u16, perm, a),

        117 => scrambled_radical_inverse_specialized(647_u16, perm, a),

        118 => scrambled_radical_inverse_specialized(653_u16, perm, a),

        119 => scrambled_radical_inverse_specialized(659_u16, perm, a),

        120 => scrambled_radical_inverse_specialized(661_u16, perm, a),

        121 => scrambled_radical_inverse_specialized(673_u16, perm, a),

        122 => scrambled_radical_inverse_specialized(677_u16, perm, a),

        123 => scrambled_radical_inverse_specialized(683_u16, perm, a),

        124 => scrambled_radical_inverse_specialized(691_u16, perm, a),

        125 => scrambled_radical_inverse_specialized(701_u16, perm, a),

        126 => scrambled_radical_inverse_specialized(709_u16, perm, a),

        127 => scrambled_radical_inverse_specialized(719_u16, perm, a),

        128 => scrambled_radical_inverse_specialized(727_u16, perm, a),

        129 => scrambled_radical_inverse_specialized(733_u16, perm, a),

        130 => scrambled_radical_inverse_specialized(739_u16, perm, a),

        131 => scrambled_radical_inverse_specialized(743_u16, perm, a),

        132 => scrambled_radical_inverse_specialized(751_u16, perm, a),

        133 => scrambled_radical_inverse_specialized(757_u16, perm, a),

        134 => scrambled_radical_inverse_specialized(761_u16, perm, a),

        135 => scrambled_radical_inverse_specialized(769_u16, perm, a),

        136 => scrambled_radical_inverse_specialized(773_u16, perm, a),

        137 => scrambled_radical_inverse_specialized(787_u16, perm, a),

        138 => scrambled_radical_inverse_specialized(797_u16, perm, a),

        139 => scrambled_radical_inverse_specialized(809_u16, perm, a),

        140 => scrambled_radical_inverse_specialized(811_u16, perm, a),

        141 => scrambled_radical_inverse_specialized(821_u16, perm, a),

        142 => scrambled_radical_inverse_specialized(823_u16, perm, a),

        143 => scrambled_radical_inverse_specialized(827_u16, perm, a),

        144 => scrambled_radical_inverse_specialized(829_u16, perm, a),

        145 => scrambled_radical_inverse_specialized(839_u16, perm, a),

        146 => scrambled_radical_inverse_specialized(853_u16, perm, a),

        147 => scrambled_radical_inverse_specialized(857_u16, perm, a),

        148 => scrambled_radical_inverse_specialized(859_u16, perm, a),

        149 => scrambled_radical_inverse_specialized(863_u16, perm, a),

        150 => scrambled_radical_inverse_specialized(877_u16, perm, a),

        151 => scrambled_radical_inverse_specialized(881_u16, perm, a),

        152 => scrambled_radical_inverse_specialized(883_u16, perm, a),

        153 => scrambled_radical_inverse_specialized(887_u16, perm, a),

        154 => scrambled_radical_inverse_specialized(907_u16, perm, a),

        155 => scrambled_radical_inverse_specialized(911_u16, perm, a),

        156 => scrambled_radical_inverse_specialized(919_u16, perm, a),

        157 => scrambled_radical_inverse_specialized(929_u16, perm, a),

        158 => scrambled_radical_inverse_specialized(937_u16, perm, a),

        159 => scrambled_radical_inverse_specialized(941_u16, perm, a),

        160 => scrambled_radical_inverse_specialized(947_u16, perm, a),

        161 => scrambled_radical_inverse_specialized(953_u16, perm, a),

        162 => scrambled_radical_inverse_specialized(967_u16, perm, a),

        163 => scrambled_radical_inverse_specialized(971_u16, perm, a),

        164 => scrambled_radical_inverse_specialized(977_u16, perm, a),

        165 => scrambled_radical_inverse_specialized(983_u16, perm, a),

        166 => scrambled_radical_inverse_specialized(991_u16, perm, a),

        167 => scrambled_radical_inverse_specialized(997_u16, perm, a),

        168 => scrambled_radical_inverse_specialized(1009_u16, perm, a),

        169 => scrambled_radical_inverse_specialized(1013_u16, perm, a),

        170 => scrambled_radical_inverse_specialized(1019_u16, perm, a),

        171 => scrambled_radical_inverse_specialized(1021_u16, perm, a),

        172 => scrambled_radical_inverse_specialized(1031_u16, perm, a),

        173 => scrambled_radical_inverse_specialized(1033_u16, perm, a),

        174 => scrambled_radical_inverse_specialized(1039_u16, perm, a),

        175 => scrambled_radical_inverse_specialized(1049_u16, perm, a),

        176 => scrambled_radical_inverse_specialized(1051_u16, perm, a),

        177 => scrambled_radical_inverse_specialized(1061_u16, perm, a),

        178 => scrambled_radical_inverse_specialized(1063_u16, perm, a),

        179 => scrambled_radical_inverse_specialized(1069_u16, perm, a),

        180 => scrambled_radical_inverse_specialized(1087_u16, perm, a),

        181 => scrambled_radical_inverse_specialized(1091_u16, perm, a),

        182 => scrambled_radical_inverse_specialized(1093_u16, perm, a),

        183 => scrambled_radical_inverse_specialized(1097_u16, perm, a),

        184 => scrambled_radical_inverse_specialized(1103_u16, perm, a),

        185 => scrambled_radical_inverse_specialized(1109_u16, perm, a),

        186 => scrambled_radical_inverse_specialized(1117_u16, perm, a),

        187 => scrambled_radical_inverse_specialized(1123_u16, perm, a),

        188 => scrambled_radical_inverse_specialized(1129_u16, perm, a),

        189 => scrambled_radical_inverse_specialized(1151_u16, perm, a),

        190 => scrambled_radical_inverse_specialized(1153_u16, perm, a),

        191 => scrambled_radical_inverse_specialized(1163_u16, perm, a),

        192 => scrambled_radical_inverse_specialized(1171_u16, perm, a),

        193 => scrambled_radical_inverse_specialized(1181_u16, perm, a),

        194 => scrambled_radical_inverse_specialized(1187_u16, perm, a),

        195 => scrambled_radical_inverse_specialized(1193_u16, perm, a),

        196 => scrambled_radical_inverse_specialized(1201_u16, perm, a),

        197 => scrambled_radical_inverse_specialized(1213_u16, perm, a),

        198 => scrambled_radical_inverse_specialized(1217_u16, perm, a),

        199 => scrambled_radical_inverse_specialized(1223_u16, perm, a),

        200 => scrambled_radical_inverse_specialized(1229_u16, perm, a),

        201 => scrambled_radical_inverse_specialized(1231_u16, perm, a),

        202 => scrambled_radical_inverse_specialized(1237_u16, perm, a),

        203 => scrambled_radical_inverse_specialized(1249_u16, perm, a),

        204 => scrambled_radical_inverse_specialized(1259_u16, perm, a),

        205 => scrambled_radical_inverse_specialized(1277_u16, perm, a),

        206 => scrambled_radical_inverse_specialized(1279_u16, perm, a),

        207 => scrambled_radical_inverse_specialized(1283_u16, perm, a),

        208 => scrambled_radical_inverse_specialized(1289_u16, perm, a),

        209 => scrambled_radical_inverse_specialized(1291_u16, perm, a),

        210 => scrambled_radical_inverse_specialized(1297_u16, perm, a),

        211 => scrambled_radical_inverse_specialized(1301_u16, perm, a),

        212 => scrambled_radical_inverse_specialized(1303_u16, perm, a),

        213 => scrambled_radical_inverse_specialized(1307_u16, perm, a),

        214 => scrambled_radical_inverse_specialized(1319_u16, perm, a),

        215 => scrambled_radical_inverse_specialized(1321_u16, perm, a),

        216 => scrambled_radical_inverse_specialized(1327_u16, perm, a),

        217 => scrambled_radical_inverse_specialized(1361_u16, perm, a),

        218 => scrambled_radical_inverse_specialized(1367_u16, perm, a),

        219 => scrambled_radical_inverse_specialized(1373_u16, perm, a),

        220 => scrambled_radical_inverse_specialized(1381_u16, perm, a),

        221 => scrambled_radical_inverse_specialized(1399_u16, perm, a),

        222 => scrambled_radical_inverse_specialized(1409_u16, perm, a),

        223 => scrambled_radical_inverse_specialized(1423_u16, perm, a),

        224 => scrambled_radical_inverse_specialized(1427_u16, perm, a),

        225 => scrambled_radical_inverse_specialized(1429_u16, perm, a),

        226 => scrambled_radical_inverse_specialized(1433_u16, perm, a),

        227 => scrambled_radical_inverse_specialized(1439_u16, perm, a),

        228 => scrambled_radical_inverse_specialized(1447_u16, perm, a),

        229 => scrambled_radical_inverse_specialized(1451_u16, perm, a),

        230 => scrambled_radical_inverse_specialized(1453_u16, perm, a),

        231 => scrambled_radical_inverse_specialized(1459_u16, perm, a),

        232 => scrambled_radical_inverse_specialized(1471_u16, perm, a),

        233 => scrambled_radical_inverse_specialized(1481_u16, perm, a),

        234 => scrambled_radical_inverse_specialized(1483_u16, perm, a),

        235 => scrambled_radical_inverse_specialized(1487_u16, perm, a),

        236 => scrambled_radical_inverse_specialized(1489_u16, perm, a),

        237 => scrambled_radical_inverse_specialized(1493_u16, perm, a),

        238 => scrambled_radical_inverse_specialized(1499_u16, perm, a),

        239 => scrambled_radical_inverse_specialized(1511_u16, perm, a),

        240 => scrambled_radical_inverse_specialized(1523_u16, perm, a),

        241 => scrambled_radical_inverse_specialized(1531_u16, perm, a),

        242 => scrambled_radical_inverse_specialized(1543_u16, perm, a),

        243 => scrambled_radical_inverse_specialized(1549_u16, perm, a),

        244 => scrambled_radical_inverse_specialized(1553_u16, perm, a),

        245 => scrambled_radical_inverse_specialized(1559_u16, perm, a),

        246 => scrambled_radical_inverse_specialized(1567_u16, perm, a),

        247 => scrambled_radical_inverse_specialized(1571_u16, perm, a),

        248 => scrambled_radical_inverse_specialized(1579_u16, perm, a),

        249 => scrambled_radical_inverse_specialized(1583_u16, perm, a),

        250 => scrambled_radical_inverse_specialized(1597_u16, perm, a),

        251 => scrambled_radical_inverse_specialized(1601_u16, perm, a),

        252 => scrambled_radical_inverse_specialized(1607_u16, perm, a),

        253 => scrambled_radical_inverse_specialized(1609_u16, perm, a),

        254 => scrambled_radical_inverse_specialized(1613_u16, perm, a),

        255 => scrambled_radical_inverse_specialized(1619_u16, perm, a),

        256 => scrambled_radical_inverse_specialized(1621_u16, perm, a),

        257 => scrambled_radical_inverse_specialized(1627_u16, perm, a),

        258 => scrambled_radical_inverse_specialized(1637_u16, perm, a),

        259 => scrambled_radical_inverse_specialized(1657_u16, perm, a),

        260 => scrambled_radical_inverse_specialized(1663_u16, perm, a),

        261 => scrambled_radical_inverse_specialized(1667_u16, perm, a),

        262 => scrambled_radical_inverse_specialized(1669_u16, perm, a),

        263 => scrambled_radical_inverse_specialized(1693_u16, perm, a),

        264 => scrambled_radical_inverse_specialized(1697_u16, perm, a),

        265 => scrambled_radical_inverse_specialized(1699_u16, perm, a),

        266 => scrambled_radical_inverse_specialized(1709_u16, perm, a),

        267 => scrambled_radical_inverse_specialized(1721_u16, perm, a),

        268 => scrambled_radical_inverse_specialized(1723_u16, perm, a),

        269 => scrambled_radical_inverse_specialized(1733_u16, perm, a),

        270 => scrambled_radical_inverse_specialized(1741_u16, perm, a),

        271 => scrambled_radical_inverse_specialized(1747_u16, perm, a),

        272 => scrambled_radical_inverse_specialized(1753_u16, perm, a),

        273 => scrambled_radical_inverse_specialized(1759_u16, perm, a),

        274 => scrambled_radical_inverse_specialized(1777_u16, perm, a),

        275 => scrambled_radical_inverse_specialized(1783_u16, perm, a),

        276 => scrambled_radical_inverse_specialized(1787_u16, perm, a),

        277 => scrambled_radical_inverse_specialized(1789_u16, perm, a),

        278 => scrambled_radical_inverse_specialized(1801_u16, perm, a),

        279 => scrambled_radical_inverse_specialized(1811_u16, perm, a),

        280 => scrambled_radical_inverse_specialized(1823_u16, perm, a),

        281 => scrambled_radical_inverse_specialized(1831_u16, perm, a),

        282 => scrambled_radical_inverse_specialized(1847_u16, perm, a),

        283 => scrambled_radical_inverse_specialized(1861_u16, perm, a),

        284 => scrambled_radical_inverse_specialized(1867_u16, perm, a),

        285 => scrambled_radical_inverse_specialized(1871_u16, perm, a),

        286 => scrambled_radical_inverse_specialized(1873_u16, perm, a),

        287 => scrambled_radical_inverse_specialized(1877_u16, perm, a),

        288 => scrambled_radical_inverse_specialized(1879_u16, perm, a),

        289 => scrambled_radical_inverse_specialized(1889_u16, perm, a),

        290 => scrambled_radical_inverse_specialized(1901_u16, perm, a),

        291 => scrambled_radical_inverse_specialized(1907_u16, perm, a),

        292 => scrambled_radical_inverse_specialized(1913_u16, perm, a),

        293 => scrambled_radical_inverse_specialized(1931_u16, perm, a),

        294 => scrambled_radical_inverse_specialized(1933_u16, perm, a),

        295 => scrambled_radical_inverse_specialized(1949_u16, perm, a),

        296 => scrambled_radical_inverse_specialized(1951_u16, perm, a),

        297 => scrambled_radical_inverse_specialized(1973_u16, perm, a),

        298 => scrambled_radical_inverse_specialized(1979_u16, perm, a),

        299 => scrambled_radical_inverse_specialized(1987_u16, perm, a),

        300 => scrambled_radical_inverse_specialized(1993_u16, perm, a),

        301 => scrambled_radical_inverse_specialized(1997_u16, perm, a),

        302 => scrambled_radical_inverse_specialized(1999_u16, perm, a),

        303 => scrambled_radical_inverse_specialized(2003_u16, perm, a),

        304 => scrambled_radical_inverse_specialized(2011_u16, perm, a),

        305 => scrambled_radical_inverse_specialized(2017_u16, perm, a),

        306 => scrambled_radical_inverse_specialized(2027_u16, perm, a),

        307 => scrambled_radical_inverse_specialized(2029_u16, perm, a),

        308 => scrambled_radical_inverse_specialized(2039_u16, perm, a),

        309 => scrambled_radical_inverse_specialized(2053_u16, perm, a),

        310 => scrambled_radical_inverse_specialized(2063_u16, perm, a),

        311 => scrambled_radical_inverse_specialized(2069_u16, perm, a),

        312 => scrambled_radical_inverse_specialized(2081_u16, perm, a),

        313 => scrambled_radical_inverse_specialized(2083_u16, perm, a),

        314 => scrambled_radical_inverse_specialized(2087_u16, perm, a),

        315 => scrambled_radical_inverse_specialized(2089_u16, perm, a),

        316 => scrambled_radical_inverse_specialized(2099_u16, perm, a),

        317 => scrambled_radical_inverse_specialized(2111_u16, perm, a),

        318 => scrambled_radical_inverse_specialized(2113_u16, perm, a),

        319 => scrambled_radical_inverse_specialized(2129_u16, perm, a),

        320 => scrambled_radical_inverse_specialized(2131_u16, perm, a),

        321 => scrambled_radical_inverse_specialized(2137_u16, perm, a),

        322 => scrambled_radical_inverse_specialized(2141_u16, perm, a),

        323 => scrambled_radical_inverse_specialized(2143_u16, perm, a),

        324 => scrambled_radical_inverse_specialized(2153_u16, perm, a),

        325 => scrambled_radical_inverse_specialized(2161_u16, perm, a),

        326 => scrambled_radical_inverse_specialized(2179_u16, perm, a),

        327 => scrambled_radical_inverse_specialized(2203_u16, perm, a),

        328 => scrambled_radical_inverse_specialized(2207_u16, perm, a),

        329 => scrambled_radical_inverse_specialized(2213_u16, perm, a),

        330 => scrambled_radical_inverse_specialized(2221_u16, perm, a),

        331 => scrambled_radical_inverse_specialized(2237_u16, perm, a),

        332 => scrambled_radical_inverse_specialized(2239_u16, perm, a),

        333 => scrambled_radical_inverse_specialized(2243_u16, perm, a),

        334 => scrambled_radical_inverse_specialized(2251_u16, perm, a),

        335 => scrambled_radical_inverse_specialized(2267_u16, perm, a),

        336 => scrambled_radical_inverse_specialized(2269_u16, perm, a),

        337 => scrambled_radical_inverse_specialized(2273_u16, perm, a),

        338 => scrambled_radical_inverse_specialized(2281_u16, perm, a),

        339 => scrambled_radical_inverse_specialized(2287_u16, perm, a),

        340 => scrambled_radical_inverse_specialized(2293_u16, perm, a),

        341 => scrambled_radical_inverse_specialized(2297_u16, perm, a),

        342 => scrambled_radical_inverse_specialized(2309_u16, perm, a),

        343 => scrambled_radical_inverse_specialized(2311_u16, perm, a),

        344 => scrambled_radical_inverse_specialized(2333_u16, perm, a),

        345 => scrambled_radical_inverse_specialized(2339_u16, perm, a),

        346 => scrambled_radical_inverse_specialized(2341_u16, perm, a),

        347 => scrambled_radical_inverse_specialized(2347_u16, perm, a),

        348 => scrambled_radical_inverse_specialized(2351_u16, perm, a),

        349 => scrambled_radical_inverse_specialized(2357_u16, perm, a),

        350 => scrambled_radical_inverse_specialized(2371_u16, perm, a),

        351 => scrambled_radical_inverse_specialized(2377_u16, perm, a),

        352 => scrambled_radical_inverse_specialized(2381_u16, perm, a),

        353 => scrambled_radical_inverse_specialized(2383_u16, perm, a),

        354 => scrambled_radical_inverse_specialized(2389_u16, perm, a),

        355 => scrambled_radical_inverse_specialized(2393_u16, perm, a),

        356 => scrambled_radical_inverse_specialized(2399_u16, perm, a),

        357 => scrambled_radical_inverse_specialized(2411_u16, perm, a),

        358 => scrambled_radical_inverse_specialized(2417_u16, perm, a),

        359 => scrambled_radical_inverse_specialized(2423_u16, perm, a),

        360 => scrambled_radical_inverse_specialized(2437_u16, perm, a),

        361 => scrambled_radical_inverse_specialized(2441_u16, perm, a),

        362 => scrambled_radical_inverse_specialized(2447_u16, perm, a),

        363 => scrambled_radical_inverse_specialized(2459_u16, perm, a),

        364 => scrambled_radical_inverse_specialized(2467_u16, perm, a),

        365 => scrambled_radical_inverse_specialized(2473_u16, perm, a),

        366 => scrambled_radical_inverse_specialized(2477_u16, perm, a),

        367 => scrambled_radical_inverse_specialized(2503_u16, perm, a),

        368 => scrambled_radical_inverse_specialized(2521_u16, perm, a),

        369 => scrambled_radical_inverse_specialized(2531_u16, perm, a),

        370 => scrambled_radical_inverse_specialized(2539_u16, perm, a),

        371 => scrambled_radical_inverse_specialized(2543_u16, perm, a),

        372 => scrambled_radical_inverse_specialized(2549_u16, perm, a),

        373 => scrambled_radical_inverse_specialized(2551_u16, perm, a),

        374 => scrambled_radical_inverse_specialized(2557_u16, perm, a),

        375 => scrambled_radical_inverse_specialized(2579_u16, perm, a),

        376 => scrambled_radical_inverse_specialized(2591_u16, perm, a),

        377 => scrambled_radical_inverse_specialized(2593_u16, perm, a),

        378 => scrambled_radical_inverse_specialized(2609_u16, perm, a),

        379 => scrambled_radical_inverse_specialized(2617_u16, perm, a),

        380 => scrambled_radical_inverse_specialized(2621_u16, perm, a),

        381 => scrambled_radical_inverse_specialized(2633_u16, perm, a),

        382 => scrambled_radical_inverse_specialized(2647_u16, perm, a),

        383 => scrambled_radical_inverse_specialized(2657_u16, perm, a),

        384 => scrambled_radical_inverse_specialized(2659_u16, perm, a),

        385 => scrambled_radical_inverse_specialized(2663_u16, perm, a),

        386 => scrambled_radical_inverse_specialized(2671_u16, perm, a),

        387 => scrambled_radical_inverse_specialized(2677_u16, perm, a),

        388 => scrambled_radical_inverse_specialized(2683_u16, perm, a),

        389 => scrambled_radical_inverse_specialized(2687_u16, perm, a),

        390 => scrambled_radical_inverse_specialized(2689_u16, perm, a),

        391 => scrambled_radical_inverse_specialized(2693_u16, perm, a),

        392 => scrambled_radical_inverse_specialized(2699_u16, perm, a),

        393 => scrambled_radical_inverse_specialized(2707_u16, perm, a),

        394 => scrambled_radical_inverse_specialized(2711_u16, perm, a),

        395 => scrambled_radical_inverse_specialized(2713_u16, perm, a),

        396 => scrambled_radical_inverse_specialized(2719_u16, perm, a),

        397 => scrambled_radical_inverse_specialized(2729_u16, perm, a),

        398 => scrambled_radical_inverse_specialized(2731_u16, perm, a),

        399 => scrambled_radical_inverse_specialized(2741_u16, perm, a),

        400 => scrambled_radical_inverse_specialized(2749_u16, perm, a),

        401 => scrambled_radical_inverse_specialized(2753_u16, perm, a),

        402 => scrambled_radical_inverse_specialized(2767_u16, perm, a),

        403 => scrambled_radical_inverse_specialized(2777_u16, perm, a),

        404 => scrambled_radical_inverse_specialized(2789_u16, perm, a),

        405 => scrambled_radical_inverse_specialized(2791_u16, perm, a),

        406 => scrambled_radical_inverse_specialized(2797_u16, perm, a),

        407 => scrambled_radical_inverse_specialized(2801_u16, perm, a),

        408 => scrambled_radical_inverse_specialized(2803_u16, perm, a),

        409 => scrambled_radical_inverse_specialized(2819_u16, perm, a),

        410 => scrambled_radical_inverse_specialized(2833_u16, perm, a),

        411 => scrambled_radical_inverse_specialized(2837_u16, perm, a),

        412 => scrambled_radical_inverse_specialized(2843_u16, perm, a),

        413 => scrambled_radical_inverse_specialized(2851_u16, perm, a),

        414 => scrambled_radical_inverse_specialized(2857_u16, perm, a),

        415 => scrambled_radical_inverse_specialized(2861_u16, perm, a),

        416 => scrambled_radical_inverse_specialized(2879_u16, perm, a),

        417 => scrambled_radical_inverse_specialized(2887_u16, perm, a),

        418 => scrambled_radical_inverse_specialized(2897_u16, perm, a),

        419 => scrambled_radical_inverse_specialized(2903_u16, perm, a),

        420 => scrambled_radical_inverse_specialized(2909_u16, perm, a),

        421 => scrambled_radical_inverse_specialized(2917_u16, perm, a),

        422 => scrambled_radical_inverse_specialized(2927_u16, perm, a),

        423 => scrambled_radical_inverse_specialized(2939_u16, perm, a),

        424 => scrambled_radical_inverse_specialized(2953_u16, perm, a),

        425 => scrambled_radical_inverse_specialized(2957_u16, perm, a),

        426 => scrambled_radical_inverse_specialized(2963_u16, perm, a),

        427 => scrambled_radical_inverse_specialized(2969_u16, perm, a),

        428 => scrambled_radical_inverse_specialized(2971_u16, perm, a),

        429 => scrambled_radical_inverse_specialized(2999_u16, perm, a),

        430 => scrambled_radical_inverse_specialized(3001_u16, perm, a),

        431 => scrambled_radical_inverse_specialized(3011_u16, perm, a),

        432 => scrambled_radical_inverse_specialized(3019_u16, perm, a),

        433 => scrambled_radical_inverse_specialized(3023_u16, perm, a),

        434 => scrambled_radical_inverse_specialized(3037_u16, perm, a),

        435 => scrambled_radical_inverse_specialized(3041_u16, perm, a),

        436 => scrambled_radical_inverse_specialized(3049_u16, perm, a),

        437 => scrambled_radical_inverse_specialized(3061_u16, perm, a),

        438 => scrambled_radical_inverse_specialized(3067_u16, perm, a),

        439 => scrambled_radical_inverse_specialized(3079_u16, perm, a),

        440 => scrambled_radical_inverse_specialized(3083_u16, perm, a),

        441 => scrambled_radical_inverse_specialized(3089_u16, perm, a),

        442 => scrambled_radical_inverse_specialized(3109_u16, perm, a),

        443 => scrambled_radical_inverse_specialized(3119_u16, perm, a),

        444 => scrambled_radical_inverse_specialized(3121_u16, perm, a),

        445 => scrambled_radical_inverse_specialized(3137_u16, perm, a),

        446 => scrambled_radical_inverse_specialized(3163_u16, perm, a),

        447 => scrambled_radical_inverse_specialized(3167_u16, perm, a),

        448 => scrambled_radical_inverse_specialized(3169_u16, perm, a),

        449 => scrambled_radical_inverse_specialized(3181_u16, perm, a),

        450 => scrambled_radical_inverse_specialized(3187_u16, perm, a),

        451 => scrambled_radical_inverse_specialized(3191_u16, perm, a),

        452 => scrambled_radical_inverse_specialized(3203_u16, perm, a),

        453 => scrambled_radical_inverse_specialized(3209_u16, perm, a),

        454 => scrambled_radical_inverse_specialized(3217_u16, perm, a),

        455 => scrambled_radical_inverse_specialized(3221_u16, perm, a),

        456 => scrambled_radical_inverse_specialized(3229_u16, perm, a),

        457 => scrambled_radical_inverse_specialized(3251_u16, perm, a),

        458 => scrambled_radical_inverse_specialized(3253_u16, perm, a),

        459 => scrambled_radical_inverse_specialized(3257_u16, perm, a),

        460 => scrambled_radical_inverse_specialized(3259_u16, perm, a),

        461 => scrambled_radical_inverse_specialized(3271_u16, perm, a),

        462 => scrambled_radical_inverse_specialized(3299_u16, perm, a),

        463 => scrambled_radical_inverse_specialized(3301_u16, perm, a),

        464 => scrambled_radical_inverse_specialized(3307_u16, perm, a),

        465 => scrambled_radical_inverse_specialized(3313_u16, perm, a),

        466 => scrambled_radical_inverse_specialized(3319_u16, perm, a),

        467 => scrambled_radical_inverse_specialized(3323_u16, perm, a),

        468 => scrambled_radical_inverse_specialized(3329_u16, perm, a),

        469 => scrambled_radical_inverse_specialized(3331_u16, perm, a),

        470 => scrambled_radical_inverse_specialized(3343_u16, perm, a),

        471 => scrambled_radical_inverse_specialized(3347_u16, perm, a),

        472 => scrambled_radical_inverse_specialized(3359_u16, perm, a),

        473 => scrambled_radical_inverse_specialized(3361_u16, perm, a),

        474 => scrambled_radical_inverse_specialized(3371_u16, perm, a),

        475 => scrambled_radical_inverse_specialized(3373_u16, perm, a),

        476 => scrambled_radical_inverse_specialized(3389_u16, perm, a),

        477 => scrambled_radical_inverse_specialized(3391_u16, perm, a),

        478 => scrambled_radical_inverse_specialized(3407_u16, perm, a),

        479 => scrambled_radical_inverse_specialized(3413_u16, perm, a),

        480 => scrambled_radical_inverse_specialized(3433_u16, perm, a),

        481 => scrambled_radical_inverse_specialized(3449_u16, perm, a),

        482 => scrambled_radical_inverse_specialized(3457_u16, perm, a),

        483 => scrambled_radical_inverse_specialized(3461_u16, perm, a),

        484 => scrambled_radical_inverse_specialized(3463_u16, perm, a),

        485 => scrambled_radical_inverse_specialized(3467_u16, perm, a),

        486 => scrambled_radical_inverse_specialized(3469_u16, perm, a),

        487 => scrambled_radical_inverse_specialized(3491_u16, perm, a),

        488 => scrambled_radical_inverse_specialized(3499_u16, perm, a),

        489 => scrambled_radical_inverse_specialized(3511_u16, perm, a),

        490 => scrambled_radical_inverse_specialized(3517_u16, perm, a),

        491 => scrambled_radical_inverse_specialized(3527_u16, perm, a),

        492 => scrambled_radical_inverse_specialized(3529_u16, perm, a),

        493 => scrambled_radical_inverse_specialized(3533_u16, perm, a),

        494 => scrambled_radical_inverse_specialized(3539_u16, perm, a),

        495 => scrambled_radical_inverse_specialized(3541_u16, perm, a),

        496 => scrambled_radical_inverse_specialized(3547_u16, perm, a),

        497 => scrambled_radical_inverse_specialized(3557_u16, perm, a),

        498 => scrambled_radical_inverse_specialized(3559_u16, perm, a),

        499 => scrambled_radical_inverse_specialized(3571_u16, perm, a),

        500 => scrambled_radical_inverse_specialized(3581_u16, perm, a),

        501 => scrambled_radical_inverse_specialized(3583_u16, perm, a),

        502 => scrambled_radical_inverse_specialized(3593_u16, perm, a),

        503 => scrambled_radical_inverse_specialized(3607_u16, perm, a),

        504 => scrambled_radical_inverse_specialized(3613_u16, perm, a),

        505 => scrambled_radical_inverse_specialized(3617_u16, perm, a),

        506 => scrambled_radical_inverse_specialized(3623_u16, perm, a),

        507 => scrambled_radical_inverse_specialized(3631_u16, perm, a),

        508 => scrambled_radical_inverse_specialized(3637_u16, perm, a),

        509 => scrambled_radical_inverse_specialized(3643_u16, perm, a),

        510 => scrambled_radical_inverse_specialized(3659_u16, perm, a),

        511 => scrambled_radical_inverse_specialized(3671_u16, perm, a),

        512 => scrambled_radical_inverse_specialized(3673_u16, perm, a),

        513 => scrambled_radical_inverse_specialized(3677_u16, perm, a),

        514 => scrambled_radical_inverse_specialized(3691_u16, perm, a),

        515 => scrambled_radical_inverse_specialized(3697_u16, perm, a),

        516 => scrambled_radical_inverse_specialized(3701_u16, perm, a),

        517 => scrambled_radical_inverse_specialized(3709_u16, perm, a),

        518 => scrambled_radical_inverse_specialized(3719_u16, perm, a),

        519 => scrambled_radical_inverse_specialized(3727_u16, perm, a),

        520 => scrambled_radical_inverse_specialized(3733_u16, perm, a),

        521 => scrambled_radical_inverse_specialized(3739_u16, perm, a),

        522 => scrambled_radical_inverse_specialized(3761_u16, perm, a),

        523 => scrambled_radical_inverse_specialized(3767_u16, perm, a),

        524 => scrambled_radical_inverse_specialized(3769_u16, perm, a),

        525 => scrambled_radical_inverse_specialized(3779_u16, perm, a),

        526 => scrambled_radical_inverse_specialized(3793_u16, perm, a),

        527 => scrambled_radical_inverse_specialized(3797_u16, perm, a),

        528 => scrambled_radical_inverse_specialized(3803_u16, perm, a),

        529 => scrambled_radical_inverse_specialized(3821_u16, perm, a),

        530 => scrambled_radical_inverse_specialized(3823_u16, perm, a),

        531 => scrambled_radical_inverse_specialized(3833_u16, perm, a),

        532 => scrambled_radical_inverse_specialized(3847_u16, perm, a),

        533 => scrambled_radical_inverse_specialized(3851_u16, perm, a),

        534 => scrambled_radical_inverse_specialized(3853_u16, perm, a),

        535 => scrambled_radical_inverse_specialized(3863_u16, perm, a),

        536 => scrambled_radical_inverse_specialized(3877_u16, perm, a),

        537 => scrambled_radical_inverse_specialized(3881_u16, perm, a),

        538 => scrambled_radical_inverse_specialized(3889_u16, perm, a),

        539 => scrambled_radical_inverse_specialized(3907_u16, perm, a),

        540 => scrambled_radical_inverse_specialized(3911_u16, perm, a),

        541 => scrambled_radical_inverse_specialized(3917_u16, perm, a),

        542 => scrambled_radical_inverse_specialized(3919_u16, perm, a),

        543 => scrambled_radical_inverse_specialized(3923_u16, perm, a),

        544 => scrambled_radical_inverse_specialized(3929_u16, perm, a),

        545 => scrambled_radical_inverse_specialized(3931_u16, perm, a),

        546 => scrambled_radical_inverse_specialized(3943_u16, perm, a),

        547 => scrambled_radical_inverse_specialized(3947_u16, perm, a),

        548 => scrambled_radical_inverse_specialized(3967_u16, perm, a),

        549 => scrambled_radical_inverse_specialized(3989_u16, perm, a),

        550 => scrambled_radical_inverse_specialized(4001_u16, perm, a),

        551 => scrambled_radical_inverse_specialized(4003_u16, perm, a),

        552 => scrambled_radical_inverse_specialized(4007_u16, perm, a),

        553 => scrambled_radical_inverse_specialized(4013_u16, perm, a),

        554 => scrambled_radical_inverse_specialized(4019_u16, perm, a),

        555 => scrambled_radical_inverse_specialized(4021_u16, perm, a),

        556 => scrambled_radical_inverse_specialized(4027_u16, perm, a),

        557 => scrambled_radical_inverse_specialized(4049_u16, perm, a),

        558 => scrambled_radical_inverse_specialized(4051_u16, perm, a),

        559 => scrambled_radical_inverse_specialized(4057_u16, perm, a),

        560 => scrambled_radical_inverse_specialized(4073_u16, perm, a),

        561 => scrambled_radical_inverse_specialized(4079_u16, perm, a),

        562 => scrambled_radical_inverse_specialized(4091_u16, perm, a),

        563 => scrambled_radical_inverse_specialized(4093_u16, perm, a),

        564 => scrambled_radical_inverse_specialized(4099_u16, perm, a),

        565 => scrambled_radical_inverse_specialized(4111_u16, perm, a),

        566 => scrambled_radical_inverse_specialized(4127_u16, perm, a),

        567 => scrambled_radical_inverse_specialized(4129_u16, perm, a),

        568 => scrambled_radical_inverse_specialized(4133_u16, perm, a),

        569 => scrambled_radical_inverse_specialized(4139_u16, perm, a),

        570 => scrambled_radical_inverse_specialized(4153_u16, perm, a),

        571 => scrambled_radical_inverse_specialized(4157_u16, perm, a),

        572 => scrambled_radical_inverse_specialized(4159_u16, perm, a),

        573 => scrambled_radical_inverse_specialized(4177_u16, perm, a),

        574 => scrambled_radical_inverse_specialized(4201_u16, perm, a),

        575 => scrambled_radical_inverse_specialized(4211_u16, perm, a),

        576 => scrambled_radical_inverse_specialized(4217_u16, perm, a),

        577 => scrambled_radical_inverse_specialized(4219_u16, perm, a),

        578 => scrambled_radical_inverse_specialized(4229_u16, perm, a),

        579 => scrambled_radical_inverse_specialized(4231_u16, perm, a),

        580 => scrambled_radical_inverse_specialized(4241_u16, perm, a),

        581 => scrambled_radical_inverse_specialized(4243_u16, perm, a),

        582 => scrambled_radical_inverse_specialized(4253_u16, perm, a),

        583 => scrambled_radical_inverse_specialized(4259_u16, perm, a),

        584 => scrambled_radical_inverse_specialized(4261_u16, perm, a),

        585 => scrambled_radical_inverse_specialized(4271_u16, perm, a),

        586 => scrambled_radical_inverse_specialized(4273_u16, perm, a),

        587 => scrambled_radical_inverse_specialized(4283_u16, perm, a),

        588 => scrambled_radical_inverse_specialized(4289_u16, perm, a),

        589 => scrambled_radical_inverse_specialized(4297_u16, perm, a),

        590 => scrambled_radical_inverse_specialized(4327_u16, perm, a),

        591 => scrambled_radical_inverse_specialized(4337_u16, perm, a),

        592 => scrambled_radical_inverse_specialized(4339_u16, perm, a),

        593 => scrambled_radical_inverse_specialized(4349_u16, perm, a),

        594 => scrambled_radical_inverse_specialized(4357_u16, perm, a),

        595 => scrambled_radical_inverse_specialized(4363_u16, perm, a),

        596 => scrambled_radical_inverse_specialized(4373_u16, perm, a),

        597 => scrambled_radical_inverse_specialized(4391_u16, perm, a),

        598 => scrambled_radical_inverse_specialized(4397_u16, perm, a),

        599 => scrambled_radical_inverse_specialized(4409_u16, perm, a),

        600 => scrambled_radical_inverse_specialized(4421_u16, perm, a),

        601 => scrambled_radical_inverse_specialized(4423_u16, perm, a),

        602 => scrambled_radical_inverse_specialized(4441_u16, perm, a),

        603 => scrambled_radical_inverse_specialized(4447_u16, perm, a),

        604 => scrambled_radical_inverse_specialized(4451_u16, perm, a),

        605 => scrambled_radical_inverse_specialized(4457_u16, perm, a),

        606 => scrambled_radical_inverse_specialized(4463_u16, perm, a),

        607 => scrambled_radical_inverse_specialized(4481_u16, perm, a),

        608 => scrambled_radical_inverse_specialized(4483_u16, perm, a),

        609 => scrambled_radical_inverse_specialized(4493_u16, perm, a),

        610 => scrambled_radical_inverse_specialized(4507_u16, perm, a),

        611 => scrambled_radical_inverse_specialized(4513_u16, perm, a),

        612 => scrambled_radical_inverse_specialized(4517_u16, perm, a),

        613 => scrambled_radical_inverse_specialized(4519_u16, perm, a),

        614 => scrambled_radical_inverse_specialized(4523_u16, perm, a),

        615 => scrambled_radical_inverse_specialized(4547_u16, perm, a),

        616 => scrambled_radical_inverse_specialized(4549_u16, perm, a),

        617 => scrambled_radical_inverse_specialized(4561_u16, perm, a),

        618 => scrambled_radical_inverse_specialized(4567_u16, perm, a),

        619 => scrambled_radical_inverse_specialized(4583_u16, perm, a),

        620 => scrambled_radical_inverse_specialized(4591_u16, perm, a),

        621 => scrambled_radical_inverse_specialized(4597_u16, perm, a),

        622 => scrambled_radical_inverse_specialized(4603_u16, perm, a),

        623 => scrambled_radical_inverse_specialized(4621_u16, perm, a),

        624 => scrambled_radical_inverse_specialized(4637_u16, perm, a),

        625 => scrambled_radical_inverse_specialized(4639_u16, perm, a),

        626 => scrambled_radical_inverse_specialized(4643_u16, perm, a),

        627 => scrambled_radical_inverse_specialized(4649_u16, perm, a),

        628 => scrambled_radical_inverse_specialized(4651_u16, perm, a),

        629 => scrambled_radical_inverse_specialized(4657_u16, perm, a),

        630 => scrambled_radical_inverse_specialized(4663_u16, perm, a),

        631 => scrambled_radical_inverse_specialized(4673_u16, perm, a),

        632 => scrambled_radical_inverse_specialized(4679_u16, perm, a),

        633 => scrambled_radical_inverse_specialized(4691_u16, perm, a),

        634 => scrambled_radical_inverse_specialized(4703_u16, perm, a),

        635 => scrambled_radical_inverse_specialized(4721_u16, perm, a),

        636 => scrambled_radical_inverse_specialized(4723_u16, perm, a),

        637 => scrambled_radical_inverse_specialized(4729_u16, perm, a),

        638 => scrambled_radical_inverse_specialized(4733_u16, perm, a),

        639 => scrambled_radical_inverse_specialized(4751_u16, perm, a),

        640 => scrambled_radical_inverse_specialized(4759_u16, perm, a),

        641 => scrambled_radical_inverse_specialized(4783_u16, perm, a),

        642 => scrambled_radical_inverse_specialized(4787_u16, perm, a),

        643 => scrambled_radical_inverse_specialized(4789_u16, perm, a),

        644 => scrambled_radical_inverse_specialized(4793_u16, perm, a),

        645 => scrambled_radical_inverse_specialized(4799_u16, perm, a),

        646 => scrambled_radical_inverse_specialized(4801_u16, perm, a),

        647 => scrambled_radical_inverse_specialized(4813_u16, perm, a),

        648 => scrambled_radical_inverse_specialized(4817_u16, perm, a),

        649 => scrambled_radical_inverse_specialized(4831_u16, perm, a),

        650 => scrambled_radical_inverse_specialized(4861_u16, perm, a),

        651 => scrambled_radical_inverse_specialized(4871_u16, perm, a),

        652 => scrambled_radical_inverse_specialized(4877_u16, perm, a),

        653 => scrambled_radical_inverse_specialized(4889_u16, perm, a),

        654 => scrambled_radical_inverse_specialized(4903_u16, perm, a),

        655 => scrambled_radical_inverse_specialized(4909_u16, perm, a),

        656 => scrambled_radical_inverse_specialized(4919_u16, perm, a),

        657 => scrambled_radical_inverse_specialized(4931_u16, perm, a),

        658 => scrambled_radical_inverse_specialized(4933_u16, perm, a),

        659 => scrambled_radical_inverse_specialized(4937_u16, perm, a),

        660 => scrambled_radical_inverse_specialized(4943_u16, perm, a),

        661 => scrambled_radical_inverse_specialized(4951_u16, perm, a),

        662 => scrambled_radical_inverse_specialized(4957_u16, perm, a),

        663 => scrambled_radical_inverse_specialized(4967_u16, perm, a),

        664 => scrambled_radical_inverse_specialized(4969_u16, perm, a),

        665 => scrambled_radical_inverse_specialized(4973_u16, perm, a),

        666 => scrambled_radical_inverse_specialized(4987_u16, perm, a),

        667 => scrambled_radical_inverse_specialized(4993_u16, perm, a),

        668 => scrambled_radical_inverse_specialized(4999_u16, perm, a),

        669 => scrambled_radical_inverse_specialized(5003_u16, perm, a),

        670 => scrambled_radical_inverse_specialized(5009_u16, perm, a),

        671 => scrambled_radical_inverse_specialized(5011_u16, perm, a),

        672 => scrambled_radical_inverse_specialized(5021_u16, perm, a),

        673 => scrambled_radical_inverse_specialized(5023_u16, perm, a),

        674 => scrambled_radical_inverse_specialized(5039_u16, perm, a),

        675 => scrambled_radical_inverse_specialized(5051_u16, perm, a),

        676 => scrambled_radical_inverse_specialized(5059_u16, perm, a),

        677 => scrambled_radical_inverse_specialized(5077_u16, perm, a),

        678 => scrambled_radical_inverse_specialized(5081_u16, perm, a),

        679 => scrambled_radical_inverse_specialized(5087_u16, perm, a),

        680 => scrambled_radical_inverse_specialized(5099_u16, perm, a),

        681 => scrambled_radical_inverse_specialized(5101_u16, perm, a),

        682 => scrambled_radical_inverse_specialized(5107_u16, perm, a),

        683 => scrambled_radical_inverse_specialized(5113_u16, perm, a),

        684 => scrambled_radical_inverse_specialized(5119_u16, perm, a),

        685 => scrambled_radical_inverse_specialized(5147_u16, perm, a),

        686 => scrambled_radical_inverse_specialized(5153_u16, perm, a),

        687 => scrambled_radical_inverse_specialized(5167_u16, perm, a),

        688 => scrambled_radical_inverse_specialized(5171_u16, perm, a),

        689 => scrambled_radical_inverse_specialized(5179_u16, perm, a),

        690 => scrambled_radical_inverse_specialized(5189_u16, perm, a),

        691 => scrambled_radical_inverse_specialized(5197_u16, perm, a),

        692 => scrambled_radical_inverse_specialized(5209_u16, perm, a),

        693 => scrambled_radical_inverse_specialized(5227_u16, perm, a),

        694 => scrambled_radical_inverse_specialized(5231_u16, perm, a),

        695 => scrambled_radical_inverse_specialized(5233_u16, perm, a),

        696 => scrambled_radical_inverse_specialized(5237_u16, perm, a),

        697 => scrambled_radical_inverse_specialized(5261_u16, perm, a),

        698 => scrambled_radical_inverse_specialized(5273_u16, perm, a),

        699 => scrambled_radical_inverse_specialized(5279_u16, perm, a),

        700 => scrambled_radical_inverse_specialized(5281_u16, perm, a),

        701 => scrambled_radical_inverse_specialized(5297_u16, perm, a),

        702 => scrambled_radical_inverse_specialized(5303_u16, perm, a),

        703 => scrambled_radical_inverse_specialized(5309_u16, perm, a),

        704 => scrambled_radical_inverse_specialized(5323_u16, perm, a),

        705 => scrambled_radical_inverse_specialized(5333_u16, perm, a),

        706 => scrambled_radical_inverse_specialized(5347_u16, perm, a),

        707 => scrambled_radical_inverse_specialized(5351_u16, perm, a),

        708 => scrambled_radical_inverse_specialized(5381_u16, perm, a),

        709 => scrambled_radical_inverse_specialized(5387_u16, perm, a),

        710 => scrambled_radical_inverse_specialized(5393_u16, perm, a),

        711 => scrambled_radical_inverse_specialized(5399_u16, perm, a),

        712 => scrambled_radical_inverse_specialized(5407_u16, perm, a),

        713 => scrambled_radical_inverse_specialized(5413_u16, perm, a),

        714 => scrambled_radical_inverse_specialized(5417_u16, perm, a),

        715 => scrambled_radical_inverse_specialized(5419_u16, perm, a),

        716 => scrambled_radical_inverse_specialized(5431_u16, perm, a),

        717 => scrambled_radical_inverse_specialized(5437_u16, perm, a),

        718 => scrambled_radical_inverse_specialized(5441_u16, perm, a),

        719 => scrambled_radical_inverse_specialized(5443_u16, perm, a),

        720 => scrambled_radical_inverse_specialized(5449_u16, perm, a),

        721 => scrambled_radical_inverse_specialized(5471_u16, perm, a),

        722 => scrambled_radical_inverse_specialized(5477_u16, perm, a),

        723 => scrambled_radical_inverse_specialized(5479_u16, perm, a),

        724 => scrambled_radical_inverse_specialized(5483_u16, perm, a),

        725 => scrambled_radical_inverse_specialized(5501_u16, perm, a),

        726 => scrambled_radical_inverse_specialized(5503_u16, perm, a),

        727 => scrambled_radical_inverse_specialized(5507_u16, perm, a),

        728 => scrambled_radical_inverse_specialized(5519_u16, perm, a),

        729 => scrambled_radical_inverse_specialized(5521_u16, perm, a),

        730 => scrambled_radical_inverse_specialized(5527_u16, perm, a),

        731 => scrambled_radical_inverse_specialized(5531_u16, perm, a),

        732 => scrambled_radical_inverse_specialized(5557_u16, perm, a),

        733 => scrambled_radical_inverse_specialized(5563_u16, perm, a),

        734 => scrambled_radical_inverse_specialized(5569_u16, perm, a),

        735 => scrambled_radical_inverse_specialized(5573_u16, perm, a),

        736 => scrambled_radical_inverse_specialized(5581_u16, perm, a),

        737 => scrambled_radical_inverse_specialized(5591_u16, perm, a),

        738 => scrambled_radical_inverse_specialized(5623_u16, perm, a),

        739 => scrambled_radical_inverse_specialized(5639_u16, perm, a),

        740 => scrambled_radical_inverse_specialized(5641_u16, perm, a),

        741 => scrambled_radical_inverse_specialized(5647_u16, perm, a),

        742 => scrambled_radical_inverse_specialized(5651_u16, perm, a),

        743 => scrambled_radical_inverse_specialized(5653_u16, perm, a),

        744 => scrambled_radical_inverse_specialized(5657_u16, perm, a),

        745 => scrambled_radical_inverse_specialized(5659_u16, perm, a),

        746 => scrambled_radical_inverse_specialized(5669_u16, perm, a),

        747 => scrambled_radical_inverse_specialized(5683_u16, perm, a),

        748 => scrambled_radical_inverse_specialized(5689_u16, perm, a),

        749 => scrambled_radical_inverse_specialized(5693_u16, perm, a),

        750 => scrambled_radical_inverse_specialized(5701_u16, perm, a),

        751 => scrambled_radical_inverse_specialized(5711_u16, perm, a),

        752 => scrambled_radical_inverse_specialized(5717_u16, perm, a),

        753 => scrambled_radical_inverse_specialized(5737_u16, perm, a),

        754 => scrambled_radical_inverse_specialized(5741_u16, perm, a),

        755 => scrambled_radical_inverse_specialized(5743_u16, perm, a),

        756 => scrambled_radical_inverse_specialized(5749_u16, perm, a),

        757 => scrambled_radical_inverse_specialized(5779_u16, perm, a),

        758 => scrambled_radical_inverse_specialized(5783_u16, perm, a),

        759 => scrambled_radical_inverse_specialized(5791_u16, perm, a),

        760 => scrambled_radical_inverse_specialized(5801_u16, perm, a),

        761 => scrambled_radical_inverse_specialized(5807_u16, perm, a),

        762 => scrambled_radical_inverse_specialized(5813_u16, perm, a),

        763 => scrambled_radical_inverse_specialized(5821_u16, perm, a),

        764 => scrambled_radical_inverse_specialized(5827_u16, perm, a),

        765 => scrambled_radical_inverse_specialized(5839_u16, perm, a),

        766 => scrambled_radical_inverse_specialized(5843_u16, perm, a),

        767 => scrambled_radical_inverse_specialized(5849_u16, perm, a),

        768 => scrambled_radical_inverse_specialized(5851_u16, perm, a),

        769 => scrambled_radical_inverse_specialized(5857_u16, perm, a),

        770 => scrambled_radical_inverse_specialized(5861_u16, perm, a),

        771 => scrambled_radical_inverse_specialized(5867_u16, perm, a),

        772 => scrambled_radical_inverse_specialized(5869_u16, perm, a),

        773 => scrambled_radical_inverse_specialized(5879_u16, perm, a),

        774 => scrambled_radical_inverse_specialized(5881_u16, perm, a),

        775 => scrambled_radical_inverse_specialized(5897_u16, perm, a),

        776 => scrambled_radical_inverse_specialized(5903_u16, perm, a),

        777 => scrambled_radical_inverse_specialized(5923_u16, perm, a),

        778 => scrambled_radical_inverse_specialized(5927_u16, perm, a),

        779 => scrambled_radical_inverse_specialized(5939_u16, perm, a),

        780 => scrambled_radical_inverse_specialized(5953_u16, perm, a),

        781 => scrambled_radical_inverse_specialized(5981_u16, perm, a),

        782 => scrambled_radical_inverse_specialized(5987_u16, perm, a),

        783 => scrambled_radical_inverse_specialized(6007_u16, perm, a),

        784 => scrambled_radical_inverse_specialized(6011_u16, perm, a),

        785 => scrambled_radical_inverse_specialized(6029_u16, perm, a),

        786 => scrambled_radical_inverse_specialized(6037_u16, perm, a),

        787 => scrambled_radical_inverse_specialized(6043_u16, perm, a),

        788 => scrambled_radical_inverse_specialized(6047_u16, perm, a),

        789 => scrambled_radical_inverse_specialized(6053_u16, perm, a),

        790 => scrambled_radical_inverse_specialized(6067_u16, perm, a),

        791 => scrambled_radical_inverse_specialized(6073_u16, perm, a),

        792 => scrambled_radical_inverse_specialized(6079_u16, perm, a),

        793 => scrambled_radical_inverse_specialized(6089_u16, perm, a),

        794 => scrambled_radical_inverse_specialized(6091_u16, perm, a),

        795 => scrambled_radical_inverse_specialized(6101_u16, perm, a),

        796 => scrambled_radical_inverse_specialized(6113_u16, perm, a),

        797 => scrambled_radical_inverse_specialized(6121_u16, perm, a),

        798 => scrambled_radical_inverse_specialized(6131_u16, perm, a),

        799 => scrambled_radical_inverse_specialized(6133_u16, perm, a),

        800 => scrambled_radical_inverse_specialized(6143_u16, perm, a),

        801 => scrambled_radical_inverse_specialized(6151_u16, perm, a),

        802 => scrambled_radical_inverse_specialized(6163_u16, perm, a),

        803 => scrambled_radical_inverse_specialized(6173_u16, perm, a),

        804 => scrambled_radical_inverse_specialized(6197_u16, perm, a),

        805 => scrambled_radical_inverse_specialized(6199_u16, perm, a),

        806 => scrambled_radical_inverse_specialized(6203_u16, perm, a),

        807 => scrambled_radical_inverse_specialized(6211_u16, perm, a),

        808 => scrambled_radical_inverse_specialized(6217_u16, perm, a),

        809 => scrambled_radical_inverse_specialized(6221_u16, perm, a),

        810 => scrambled_radical_inverse_specialized(6229_u16, perm, a),

        811 => scrambled_radical_inverse_specialized(6247_u16, perm, a),

        812 => scrambled_radical_inverse_specialized(6257_u16, perm, a),

        813 => scrambled_radical_inverse_specialized(6263_u16, perm, a),

        814 => scrambled_radical_inverse_specialized(6269_u16, perm, a),

        815 => scrambled_radical_inverse_specialized(6271_u16, perm, a),

        816 => scrambled_radical_inverse_specialized(6277_u16, perm, a),

        817 => scrambled_radical_inverse_specialized(6287_u16, perm, a),

        818 => scrambled_radical_inverse_specialized(6299_u16, perm, a),

        819 => scrambled_radical_inverse_specialized(6301_u16, perm, a),

        820 => scrambled_radical_inverse_specialized(6311_u16, perm, a),

        821 => scrambled_radical_inverse_specialized(6317_u16, perm, a),

        822 => scrambled_radical_inverse_specialized(6323_u16, perm, a),

        823 => scrambled_radical_inverse_specialized(6329_u16, perm, a),

        824 => scrambled_radical_inverse_specialized(6337_u16, perm, a),

        825 => scrambled_radical_inverse_specialized(6343_u16, perm, a),

        826 => scrambled_radical_inverse_specialized(6353_u16, perm, a),

        827 => scrambled_radical_inverse_specialized(6359_u16, perm, a),

        828 => scrambled_radical_inverse_specialized(6361_u16, perm, a),

        829 => scrambled_radical_inverse_specialized(6367_u16, perm, a),

        830 => scrambled_radical_inverse_specialized(6373_u16, perm, a),

        831 => scrambled_radical_inverse_specialized(6379_u16, perm, a),

        832 => scrambled_radical_inverse_specialized(6389_u16, perm, a),

        833 => scrambled_radical_inverse_specialized(6397_u16, perm, a),

        834 => scrambled_radical_inverse_specialized(6421_u16, perm, a),

        835 => scrambled_radical_inverse_specialized(6427_u16, perm, a),

        836 => scrambled_radical_inverse_specialized(6449_u16, perm, a),

        837 => scrambled_radical_inverse_specialized(6451_u16, perm, a),

        838 => scrambled_radical_inverse_specialized(6469_u16, perm, a),

        839 => scrambled_radical_inverse_specialized(6473_u16, perm, a),

        840 => scrambled_radical_inverse_specialized(6481_u16, perm, a),

        841 => scrambled_radical_inverse_specialized(6491_u16, perm, a),

        842 => scrambled_radical_inverse_specialized(6521_u16, perm, a),

        843 => scrambled_radical_inverse_specialized(6529_u16, perm, a),

        844 => scrambled_radical_inverse_specialized(6547_u16, perm, a),

        845 => scrambled_radical_inverse_specialized(6551_u16, perm, a),

        846 => scrambled_radical_inverse_specialized(6553_u16, perm, a),

        847 => scrambled_radical_inverse_specialized(6563_u16, perm, a),

        848 => scrambled_radical_inverse_specialized(6569_u16, perm, a),

        849 => scrambled_radical_inverse_specialized(6571_u16, perm, a),

        850 => scrambled_radical_inverse_specialized(6577_u16, perm, a),

        851 => scrambled_radical_inverse_specialized(6581_u16, perm, a),

        852 => scrambled_radical_inverse_specialized(6599_u16, perm, a),

        853 => scrambled_radical_inverse_specialized(6607_u16, perm, a),

        854 => scrambled_radical_inverse_specialized(6619_u16, perm, a),

        855 => scrambled_radical_inverse_specialized(6637_u16, perm, a),

        856 => scrambled_radical_inverse_specialized(6653_u16, perm, a),

        857 => scrambled_radical_inverse_specialized(6659_u16, perm, a),

        858 => scrambled_radical_inverse_specialized(6661_u16, perm, a),

        859 => scrambled_radical_inverse_specialized(6673_u16, perm, a),

        860 => scrambled_radical_inverse_specialized(6679_u16, perm, a),

        861 => scrambled_radical_inverse_specialized(6689_u16, perm, a),

        862 => scrambled_radical_inverse_specialized(6691_u16, perm, a),

        863 => scrambled_radical_inverse_specialized(6701_u16, perm, a),

        864 => scrambled_radical_inverse_specialized(6703_u16, perm, a),

        865 => scrambled_radical_inverse_specialized(6709_u16, perm, a),

        866 => scrambled_radical_inverse_specialized(6719_u16, perm, a),

        867 => scrambled_radical_inverse_specialized(6733_u16, perm, a),

        868 => scrambled_radical_inverse_specialized(6737_u16, perm, a),

        869 => scrambled_radical_inverse_specialized(6761_u16, perm, a),

        870 => scrambled_radical_inverse_specialized(6763_u16, perm, a),

        871 => scrambled_radical_inverse_specialized(6779_u16, perm, a),

        872 => scrambled_radical_inverse_specialized(6781_u16, perm, a),

        873 => scrambled_radical_inverse_specialized(6791_u16, perm, a),

        874 => scrambled_radical_inverse_specialized(6793_u16, perm, a),

        875 => scrambled_radical_inverse_specialized(6803_u16, perm, a),

        876 => scrambled_radical_inverse_specialized(6823_u16, perm, a),

        877 => scrambled_radical_inverse_specialized(6827_u16, perm, a),

        878 => scrambled_radical_inverse_specialized(6829_u16, perm, a),

        879 => scrambled_radical_inverse_specialized(6833_u16, perm, a),

        880 => scrambled_radical_inverse_specialized(6841_u16, perm, a),

        881 => scrambled_radical_inverse_specialized(6857_u16, perm, a),

        882 => scrambled_radical_inverse_specialized(6863_u16, perm, a),

        883 => scrambled_radical_inverse_specialized(6869_u16, perm, a),

        884 => scrambled_radical_inverse_specialized(6871_u16, perm, a),

        885 => scrambled_radical_inverse_specialized(6883_u16, perm, a),

        886 => scrambled_radical_inverse_specialized(6899_u16, perm, a),

        887 => scrambled_radical_inverse_specialized(6907_u16, perm, a),

        888 => scrambled_radical_inverse_specialized(6911_u16, perm, a),

        889 => scrambled_radical_inverse_specialized(6917_u16, perm, a),

        890 => scrambled_radical_inverse_specialized(6947_u16, perm, a),

        891 => scrambled_radical_inverse_specialized(6949_u16, perm, a),

        892 => scrambled_radical_inverse_specialized(6959_u16, perm, a),

        893 => scrambled_radical_inverse_specialized(6961_u16, perm, a),

        894 => scrambled_radical_inverse_specialized(6967_u16, perm, a),

        895 => scrambled_radical_inverse_specialized(6971_u16, perm, a),

        896 => scrambled_radical_inverse_specialized(6977_u16, perm, a),

        897 => scrambled_radical_inverse_specialized(6983_u16, perm, a),

        898 => scrambled_radical_inverse_specialized(6991_u16, perm, a),

        899 => scrambled_radical_inverse_specialized(6997_u16, perm, a),

        900 => scrambled_radical_inverse_specialized(7001_u16, perm, a),

        901 => scrambled_radical_inverse_specialized(7013_u16, perm, a),

        902 => scrambled_radical_inverse_specialized(7019_u16, perm, a),

        903 => scrambled_radical_inverse_specialized(7027_u16, perm, a),

        904 => scrambled_radical_inverse_specialized(7039_u16, perm, a),

        905 => scrambled_radical_inverse_specialized(7043_u16, perm, a),

        906 => scrambled_radical_inverse_specialized(7057_u16, perm, a),

        907 => scrambled_radical_inverse_specialized(7069_u16, perm, a),

        908 => scrambled_radical_inverse_specialized(7079_u16, perm, a),

        909 => scrambled_radical_inverse_specialized(7103_u16, perm, a),

        910 => scrambled_radical_inverse_specialized(7109_u16, perm, a),

        911 => scrambled_radical_inverse_specialized(7121_u16, perm, a),

        912 => scrambled_radical_inverse_specialized(7127_u16, perm, a),

        913 => scrambled_radical_inverse_specialized(7129_u16, perm, a),

        914 => scrambled_radical_inverse_specialized(7151_u16, perm, a),

        915 => scrambled_radical_inverse_specialized(7159_u16, perm, a),

        916 => scrambled_radical_inverse_specialized(7177_u16, perm, a),

        917 => scrambled_radical_inverse_specialized(7187_u16, perm, a),

        918 => scrambled_radical_inverse_specialized(7193_u16, perm, a),

        919 => scrambled_radical_inverse_specialized(7207_u16, perm, a),

        920 => scrambled_radical_inverse_specialized(7211_u16, perm, a),

        921 => scrambled_radical_inverse_specialized(7213_u16, perm, a),

        922 => scrambled_radical_inverse_specialized(7219_u16, perm, a),

        923 => scrambled_radical_inverse_specialized(7229_u16, perm, a),

        924 => scrambled_radical_inverse_specialized(7237_u16, perm, a),

        925 => scrambled_radical_inverse_specialized(7243_u16, perm, a),

        926 => scrambled_radical_inverse_specialized(7247_u16, perm, a),

        927 => scrambled_radical_inverse_specialized(7253_u16, perm, a),

        928 => scrambled_radical_inverse_specialized(7283_u16, perm, a),

        929 => scrambled_radical_inverse_specialized(7297_u16, perm, a),

        930 => scrambled_radical_inverse_specialized(7307_u16, perm, a),

        931 => scrambled_radical_inverse_specialized(7309_u16, perm, a),

        932 => scrambled_radical_inverse_specialized(7321_u16, perm, a),

        933 => scrambled_radical_inverse_specialized(7331_u16, perm, a),

        934 => scrambled_radical_inverse_specialized(7333_u16, perm, a),

        935 => scrambled_radical_inverse_specialized(7349_u16, perm, a),

        936 => scrambled_radical_inverse_specialized(7351_u16, perm, a),

        937 => scrambled_radical_inverse_specialized(7369_u16, perm, a),

        938 => scrambled_radical_inverse_specialized(7393_u16, perm, a),

        939 => scrambled_radical_inverse_specialized(7411_u16, perm, a),

        940 => scrambled_radical_inverse_specialized(7417_u16, perm, a),

        941 => scrambled_radical_inverse_specialized(7433_u16, perm, a),

        942 => scrambled_radical_inverse_specialized(7451_u16, perm, a),

        943 => scrambled_radical_inverse_specialized(7457_u16, perm, a),

        944 => scrambled_radical_inverse_specialized(7459_u16, perm, a),

        945 => scrambled_radical_inverse_specialized(7477_u16, perm, a),

        946 => scrambled_radical_inverse_specialized(7481_u16, perm, a),

        947 => scrambled_radical_inverse_specialized(7487_u16, perm, a),

        948 => scrambled_radical_inverse_specialized(7489_u16, perm, a),

        949 => scrambled_radical_inverse_specialized(7499_u16, perm, a),

        950 => scrambled_radical_inverse_specialized(7507_u16, perm, a),

        951 => scrambled_radical_inverse_specialized(7517_u16, perm, a),

        952 => scrambled_radical_inverse_specialized(7523_u16, perm, a),

        953 => scrambled_radical_inverse_specialized(7529_u16, perm, a),

        954 => scrambled_radical_inverse_specialized(7537_u16, perm, a),

        955 => scrambled_radical_inverse_specialized(7541_u16, perm, a),

        956 => scrambled_radical_inverse_specialized(7547_u16, perm, a),

        957 => scrambled_radical_inverse_specialized(7549_u16, perm, a),

        958 => scrambled_radical_inverse_specialized(7559_u16, perm, a),

        959 => scrambled_radical_inverse_specialized(7561_u16, perm, a),

        960 => scrambled_radical_inverse_specialized(7573_u16, perm, a),

        961 => scrambled_radical_inverse_specialized(7577_u16, perm, a),

        962 => scrambled_radical_inverse_specialized(7583_u16, perm, a),

        963 => scrambled_radical_inverse_specialized(7589_u16, perm, a),

        964 => scrambled_radical_inverse_specialized(7591_u16, perm, a),

        965 => scrambled_radical_inverse_specialized(7603_u16, perm, a),

        966 => scrambled_radical_inverse_specialized(7607_u16, perm, a),

        967 => scrambled_radical_inverse_specialized(7621_u16, perm, a),

        968 => scrambled_radical_inverse_specialized(7639_u16, perm, a),

        969 => scrambled_radical_inverse_specialized(7643_u16, perm, a),

        970 => scrambled_radical_inverse_specialized(7649_u16, perm, a),

        971 => scrambled_radical_inverse_specialized(7669_u16, perm, a),

        972 => scrambled_radical_inverse_specialized(7673_u16, perm, a),

        973 => scrambled_radical_inverse_specialized(7681_u16, perm, a),

        974 => scrambled_radical_inverse_specialized(7687_u16, perm, a),

        975 => scrambled_radical_inverse_specialized(7691_u16, perm, a),

        976 => scrambled_radical_inverse_specialized(7699_u16, perm, a),

        977 => scrambled_radical_inverse_specialized(7703_u16, perm, a),

        978 => scrambled_radical_inverse_specialized(7717_u16, perm, a),

        979 => scrambled_radical_inverse_specialized(7723_u16, perm, a),

        980 => scrambled_radical_inverse_specialized(7727_u16, perm, a),

        981 => scrambled_radical_inverse_specialized(7741_u16, perm, a),

        982 => scrambled_radical_inverse_specialized(7753_u16, perm, a),

        983 => scrambled_radical_inverse_specialized(7757_u16, perm, a),

        984 => scrambled_radical_inverse_specialized(7759_u16, perm, a),

        985 => scrambled_radical_inverse_specialized(7789_u16, perm, a),

        986 => scrambled_radical_inverse_specialized(7793_u16, perm, a),

        987 => scrambled_radical_inverse_specialized(7817_u16, perm, a),

        988 => scrambled_radical_inverse_specialized(7823_u16, perm, a),

        989 => scrambled_radical_inverse_specialized(7829_u16, perm, a),

        990 => scrambled_radical_inverse_specialized(7841_u16, perm, a),

        991 => scrambled_radical_inverse_specialized(7853_u16, perm, a),

        992 => scrambled_radical_inverse_specialized(7867_u16, perm, a),

        993 => scrambled_radical_inverse_specialized(7873_u16, perm, a),

        994 => scrambled_radical_inverse_specialized(7877_u16, perm, a),

        995 => scrambled_radical_inverse_specialized(7879_u16, perm, a),

        996 => scrambled_radical_inverse_specialized(7883_u16, perm, a),

        997 => scrambled_radical_inverse_specialized(7901_u16, perm, a),

        998 => scrambled_radical_inverse_specialized(7907_u16, perm, a),

        999 => scrambled_radical_inverse_specialized(7919_u16, perm, a),

        1000 => scrambled_radical_inverse_specialized(7927_u16, perm, a),

        1001 => scrambled_radical_inverse_specialized(7933_u16, perm, a),

        1002 => scrambled_radical_inverse_specialized(7937_u16, perm, a),

        1003 => scrambled_radical_inverse_specialized(7949_u16, perm, a),

        1004 => scrambled_radical_inverse_specialized(7951_u16, perm, a),

        1005 => scrambled_radical_inverse_specialized(7963_u16, perm, a),

        1006 => scrambled_radical_inverse_specialized(7993_u16, perm, a),

        1007 => scrambled_radical_inverse_specialized(8009_u16, perm, a),

        1008 => scrambled_radical_inverse_specialized(8011_u16, perm, a),

        1009 => scrambled_radical_inverse_specialized(8017_u16, perm, a),

        1010 => scrambled_radical_inverse_specialized(8039_u16, perm, a),

        1011 => scrambled_radical_inverse_specialized(8053_u16, perm, a),

        1012 => scrambled_radical_inverse_specialized(8059_u16, perm, a),

        1013 => scrambled_radical_inverse_specialized(8069_u16, perm, a),

        1014 => scrambled_radical_inverse_specialized(8081_u16, perm, a),

        1015 => scrambled_radical_inverse_specialized(8087_u16, perm, a),

        1016 => scrambled_radical_inverse_specialized(8089_u16, perm, a),

        1017 => scrambled_radical_inverse_specialized(8093_u16, perm, a),

        1018 => scrambled_radical_inverse_specialized(8101_u16, perm, a),

        1019 => scrambled_radical_inverse_specialized(8111_u16, perm, a),

        1020 => scrambled_radical_inverse_specialized(8117_u16, perm, a),

        1021 => scrambled_radical_inverse_specialized(8123_u16, perm, a),

        1022 => scrambled_radical_inverse_specialized(8147_u16, perm, a),

        1023 => scrambled_radical_inverse_specialized(8161_u16, perm, a),
        _ => {
            panic!("TODO: scrambled_radical_inverse({:?}, {:?})", base_index, a);
        }
    }
}
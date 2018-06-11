
const int16 sin_tab[1024]={0,25,50,75,101,126,151,176,201,226,251,
276,302,327,352,377,402,427,452,477,503,
528,553,578,603,628,653,678,704,729,754,
779,804,829,854,879,904,929,955,980,1005,1030,
1055,1080,1105,1130,1155,1180,1205,1230,
1255,1280,1306,1331,1356,1381,1406,1431,
1456,1481,1506,1531,1556,1581,1606,1631,
1656,1681,1706,1731,1756,1781,1806,1831,
1856,1881,1906,1931,1956,1981,2006,2031,
2055,2080,2105,2130,2155,2180,2205,2230,
2255,2280,2305,2329,2354,2379,2404,2429,
2454,2479,2503,2528,2553,2578,2603,2628,
2652,2677,2702,2727,2752,2776,2801,2826,
2851,2875,2900,2925,2949,2974,2999,3024,
3048,3073,3098,3122,3147,3172,3196,3221,
3246,3270,3295,3320,3344,3369,3393,3418,
3442,3467,3492,3516,3541,3565,3590,3614,
3639,3663,3688,3712,3737,3761,3786,3810,
3835,3859,3883,3908,3932,3957,3981,4005,
4030,4054,4078,4103,4127,4151,4176,4200,
4224,4249,4273,4297,4321,4346,4370,4394,
4418,4442,4467,4491,4515,4539,4563,4587,
4612,4636,4660,4684,4708,4732,4756,4780,
4804,4828,4852,4876,4900,4924,4948,4972,
4996,5020,5044,5068,5092,5115,5139,5163,
5187,5211,5235,5259,5282,5306,5330,5354,
5377,5401,5425,5449,5472,5496,5520,5543,
5567,5591,5614,5638,5661,5685,5708,5732,
5756,5779,5803,5826,5850,5873,5897,5920,
5943,5967,5990,6014,6037,6060,6084,6107,
6130,6154,6177,6200,6223,6247,6270,6293,
6316,6339,6363,6386,6409,6432,6455,6478,
6501,6524,6547,6570,6593,6616,6639,6662,
6685,6708,6731,6754,6777,6800,6823,6846,
6868,6891,6914,6937,6960,6982,7005,7028,
7050,7073,7096,7118,7141,7164,7186,7209,
7231,7254,7276,7299,7321,7344,7366,7389,
7411,7434,7456,7478,7501,7523,7545,7568,
7590,7612,7635,7657,7679,7701,7723,7746,
7768,7790,7812,7834,7856,7878,7900,7922,
7944,7966,7988,8010,8032,8054,8076,8098,
8119,8141,8163,8185,8207,8228,8250,8272,
8293,8315,8337,8358,8380,8401,8423,8445,
8466,8488,8509,8531,8552,8573,8595,8616,
8638,8659,8680,8702,8723,8744,8765,8787,
8808,8829,8850,8871,8892,8914,8935,8956,
8977,8998,9019,9040,9061,9082,9102,9123,
9144,9165,9186,9207,9227,9248,9269,9290,
9310,9331,9352,9372,9393,9413,9434,9455,
9475,9496,9516,9537,9557,9577,9598,9618,
9638,9659,9679,9699,9720,9740,9760,9780,
9800,9820,9841,9861,9881,9901,9921,9941,
9961,9981,10001,10020,10040,10060,10080,
10100,10120,10139,10159,10179,10198,10218,
10238,10257,10277,10296,10316,10336,10355,
10374,10394,10413,10433,10452,10471,10491,
10510,10529,10549,10568,10587,10606,10625,
10644,10663,10683,10702,10721,10740,10759,
10778,10796,10815,10834,10853,10872,10891,
10909,10928,10947,10966,10984,11003,11021,
11040,11059,11077,11096,11114,11133,11151,
11169,11188,11206,11224,11243,11261,11279,
11297,11316,11334,11352,11370,11388,11406,
11424,11442,11460,11478,11496,11514,11532,
11550,11567,11585,11603,11621,11638,11656,
11674,11691,11709,11727,11744,11762,11779,
11797,11814,11831,11849,11866,11883,11901,
11918,11935,11952,11970,11987,12004,12021,
12038,12055,12072,12089,12106,12123,12140,
12157,12173,12190,12207,12224,12240,12257,
12274,12290,12307,12324,12340,12357,12373,
12390,12406,12423,12439,12455,12472,12488,
12504,12520,12537,12553,12569,12585,12601,
12617,12633,12649,12665,12681,12697,12713,
12729,12744,12760,12776,12792,12807,12823,
12839,12854,12870,12885,12901,12916,12932,
12947,12963,12978,12993,13008,13024,13039,
13054,13069,13085,13100,13115,13130,13145,
13160,13175,13190,13205,13219,13234,13249,
13264,13279,13293,13308,13323,13337,13352,
13366,13381,13395,13410,13424,13439,13453,
13467,13482,13496,13510,13524,13538,13553,
13567,13581,13595,13609,13623,13637,13651,
13665,13678,13692,13706,13720,13733,13747,
13761,13774,13788,13802,13815,13829,13842,
13856,13869,13882,13896,13909,13922,13935,
13949,13962,13975,13988,14001,14014,14027,
14040,14053,14066,14079,14092,14104,14117,
14130,14143,14155,14168,14181,14193,14206,
14218,14231,14243,14256,14268,14280,14293,
14305,14317,14329,14341,14354,14366,14378,
14390,14402,14414,14426,14438,14449,14461,
14473,14485,14497,14508,14520,14531,14543,
14555,14566,14578,14589,14601,14612,14623,
14635,14646,14657,14668,14680,14691,14702,
14713,14724,14735,14746,14757,14768,14779,
14789,14800,14811,14822,14832,14843,14854,
14864,14875,14885,14896,14906,14917,14927,
14937,14948,14958,14968,14978,14989,14999,
15009,15019,15029,15039,15049,15059,15069,
15078,15088,15098,15108,15118,15127,15137,
15146,15156,15166,15175,15184,15194,15203,
15213,15222,15231,15240,15250,15259,15268,
15277,15286,15295,15304,15313,15322,15331,
15340,15349,15357,15366,15375,15383,15392,
15401,15409,15418,15426,15435,15443,15451,
15460,15468,15476,15485,15493,15501,15509,
15517,15525,15533,15541,15549,15557,15565,
15573,15581,15588,15596,15604,15611,15619,
15627,15634,15642,15649,15656,15664,15671,
15679,15686,15693,15700,15707,15715,15722,
15729,15736,15743,15750,15757,15763,15770,
15777,15784,15791,15797,15804,15810,15817,
15824,15830,15837,15843,15849,15856,15862,
15868,15875,15881,15887,15893,15899,15905,
15911,15917,15923,15929,15935,15941,15946,
15952,15958,15964,15969,15975,15980,15986,
15991,15997,16002,16008,16013,16018,16024,
16029,16034,16039,16044,16049,16054,16059,
16064,16069,16074,16079,16084,16088,16093,
16098,16103,16107,16112,16116,16121,16125,
16130,16134,16138,16143,16147,16151,16156,
16160,16164,16168,16172,16176,16180,16184,
16188,16192,16195,16199,16203,16207,16210,
16214,16218,16221,16225,16228,16232,16235,
16238,16242,16245,16248,16251,16255,16258,
16261,16264,16267,16270,16273,16276,16279,
16281,16284,16287,16290,16292,16295,16298,
16300,16303,16305,16308,16310,16312,16315,
16317,16319,16321,16324,16326,16328,16330,
16332,16334,16336,16338,16340,16341,16343,
16345,16347,16348,16350,16352,16353,16355,
16356,16358,16359,16360,16362,16363,16364,
16365,16367,16368,16369,16370,16371,16372,
16373,16374,16375,16375,16376,16377,16378,
16378,16379,16380,16380,16381,16381,16382,
16382,16382,16383,16383,16383,16384,16384,16384,16384,16384};




//===============================================
// No more.
//===============================================

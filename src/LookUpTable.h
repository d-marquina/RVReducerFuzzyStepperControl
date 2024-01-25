float sat_sup = 2.3;
float sat_inf = -2.3;

float d0_min = -23;
float d0_cond = 10;
int d0_n = 46; // Index from 0 to 46

float d1_min = -23;
float d1_cond = 10;
int d1_n = 46;

static float look_up_table[47][47] = {
    {-195.2813, -195.2883, -195.2952, -195.3022, -195.3091, -195.316, -195.3223, -195.3269, -195.3257, -195.3082, -195.2484, -195.0883, -194.7132, -193.9224, -192.4151, -189.8298, -185.8712, -180.5031, -174.0912, -167.3568, -161.1476, -156.191, -152.9766, -151.7682, -152.6615, -155.6091, -160.3888, -166.5374, -173.3286, -179.8877, -185.4465, -189.5921, -192.3301, -193.9468, -194.8092, -195.2292, -195.4182, -195.4986, -195.5327, -195.5487, -195.5583, -195.5659, -195.573, -195.5799, -195.5868, -195.5938, -195.6007},
{-186.7185, -186.7255, -186.7324, -186.7394, -186.7465, -186.7535, -186.7602, -186.7653, -186.7652, -186.7497, -186.6933, -186.5394, -186.1755, -185.405, -183.9366, -181.4283, -177.6209, -172.5267, -166.5461, -160.3824, -154.7991, -150.4019, -147.5656, -146.4784, -147.2104, -149.7423, -153.9314, -159.4342, -165.651, -171.7919, -177.102, -181.1253, -183.8117, -185.4078, -186.261, -186.6758, -186.8615, -186.9396, -186.9721, -186.9872, -186.9962, -187.0035, -187.0104, -187.0172, -187.0241, -187.031, -187.0379},
{-178.1557, -178.1627, -178.1697, -178.1768, -178.1841, -178.1915, -178.1989, -178.2056, -178.2085, -178.1987, -178.154, -178.0237, -177.7063, -177.0246, -175.7176, -173.4851, -170.1151, -165.6532, -160.488, -155.2458, -150.5626, -146.9081, -144.5519, -143.6203, -144.1643, -146.1852, -149.6046, -154.1887, -159.4783, -164.8132, -169.5122, -173.1239, -175.5575, -177.0087, -177.7832, -178.1568, -178.3214, -178.3887, -178.4156, -178.4277, -178.4351, -178.4416, -178.448, -178.4546, -178.4614, -178.4682, -178.4751},
{-169.5929, -169.6, -169.6071, -169.6144, -169.6221, -169.6304, -169.6395, -169.6494, -169.6583, -169.6604, -169.6389, -169.5546, -169.3268, -168.8129, -167.8023, -166.0552, -163.4092, -159.917, -155.9037, -151.8657, -148.2837, -145.4952, -143.6829, -142.9296, -143.2736, -144.7292, -147.263, -150.731, -154.8105, -158.999, -162.7438, -165.6512, -167.6176, -168.7853, -169.3997, -169.6874, -169.8072, -169.8514, -169.8663, -169.872, -169.8761, -169.8807, -169.8862, -169.8923, -169.8988, -169.9055, -169.9123},
{-161.0302, -161.0373, -161.0447, -161.0525, -161.061, -161.0711, -161.0835, -161.0996, -161.12, -161.1435, -161.1623, -161.1538, -161.0674, -160.8092, -160.2353, -159.1778, -157.5181, -155.2834, -152.6863, -150.0541, -147.7014, -145.8474, -144.6112, -144.0499, -144.1951, -145.0668, -146.6574, -148.888, -151.5571, -154.3312, -156.8264, -158.7585, -160.0452, -160.7828, -161.1446, -161.2913, -161.3347, -161.3376, -161.3301, -161.3236, -161.3209, -161.3218, -161.3253, -161.3304, -161.3364, -161.3429, -161.3496},
{-152.4675, -152.4749, -152.4826, -152.4913, -152.5016, -152.5151, -152.5339, -152.5615, -152.6027, -152.6633, -152.748, -152.8567, -152.9769, -153.0728, -153.0772, -152.8956, -152.4382, -151.673, -150.6619, -149.5416, -148.4665, -147.5597, -146.9006, -146.5369, -146.4985, -146.8046, -147.4554, -148.4109, -149.567, -150.7527, -151.7763, -152.5044, -152.9129, -153.0689, -153.0721, -153.008, -152.9302, -152.864, -152.817, -152.788, -152.7727, -152.7666, -152.7664, -152.7695, -152.7745, -152.7805, -152.787},
{-143.905, -143.9127, -143.9212, -143.9315, -143.9451, -143.9648, -143.9951, -144.0435, -144.1212, -144.2444, -144.435, -144.7211, -145.1337, -145.7, -146.4273, -147.2825, -148.1786, -148.9915, -149.6095, -149.9821, -150.1278, -150.1058, -149.9825, -149.8111, -149.6246, -149.4342, -149.2302, -148.9826, -148.65, -148.1987, -147.6286, -146.9838, -146.3359, -145.751, -145.2689, -144.9003, -144.636, -144.4573, -144.3431, -144.2747, -144.2366, -144.2178, -144.2106, -144.2101, -144.2133, -144.2184, -144.2245},
{-135.3427, -135.351, -135.3609, -135.3741, -135.3933, -135.4238, -135.4742, -135.5584, -135.6977, -135.9238, -136.2823, -136.8347, -137.6595, -138.8423, -140.449, -142.4755, -144.7955, -147.1592, -149.2791, -150.9501, -152.1073, -152.795, -153.0996, -153.0972, -152.8276, -152.2871, -151.4349, -150.2141, -148.5943, -146.6264, -144.4714, -142.3596, -140.4971, -138.9952, -137.8663, -137.0628, -136.5156, -136.1575, -135.9324, -135.7972, -135.7201, -135.6793, -135.6601, -135.6533, -135.6533, -135.6569, -135.6622},
{-126.7807, -126.7901, -126.8023, -126.8201, -126.8488, -126.8972, -126.9807, -127.1234, -127.3626, -127.7529, -128.3714, -129.3205, -130.726, -132.719, -135.3885, -138.7011, -142.4255, -146.1488, -149.4267, -151.9719, -153.7257, -154.7916, -155.3219, -155.4372, -155.1891, -154.5497, -153.4211, -151.6687, -149.1952, -146.0459, -142.4746, -138.8854, -135.6638, -133.036, -131.0472, -129.626, -128.6557, -128.0191, -127.6175, -127.3743, -127.2336, -127.1566, -127.1177, -127.1005, -127.0952, -127.0962, -127.1004},
{-118.2192, -118.2302, -118.246, -118.2713, -118.3146, -118.3912, -118.5263, -118.7602, -119.1538, -119.7951, -120.8038, -122.3317, -124.5502, -127.6122, -131.5735, -136.2822, -141.309, -146.0324, -149.8944, -152.6411, -154.3527, -155.2939, -155.7428, -155.8914, -155.8085, -155.4323, -154.5758, -152.9617, -150.3194, -146.5543, -141.9014, -136.9036, -132.1792, -128.1675, -125.0374, -122.7494, -121.1619, -120.1084, -119.4379, -119.0285, -118.7892, -118.6561, -118.5865, -118.5531, -118.5397, -118.5367, -118.539},
{-109.6582, -109.6718, -109.6929, -109.7291, -109.7942, -109.9123, -110.1236, -110.4916, -111.1114, -112.1177, -113.6877, -116.0328, -119.3667, -123.8314, -129.3734, -135.6059, -141.7832, -147.026, -150.7289, -152.8244, -153.6962, -153.8903, -153.8769, -153.9472, -154.1939, -154.5115, -154.5926, -153.9456, -152.0013, -148.3586, -143.0832, -136.7983, -130.412, -124.6999, -120.0727, -116.5991, -114.1438, -112.4938, -111.4344, -110.7831, -110.3999, -110.1847, -110.07, -110.013, -109.9875, -109.9787, -109.9783},
{-101.0981, -101.1151, -101.1435, -101.195, -101.2903, -101.4661, -101.7831, -102.3367, -103.2689, -104.7771, -107.113, -110.5603, -115.3703, -121.6362, -129.1072, -137.0302, -144.217, -149.4879, -152.2723, -152.856, -152.0919, -150.9215, -150.0664, -149.9403, -150.6656, -152.0867, -153.7504, -154.8887, -154.515, -151.753, -146.3343, -138.8843, -130.6496, -122.8713, -116.3342, -111.3026, -107.6859, -105.2282, -103.6385, -102.6561, -102.0754, -101.7474, -101.5709, -101.4813, -101.4394, -101.4226, -101.4184},
{-92.5387, -92.5602, -92.5982, -92.6695, -92.8041, -93.0548, -93.5089, -94.3032, -95.6396, -97.7948, -101.1133, -105.9635, -112.6299, -121.1181, -130.8956, -140.7229, -148.8635, -153.8126, -155.1148, -153.5453, -150.5529, -147.557, -145.5718, -145.1506, -146.4497, -149.26, -152.9662, -156.4817, -158.3195, -157.0011, -151.7885, -143.231, -132.9376, -122.7195, -113.854, -106.8853, -101.8062, -98.3236, -96.0576, -94.6518, -93.8181, -93.3455, -93.0897, -92.9584, -92.8954, -92.8683, -92.8593},
{-83.9801, -84.007, -84.0566, -84.1518, -84.3339, -84.6751, -85.2945, -86.3787, -88.2009, -91.1317, -95.6232, -102.1381, -110.9891, -122.0631, -134.4851, -146.4544, -155.6335, -160.1929, -159.8371, -155.8895, -150.4436, -145.4321, -142.1851, -141.4087, -143.2972, -147.5849, -153.4823, -159.5641, -163.8125, -164.104, -159.1757, -149.4533, -136.9001, -123.9402, -112.4125, -103.2003, -96.4118, -91.7233, -88.6585, -86.7512, -85.6176, -84.9734, -84.6236, -84.4428, -84.3548, -84.3156, -84.3009},
{-75.422, -75.4548, -75.5174, -75.6392, -75.8741, -76.3156, -77.1184, -78.5234, -80.882, -84.6667, -90.4437, -98.7728, -109.988, -123.8418, -139.1013, -153.407, -163.8484, -168.2948, -166.5926, -160.5582, -152.8801, -145.9868, -141.5317, -140.3869, -142.7905, -148.4136, -156.2814, -164.6289, -170.9371, -172.5015, -167.5977, -156.5401, -141.6053, -125.7847, -111.4638, -99.8768, -91.264, -85.2807, -81.3545, -78.9052, -77.447, -76.617, -76.1654, -75.9311, -75.816, -75.7636, -75.7429},
{-66.8639, -66.9029, -66.9785, -67.127, -67.4148, -67.9569, -68.943, -70.668, -73.5602, -78.1905, -85.2335, -95.3365, -108.8463, -125.3863, -143.4083, -160.0919, -172.0722, -176.9669, -174.6789, -167.3449, -158.0955, -149.7785, -144.3567, -142.8782, -145.6307, -152.215, -161.4635, -171.2957, -178.7873, -180.8264, -175.4093, -162.8149, -145.5601, -127.0601, -110.1339, -96.3166, -85.9756, -78.7566, -74.0041, -71.0331, -69.2621, -68.253, -67.7033, -67.4174, -67.2763, -67.2113, -67.1846},
{-58.3104, -58.3555, -58.4431, -58.6154, -58.9493, -59.5786, -60.7233, -62.7244, -66.0749, -71.4274, -79.5428, -91.1327, -106.5458, -125.3028, -145.6406, -164.4609, -178.1446, -184.1393, -182.2855, -174.7868, -164.9738, -155.9681, -149.9895, -148.2557, -151.1057, -158.0689, -167.7972, -177.9626, -185.4027, -186.8612, -180.3127, -166.1051, -146.8887, -126.279, -107.3291, -91.7666, -80.0567, -71.8484, -66.4294, -63.0358, -61.0108, -59.8563, -59.2267, -58.8986, -58.736, -58.6603, -58.6283},
{-49.9001, -49.9731, -50.0868, -50.2851, -50.651, -51.3306, -52.5658, -54.7303, -58.3619, -64.1658, -72.9538, -85.4678, -102.0449, -122.147, -143.9332, -164.2592, -179.4985, -187.0493, -186.5929, -180.1357, -170.9157, -162.1162, -156.1031, -154.223, -156.8591, -163.4835, -172.6207, -181.8243, -187.9269, -187.83, -179.7445, -164.1644, -143.7335, -122.005, -102.0208, -85.554, -73.117, -64.3726, -58.5879, -54.96, -52.791, -51.5492, -50.8649, -50.4996, -50.3085, -50.2088, -50.1556},
{-43.7526, -44.2158, -44.6263, -44.9894, -45.3684, -45.9143, -46.8993, -48.7484, -52.0604, -57.5969, -66.2055, -78.6302, -95.18, -115.2936, -137.1794, -157.8672, -173.9673, -182.9827, -184.4127, -179.8711, -172.2253, -164.467, -158.9365, -157.0227, -159.1326, -164.7174, -172.2796, -179.4455, -183.3091, -181.209, -171.7789, -155.6758, -135.3798, -114.1272, -94.699, -78.7381, -66.7198, -58.3029, -52.7527, -49.2614, -47.1267, -45.8183, -44.9759, -44.3776, -43.9006, -43.4864, -43.1134},
{-56.1528, -59.9511, -62.8736, -64.5946, -64.9965, -64.2473, -62.8376, -61.5686, -61.4851, -63.7486, -69.4457, -79.3331, -93.5384, -111.2771, -130.7147, -149.1633, -163.7465, -172.3812, -174.5725, -171.5203, -165.5167, -159.0783, -154.2694, -152.3607, -153.7295, -157.8587, -163.3838, -168.2475, -170.091, -166.9402, -157.9966, -144.0896, -127.4131, -110.6478, -96.0005, -84.6554, -76.7567, -71.7273, -68.668, -66.6718, -65.006, -63.1813, -60.9475, -58.251, -55.1772, -51.8921, -48.5902},
{-136.1431, -152.9559, -165.5067, -172.3995, -173.0038, -167.6583, -157.7493, -145.615, -134.2017, -126.4314, -124.3767, -128.5511, -137.7007, -149.2712, -160.3208, -168.4155, -172.1636, -171.3306, -166.6613, -159.5503, -151.6616, -144.5746, -139.5171, -137.2006, -137.7385, -140.628, -144.8072, -148.8389, -151.2782, -151.1916, -148.6314, -144.758, -141.4266, -140.4014, -142.6286, -147.9327, -155.1902, -162.7752, -169.0234, -172.5643, -172.4945, -168.4265, -160.4613, -149.1144, -135.2122, -119.7668, -103.8395},
{-262.3769, -292.9864, -314.8929, -326.5322, -327.419, -318.2865, -301.3108, -280.2819, -260.3149, -246.5943, -242.2138, -246.3457, -254.3655, -260.0757, -258.4372, -247.2809, -227.6283, -202.8833, -177.3017, -154.4086, -136.1563, -123.0542, -114.7862, -110.736, -110.2143, -112.4948, -116.832, -122.5785, -129.4202, -137.6344, -148.164, -162.317, -181.1077, -204.5686, -231.4558, -259.515, -286.0916, -308.724, -325.4944, -335.1301, -336.959, -330.8273, -317.0383, -296.3248, -269.8375, -239.1133, -205.9893},
{-229.682, -250.8636, -265.2765, -272.6224, -273.083, -267.3714, -256.9912, -244.5751, -233.8512, -228.5557, -230.3208, -237.2193, -244.6918, -247.9909, -243.9401, -231.3794, -211.0543, -185.4622, -158.3947, -133.7929, -114.3454, -100.8672, -92.7713, -88.9304, -88.2486, -89.8558, -93.1426, -97.8204, -104.0355, -112.4303, -123.9807, -139.5293, -159.1927, -182.0304, -206.2308, -229.6753, -250.4925, -267.3421, -279.4206, -286.3242, -287.8946, -284.1112, -275.0478, -260.8887, -241.9878, -218.9523, -192.7145},
{-0.77046, -0.36562, 0.1475, 0.68473, 1.1294, 1.3238, 1.08, 0.24993, -1.1216, -2.6453, -3.6687, -3.6782, -2.6684, -1.1296, 0.23012, 0.77327, 0.17169, -1.3979, -3.2406, -4.4895, -4.643, -3.8118, -2.5101, -1.2915, -0.52163, -0.32278, -0.61436, -1.1841, -1.764, -2.1067, -2.057, -1.6054, -0.89003, -0.12921, 0.48337, 0.84849, 0.96248, 0.87656, 0.65657, 0.3614, 0.037061, -0.28164, -0.56761, -0.79875, -0.95675, -1.0276, -1.0032},
{227.1131, 248.8875, 264.1491, 272.4629, 273.8504, 268.8631, 258.9052, 246.6933, 236.3341, 232.1482, 236.1853, 246.4245, 257.7155, 264.423, 262.3298, 249.2297, 225.1304, 192.622, 156.8698, 124.2085, 99.4942, 84.1861, 76.6942, 74.1889, 74.1932, 75.2767, 77.1033, 80.22, 85.768, 95.1166, 109.3851, 128.9215, 152.9626, 179.7145, 206.8497, 232.1337, 253.8525, 270.9267, 282.8025, 289.2695, 290.3033, 285.9724, 276.4099, 261.8392, 242.64, 219.4331, 193.1526},
{258.5071, 288.9938, 311.1116, 323.2632, 324.9564, 317.016, 301.896, 283.9043, 268.7908, 262.0648, 266.2752, 279.0447, 293.7646, 302.5799, 299.4778, 282.0537, 251.9874, 214.485, 176.5698, 144.4971, 121.5886, 107.8036, 100.9498, 98.3183, 97.7745, 98.1494, 99.2088, 101.475, 106.0172, 114.1995, 127.3387, 146.2678, 170.9208, 200.1331, 231.7924, 263.2855, 292.0189, 315.8068, 333.0444, 342.7165, 344.3302, 337.844, 323.6268, 302.4492, 275.4837, 244.2819, 210.6934},
{134.8621, 151.1917, 163.4656, 170.3215, 171.1571, 166.358, 157.3987, 146.7417, 137.4133, 132.1968, 132.6039, 138.108, 146.1988, 153.4118, 156.848, 155.379, 149.9497, 142.899, 136.736, 133.0628, 132.1607, 133.2616, 135.1345, 136.6058, 136.8442, 135.4457, 132.4153, 128.1242, 123.2659, 118.7934, 115.8006, 115.3242, 118.0899, 124.2811, 133.4302, 144.4838, 156.0138, 166.4895, 174.5236, 179.0471, 179.4067, 175.3968, 167.243, 155.5468, 141.1971, 125.2547, 108.8208},
{58.2038, 61.7924, 64.532, 66.0871, 66.304, 65.2764, 63.3686, 61.1828, 59.4682, 58.9767, 60.297, 63.7199, 69.1988, 76.4331, 85.0336, 94.6689, 105.1014, 116.1024, 127.3275, 138.2522, 148.2092, 156.4953, 162.484, 165.702, 165.8639, 162.883, 156.8775, 148.1837, 137.3676, 125.2157, 112.6785, 100.7547, 90.3287, 82.0109, 76.0368, 72.2631, 70.2521, 69.4054, 69.0994, 68.7909, 68.0801, 66.7356, 64.6881, 62.0035, 58.8439, 55.4237, 51.968},
{46.8885, 47.2821, 47.5945, 47.7993, 47.8988, 47.938, 48.0155, 48.2916, 48.992, 50.406, 52.8747, 56.7663, 62.4332, 70.1497, 80.037, 91.9897, 105.6311, 120.3193, 135.2119, 149.3729, 161.8885, 171.9613, 178.9667, 182.4773, 182.2686, 178.3228, 170.8383, 160.2436, 147.2009, 132.5806, 117.3881, 102.6414, 89.2247, 77.7607, 68.546, 61.5642, 56.5634, 53.1618, 50.9446, 49.534, 48.6258, 47.9995, 47.5116, 47.0805, 46.6686, 46.2667, 45.8805},
{53.1909, 53.2169, 53.2488, 53.2959, 53.3783, 53.5345, 53.8299, 54.3695, 55.3101, 56.8713, 59.339, 63.0526, 68.3704, 75.6067, 84.9484, 96.3676, 109.5603, 123.9356, 138.6682, 152.8005, 165.3629, 175.483, 182.4625, 185.8224, 185.3249, 180.9844, 173.0743, 162.1242, 148.8977, 134.3355, 119.4562, 105.2277, 92.4364, 81.5933, 72.9044, 66.306, 61.5449, 58.2722, 56.1233, 54.771, 53.9519, 53.471, 53.1939, 53.0336, 52.9375, 52.8756, 52.8316},
{61.6142, 61.6163, 61.6279, 61.6587, 61.7268, 61.8641, 62.1247, 62.594, 63.4009, 64.7277, 66.8138, 69.948, 74.4425, 80.5829, 88.5604, 98.3948, 109.8727, 122.5217, 135.6365, 148.354, 159.7581, 168.9893, 175.3381, 178.3106, 177.6698, 173.4541, 165.9791, 155.8169, 143.7491, 130.6864, 117.5606, 105.2052, 94.2543, 85.0831, 77.8054, 72.3197, 68.3824, 65.6858, 63.9196, 62.8111, 62.143, 61.7553, 61.5375, 61.4183, 61.3537, 61.3181, 61.2973},
{70.171, 70.1705, 70.1776, 70.1998, 70.2514, 70.3571, 70.5587, 70.9227, 71.5491, 72.5795, 74.201, 76.6412, 80.1494, 84.9612, 91.2465, 99.0492, 108.233, 118.4506, 129.1506, 139.626, 149.0953, 156.7991, 162.0931, 164.5227, 163.874, 160.1975, 153.8043, 145.2347, 135.1968, 124.481, 113.859, 103.9877, 95.3384, 88.1654, 82.5189, 78.2893, 75.2678, 73.2052, 71.8573, 71.0124, 70.5034, 70.2077, 70.0413, 69.9497, 69.8996, 69.8714, 69.8545},
{78.7324, 78.7301, 78.7334, 78.7477, 78.7833, 78.8583, 79.0029, 79.2655, 79.7184, 80.4648, 81.6408, 83.4133, 85.967, 89.4802, 94.0879, 99.8381, 106.6492, 114.2819, 122.3364, 130.2808, 137.5082, 143.4126, 147.4691, 149.3032, 148.741, 145.8323, 140.8453, 134.2312, 126.5647, 118.4665, 110.5216, 103.2088, 96.8558, 91.6252, 87.5319, 84.4795, 82.3061, 80.8259, 79.8598, 79.2543, 78.8892, 78.6764, 78.5558, 78.4887, 78.451, 78.429, 78.415},
{87.2941, 87.2903, 87.2904, 87.298, 87.3201, 87.3691, 87.4656, 87.6425, 87.9493, 88.4561, 89.2562, 90.4639, 92.207, 94.6102, 97.7712, 101.7305, 106.441, 111.7465, 117.3755, 122.9566, 128.0571, 132.2361, 135.1061, 136.3884, 135.9549, 133.8472, 130.2704, 125.5635, 120.1489, 114.4726, 108.9446, 103.8909, 99.5267, 95.9516, 93.1649, 91.0931, 89.6211, 88.6196, 87.9661, 87.5562, 87.3082, 87.1629, 87.0797, 87.0323, 87.0047, 86.9877, 86.9759},
{95.856, 95.8511, 95.8487, 95.8511, 95.8629, 95.892, 95.9516, 96.063, 96.2578, 96.5812, 97.0933, 97.8677, 98.9873, 100.5336, 102.5716, 105.1305, 108.1838, 111.6341, 115.3075, 118.9621, 122.3114, 125.0602, 126.9464, 127.7805, 127.4762, 126.0638, 123.6856, 120.5734, 117.0122, 113.2982, 109.6991, 106.4237, 103.6063, 101.3058, 99.5171, 98.1896, 97.2473, 96.6063, 96.1875, 95.924, 95.7638, 95.6688, 95.6134, 95.5807, 95.5607, 95.5474, 95.5374},
{104.4182, 104.4125, 104.4083, 104.4071, 104.4115, 104.4265, 104.4601, 104.5251, 104.6408, 104.8348, 105.1433, 105.6114, 106.2896, 107.2278, 108.4664, 110.0242, 111.8865, 113.9952, 116.245, 118.4876, 120.5461, 122.2365, 123.3947, 123.9018, 123.7039, 122.8209, 121.3436, 119.4181, 117.2229, 114.9413, 112.7373, 110.7372, 109.021, 107.6222, 106.5359, 105.7302, 105.1581, 104.7684, 104.5131, 104.3515, 104.2521, 104.1922, 104.1561, 104.1336, 104.1188, 104.108, 104.0993},
{112.9806, 112.9744, 112.969, 112.9654, 112.965, 112.9707, 112.9871, 113.0216, 113.0853, 113.1941, 113.3688, 113.6354, 114.023, 114.5605, 115.2712, 116.1665, 117.2383, 118.4534, 119.7513, 121.0465, 122.236, 123.2125, 123.88, 124.1689, 124.0477, 123.5285, 122.6652, 121.5436, 120.2682, 118.9456, 117.6704, 116.5151, 115.525, 114.7185, 114.0924, 113.6276, 113.2969, 113.0709, 112.9218, 112.8264, 112.7667, 112.7294, 112.7058, 112.69, 112.6786, 112.6694, 112.6615},
{121.5432, 121.5366, 121.5305, 121.5254, 121.522, 121.522, 121.5278, 121.5436, 121.5752, 121.6315, 121.7238, 121.8663, 122.0748, 122.3652, 122.7503, 123.2364, 123.819, 124.4803, 125.1873, 125.8929, 126.5409, 127.0723, 127.4341, 127.5881, 127.517, 127.2277, 126.7501, 126.1316, 125.4297, 124.703, 124.003, 123.3693, 122.8263, 122.3838, 122.0398, 121.7838, 121.6009, 121.4748, 121.3907, 121.3357, 121.3001, 121.2767, 121.2607, 121.2489, 121.2395, 121.2314, 121.2239},
{130.1058, 130.0991, 130.0926, 130.0866, 130.0815, 130.0782, 130.0778, 130.0827, 130.0958, 130.1216, 130.1662, 130.2367, 130.3416, 130.4888, 130.6852, 130.934, 131.2329, 131.5726, 131.936, 132.2988, 132.6317, 132.9039, 133.088, 133.1641, 133.1232, 132.969, 132.7171, 132.3922, 132.0245, 131.6441, 131.2779, 130.9463, 130.6618, 130.4296, 130.2483, 130.1126, 130.0147, 129.9463, 129.8995, 129.8677, 129.846, 129.8305, 129.8189, 129.8095, 129.8013, 129.7937, 129.7865},
{138.6686, 138.6617, 138.655, 138.6485, 138.6425, 138.6373, 138.6337, 138.6325, 138.6355, 138.6447, 138.6632, 138.6944, 138.7426, 138.8117, 138.905, 139.0241, 139.1679, 139.3318, 139.5074, 139.6826, 139.8431, 139.9736, 140.0607, 140.0944, 140.0705, 139.9907, 139.8627, 139.699, 139.5141, 139.3231, 139.1392, 138.9725, 138.829, 138.7112, 138.6185, 138.5483, 138.4966, 138.4595, 138.4329, 138.4138, 138.3996, 138.3885, 138.3792, 138.3711, 138.3635, 138.3563, 138.3492},
{147.2313, 147.2244, 147.2176, 147.2109, 147.2044, 147.1983, 147.1928, 147.1886, 147.1863, 147.187, 147.192, 147.2029, 147.2218, 147.2506, 147.2906, 147.3428, 147.4065, 147.4797, 147.5583, 147.6367, 147.7082, 147.7656, 147.8027, 147.8147, 147.7997, 147.7585, 147.6947, 147.6142, 147.5238, 147.4306, 147.3408, 147.259, 147.1881, 147.1293, 147.0823, 147.0457, 147.0179, 146.9968, 146.9807, 146.968, 146.9577, 146.9488, 146.9408, 146.9333, 146.926, 146.919, 146.912},
{155.7941, 155.7872, 155.7803, 155.7735, 155.7667, 155.7601, 155.7539, 155.7482, 155.7433, 155.7397, 155.7381, 155.7392, 155.7438, 155.7528, 155.7668, 155.7862, 155.8108, 155.8396, 155.8709, 155.902, 155.93, 155.9518, 155.9644, 155.9659, 155.9553, 155.933, 155.9007, 155.8609, 155.8168, 155.7713, 155.7274, 155.6871, 155.6517, 155.6216, 155.5968, 155.5767, 155.5604, 155.5472, 155.5362, 155.5267, 155.5182, 155.5104, 155.503, 155.4958, 155.4887, 155.4817, 155.4747},
{164.3569, 164.35, 164.3431, 164.3362, 164.3293, 164.3225, 164.3159, 164.3095, 164.3034, 164.2979, 164.2932, 164.2897, 164.2876, 164.2875, 164.2894, 164.2937, 164.3002, 164.3084, 164.3177, 164.3269, 164.3348, 164.3401, 164.3414, 164.3381, 164.3295, 164.3161, 164.2983, 164.2774, 164.2547, 164.2314, 164.2087, 164.1876, 164.1686, 164.1518, 164.1373, 164.1248, 164.1139, 164.1043, 164.0956, 164.0876, 164.08, 164.0727, 164.0655, 164.0585, 164.0515, 164.0445, 164.0375},
{172.9197, 172.9128, 172.9059, 172.8989, 172.892, 172.8852, 172.8783, 172.8716, 172.865, 172.8587, 172.8526, 172.8471, 172.8421, 172.8379, 172.8346, 172.8322, 172.8307, 172.8299, 172.8295, 172.8291, 172.8282, 172.8261, 172.8226, 172.817, 172.8095, 172.7999, 172.7886, 172.776, 172.7626, 172.7491, 172.7358, 172.7231, 172.7112, 172.7003, 172.6903, 172.6811, 172.6726, 172.6645, 172.6569, 172.6495, 172.6423, 172.6352, 172.6282, 172.6212, 172.6143, 172.6073, 172.6004},
{181.4826, 181.4756, 181.4687, 181.4617, 181.4548, 181.4479, 181.441, 181.4341, 181.4273, 181.4206, 181.414, 181.4076, 181.4014, 181.3955, 181.39, 181.3848, 181.38, 181.3754, 181.371, 181.3666, 181.3619, 181.3569, 181.3512, 181.3449, 181.3377, 181.3297, 181.3211, 181.3119, 181.3025, 181.293, 181.2836, 181.2745, 181.2656, 181.2572, 181.249, 181.2412, 181.2337, 181.2263, 181.2191, 181.212, 181.2049, 181.1979, 181.191, 181.184, 181.1771, 181.1701, 181.1632},
{190.0454, 190.0384, 190.0315, 190.0245, 190.0176, 190.0107, 190.0037, 189.9968, 189.9899, 189.9831, 189.9763, 189.9695, 189.9629, 189.9563, 189.9499, 189.9436, 189.9374, 189.9313, 189.9253, 189.9193, 189.9132, 189.907, 189.9005, 189.8938, 189.8867, 189.8794, 189.8718, 189.8641, 189.8562, 189.8484, 189.8405, 189.8328, 189.8251, 189.8176, 189.8102, 189.803, 189.7958, 189.7887, 189.7817, 189.7747, 189.7677, 189.7607, 189.7538, 189.7468, 189.7399, 189.7329, 189.726},
{198.6082, 198.6012, 198.5943, 198.5874, 198.5804, 198.5735, 198.5665, 198.5596, 198.5527, 198.5458, 198.5389, 198.532, 198.5252, 198.5183, 198.5116, 198.5049, 198.4982, 198.4916, 198.4849, 198.4783, 198.4717, 198.465, 198.4582, 198.4513, 198.4444, 198.4373, 198.4301, 198.4229, 198.4156, 198.4084, 198.4011, 198.3939, 198.3867, 198.3795, 198.3724, 198.3654, 198.3584, 198.3514, 198.3444, 198.3374, 198.3305, 198.3235, 198.3166, 198.3096, 198.3027, 198.2957, 198.2888}

};//*/

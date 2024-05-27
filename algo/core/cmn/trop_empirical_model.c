#include "trop_empirical_model.h"
#include "gnss_def.h"
#include "gnss_common.h"
#include <math.h>

#define NMAX  (9)
#define MMAX  (9)
#define ROW_NUM_MAX (NMAX+1)
#define COL_NUM_MAX (MMAX+1)

const double gd_TEMP0 = 15.0; /* temparature at sea level */
const double gpd_A_GEOID[55] = {
		-5.6195e-001,-6.0794e-002,-2.0125e-001,-6.4180e-002,-3.6997e-002,
		+1.0098e+001,+1.6436e+001,+1.4065e+001,+1.9881e+000,+6.4414e-001,
		-4.7482e+000,-3.2290e+000,+5.0652e-001,+3.8279e-001,-2.6646e-002,
		+1.7224e+000,-2.7970e-001,+6.8177e-001,-9.6658e-002,-1.5113e-002,
		+2.9206e-003,-3.4621e+000,-3.8198e-001,+3.2306e-002,+6.9915e-003,
		-2.3068e-003,-1.3548e-003,+4.7324e-006,+2.3527e+000,+1.2985e+000,
		+2.1232e-001,+2.2571e-002,-3.7855e-003,+2.9449e-005,-1.6265e-004,
		+1.1711e-007,+1.6732e+000,+1.9858e-001,+2.3975e-002,-9.0013e-004,
		-2.2475e-003,-3.3095e-005,-1.2040e-005,+2.2010e-006,-1.0083e-006,
		+8.6297e-001,+5.8231e-001,+2.0545e-002,-7.8110e-003,-1.4085e-004,
		-8.8459e-006,+5.7256e-006,-1.5068e-006,+4.0095e-007,-2.4185e-008 };
const double gpd_B_GEOID[55] = {
	+0.0000e+000,+0.0000e+000,-6.5993e-002,+0.0000e+000,+6.5364e-002,
	-5.8320e+000,+0.0000e+000,+1.6961e+000,-1.3557e+000,+1.2694e+000,
	0.0000e+000,-2.9310e+000,+9.4805e-001,-7.6243e-002,+4.1076e-002,
	+0.0000e+000,-5.1808e-001,-3.4583e-001,-4.3632e-002,+2.2101e-003,
	-1.0663e-002,+0.0000e+000,+1.0927e-001,-2.9463e-001,+1.4371e-003,
	-1.1452e-002,-2.8156e-003,-3.5330e-004,+0.0000e+000,+4.4049e-001,
	+5.5653e-002,-2.0396e-002,-1.7312e-003,+3.5805e-005,+7.2682e-005,
	+2.2535e-006,+0.0000e+000,+1.9502e-002,+2.7919e-002,-8.1812e-003,
	+4.4540e-004,+8.8663e-005,+5.5596e-005,+2.4826e-006,+1.0279e-006,
	+0.0000e+000,+6.0529e-002,-3.5824e-002,-5.1367e-003,+3.0119e-005,
	-2.9911e-005,+1.9844e-005,-1.2349e-006,-7.6756e-009,+5.0100e-008
};
const double gpd_AP_MEAN[55] = {
	+1.0108e+003,+8.4886e+000,+1.4799e+000,-1.3897e+001,+3.7516e-003,
	-1.4936e-001,+1.2232e+001,-7.6615e-001,-6.7699e-002,+8.1002e-003,
	-1.5874e+001,+3.6614e-001,-6.7807e-002,-3.6309e-003,+5.9966e-004,
	+4.8163e+000,-3.7363e-001,-7.2071e-002,+1.9998e-003,-6.2385e-004,
	-3.7916e-004,+4.7609e+000,-3.9534e-001,+8.6667e-003,+1.1569e-002,
	+1.1441e-003,-1.4193e-004,-8.5723e-005,+6.5008e-001,-5.0889e-001,
	-1.5754e-002,-2.8305e-003,+5.7458e-004,+3.2577e-005,-9.6052e-006,
	-2.7974e-006,+1.3530e+000,-2.7271e-001,-3.0276e-004,+3.6286e-003,
	-2.0398e-004,+1.5846e-005,-7.7787e-006,+1.1210e-006,+9.9020e-008,
	+5.5046e-001,-2.7312e-001,+3.2532e-003,-2.4277e-003,+1.1596e-004,
	+2.6421e-007,-1.3263e-006,+2.7322e-007,+1.4058e-007,+4.9414e-009
};
const double gpd_BP_MEAN[55] = {
	+0.0000e+000,+0.0000e+000,-1.2878e+000,+0.0000e+000,+7.0444e-001,
	+3.3222e-001,+0.0000e+000,-2.9636e-001,+7.2248e-003,+7.9655e-003,
	+0.0000e+000,+1.0854e+000,+1.1145e-002,-3.6513e-002,+3.1527e-003,
	+0.0000e+000,-4.8434e-001,+5.2023e-002,-1.3091e-002,+1.8515e-003,
	+1.5422e-004,+0.0000e+000,+6.8298e-001,+2.5261e-003,-9.9703e-004,
	-1.0829e-003,+1.7688e-004,-3.1418e-005,+0.0000e+000,-3.7018e-001,
	+4.3234e-002,+7.2559e-003,+3.1516e-004,+2.0024e-005,-8.0581e-006,
	-2.3653e-006,+0.0000e+000,+1.0298e-001,-1.5086e-002,+5.6186e-003,
	+3.2613e-005,+4.0567e-005,-1.3925e-006,-3.6219e-007,-2.0176e-008,
	+0.0000e+000,-1.8364e-001,+1.8508e-002,+7.5016e-004,-9.6139e-005,
	-3.1995e-006,+1.3868e-007,-1.9486e-007,+3.0165e-010,-6.4376e-010
};
const double gpd_AP_AMP[55] = {
	-1.0444e-001,+1.6618e-001,-6.3974e-002,+1.0922e+000,+5.7472e-001,
	-3.0277e-001,-3.5087e+000,+7.1264e-003,-1.4030e-001,+3.7050e-002,
	+4.0208e-001,-3.0431e-001,-1.3292e-001,+4.6746e-003,-1.5902e-004,
	+2.8624e+000,-3.9315e-001,-6.4371e-002,+1.6444e-002,-2.3403e-003,
	+4.2127e-005,+1.9945e+000,-6.0907e-001,-3.5386e-002,-1.0910e-003,
	-1.2799e-004,+4.0970e-005,+2.2131e-005,-5.3292e-001,-2.9765e-001,
	-3.2877e-002,+1.7691e-003,+5.9692e-005,+3.1725e-005,+2.0741e-005,
	-3.7622e-007,+2.6372e+000,-3.1165e-001,+1.6439e-002,+2.1633e-004,
	+1.7485e-004,+2.1587e-005,+6.1064e-006,-1.3755e-008,-7.8748e-008,
	-5.9152e-001,-1.7676e-001,+8.1807e-003,+1.0445e-003,+2.3432e-004,
	+9.3421e-006,+2.8104e-006,-1.5788e-007,-3.0648e-008,+2.6421e-010
};
const double gpd_BP_AMP[55] = {
	+0.0000e+000,+0.0000e+000,+9.3340e-001,+0.0000e+000,+8.2346e-001,
	+2.2082e-001,+0.0000e+000,+9.6177e-001,-1.5650e-002,+1.2708e-003,
	+0.0000e+000,-3.9913e-001,+2.8020e-002,+2.8334e-002,+8.5980e-004,
	+0.0000e+000,+3.0545e-001,-2.1691e-002,+6.4067e-004,-3.6528e-005,
	-1.1166e-004,+0.0000e+000,-7.6974e-002,-1.8986e-002,+5.6896e-003,
	-2.4159e-004,-2.3033e-004,-9.6783e-006,+0.0000e+000,-1.0218e-001,
	-1.3916e-002,-4.1025e-003,-5.1340e-005,-7.0114e-005,-3.3152e-007,
	+1.6901e-006,+0.0000e+000,-1.2422e-002,+2.5072e-003,+1.1205e-003,
	-1.3034e-004,-2.3971e-005,-2.6622e-006,+5.7852e-007,+4.5847e-008,
	+0.0000e+000,+4.4777e-002,-3.0421e-003,+2.6062e-005,-7.2421e-005,
	+1.9119e-006,+3.9236e-007,+2.2390e-007,+2.9765e-009,-4.6452e-009
};
const double gpd_AT_MEAN[55] = {
	+1.6257e+001,+2.1224e+000,+9.2569e-001,-2.5974e+001,+1.4510e+000,
	+9.2468e-002,-5.3192e-001,+2.1094e-001,-6.9210e-002,-3.4060e-002,
	-4.6569e+000,+2.6385e-001,-3.6093e-002,+1.0198e-002,-1.8783e-003,
	+7.4983e-001,+1.1741e-001,+3.9940e-002,+5.1348e-003,+5.9111e-003,
	+8.6133e-006,+6.3057e-001,+1.5203e-001,+3.9702e-002,+4.6334e-003,
	+2.4406e-004,+1.5189e-004,+1.9581e-007,+5.4414e-001,+3.5722e-001,
	+5.2763e-002,+4.1147e-003,-2.7239e-004,-5.9957e-005,+1.6394e-006,
	-7.3045e-007,-2.9394e+000,+5.5579e-002,+1.8852e-002,+3.4272e-003,
	-2.3193e-005,-2.9349e-005,+3.6397e-007,+2.0490e-006,-6.4719e-008,
	-5.2225e-001,+2.0799e-001,+1.3477e-003,+3.1613e-004,-2.2285e-004,
	-1.8137e-005,-1.5177e-007,+6.1343e-007,+7.8566e-008,+1.0749e-009
};
const double gpd_BT_MEAN[55] = {
	+0.0000e+000,+0.0000e+000,+1.0210e+000,+0.0000e+000,+6.0194e-001,
	+1.2292e-001,+0.0000e+000,-4.2184e-001,+1.8230e-001,+4.2329e-002,
	+0.0000e+000,+9.3312e-002,+9.5346e-002,-1.9724e-003,+5.8776e-003,
	+0.0000e+000,-2.0940e-001,+3.4199e-002,-5.7672e-003,-2.1590e-003,
	+5.6815e-004,+0.0000e+000,+2.2858e-001,+1.2283e-002,-9.3679e-003,
	-1.4233e-003,-1.5962e-004,+4.0160e-005,+0.0000e+000,+3.6353e-002,
	-9.4263e-004,-3.6762e-003,+5.8608e-005,-2.6391e-005,+3.2095e-006,
	-1.1605e-006,+0.0000e+000,+1.6306e-001,+1.3293e-002,-1.1395e-003,
	+5.1097e-005,+3.3977e-005,+7.6449e-006,-1.7602e-007,-7.6558e-008,
	+0.0000e+000,-4.5415e-002,-1.8027e-002,+3.6561e-004,-1.1274e-004,
	+1.3047e-005,+2.0001e-006,-1.5152e-007,-2.7807e-008,+7.7491e-009
};
const double gpd_AT_AMP[55] = {
	-1.8654e+000,-9.0041e+000,-1.2974e-001,-3.6053e+000,+2.0284e-002,
	+2.1872e-001,-1.3015e+000,+4.0355e-001,+2.2216e-001,-4.0605e-003,
	+1.9623e+000,+4.2887e-001,+2.1437e-001,-1.0061e-002,-1.1368e-003,
	-6.9235e-002,+5.6758e-001,+1.1917e-001,-7.0765e-003,+3.0017e-004,
	+3.0601e-004,+1.6559e+000,+2.0722e-001,+6.0013e-002,+1.7023e-004,
	-9.2424e-004,+1.1269e-005,-6.9911e-006,-2.0886e+000,-6.7879e-002,
	-8.5922e-004,-1.6087e-003,-4.5549e-005,+3.3178e-005,-6.1715e-006,
	-1.4446e-006,-3.7210e-001,+1.5775e-001,-1.7827e-003,-4.4396e-004,
	+2.2844e-004,-1.1215e-005,-2.1120e-006,-9.6421e-007,-1.4170e-008,
	+7.8720e-001,-4.4238e-002,-1.5120e-003,-9.4119e-004,+4.0645e-006,
	-4.9253e-006,-1.8656e-006,-4.0736e-007,-4.9594e-008,+1.6134e-009
};
const double gpd_BT_AMP[55] = {
	+0.0000e+000,+0.0000e+000,-8.9895e-001,+0.0000e+000,-1.0790e+000,
	-1.2699e-001,+0.0000e+000,-5.9033e-001,+3.4865e-002,-3.2614e-002,
	+0.0000e+000,-2.4310e-002,+1.5607e-002,-2.9833e-002,-5.9048e-003,
	+0.0000e+000,+2.8383e-001,+4.0509e-002,-1.8834e-002,-1.2654e-003,
	-1.3794e-004,+0.0000e+000,+1.3306e-001,+3.4960e-002,-3.6799e-003,
	-3.5626e-004,+1.4814e-004,+3.7932e-006,+0.0000e+000,+2.0801e-001,
	+6.5640e-003,-3.4893e-003,-2.7395e-004,+7.4296e-005,-7.9927e-006,
	-1.0277e-006,+0.0000e+000,+3.6515e-002,-7.4319e-003,-6.2873e-004,
	-8.2461e-005,+3.1095e-005,-5.3860e-007,-1.2055e-007,-1.1517e-007,
	+0.0000e+000,+3.1404e-002,+1.5580e-002,-1.1428e-003,+3.3529e-005,
	+1.0387e-005,-1.9378e-006,-2.7327e-007,+7.5833e-009,-9.2323e-009
};

const double gpd_AH_MEAN[55] = {
	+1.2517e+02, +8.503e-01, +6.936e-02, -6.760e+00, +1.771e-01,
	+1.130e-02, +5.963e-01, +1.808e-02, +2.801e-03, -1.414e-03,
	-1.212e+00, +9.300e-02, +3.683e-03, +1.095e-03, +4.671e-05,
	+3.959e-01, -3.867e-02, +5.413e-03, -5.289e-04, +3.229e-04,
	+2.067e-05, +3.000e-01, +2.031e-02, +5.900e-03, +4.573e-04,
	-7.619e-05, +2.327e-06, +3.845e-06, +1.182e-01, +1.158e-02,
	+5.445e-03, +6.219e-05, +4.204e-06, -2.093e-06, +1.540e-07,
	-4.280e-08, -4.751e-01, -3.490e-02, +1.758e-03, +4.019e-04,
	-2.799e-06, -1.287e-06, +5.468e-07, +7.580e-08, -6.300e-09,
	-1.160e-01, +8.301e-03, +8.771e-04, +9.955e-05, -1.718e-06,
	-2.012e-06, +1.170e-08, +1.790e-08, -1.300e-09, +1.000e-10
};
const double gpd_BH_MEAN[55] = {
	+0.000e+00, +0.000e+00, +3.249e-02, +0.000e+00, +3.324e-02,
	+1.850e-02, +0.000e+00, -1.115e-01, +2.519e-02, +4.923e-03,
	+0.000e+00, +2.737e-02, +1.595e-02, -7.332e-04, +1.933e-04,
	+0.000e+00, -4.796e-02, +6.381e-03, -1.599e-04, -3.685e-04,
	+1.815e-05, +0.000e+00, +7.033e-02, +2.426e-03, -1.111e-03,
	-1.357e-04, -7.828e-06, +2.547e-06, +0.000e+00, +5.779e-03,
	+3.133e-03, -5.312e-04, -2.028e-05, +2.323e-07, -9.100e-08,
	-1.650e-08, +0.000e+00, +3.688e-02, -8.638e-04, -8.514e-05,
	-2.828e-05, +5.403e-07, +4.390e-07, +1.350e-08, +1.800e-09,
	+0.000e+00, -2.736e-02, -2.977e-04, +8.113e-05, +2.329e-07,
	+8.451e-07, +4.490e-08, -8.100e-09, -1.500e-09, +2.000e-10
};
const double gpd_AH_AMP[55] = {
	-2.738e-01, -2.837e+00, +1.298e-02, -3.588e-01, +2.413e-02,
	+3.427e-02, -7.624e-01, +7.272e-02, +2.160e-02, -3.385e-03,
	+4.424e-01, +3.722e-02, +2.195e-02, -1.503e-03, +2.426e-04,
	+3.013e-01, +5.762e-02, +1.019e-02, -4.476e-04, +6.790e-05,
	+3.227e-05, +3.123e-01, -3.535e-02, +4.840e-03, +3.025e-06,
	-4.363e-05, +2.854e-07, -1.286e-06, -6.725e-01, -3.730e-02,
	+8.964e-04, +1.399e-04, -3.990e-06, +7.431e-06, -2.796e-07,
	-1.601e-07, +4.068e-02, -1.352e-02, +7.282e-04, +9.594e-05,
	+2.070e-06, -9.620e-08, -2.742e-07, -6.370e-08, -6.300e-09,
	+8.625e-02, -5.971e-03, +4.705e-04, +2.335e-05, +4.226e-06,
	+2.475e-07, -8.850e-08, -3.600e-08, -2.900e-09, +0.000e+00
};
const double gpd_BH_AMP[55] = {
	+0.000e+00, +0.000e+00, -1.136e-01, +0.000e+00, -1.868e-01,
	-1.399e-02, +0.000e+00, -1.043e-01, +1.175e-02, -2.240e-03,
	+0.000e+00, -3.222e-02, +1.333e-02, -2.647e-03, -2.316e-05,
	+0.000e+00, +5.339e-02, +1.107e-02, -3.116e-03, -1.079e-04,
	-1.299e-05, +0.000e+00, +4.861e-03, +8.891e-03, -6.448e-04,
	-1.279e-05, +6.358e-06, -1.417e-07, +0.000e+00, +3.041e-02,
	+1.150e-03, -8.743e-04, -2.781e-05, +6.367e-07, -1.140e-08,
	-4.200e-08, +0.000e+00, -2.982e-02, -3.000e-03, +1.394e-05,
	-3.290e-05, -1.705e-07, +7.440e-08, +2.720e-08, -6.600e-09,
	+0.000e+00, +1.236e-02, -9.981e-04, -3.792e-05, -1.355e-05,
	+1.162e-06, -1.789e-07, +1.470e-08, -2.400e-09, -4.000e-10
};
const double gpd_AW_MEAN[55] = {
	+5.640e+01, +1.555e+00, -1.011e+00, -3.975e+00, +3.171e-02,
	+1.065e-01, +6.175e-01, +1.376e-01, +4.229e-02, +3.028e-03,
	+1.688e+00, -1.692e-01, +5.478e-02, +2.473e-02, +6.059e-04,
	+2.278e+00, +6.614e-03, -3.505e-04, -6.697e-03, +8.402e-04,
	+7.033e-04, -3.236e+00, +2.184e-01, -4.611e-02, -1.613e-02,
	-1.604e-03, +5.420e-05, +7.922e-05, -2.711e-01, -4.406e-01,
	-3.376e-02, -2.801e-03, -4.090e-04, -2.056e-05, +6.894e-06,
	+2.317e-06, +1.941e+00, -2.562e-01, +1.598e-02, +5.449e-03,
	+3.544e-04, +1.148e-05, +7.503e-06, -5.667e-07, -3.660e-08,
	+8.683e-01, -5.931e-02, -1.864e-03, -1.277e-04, +2.029e-04,
	+1.269e-05, +1.629e-06, +9.660e-08, -1.015e-07, -5.000e-10
};
const double gpd_BW_MEAN[55] = {
	+0.000e+00, +0.000e+00, +2.592e-01, +0.000e+00, +2.974e-02,
	-5.471e-01, +0.000e+00, -5.926e-01, -1.030e-01, -1.567e-02,
	+0.000e+00, +1.710e-01, +9.025e-02, +2.689e-02, +2.243e-03,
	+0.000e+00, +3.439e-01, +2.402e-02, +5.410e-03, +1.601e-03,
	+9.669e-05, +0.000e+00, +9.502e-02, -3.063e-02, -1.055e-03,
	-1.067e-04, -1.130e-04, +2.124e-05, +0.000e+00, -3.129e-01,
	+8.463e-03, +2.253e-04, +7.413e-05, -9.376e-05, -1.606e-06,
	+2.060e-06, +0.000e+00, +2.739e-01, +1.167e-03, -2.246e-05,
	-1.287e-04, -2.438e-05, -7.561e-07, +1.158e-06, +4.950e-08,
	+0.000e+00, -1.344e-01, +5.342e-03, +3.775e-04, -6.756e-05,
	-1.686e-06, -1.184e-06, +2.768e-07, +2.730e-08, +5.700e-09
};
const double gpd_AW_AMP[55] = {
	+1.023e-01, -2.695e+00, +3.417e-01, -1.405e-01, +3.175e-01,
	+2.116e-01, +3.536e+00, -1.505e-01, -1.660e-02, +2.967e-02,
	+3.819e-01, -1.695e-01, -7.444e-02, +7.409e-03, -6.262e-03,
	-1.836e+00, -1.759e-02, -6.256e-02, -2.371e-03, +7.947e-04,
	+1.501e-04, -8.603e-01, -1.360e-01, -3.629e-02, -3.706e-03,
	-2.976e-04, +1.857e-05, +3.021e-05, +2.248e+00, -1.178e-01,
	+1.255e-02, +1.134e-03, -2.161e-04, -5.817e-06, +8.836e-07,
	-1.769e-07, +7.313e-01, -1.188e-01, +1.145e-02, +1.011e-03,
	+1.083e-04, +2.570e-06, -2.140e-06, -5.710e-08, +2.000e-08,
	-1.632e+00, -6.948e-03, -3.893e-03, +8.592e-04, +7.577e-05,
	+4.539e-06, -3.852e-07, -2.213e-07, -1.370e-08, +5.800e-09
};
const double gpd_BW_AMP[55] = {
	+0.000e+00, +0.000e+00, -8.865e-02, +0.000e+00, -4.309e-01,
	+6.340e-02, +0.000e+00, +1.162e-01, +6.176e-02, -4.234e-03,
	+0.000e+00, +2.530e-01, +4.017e-02, -6.204e-03, +4.977e-03,
	+0.000e+00, -1.737e-01, -5.638e-03, +1.488e-04, +4.857e-04,
	-1.809e-04, +0.000e+00, -1.514e-01, -1.685e-02, +5.333e-03,
	-7.611e-05, +2.394e-05, +8.195e-06, +0.000e+00, +9.326e-02,
	-1.275e-02, -3.071e-04, +5.374e-05, -3.391e-05, -7.436e-06,
	+6.747e-07, +0.000e+00, -8.637e-02, -3.807e-03, -6.833e-04,
	-3.861e-05, -2.268e-05, +1.454e-06, +3.860e-07, -1.068e-07,
	+0.000e+00, -2.658e-02, -1.947e-03, +7.131e-04, -3.506e-05,
	+1.885e-07, +5.792e-07, +3.990e-08, +2.000e-08, -5.700e-09
};
/**
 * @brief global pressure temperature(GPT) model for troposphere
 * @param[in]  d_mjd Modified Julian Day
 * @param[in]  d_lat latitude in geodetic coordinates of site
 * @param[in]  d_lon longitude in geodetic coordinates of site
 * @param[in]  d_hgt height in geodetic coordinates of site
 * @param[out] pd_pres is pressure in site location
 * @param[out] pd_temperature is temperature in site location
 * @param[out] pd_undu is the humidity in site location
 * @return     void
 */
void tropGPTmodel(double d_mjd, double d_lat, double d_lon, double d_hgt, double* pd_pres, double* pd_temperature, double* pd_undu)
{
	uint16_t w_i = 0;
	uint16_t w_j = 0;
	uint16_t w_n = 0;
	uint16_t w_m = 0;
	uint16_t w_nMax = 0;
	uint16_t w_mMax = 0;
	uint16_t w_dM = 0;
	uint16_t w_dN = 0;
	uint16_t w_m1Index = 0;
	double pd_V[10][10];
	double pd_W[10][10];
	double d_doy = 0.0;
	double d_cosLat = 0.0;
	double d_temperature0 = 0.0;
	double d_pres0 = 0.0;
	double d_apm = 0.0;
	double d_apa = 0.0;
	double d_atm = 0.0;
	double d_ata = 0.0;
	double d_hort = 0.0;
	double d_x = 0.0;
	double d_y = 0.0;
	double d_z = 0.0;
	double d_rec = 0.0;
	for (w_i = 0; w_i < 10; ++w_i)
	{
		for (w_j = 0; w_j < 10; ++w_j)
		{
			pd_V[w_i][w_j] = pd_W[w_i][w_j] = 0.0;
		}
	}
	//Reference day is 28 January 1980                                           
	//This is taken from Niell (1996) to be consistent (See References)          
	//For constant values use: doy = 91.3125
	d_doy = d_mjd - 44239 + 1 - 28;
	//Define degree n and order m EGM  
	w_nMax = w_mMax = 9;
	//Define unit vector
	d_cosLat = cos(d_lat);
	d_x = d_cosLat * cos(d_lon);
	d_y = d_cosLat * sin(d_lon);
	d_z = sin(d_lat);
	//Legendre polynomials                                                       
	pd_V[0][0] = 1.0;
	pd_W[0][0] = 0.0;
	pd_V[1][0] = d_z * pd_V[0][0];
	pd_W[1][0] = 0.0;

	for (w_n = 2; w_n <= w_nMax; w_n++)
	{
		pd_V[w_n][0] = ((2.0 * w_n - 1) * d_z * pd_V[w_n - 1][0] - (w_n - 1) * pd_V[w_n - 2][0]) / w_n;
		pd_W[w_n][0] = 0.0;
	}
	for (w_m = 1; w_m <= w_nMax; w_m++)
	{
		w_dM = 2 * w_m;
		w_m1Index = w_m - 1;
		pd_V[w_m][w_m] = (w_dM - 1.0) * (d_x * pd_V[w_m1Index][w_m1Index] - d_y * pd_W[w_m1Index][w_m1Index]);
		pd_W[w_m][w_m] = (w_dM - 1.0) * (d_x * pd_W[w_m1Index][w_m1Index] + d_y * pd_V[w_m1Index][w_m1Index]);
		if (w_m < w_nMax)
		{
			pd_V[w_m + 1][w_m] = (w_dM + 1.0) * d_z * pd_V[w_m][w_m];
			pd_W[w_m + 1][w_m] = (w_dM + 1.0) * d_z * pd_W[w_m][w_m];
		}
		for (w_n = w_m + 2; w_n <= w_nMax; w_n++)
		{
			w_dN = 2 * w_n;
			d_rec = 1.0 / (w_n - w_m);
			pd_V[w_n][w_m] = ((w_dN - 1.0) * d_z * pd_V[w_n - 1][w_m] - (w_n + w_m1Index) * pd_V[w_n - 2][w_m]) * d_rec;
			pd_W[w_n][w_m] = ((w_dN - 1.0) * d_z * pd_W[w_n - 1][w_m] - (w_n + w_m1Index) * pd_W[w_n - 2][w_m]) * d_rec;
		}
	}
	//Geoidal height                                                             
	*pd_undu = 0.0;
	w_i = 0;
	for (w_n = 0; w_n <= w_nMax; w_n++)
	{
		for (w_m = 0; w_m <= w_n; w_m++)
		{
			*pd_undu = *pd_undu + (gpd_A_GEOID[w_i] * pd_V[w_n][w_m] + gpd_B_GEOID[w_i] * pd_W[w_n][w_m]);
			w_i = w_i + 1;
		}
	}
	//orthometric height                                                         
	d_hort = d_hgt - *pd_undu;
	//Surface pressure on the geoid                                              
	d_apm = 0.0;
	d_apa = 0.0;
	w_i = 0;
	for (w_n = 0; w_n <= w_nMax; w_n++)
	{
		for (w_m = 0; w_m <= w_n; w_m++)
		{
			d_apm = d_apm + (gpd_AP_MEAN[w_i] * pd_V[w_n][w_m] + gpd_BP_MEAN[w_i] * pd_W[w_n][w_m]);
			d_apa = d_apa + (gpd_AP_AMP[w_i] * pd_V[w_n][w_m] + gpd_BP_AMP[w_i] * pd_W[w_n][w_m]);
			w_i = w_i + 1;
		}
	}
	d_pres0 = d_apm + d_apa * cos(d_doy / 365.25 * 2.0 * PI);
	//height correction for pressure                                             
	*pd_pres = d_pres0 * pow(1.0 - 0.0000226 * d_hort, 5.225);
	//Surface temperature on the geoid                                           
	d_atm = 0.0;
	d_ata = 0.0;
	w_i = 0;
	for (w_n = 0; w_n <= w_nMax; w_n++)
	{
		for (w_m = 0; w_m <= w_n; w_m++)
		{
			d_atm = d_atm + (gpd_AT_MEAN[w_i] * pd_V[w_n][w_m] + gpd_BT_MEAN[w_i] * pd_W[w_n][w_m]);
			d_ata = d_ata + (gpd_AT_AMP[w_i] * pd_V[w_n][w_m] + gpd_BT_AMP[w_i] * pd_W[w_n][w_m]);
			w_i = w_i + 1;
		}
	}
	d_temperature0 = d_atm + d_ata * cos(d_doy / 365.25 * 2.0 * PI);
	//height correction for temperature 
	*pd_temperature = d_temperature0 - 0.0065 * d_hort;
	return;
}
/**
 * @brief using Saastamoinen(SAAS) model to calculate the empirical value of troposphere
 * @param[in]  z_obsTime observation time of calculating the empirical value of troposphere
 * @param[in]  pd_LLA geodetic coordinates of site, rad
 * @param[in]  d_humi the humidity of site,usually set to 0.7
 * @param[in]  u_metOpt whether using global pressure temperature(GPT) model,1 represent using GPT
 * @param[out] pd_zenithDry is the zenith tropospheric delaly of dry part
 * @param[out] pd_zenithWet is the zenith tropospheric delaly of wet part
 * @return 1 represent failure, 0 reprenset success
 */
extern uint8_t usingSaasModelCalZenithTropDelay(GpsTime_t z_obsTime, const double pd_LLA[3], double d_humi, uint8_t u_metOpt,
                                         double* pd_zenithDry, double* pd_zenithWet)
{
	double d_hgt = 0.0;
	double d_pres = 0.0;
	double d_temperature = 0.0;
	double d_e = 0.0;
	double d_mjd = 0.0;
	double d_undu = 0.0;
	EpochTime_t z_calenderTime0 = { 0 };
	UtcTime_t z_utcTime0 = { 0 };
	GpsTime_t z_gpsTime0 = { 0 };
	z_calenderTime0.year = 2000;
	z_calenderTime0.month = 1;
	z_calenderTime0.day = 1;
	z_calenderTime0.hour = 12;
	z_calenderTime0.min = 0;
	z_calenderTime0.second = 0.0;

	if (pd_LLA[2] < -100.0 || 1e4 < pd_LLA[2])
	{
		return 1;
	}

	/* standard atmosphere */
	d_hgt = pd_LLA[2] < 0.0 ? 0.0 : (pd_LLA[2] < 5e3 ? pd_LLA[2] : 5e3);//d_hgt=pd_LLA[2]<0.0?0.0:pd_LLA[2];

	if (u_metOpt)
	{
		tm_cvt_EpochToUtcTime(&z_calenderTime0, &z_utcTime0);
		tm_cvt_UtcTimeToGpst(&z_utcTime0, &z_gpsTime0);
		d_mjd = 51544.5 + tm_GpsTimeDiff(&z_obsTime, &z_gpsTime0) / 86400.0;
		tropGPTmodel(d_mjd, pd_LLA[0], pd_LLA[1], d_hgt, &d_pres, &d_temperature, &d_undu);//d_pres:hPa(=mbar), d_temperature:Celsius deg, undu:m
		d_temperature += 273.15;	//ppy, Celsius-> Kelvin
	}
	else
	{
		/* standard atmosphere */
		d_pres = 1013.25 * pow(1.0 - 2.2557e-5 * d_hgt, 5.2568);
		d_temperature = gd_TEMP0 - 6.5e-3 * d_hgt + 273.16;
	}
	d_e = 6.108 * d_humi * exp((17.15 * d_temperature - 4684.0) / (d_temperature - 38.45));
	//trph = 0.0022768*pres / (1.0 - 0.00266*cos(2.0*pos[0]) - 0.00028*d_hgt / 1e3) / cos(z);
	//trpw = 0.002277*(1255.0 / d_temperature + 0.05)*e / cos(z);
	*pd_zenithDry = 0.0022768 * d_pres / (1.0 - 0.00266 * cos(2.0 * pd_LLA[0]) - 0.00028 * d_hgt / 1e3); //Davis et al. (1985)
	*pd_zenithWet = 0.002277 * (1255.0 / d_temperature + 0.05) * d_e / (1.0 - 0.00266 * cos(2.0 * pd_LLA[0]) - 0.00028 * d_hgt / 1e3);
	return 0;
}
/**
 * @brief using GMF model to calculate the map value of troposphere
 * @param[in]  d_ele the elevation of satellite
 * @param[in]  d_a
 * @param[in]  d_b
 * @param[in]  d_c
 * @return     the map value of troposphere calculating by GMF
 */
double mapfuncByGMF(double d_ele, double d_a, double d_b, double d_c)
{
	double d_sinEle = sin(d_ele);
	return (1.0 + d_a / (1.0 + d_b / (1.0 + d_c))) / (d_sinEle + (d_a / (d_sinEle + d_b / (d_sinEle + d_c))));
}
/**
 * @brief using GMF model to calculate the map value of troposphere
 * @param[in]  d_doy is the day of year
 * @param[in]  pd_LLA geodetic coordinates of site, rad
 * @param[in]  d_ele the elevation of satellite, rad
 * @param[out] pd_mapDry is the tropospheric map of dry part
 * @param[out] pd_mapWet is the tropospheric map of wet part
 * @return     uint8_t  1 represent failure, 0 reprenset success
 */
extern uint8_t usingGMFcalculateTropMap(double d_doy, const double pd_LLA[3], double d_ele, double* pd_mapDry, double* pd_mapWet)
{
	uint16_t w_i = 0;
	uint16_t w_j = 0;
	uint16_t w_n = 0;
	uint16_t w_m = 0;
	uint16_t w_dM = 0;
	uint16_t w_dN = 0;
	uint16_t w_m1Index = 0;
	double pd_aht[3] = { 2.53e-5, 5.49e-3, 1.14e-3 }; /* height correction */
	double pd_V[ROW_NUM_MAX][COL_NUM_MAX];
	double pd_W[ROW_NUM_MAX][COL_NUM_MAX];
	double d_x = 0.0;
	double d_y = 0.0;
	double d_z = 0.0;
	double pd_ah[3] = { 0.0 };
	double pd_aw[3] = { 0.0 };
	double d_c0h = 0.0;
	double d_phh = 0.0;
	double d_c11h = 0.0;
	double d_c10h = 0.0;
	double d_ahm = 0.0;
	double d_aha = 0.0;
	double d_awm = 0.0;
	double d_awa = 0.0;
	double d_dm = 0.0;
	double d_cosLat = 0.0;
	double d_rec = 0.0;
	double d_doyTran = (d_doy - 28) / 365.25 * 2 * PI;
	if (d_ele <= 0.0)
	{
		return 1;
	}
	for (w_i = 0; w_i < ROW_NUM_MAX; ++w_i)
	{
		for (w_j = 0; w_j < COL_NUM_MAX; ++w_j)
		{
			pd_V[w_i][w_j] = pd_W[w_i][w_j] = 0.0;
		}
	}
	//Define unit vector
	d_cosLat = cos(pd_LLA[0]);
	d_x = d_cosLat * cos(pd_LLA[1]);
	d_y = d_cosLat * sin(pd_LLA[1]);
	d_z = sin(pd_LLA[0]);

	//Legendre polynomials                                                       
	pd_V[0][0] = 1.0;
	pd_W[0][0] = 0.0;
	pd_V[1][0] = d_z * pd_V[0][0];
	pd_W[1][0] = 0.0;

	for (w_n = 2; w_n <= NMAX; w_n++)
	{
		pd_V[w_n][0] = ((2.0 * w_n - 1) * d_z * pd_V[w_n - 1][0] - (w_n - 1) * pd_V[w_n - 2][0]) / w_n;
		pd_W[w_n][0] = 0.0;
	}
	for (w_m = 1; w_m <= NMAX; w_m++)
	{
		w_dM = 2 * w_m;
		w_m1Index = w_m - 1;
		pd_V[w_m][w_m] = (w_dM - 1) * (d_x * pd_V[w_m1Index][w_m1Index] - d_y * pd_W[w_m1Index][w_m1Index]);
		pd_W[w_m][w_m] = (w_dM - 1) * (d_x * pd_W[w_m1Index][w_m1Index] + d_y * pd_V[w_m1Index][w_m1Index]);
		if (w_m < NMAX)
		{
			pd_V[w_m + 1][w_m] = (w_dM + 1.0) * d_z * pd_V[w_m][w_m];
			pd_W[w_m + 1][w_m] = (w_dM + 1.0) * d_z * pd_W[w_m][w_m];
		}
		for (w_n = w_m + 2; w_n <= NMAX; w_n++)
		{
			w_dN = 2 * w_n;
			d_rec = 1.0 / (w_n - w_m);
			pd_V[w_n][w_m] = ((w_dN - 1) * d_z * pd_V[w_n - 1][w_m] - (w_n + w_m1Index) * pd_V[w_n - 2][w_m]) * d_rec;
			pd_W[w_n][w_m] = ((w_dN - 1) * d_z * pd_W[w_n - 1][w_m] - (w_n + w_m1Index) * pd_W[w_n - 2][w_m]) * d_rec;
		}
	}

	/* hydrostatic */
	pd_ah[1] = 0.0029; 
	d_c0h = 0.062;
	if (pd_LLA[0] < 0)/* southern hemisphere */
	{
		d_phh = PI; 
		d_c11h = 0.007;
		d_c10h = 0.002;
	}
	else/* northern hemisphere */
	{
		d_phh = 0.0;
		d_c11h = 0.005;
		d_c10h = 0.001;
	}

	pd_ah[2] = d_c0h + ((cos(d_doyTran + d_phh) + 1) * d_c11h / 2 + d_c10h) * (1 - d_cosLat);
	d_ahm = 0.0; 
	d_aha = 0.0;
	w_i = 0;
	for (w_n = 0; w_n <= NMAX; w_n++)
	{
		for (w_m = 0; w_m <= w_n; w_m++)
		{
			d_ahm += (gpd_AH_MEAN[w_i] * pd_V[w_n][w_m] + gpd_BH_MEAN[w_i] * pd_W[w_n][w_m]) * 1.0e-5;
			d_aha += (gpd_AH_AMP[w_i] * pd_V[w_n][w_m] + gpd_BH_AMP[w_i] * pd_W[w_n][w_m]) * 1.0e-5;
			w_i = w_i + 1;
		}
	}
	double cos_store = cos(d_doyTran);
	pd_ah[0] = d_ahm + d_aha * cos_store;
	d_dm = (1.0 / sin(d_ele) - mapfuncByGMF(d_ele, pd_aht[0], pd_aht[1], pd_aht[2])) * pd_LLA[2] / 1e3;
	*pd_mapDry = mapfuncByGMF(d_ele, pd_ah[0], pd_ah[1], pd_ah[2]) + d_dm;

	//wet
	pd_aw[1] = 0.00146; 
	pd_aw[2] = 0.04391;
	d_awm = 0.0; 
	d_awa = 0.0;
	w_i = 0;
	for (w_n = 0; w_n <= NMAX; w_n++)
	{
		for (w_m = 0; w_m <= w_n; w_m++)
		{
			d_awm += (gpd_AW_MEAN[w_i] * pd_V[w_n][w_m] + gpd_BW_MEAN[w_i] * pd_W[w_n][w_m]) * 1.0e-5;
			d_awa += (gpd_AW_AMP[w_i] * pd_V[w_n][w_m] + gpd_BW_AMP[w_i] * pd_W[w_n][w_m]) * 1.0e-5;
			w_i = w_i + 1;
		}
	}
	pd_aw[0] = d_awm + d_awa * cos_store;
	*pd_mapWet = mapfuncByGMF(d_ele, pd_aw[0], pd_aw[1], pd_aw[2]);
	return 0;
}
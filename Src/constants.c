#include <stdio.h>

float CC_Weights[13] = {1.0000,2.5643,4.0902,5.5399,6.8779,8.0711,9.0902,9.9101,10.5106,10.8769,11.0000,10.8769,10.5106};

float hamming[320] = {0.08,0.080089226,0.08035687,0.080802828,0.081426926,0.082228923,0.083208508,0.084365301,0.085698852,0.087208645,0.088894094,0.090754544,0.092789275,0.094997496,0.097378352,0.099930918,0.102654205,0.105547155,0.108608647,0.111837493,0.11523244,0.118792171,0.122515306,0.126400399,0.130445944,0.134650372,0.13901205,0.143529288,0.148200333,0.153023372,0.157996535,0.163117893,0.168385458,0.173797187,0.17935098,0.185044684,0.190876089,0.196842933,0.202942901,0.209173628,0.215532694,0.222017635,0.228625934,0.235355027,0.242202304,0.249165108,0.256240739,0.263426452,0.270719458,0.278116929,0.285615995,0.293213746,0.300907236,0.30869348,0.316569456,0.32453211,0.332578352,0.340705062,0.348909086,0.357187242,0.365536318,0.373953075,0.382434249,0.390976549,0.399576661,0.408231248,0.416936954,0.425690402,0.434488194,0.443326918,0.452203146,0.461113434,0.470054325,0.479022351,0.488014032,0.497025881,0.506054401,0.51509609,0.52414744,0.53320494,0.542265077,0.551324334,0.560379198,0.569426157,0.5784617,0.587482322,0.596484523,0.605464812,0.614419705,0.623345727,0.632239417,0.641097322,0.649916009,0.658692054,0.667422054,0.676102622,0.68473039,0.693302012,0.701814162,0.710263537,0.718646861,0.72696088,0.73520237,0.743368133,0.751455001,0.759459838,0.767379537,0.775211027,0.782951269,0.790597261,0.798146036,0.805594666,0.812940261,0.820179972,0.82731099,0.834330548,0.841235924,0.848024439,0.854693458,0.861240396,0.867662711,0.873957913,0.88012356,0.886157259,0.89205667,0.897819504,0.903443526,0.908926554,0.91426646,0.919461173,0.924508679,0.929407018,0.934154291,0.938748655,0.943188329,0.94747159,0.951596777,0.955562289,0.959366588,0.963008198,0.966485706,0.969797764,0.972943086,0.975920452,0.978728707,0.981366762,0.983833593,0.986128243,0.988249822,0.990197508,0.991970544,0.993568243,0.994989984,0.996235217,0.997303458,0.998194292,0.998907375,0.99944243,0.999799249,0.999977693,0.999977693,0.99979925,0.999442432,0.998907379,0.998194297,0.997303463,0.996235223,0.994989991,0.993568251,0.991970553,0.990197518,0.988249833,0.986128255,0.983833606,0.981366775,0.978728722,0.975920467,0.972943102,0.969797781,0.966485725,0.963008218,0.959366608,0.95556231,0.951596799,0.947471613,0.943188353,0.93874868,0.934154316,0.929407044,0.924508706,0.919461201,0.914266489,0.908926583,0.903443556,0.897819535,0.892056702,0.886157292,0.880123593,0.873957947,0.867662746,0.861240431,0.854693494,0.848024475,0.841235961,0.834330586,0.827311028,0.820180011,0.812940301,0.805594706,0.798146077,0.790597302,0.782951311,0.77521107,0.76737958,0.759459881,0.751455045,0.743368177,0.735202415,0.726960925,0.718646906,0.710263583,0.701814208,0.693302058,0.684730437,0.676102669,0.667422101,0.658692102,0.649916056,0.641097371,0.632239465,0.623345776,0.614419754,0.605464861,0.596484572,0.587482371,0.578461749,0.569426206,0.560379248,0.551324383,0.542265126,0.53320499,0.52414749,0.515096139,0.50605445,0.49702593,0.488014081,0.4790224,0.470054374,0.461113483,0.452203195,0.443326967,0.434488242,0.425690449,0.416937002,0.408231296,0.399576708,0.390976595,0.382434295,0.373953121,0.365536363,0.357187287,0.348909131,0.340705106,0.332578396,0.324532153,0.316569499,0.308693522,0.300907278,0.293213788,0.285616036,0.27811697,0.270719498,0.263426491,0.256240778,0.249165147,0.242202341,0.235355064,0.22862597,0.222017671,0.215532729,0.209173662,0.202942935,0.196842966,0.190876121,0.185044715,0.179351011,0.173797217,0.168385487,0.163117921,0.157996563,0.153023399,0.148200359,0.143529313,0.139012075,0.134650395,0.130445967,0.126400421,0.122515326,0.118792191,0.115232459,0.111837511,0.108608664,0.105547171,0.10265422,0.099930932,0.097378365,0.094997509,0.092789286,0.090754555,0.088894103,0.087208654,0.08569886,0.084365308,0.083208514,0.082228928,0.08142693,0.080802831,0.080356872,0.080089227,0.08};

float H[25][257] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.27956,0.55911,0.83867,0.88835,0.62433,0.36031,0.096283,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.11165,0.37567,0.63969,0.90372,0.84158,0.59223,0.34288,0.093528,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.15842,0.40777,0.65712,0.90647,0.85284,0.61734,0.38184,0.14635,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.14716,0.38266,0.61816,0.85365,0.9158,0.69339,0.47098,0.24857,0.026161,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.084196,0.30661,0.52902,0.75143,0.97384,0.81466,0.6046,0.39455,0.1845,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.18534,0.3954,0.60545,0.8155,0.97587,0.77748,0.5791,0.38072,0.18234,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.024135,0.22252,0.4209,0.61928,0.81766,0.98485,0.79749,0.61014,0.42278,0.23542,0.048062,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.015149,0.20251,0.38986,0.57722,0.76458,0.95194,0.86844,0.6915,0.51455,0.3376,0.16065,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.13156,0.3085,0.48545,0.6624,0.83935,0.98461,0.8175,0.65038,0.48327,0.31615,0.14904,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.015388,0.1825,0.34962,0.51673,0.68385,0.85096,0.98292,0.8251,0.66727,0.50944,0.35161,0.19378,0.035948,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.017075,0.1749,0.33273,0.49056,0.64839,0.80622,0.96405,0.88489,0.73583,0.58677,0.43771,0.28865,0.13959,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.11511,0.26417,0.41323,0.56229,0.71135,0.86041,0.99106,0.85028,0.7095,0.56873,0.42795,0.28717,0.14639,0.005617,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0089421,0.14972,0.2905,0.43127,0.57205,0.71283,0.85361,0.99438,0.87235,0.73939,0.60644,0.47348,0.34053,0.20757,0.07462,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.12765,0.26061,0.39356,0.52652,0.65947,0.79243,0.92538,0.94491,0.81934,0.69377,0.5682,0.44264,0.31707,0.1915,0.065935,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.055094,0.18066,0.30623,0.4318,0.55736,0.68293,0.8085,0.93406,0.94368,0.82509,0.7065,0.58791,0.46932,0.35073,0.23214,0.11355,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.056319,0.17491,0.2935,0.41209,0.53068,0.64927,0.76786,0.88645,0.99524,0.88324,0.77124,0.65924,0.54724,0.43524,0.32324,0.21123,0.099234,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0047604,0.11676,0.22876,0.34076,0.45276,0.56476,0.67676,0.78877,0.90077,0.98794,0.88216,0.77639,0.67061,0.56483,0.45906,0.35328,0.2475,0.14172,0.035946,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.012058,0.11784,0.22361,0.32939,0.43517,0.54094,0.64672,0.7525,0.85828,0.96405,0.93405,0.83415,0.73425,0.63435,0.53445,0.43455,0.33465,0.23475,0.13485,0.034949,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.065952,0.16585,0.26575,0.36565,0.46555,0.56545,0.66535,0.76525,0.86515,0.96505,0.93866,0.84431,0.74996,0.65561,0.56126,0.46691,0.37256,0.27822,0.18387,0.089518,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.061342,0.15569,0.25004,0.34439,0.43874,0.53309,0.62744,0.72178,0.81613,0.91048,0.99544,0.90633,0.81722,0.72812,0.63901,0.5499,0.4608,0.37169,0.28259,0.19348,0.10437,0.015266,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0045629,0.093669,0.18278,0.27188,0.36099,0.4501,0.5392,0.62831,0.71741,0.80652,0.89563,0.98473,0.93026,0.84611,0.76195,0.6778,0.59364,0.50949,0.42533,0.34118,0.25702,0.17286,0.08871,0.0045544,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.069738,0.15389,0.23805,0.3222,0.40636,0.49051,0.57467,0.65882,0.74298,0.82714,0.91129,0.99545,0.92482,0.84534,0.76586,0.68638,0.60691,0.52743,0.44795,0.36847,0.28899,0.20951,0.13003,0.050551,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.075178,0.15466,0.23414,0.31362,0.39309,0.47257,0.55205,0.63153,0.71101,0.79049,0.86997,0.94945,0.97268,0.89762,0.82255,0.74749,0.67243,0.59736,0.5223,0.44724,0.37218,0.29711,0.22205,0.14699,0.071924,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.027321,0.10238,0.17745,0.25251,0.32757,0.40264,0.4777,0.55276,0.62782,0.70289,0.77795,0.85301,0.92808,0.99704,0.92614,0.85525,0.78436,0.71347,0.64258,0.57168,0.50079,0.4299,0.35901,0.28812,0.21722,0.14633,0.075439,0.0045468,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0029643,0.073856,0.14475,0.21564,0.28653,0.35742,0.42832,0.49921,0.5701,0.64099,0.71188,0.78278,0.85367,0.92456,0.99545,0.93734,0.87039,0.80344,0.73648,0.66953,0.60258,0.53562,0.46867,0.40172,0.33476,0.26781,0.20086,0.13391,0.066953,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

float DCT[13][25] = {{0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284,0.28284},
{0.28229,0.27783,0.269,0.25593,0.23881,0.21793,0.19362,0.16625,0.13627,0.10412,0.070332,0.035459,-1.0389e-06,-0.035461,-0.070334,-0.10412,-0.13627,-0.16625,-0.19362,-0.21793,-0.23881,-0.25593,-0.269,-0.27783,-0.28229},
{0.28061,0.26298,0.22881,0.1803,0.12042,0.052972,-0.017752,-0.087415,-0.15153,-0.20618,-0.24787,-0.27395,-0.28284,-0.27395,-0.24786,-0.20618,-0.15153,-0.087411,-0.017748,0.052976,0.12043,0.1803,0.22881,0.26298,0.28062},
{0.27784,0.23881,0.16622,0.070359,-0.035461,-0.1363,-0.21793,-0.26901,-0.28229,-0.25593,-0.1936,-0.10415,3.1168e-06,0.10415,0.1936,0.25593,0.28229,0.269,0.21792,0.13629,0.035455,-0.070365,-0.16623,-0.23881,-0.27784},
{0.27397,0.20618,0.087359,-0.052974,-0.1803,-0.263,-0.28061,-0.22881,-0.12047,0.017754,0.15158,0.24784,0.28284,0.24784,0.15158,0.017746,-0.12048,-0.22881,-0.28062,-0.263,-0.1803,-0.052966,0.087367,0.20619,0.27397},
{0.26901,0.16625,-5.7607e-05,-0.16622,-0.26901,-0.26898,-0.16627,3.1401e-05,0.1662,0.269,0.26899,0.16629,-5.1947e-06,-0.1663,-0.26899,-0.26899,-0.16619,-2.1012e-05,0.16628,0.26898,0.269,0.16622,4.7218e-05,-0.16625,-0.26902},
{0.263,0.12042,-0.087469,-0.24784,-0.27395,-0.15148,0.052976,0.22885,0.28062,0.1803,-0.017812,-0.20615,-0.28284,-0.20614,-0.0178,0.18031,0.28062,0.22884,0.052964,-0.15149,-0.27395,-0.24783,-0.087457,0.12043,0.263},
{0.25595,0.070332,-0.16632,-0.28228,-0.1936,0.035547,0.2388,0.26899,0.1042,-0.13625,-0.27784,-0.21798,7.2726e-06,0.21798,0.27784,0.13624,-0.10421,-0.26899,-0.23879,-0.035533,0.19361,0.28228,0.1663,-0.070346,-0.25596},
{0.24789,0.01775,-0.22888,-0.263,-0.05297,0.20626,0.27396,0.087355,-0.18022,-0.28061,-0.12036,0.15149,0.28284,0.15148,-0.12038,-0.28062,-0.18021,0.087371,0.27397,0.20625,-0.052986,-0.26301,-0.22887,0.017766,0.2479},
{0.23886,-0.035461,-0.26903,-0.19366,0.10415,0.28229,0.13629,-0.1663,-0.27785,-0.070353,0.21798,0.25596,-9.3505e-06,-0.25597,-0.21797,0.070371,0.27786,0.16628,-0.13631,-0.28229,-0.10414,0.19368,0.26903,0.035442,-0.23887},
{0.22888,-0.087415,-0.28284,-0.087465,0.22885,0.22874,-0.087365,-0.28284,-0.087515,0.22882,0.22877,-0.087315,-0.28284,-0.087296,0.22879,0.2288,-0.087535,-0.28284,-0.087346,0.22875,0.22883,-0.087485,-0.28284,-0.087395,0.22889},
{0.218,-0.13627,-0.26896,0.035379,0.28229,0.070191,-0.25591,-0.16619,0.19353,0.23882,-0.10421,-0.27785,1.1428e-05,0.27786,0.10419,-0.23883,-0.19351,0.16621,0.2559,-0.070213,-0.28229,-0.035356,0.26897,0.13625,-0.21801},
{0.20626,-0.1803,-0.22874,0.15149,0.24784,-0.12058,-0.263,0.087475,0.27399,-0.052982,-0.2806,0.017649,0.28284,0.017624,-0.2806,-0.052958,0.274,0.087451,-0.26301,-0.12056,0.24785,0.15147,-0.22876,-0.18028,0.20627}
};

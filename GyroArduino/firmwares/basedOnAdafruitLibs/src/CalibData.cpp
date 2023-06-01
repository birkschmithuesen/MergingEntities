#include "CalibData.hpp"

CalibData calibrationSets[]{

// Controller A for testing purpose
{
0,      //controllerindex
0,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{-22.309784,34.632069,-1.998663},//hardIron
{1.049076,-0.016527,-0.010604,-0.016527,0.982334,-0.009963,-0.010604,-0.009963,0.970832},//softIron
34.670895//magfield
},

{
0,      //controllerindex
1,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{4.490335,-15.055762,16.319937},//hardIron
{1.023077,-0.020309,0.010741,-0.020309,0.968477,-0.014193,0.010741,-0.014193,1.009993},//softIron
33.839920//magfield
},

{
0,      //controllerindex
2,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{-8.938557,-18.333771,-15.042858},//hardIron
{1.027766,0.009910,-0.006837,0.009910,0.983864,0.002625,-0.006837,0.002625,0.989091},//softIron
35.080391//magfield
},

{
0,      //controllerindex
3,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{4.320663,-2.024136,15.457508},//hardIron
{1.012598,-0.029107,-0.018482,-0.029107,1.005237,-0.013670,-0.018482,-0.013670,0.983770},//softIron
32.329922//magfield
},

{
0,      //controllerindex
4,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{-20.750238,2.628237,-39.692886},//hardIron
{1.040656,-0.014913,-0.015915,-0.014913,0.989629,0.005869,-0.015915,0.005869,0.971487},//softIron
35.150177//magfield
},

{
0,      //controllerindex
5,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{28.861887,5.437371,41.768616},//hardIron
{1.038859,-0.007280,-0.003225,-0.007280,0.967855,-0.002548,-0.003225,-0.002548,0.994633},//softIron
32.356808//magfield
},

{
0,      //controllerindex
6,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{-20.374104,34.377129,23.934938},//hardIron
{1.042178,0.013814,0.015849,0.013814,0.986063,-0.005018,0.015849,-0.005018,0.973540},//softIron
32.430531//magfield
},

{
0,      //controllerindex
7,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000000,0.000000,0.000000},//gyrobias
{3.983593,-1.574223,-1.285341},//hardIron
{1.027712,-0.026347,-0.010012,-0.026347,0.982606,0.003565,-0.010012,0.003565,0.991051},//softIron
33.670193//magfield
},

// Controller A from Felix
/*
{
0,      //controllerindex
0,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000616,0.000966,-0.009676},//gyrobias
{-5.932755,-29.359560,-8.725225},//hardIron
{1.034776,-0.019334,0.006597,-0.019334,0.961972,0.006315,0.006597,0.006315,1.005058},//softIron
48.525284//magfield
},
*/

{
0,      //controllerindex
1,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.007603,-0.021200,0.000050},//gyrobias
{-42.745296,-9.749575,20.779453},//hardIron
{1.028661,-0.019210,0.005685,-0.019210,0.968913,-0.000673,0.005685,-0.000673,1.003730},//softIron
47.493793//magfield
},

{
0,      //controllerindex
2,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.012882,-0.001582,-0.001674},//gyrobias
{17.364214,-5.963294,34.401352},//hardIron
{1.008628,-0.022871,-0.004684,-0.022871,0.973716,0.008953,-0.004684,0.008953,1.018853},//softIron
48.365723//magfield
},

{
0,      //controllerindex
3,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.003839,0.015830,-0.006403},//gyrobias
{2.180344,10.391054,41.147720},//hardIron
{1.029473,-0.003898,0.007350,-0.003898,0.978253,-0.006372,0.007350,-0.006372,0.993074},//softIron
47.504360//magfield
},

{
0,      //controllerindex
4,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.005796,0.000158,0.004405},//gyrobias
{-21.349653,16.564987,-10.783118},//hardIron
{1.024646,-0.004584,0.005335,-0.004584,0.972065,0.006970,0.005335,0.006970,1.004093},//softIron
48.947235//magfield
},

{
0,      //controllerindex
5,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.003597,-0.002415,-0.003647},//gyrobias
{12.595722,-3.036313,-18.672426},//hardIron
{1.024214,-0.010894,0.012126,-0.010894,0.959509,-0.001254,0.012126,-0.001254,1.017828},//softIron
48.560913//magfield
},

{
0,      //controllerindex
6,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000983,-0.000475,0.003214},//gyrobias
{5.022198,-7.981039,-8.496399},//hardIron
{1.033048,-0.043432,-0.003325,-0.043432,0.984526,0.006668,-0.003325,0.006668,0.985104},//softIron
47.305309//magfield
},

{
0,      //controllerindex
7,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.009768,-0.003497,-0.002823},//gyrobias
{-38.805309,-4.989368,-19.857349},//hardIron
{1.026313,-0.045533,0.009419,-0.045533,0.986962,0.001193,0.009419,0.001193,0.989347},//softIron
47.608818//magfield
},


/////////////controller B
{
1,      //controllerindex
0,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000824,0.007011,0.005213},//gyrobias
{-11.555323,-13.326333,15.514660},//hardIron
{1.033954,-0.031330,0.006879,-0.031330,0.971828,0.000312,0.006879,0.000312,0.996217},//softIron
46.942730//magfield
},

{
1,      //controllerindex
1,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.009726,0.009301,0.005388},//gyrobias
{5.036726,7.907860,32.520309},//hardIron
{1.018956,-0.021316,0.000071,-0.021316,0.971294,0.002078,0.000071,0.002078,1.010870},//softIron
47.842922//magfield
},

{
1,      //controllerindex
2,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.003739,-0.001524,0.002332},//gyrobias
{-19.630354,-2.264633,18.383480},//hardIron
{1.040678,-0.026753,0.014343,-0.026753,0.977153,-0.003127,0.014343,-0.003127,0.984277},//softIron
47.384380//magfield
},

{
1,      //controllerindex
3,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.020484,-0.005746,-0.009643},//gyrobias
{-6.256618,22.711586,0.177987},//hardIron
{1.029407,-0.014324,0.015034,-0.014324,0.969369,0.002950,0.015034,0.002950,1.002565},//softIron
47.742828//magfield
},

{
1,      //controllerindex
4,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.015954,-0.002240,-0.004488},//gyrobias
{-40.065765,-22.978792,6.884359},//hardIron
{1.017031,-0.006690,0.003841,-0.006690,0.973793,0.005245,0.003841,0.005245,1.009805},//softIron
48.307163//magfield
},

{
1,      //controllerindex
5,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.019843,-0.004613,-0.003231},//gyrobias
{12.063061,9.217522,-19.570223},//hardIron
{1.031424,-0.020286,0.003765,-0.020286,0.984281,0.016681,0.003765,0.016681,0.985715},//softIron
47.373386//magfield
},

{
1,      //controllerindex
6,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.007969,-0.021242,0.001890},//gyrobias
{-7.705038,-19.161366,-2.208708},//hardIron
{1.008881,-0.008134,0.003847,-0.008134,0.988886,0.002789,0.003847,0.002789,1.002426},//softIron
47.730442//magfield
},

{
1,      //controllerindex
7,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.004605,-0.001449,-0.000566},//gyrobias
{0.336281,2.357177,-29.881321},//hardIron
{1.037231,-0.000789,-0.000518,-0.000789,0.968323,0.003845,-0.000518,0.003845,0.995661},//softIron
46.815128//magfield
},
{
2,      //controllerindex
0,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.000608,0.001241,-0.001649},//gyrobias
{-36.121483,-24.739384,6.435102},//hardIron
{1.009117,-0.011831,0.000662,-0.011831,0.985828,0.007041,0.000662,0.007041,1.005404},//softIron
48.263016//magfield
},

{
2,      //controllerindex
1,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.010442,0.000899,-0.010733},//gyrobias
{-19.192587,-0.976067,5.093008},//hardIron
{1.026206,-0.015845,-0.004050,-0.015845,0.976027,-0.001331,-0.004050,-0.001331,0.998666},//softIron
47.969273//magfield
},

{
2,      //controllerindex
2,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.026779,0.016504,0.016629},//gyrobias
{5.097925,11.868370,42.920883},//hardIron
{1.024660,-0.013920,0.000915,-0.013920,0.976134,0.005554,0.000915,0.005554,1.000022},//softIron
48.445694//magfield
},

{
2,      //controllerindex
3,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.008402,-0.006004,0.003014},//gyrobias
{11.773514,-15.966524,41.473751},//hardIron
{1.023929,-0.017821,0.006221,-0.017821,0.976742,0.001748,0.006221,0.001748,1.000244},//softIron
47.869793//magfield
},

{
2,      //controllerindex
4,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.010084,0.007736,0.000125},//gyrobias
{0.286942,-1.399803,60.119743},//hardIron
{1.027195,0.006198,0.018616,0.006198,0.982882,0.005531,0.018616,0.005531,0.990885},//softIron
48.124268//magfield
},

{
2,      //controllerindex
5,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.018469,0.035131,0.000333},//gyrobias
{-11.242447,0.321085,36.816010},//hardIron
{1.010702,-0.015550,0.005082,-0.015550,0.987516,0.004040,0.005082,0.004040,1.002205},//softIron
47.822906//magfield
},

{
2,      //controllerindex
6,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.012890,-0.005945,0.000033},//gyrobias
{27.420254,8.546021,33.649261},//hardIron
{1.031953,-0.008710,-0.002949,-0.008710,0.971956,0.007310,-0.002949,0.007310,0.997135},//softIron
48.205360//magfield
},

{
2,      //controllerindex
7,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.016204,-0.000916,-0.001774},//gyrobias
{-16.066875,17.177509,22.417923},//hardIron
{1.020650,-0.002551,0.010542,-0.002551,0.976080,-0.012645,0.010542,-0.012645,1.004057},//softIron
47.284946//magfield
},

{
3,      //controllerindex
0,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.001251,-0.003591,-0.001817},//gyrobias
{-30.187073,5.579748,-8.846073},//hardIron
{1.025777,-0.012326,-0.002390,-0.012326,0.978963,0.001719,-0.002390,0.001719,0.995979},//softIron
46.307453//magfield
},

{
3,      //controllerindex
1,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.008800,0.013080,0.005939},//gyrobias
{-19.534632,-7.882393,20.663071},//hardIron
{1.013960,-0.009680,0.004089,-0.009680,0.978747,-0.009619,0.004089,-0.009619,1.007853},//softIron
46.222439//magfield
},

{
3,      //controllerindex
2,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{0.004649,0.002292,-0.007049},//gyrobias
{-25.987268,11.970175,21.550350},//hardIron
{1.034446,-0.009062,0.005118,-0.009062,0.966075,-0.000327,0.005118,-0.000327,1.000756},//softIron
47.332832//magfield
},

{
3,      //controllerindex
3,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.001355,0.002115,-0.016071},//gyrobias
{23.571960,-7.877264,-19.396173},//hardIron
{1.008608,-0.005868,0.001519,-0.005868,0.974862,-0.002539,0.001519,-0.002539,1.017076},//softIron
46.903534//magfield
},

{
3,      //controllerindex
4,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.005964,-0.006911,-0.002975},//gyrobias
{-17.490795,15.566849,8.933426},//hardIron
{1.030215,-0.026034,-0.001180,-0.026034,0.973465,0.000701,-0.001180,0.000701,0.997806},//softIron
45.017513//magfield
},

{
3,      //controllerindex
5,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.014510,-0.004778,-0.008814},//gyrobias
{3.649540,-6.390612,12.220705},//hardIron
{1.028342,-0.011722,-0.000435,-0.011722,0.973658,0.003420,-0.000435,0.003420,0.998897},//softIron
44.909649//magfield
},

{
3,      //controllerindex
6,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.013762,-0.004359,-0.003191},//gyrobias
{2.313511,29.140232,0.114925},//hardIron
{1.021010,-0.015261,0.002676,-0.015261,0.978418,-0.007592,0.002676,-0.007592,1.001326},//softIron
45.793182//magfield
},

{
3,      //controllerindex
7,      //sensorIndex
{0.000000,0.000000,0.000000},//accelBias
{-0.008491,-0.000689,0.004640},//gyrobias
{-30.208538,-17.686920,33.443115},//hardIron
{1.034116,-0.007740,0.010493,-0.007740,0.962443,-0.011050,0.010493,-0.011050,1.005037},//softIron
47.457424//magfield
}

};
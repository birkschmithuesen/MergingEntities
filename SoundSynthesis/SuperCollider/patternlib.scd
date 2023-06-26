
(

~tripples.stop;
~tripples = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\address, 'tripples',
	\dur, Pseq([1/4, 1/4, 1/4, ],inf),
	\midinote, Pseq([60, 65, 68],inf),
	\exc, ~exc[0],
	\amp, 1, //Pwhite().linlin(0, 1, 0.1, 1),
	\stretch, Pfunc {~tempo},
	\shape, ~shape[0].linlin(0.0, 1.0, 0.1, 1.0),
	\atk, 0.02, // 0.005,//Pfunc { ~attack },
	\rel, 0.1, //~rel[0].linlin(0.0, 1.0, 0.1, 0.5),
	\dev, ~atk[0].linlin(0.0, 1.0, 1.0, 1.05),
	\pan, ~pan[0].linlin(0.0, 1.0, -1.0, 1.0),
	\out, ~efxBus[0],
).play(quant:4);

~steady.stop;
~steady = Pbind(
	\instrument, \kling_t,
	\group, ~patternGroup,
	\dur, Pseq([1/12],inf),
	\midinote, 65,
	//\freq, 3000,
	\exc, ~exc[1],
	\amp, 1, //Pwhite(),
	\decay, ~shape[1].linlin(0.0, 1.0, 0.04, 0.3),
	\atk, 0.15, //~atk[1].linlin(0.0, 1.0, 0.0, 0.05), // 0.005,//Pfunc { ~attack },
	\dev, ~atk[1].linlin(0.0, 1.0, 1.0, 1.1),
	\rel, 0.1, //~rel[1].linlin(0.0, 1.0, 0.0, 0.25),
	\stretch, Pfunc{~tempo},
	\pan, ~pan[1].linlin(0.0, 1.0, -1.0, 1.0),
	\out, ~efxBus[1],
).play(quant:4);


~bass.stop;
~bass = Pbind(
	\instrument, \moog_t,
	\group, ~patternGroup,
	\dur, Pseq([1/2, 1/8, 3/8], inf),
	\midinote, Pseq([46, 47, 46], inf),
	\exc, ~exc[3],
	\amp, 2, //Pwhite(),
	\cutoff, ~shape[3].linlin(0.0, 1.0, 700, 4000),
	\depth, ~atk[3].linlin(0.0, 1.0, 0.0, 0.5), // 0.005,//Pfunc { ~attack },
	\rel, 0.3, //~rel[3].linlin(0.0, 1.0, 0.02, 0.2),
	\stretch, Pfunc{~tempo},
	\pan, ~pan[3].linlin(0.0, 1.0, -1.0, 1.0),
	\out, ~efxBus[3],
).play(quant:4);

~shaker.stop;
~shaker = Pbind(
	\instrument, \shaker,
	\group, ~patternGroup,
	\dur, Pseq([1/8],inf),
	\midinote, 72,
	\exc, ~exc[2],
	\amp, 2, //Pwhite(),
	\shape, ~shape[2],
	//\atk, ~atk[2].linlin(0.0, 1.0, 6000, 9000), // 0.005,//Pfunc { ~attack },
	\rel, ~atk[2].linlin(0.0, 1.0, 0.2, 0.08),
	\stretch, Pfunc{~tempo},
	\out, ~efxBus[2],
	\pan, ~pan[2].linlin(0.0, 1.0, -1.0, 1.0),
).play(quant:4);

~kick.stop;
~kick = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\dur, Pseq([1/4, 1/4, 1/4, 1/8, 1/8],inf), // + Pgauss(0, 0.005)),
	\midinote, Pseq([30, 30, 30, 30, 34], inf),
	\exc, Pseq([1, 1, 1, 1, 0.6], inf),
	\amp, 1.0, //Pwhite(),
	\shape, 0.5,
	\atk, 0.001, // 0.005,//Pfunc { ~attack },
	\rel, 0.08,
	\stretch, Pfunc{~tempo},
	\out, 16,
).play(quant:4);

/*
~marker = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\dur, Pseq([4],inf), // + Pgauss(0, 0.005)),
	\midinote, 40, //Pseq([20, 30, 30, 30], inf),
	\exc, 1.2,
	\amp, 1.5, //Pwhite(),
	\shape, 0.5,
	\atk, 0.001, // 0.005,//Pfunc { ~attack },
	\rel, 1,
	\stretch, Pfunc{~tempo},
	\out, 16,
).play(quant:1);
*/

)

/*
(
Pdef(\tripples,
	Pbind(
		\instrument, \sine,
		\dur, Pseq([1/4, 1/4, 1/4, ],inf),
		\midinote, Pseq([60, 65, 68],inf),
		\exc, ~exc[0],
		\amp, 1, //Pwhite().linlin(0, 1, 0.1, 1),
		\stretch, Pfunc {~tempo},
		\shape, ~shape[0],
		\atk, ~atk[0], // 0.005,//Pfunc { ~attack },
		\rel, ~rel[0],
		\out, 0,
));


Pdef(\steady,
	Pbind(
		\instrument, \sine,
		\dur, Pseq([1/6],inf),
		\midinote, 53,
		\exc, ~exc[1],
		\amp, 1, //Pwhite(),
		\shape, ~shape[1],
		\atk, ~atk[1], // 0.005,//Pfunc { ~attack },
		\rel, ~rel[1],
		\stretch, Pfunc{~tempo},
		\out, 2,
));
)
*/

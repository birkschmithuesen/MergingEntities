
(
~tripples = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\address, 'tripples',
	\dur, Pseq([1/4, 1/4, 1/4, ],inf),
	\midinote, Pseq([60, 65, 68],inf),
	\exc, ~exc[0],
	\amp, 1, //Pwhite().linlin(0, 1, 0.1, 1),
	\stretch, Pfunc {~tempo},
	\shape, ~shape[0],
	\atk, ~atk[0].linlin(0.01, 1.0, 0.0, 0.15), // 0.005,//Pfunc { ~attack },
	\rel, ~rel[0].linlin(0.0, 1.0, 0.0, 0.25),
	\pan, ~pan[0],
	\out, ~efxBus[0],
).play(quant:1);


~steady = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\dur, Pseq([1/6],inf),
	\midinote, 53,
	\exc, ~exc[1],
	\amp, 1, //Pwhite(),
	\shape, ~shape[1],
	\atk, ~atk[1].linlin(0.01, 1.0, 0.0, 0.15), // 0.005,//Pfunc { ~attack },
	\rel, ~rel[1].linlin(0.0, 1.0, 0.0, 0.25),
	\stretch, Pfunc{~tempo},
	\pan, ~pan[1],
	\out, ~efxBus[1],
).play(quant:1);

~bass = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\dur, Pseq([1/2, 1/8, 3/8], inf),
	\midinote, Pseq([46, 47, 46], inf),
	\exc, ~exc[3],
	\amp, 1, //Pwhite(),
	\shape, ~shape[3],
	\atk, ~atk[3].linlin(0.01, 1.0, 0.0, 0.15), // 0.005,//Pfunc { ~attack },
	\rel, ~rel[3].linlin(0.0, 1.0, 0.02, 0.2),
	\stretch, Pfunc{~tempo},
	\pan, ~pan[3],
	\out, ~efxBus[3],
).play(quant:1);

~shaker = Pbind(
	\instrument, \shaker,
	\group, ~patternGroup,
	\dur, Pseq([1/8],inf),
	\midinote, 72,
	\exc, ~exc[2],
	\amp, 1, //Pwhite(),
	\shape, ~shape[2],
	\atk, ~atk[2].linlin(0.01, 1.0, 6000, 9000), // 0.005,//Pfunc { ~attack },
	\rel, ~rel[2].linlin(0.0, 1.0, 0.0, 0.25),
	\stretch, Pfunc{~tempo},
	\out, ~efxBus[2],
	\pan, ~pan[2],
).play(quant:1);

~kick = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\dur, Pseq([1/4],inf), // + Pgauss(0, 0.005)),
	\midinote, 30,
	\exc, 1,
	\amp, 1.25, //Pwhite(),
	\shape, 0.5,
	\atk, 0.001, // 0.005,//Pfunc { ~attack },
	\rel, 0.08,
	\stretch, Pfunc{~tempo},
	\out, 10,
).play(quant:1);

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

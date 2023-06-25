
(
~tripples.stop;
~tripples = Pbind(
	\instrument, \sine,
	\group, ~patternGroup,
	\address, 'tripples',
	\dur, Pseq([1/4, 1/4, 1/4, 1/8, 1/8, 1/8, 1/8, 3/4],inf),
	\midinote, Pseq([Pseq([60, 65, 68, 60, 59, 60, 65, 68],1), Pseq([60, 64, 68, 60, 59, 60, 64, 68], 1)],inf),
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
	\dur, Pseq([1/6, 1/6, 1/12, 1/24, 1/24],inf),
	//\midinote, 70,
	\freq, Pseq([3000, 3000, 3000, 3000, 3000, 3000, 2900],inf),
	\exc, ~exc[1],
	\amp, 1, //Pwhite(),
	\decay, ~shape[1].linlin(0.0, 1.0, 0.04, 0.2),
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
	\dur, Pseq([1/2, 1/8, 3/8, 1/2, 1/8, 1/8, 1/8, 1/8], inf),
	\midinote, Pseq([46, 47, 46, 46, 47, 46, 43, 45], inf),
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
~shaker.stream = Pbind(
	\instrument, \shaker,
	\group, ~patternGroup,
	\dur, Pseq([1/8, 1/4, 1/8],inf),
	\exc, ~exc[2],
	\amp, Pseq([1, 2, 2],inf), //Pwhite(),
	\shape, ~shape[2],
	//\atk, ~atk[2].linlin(0.0, 1.0, 6000, 9000), // 0.005,//Pfunc { ~attack },
	\rel, ~atk[2].linlin(0.0, 1.0, 0.2, 0.08),
	\stretch, Pfunc{~tempo},
	\out, ~efxBus[2],
	\pan, ~pan[2].linlin(0.0, 1.0, -1.0, 1.0),
).play(quant:4);
)
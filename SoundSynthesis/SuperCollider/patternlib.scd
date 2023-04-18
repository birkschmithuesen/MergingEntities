
(
~tripples = Pbind(
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
).play(quant:0);


~steady = Pbind(
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
).play(quant:0);
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

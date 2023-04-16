
Scale.directory

Server.killAll;
s.plotTree;

//////////////////////////
// PATTERNS WITH SynthDef
//////////////////////////


~tempo = 60/120 * 4; //60/90 * 4 //60sec / 128 BPM * 4 bars

(
SynthDef.new(\sine, {
	arg freq=440, atk=0.005, rel=0.3, shape = 0, amp=1, pan=0, bus=0;
	var sig, osc1, osc2, env;
	osc1 = SinOsc.ar(freq);
	osc2 = VarSaw.ar(freq)*0.7;
	sig = (osc1 * (1-shape)) + (osc2 * shape);
	env = EnvGen.kr(Env.new([0, 1, 0], [atk, rel], [1,-1]), doneAction:2);
	sig = Pan2.ar(sig, pan, amp);
	sig = sig * env;
	Out.ar(bus, sig);
}).add
)

x = Synth.new(\sine, [\amp, 0.5]);


a = Pdefn(\shape, 0.0);

(
Pdef(\tripples,
Pbind(
	\instrument, \sine,
	\dur, Pseq([1/4],inf),
	\midinote, Pseq([60, 65, 68],inf),
	\amp, Pwhite().linlin(0, 1, 0.1, 1),
	\stretch, Pfunc {~tempo},
	\shape, a,
	\atk, 0.005,//Pfunc { ~attack },
));
)

y = Pdef.new(\tripples).play;

a.source = 1



~shape = 0.005

(
Pdef(\tripples,
Pbind(
	\instrument, \sine,
	\dur, Pseq([1/4],inf),
	\midinote, Pseq([60, 65, 68],inf),
	\amp, Pwhite().linlin(0, 1, 0.1, 1),
	\stretch, Pfunc {~tempo},
	\shape, Pfunc {~shape},
	\atk, 0.005,//Pfunc { ~attack },
));
)

x = Pdef.new(\tripples);

y = Pbind(
	\instrument, \sine,
	\dur, Pseq([1/6],inf),
	\midinote, 53,
	\amp, Pwhite(),
	\shape, 0,
	\stretch, Pfunc{~tempo},
).play(quant:1);
)


x.unmute;

(
OSCFunc({
	arg msg;
	if (msg[1] != nil,
		{
			~shape = msg[1].postln;
		}
	);
},
('/freq')
);
)


/*
x.stream=Pbind(
				\instrument, \sine,
				\dur, Pseq([1/4],inf),
				\midinote, Pseq([60, 65, 68],inf),
				\amp, Pwhite().linlin(0, 1, 0.1, 1),
				\stretch, ~tempo,
				\atk, msg[1].linlin(0,1,0.005,0.1),
			).asStream;
*/


(
y = Pbind(
		\instrument, \sine,
		\dur, Pseq([1/16], inf),
		\stretch, ~tempo,
		\midinote, Pseq([33], inf),
		\harmonic, Pexprand(1, 6, inf).round.trace,
		// \amp, Pkey(\harmonic).reciprocal * 0.2,
		\amp, Pkey(\harmonic).reciprocal * Pseq([Pexprand(0.4, 0.5, 1),Pexprand(0.1, 0.3, 3)], inf),
		\atk, 0.005, //Pwhite(0.002, 0.1, inf),
		\rel, Pwhite(0.005, 0.2, inf),
		\pan, Pwhite(-0.8, 0.8, inf),
	)
)
// PDEVs

(
Pdef.new(
	\sinepat,
	y
).play(quant:~tempo);//quant_(~tempo);//play(quant:~tempo);
)

y.stop;
(
y.stream = Pbind(
		\instrument, \sine,
		\dur, Pseq([1/16], inf),
		\stretch, ~tempo,
		\midinote, Pseq([33], inf),
		\harmonic, Pexprand(1, 6, inf).round.trace,
		// \amp, Pkey(\harmonic).reciprocal * 0.2,
		\amp, Pkey(\harmonic).reciprocal * Pseq([Pexprand(0.4, 0.5, 1),Pexprand(0.1, 0.3, 3)], inf),
		\atk, 0.005, //Pwhite(0.002, 0.1, inf),
		\rel, Pwhite(0.005, 0.2, inf),
		\pan, Pwhite(-0.8, 0.8, inf),
	).asStream;
)










{LFPulse.ar([48,55].midicps, 0.15)}.play;

(
{RLPF.ar(LFPulse.ar([48,55].midicps, 0.15),
	SinOsc.kr(0.1, 0, 10, 72).midicps, 0.1, 0.1)}.play;
)

150.cpsmidi
91.midicps
55.midinote
55.midinote
60.midinote
lookup

Scale


(
Ndef(\simplebpf, {
	arg amp=1, freq=440, rq=0.2, out = 0;
	var sig;
	sig = PinkNoise.ar(1!2);
	sig = BPF.ar(
		sig,
		freq,
		rq,
		1/rq.sqrt
	);
	sig = sig * amp;
	Out.ar(out, sig);
}).play;
)

// Play it with the mouse...
x = { MoogFF.ar(WhiteNoise.ar(01.1), MouseY.kr(100, 10000, 1), MouseX.kr(0, 4)) }.play(s);
x.free;

{MouseY.kr(100, 10000, 1).dump}.play;

{ Resonz.ar(WhiteNoise.ar(0.5), 2000, XLine.kr(1, 0.001, 8)) }.scope(1);



(
Ndef(\saw, {
	arg amp = 1, freq = 150, dev = 1.01, out = 0;
	var temp, sig = 0;
	10.do{
		temp = VarSaw.ar(
			freq * {Rand(-1.0,1.0).range(dev.reciprocal, dev)}!2,
			{Rand(0.0, 1.0)}!2,
			{ExpRand(0.01, 0.2)}!2
		);
		sig = sig + temp;
	};
	sig = sig * 0.025 * amp;
	Out.ar(out, sig);
}).play;
)

Ndef(\saw).edit;

(
Ndef(\moog, {
	arg amp = 1, exc = 1, freq = 120, cutoff = 2000, res = 0.5, speed = 2, depth = 0.2, out = 0;
	var sig;
	sig = Pulse.ar([freq/2.98, freq], [0.3, 0.7]) * exc;
	sig = Pan2.ar(sig[0], -0.3) + Pan2.ar(sig[1], 0.3);
	sig = MoogFF.ar(sig, cutoff * SinOsc.ar(speed, 0.0, depth, 0.5), res * 4) * 0.5 * amp;
	Out.ar(out, sig);
}).play;
)


Ndef(\moog).edit;
Ndef.clear;




(
Ndef(\kling, {
	arg amp = 1, exc = 1, dens = 1, freq = 1000, dev = 1.01, lfo_depth = 100, decay = 0.3, wide = 0.02, out = 0;
	var sig;
	//sig = Ringz.ar(Impulse.ar(dens, 0.0, exc * 0.1), [freq, freq * Rand(0.0, 1.0).range(dev.reciprocal, dev)], decay);
	sig = {arg count; Impulse.ar(dens, count * wide, exc * 0.2) * LFNoise0.kr(10, 0.5, 0.5)}!2;
	sig = Ringz.ar(sig, [
		LFSaw.kr(dens, 0.0).range(freq-lfo_depth, freq),
		LFSaw.kr(dens, 1.0).range(freq, freq+lfo_depth) * dev
	], decay);
	sig = Pan2.ar(sig[0], -0.3) + Pan2.ar(sig[1], 0.3)*amp;
	Out.ar(out, sig);
});
)
Ndef.clear;
Ndef(\kling).edit;





// read sound
b = Buffer.read(s, Platform.resourceDir +/+ "sounds/a11wlk01.wav");

(
{
	var trate, dur;
	trate = MouseY.kr(2,200,1);
	dur = 4 / trate;
	TGrains.ar(2, Impulse.ar(trate), b, 1, MouseX.kr(0,BufDur.kr(b)), dur, 0, 0.1, 2);
}.scope(2, zoom: 4);
)


// Phase Modulation
(
{
	var in;
	in = SinOsc.ar(300, 0, MouseX.kr(0.1,8pi,1).poll);
	SinOsc.ar(MouseY.kr(50, 2000, 1).poll, in, 0.2); // 0 Hz sine oscillator
}.scope(1);
)

(
Ndef(\pm, {
	arg carfreq = 440, modfreq = 440, pmindex = 0, modphase = 0;
	var sig;
	sig = PMOsc.ar(carfreq, modfreq, pmindex, modphase);
	sig = sig * 0.5;
	Out.ar(0, [sig,sig]);
}).play
)

// Interessante UGens
GrainFM

In.ar

SynthDef
s.meter;
// Parallel Structures

(
{
	// mixing sine oscillators in parallel
	var n = 16; // number of structures to make
	// mix together  parallel structures
	Mix.fill(n,
		{
			var amp;
			amp = FSinOsc.kr(exprand(0.1,1),2pi.rand).max(0);
			Pan2.ar(
				FSinOsc.ar(exprand(100,1000.0), 0, amp),
				1.0.rand2)
		}
	) / (2*n)            // scale amplitude
}.scope(2);
)

1.0.rrand(2)

// Serial Structures

(
play {
	var sig, chain;

	// The original sound source
	sig = sum({ SinOsc.ar(rrand(50,6000),0,2*Decay.ar(Dust2.ar(1),0.1)).tanh } ! 7);

	chain = sig;    // Start with the original signal
	8.do {|i|       // Loop 8 times. For each loop, connect the signal through something.

		// A simple reverb
		chain = LeakDC.ar(AllpassL.ar(LPF.ar(chain*0.9,3000), 0.2, {0.19.rand+0.01}!2, 3));
	};

	Limiter.ar(sig+chain);    // dry + wet
}
)

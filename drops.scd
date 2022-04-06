s.boot;

(
SynthDef(\boops, { |out, gate = 1, mask = 1, freq = 440, amp = 0.1, decay = 2, trigFreq = 10|
	var masterEg = EnvGen.kr(Env.asr(0.005, 1, decay), gate, doneAction: 2),
	trig = Impulse.kr(trigFreq),
	// note trig * mask: if mask == 0, then no triggers pass through
	boopEg = EnvGen.kr(Env.perc(0.01, 0.08), trig * mask),
	sig = SinOsc.ar(freq);
	Out.ar(out, (sig * (amp * boopEg * masterEg)).dup);
}).add;
)

(
p = Pbind(
	\instrument, \boops,
	\freq, Pexprand(200, 800, inf),
	\trigFreq, Pexprand(8, 14, inf),
	\dur, 1
).play;
)

p.stop;

////////////////////////////////////////////////////////////

{ Impulse.ar([5, 7]/2, [0, 1], 0.15) }.play


(
{ Splay.ar(
	Ringz.ar(
		Impulse.ar([3,2], phase:0.5, mul:0.08),
		[150, 220],
		mul:1
	)
)
}.play
)


////////////////////////
//Why is the encelope not freeing itself???
//Just happens with the asr!
/////////////////////////
(
SynthDef.new(\drops, {
	arg freq = 440, amp = 0.5, pan = 0, out = 0, oscGate = 1;
	var sig, rate, trig, dropEnv, env, oscAddress = "/drops";
	//rate = EnvGen.kr(Env([2,0]));
	rate = Line.kr(20, 0, 1);
	trig = Impulse.kr(rate);
	dropEnv = EnvGen.kr(Env.perc(0.01, 0.2), trig);
	env = EnvGen.kr(Env.asr(0.005, 1, 2), doneAction:Done.freeSelf);
	env = env * dropEnv;
	sig = SinOsc.ar([freq,freq+10]) * env;
	sig = Pan2.ar(sig, pan, amp);
	Out.ar (out, sig);
}).add;
)

x = Synth.new(\drops);

{EnvGen.kr(Env.asr(0.005, 1, 2))}.plot(duration:1);

s.plotTree;

TempoClock.default = 20;
TempoClock.default.beats;
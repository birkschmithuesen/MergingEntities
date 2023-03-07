(
SynthDef(\moog, {
	arg freq = 120, amp = 1, exc = 1, cutoff = 2000, res = 0.5, speed = 2, depth = 0.0, out = 0, lag_time = 0.02;
	var sig;
	sig = Pulse.ar([freq/2.98, freq], [0.3, 0.7]) * exc;
	sig = Pan2.ar(sig[0], -0.3) + Pan2.ar(sig[1], 0.3);
	sig = MoogFF.ar(sig, cutoff * SinOsc.ar(speed, 0.0, depth, 0.5), res * 4) * 0.5 * amp;
	Out.ar(out, sig);
}).add;
)
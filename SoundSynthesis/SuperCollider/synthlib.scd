(
SynthDef(\moog, {
	arg freq = 120, amp = 1, exc = 1, cutoff = 2000, res = 0.5, speed = 2, depth = 0.0, detune = 1, out = 0;
	var sig;
	freq = freq + (freq * (Rand(-1.0, 1.0) * detune));
	sig = Pulse.ar([freq/2.98, freq], [0.3, 0.7]) * exc;
	sig = Pan2.ar(sig[0], -0.3) + Pan2.ar(sig[1], 0.3);
	sig = MoogFF.ar(sig, cutoff * SinOsc.ar(speed, 0.0, depth, 0.5), res * 4) * 0.5 * amp;
	Out.ar(out, sig);
}).add;


SynthDef(\noise, {
	arg amp = 1, exc = 1, freq = 150, bw = 10, out = 0;
	var sig;
	sig = WhiteNoise.ar(0.25) * exc;
	sig = BBandPass.ar(sig, freq, bw);
	sig = sig * amp;
	Out.ar(out, [sig, sig]);
}).add;
)
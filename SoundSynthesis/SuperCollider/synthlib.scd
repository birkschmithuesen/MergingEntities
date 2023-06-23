(
SynthDef(\moog, {
	arg freq = 120, amp = 1, exc = 0, cutoff = 2000, res = 0.5, speed = 2, depth = 0.0, detune = 0, pan = 0.5, out = 0;
	var sig;
	freq = freq + (freq * (Rand(-1.0, 1.0) * detune));
	sig = Pulse.ar([freq/2.98, freq], [0.3, 0.7]) * exc;
	sig = Pan2.ar(sig[0], pan-0.3) + Pan2.ar(sig[1], pan+0.3);
	sig = MoogFF.ar(sig, cutoff * SinOsc.ar(speed, 0.0, depth, 0.5), res * 4) * 0.2 * amp * 1.6;
	Out.ar(out, sig);
}).add;


SynthDef(\noise, {
	arg amp = 1, exc = 0, freq_gain = 1, freq = 150, moog_gain = 0, pan = 0.5, out = 0;
	var sig;
	sig = WhiteNoise.ar(0.4) * exc;
	sig = MoogFF.ar(sig, freq, moog_gain) * freq_gain;
	sig = sig * amp * 0.4;
	sig = Pan2.ar(sig, pan);
	Out.ar(out, sig);
}).add;

/*
// replace the noise with pink noise or Moog Filter + Brown noise frokm test.scd
SynthDef(\noise, {
	arg amp = 1, exc = 1, freq = 150, bw = 10, out = 0;
	var sig;
	sig = WhiteNoise.ar(0.25) * exc;
	sig = BBandPass.ar(sig, freq, bw);
	sig = sig * amp * 0.4 * (20-(bw.explin(0.01, 0.4, 0, 19))); //is this working corretcly?
	Out.ar(out, [sig, sig]);
}).add;
*/

/* NOT USED ANY MORE
SynthDef(\saw, {
	arg amp = 1, exc = 1, degree = 12, detune = 1.01, dist = 0.5, out = 0;
	var temp, sig = 0, scale = Scale.minorPentatonic;
	10.do{
		temp = VarSaw.ar(
			(DegreeToKey.kr(scale.as(LocalBuf), degree, scale.stepsPerOctave, 1, 60)).midicps * {Rand(-1.0,1.0).range(detune.reciprocal, detune)}!2,
			{Rand(0.0, 1.0)}!2,
			{ExpRand(0.01, 0.2)}!2
		);
		sig = sig + temp;
	};
	sig = sig * 0.01 * exc;
	sig = Disintegrator.ar(sig, 0.5, dist.linlin(0.0, 1.0, 1.0, 0.0));
	sig = sig * amp;
	Out.ar(out, sig);
}).add;
*/

SynthDef(\lead, {
	arg amp = 1, exc = 0, degree = 7, detune = 0.04, out = 0, pan = 0.5;
	var scale, buffer, sig;
	scale =  FloatArray[0, 3, 5, 7, 10]; // dorian scale FloatArray[0, 2, 3.2, 5, 7, 9, 10];
	buffer = Buffer.alloc(s, scale.size,1, {|b| b.setnMsg(0, scale) });

	sig = SinOsc.ar(
		(
			DegreeToKey.kr(buffer.bufnum, degree,
				12,                    // 12 notes per octave
				1,                    // mul = 1
				72                    // offset by 72 notes
			)
			+ LFNoise1.kr([3,3], detune)    // add some low freq stereo detuning
		).midicps,                        // convert midi notes to hertz
		0,
		0.1);

	// add some 70's euro-space-rock echo
	sig = sig * exc * 0.4;
	sig = CombN.ar(sig, 0.2, 0.2, 0.5, 1, sig) * amp * 1.6;
	sig = Pan2.ar(Mix.ar(sig), pan);
	//sig = Mix.ar(sig);
	Out.ar(out, sig);
}).add;


SynthDef(\kling, {
	arg amp = 1, exc = 1, dens = 1, freq = 1000, dev = 1.01, lfodepth = 10, decay = 1.0, wide = 0.02, pan = 0.0, out = 0;
	var sig;
	sig = {arg count; Impulse.ar(dens, count * wide, exc * 0.2) * LFNoise0.kr(10, 0.5, 0.5)}!2;
	sig = Ringz.ar(sig, [
		LFSaw.kr(dens, 0.0).range(freq - lfodepth, freq),
		LFSaw.kr(dens, 1.0).range(freq, freq + lfodepth) * dev
	], decay);
	sig = Pan2.ar(sig[0], (-0.2 + pan)) + Pan2.ar(sig[1], (0.2 + pan));//Mix.new(sig * amp);
	sig = sig * amp * 1.5;
	Out.ar(out, sig);
}).add;


SynthDef(\grain, {
	arg sndbuf, amp = 1, exc = 1, dur = 0.5, rate = 0.5, pos = 0, out = 0, pan = 0.5;
    var sig, dur_mapped, rate_mapped;
	dur_mapped = dur.linlin(0.0, 1.0, 0.05, 0.5);
	rate_mapped = rate.linlin(0.0, 1.0, 0.5, 1.5);
	sig = BufGrain.ar(Impulse.kr(1/dur_mapped), dur_mapped, sndbuf, rate_mapped, pos, 2)*exc;
	sig = Pan2.ar(sig, pan);
	Out.ar(out, sig);
}).add;


SynthDef.new(\sine, {
	arg amp=1, exc = 1, freq=440, atk=0.005, rel=0.3, shape = 0, pan=0, out=0;
	var sig, osc1, osc2, env, addr;
	osc1 = SinOsc.ar(freq) * exc * 1.75;
	osc2 = VarSaw.ar(freq) * exc;
	sig = (osc1 * (1-shape)) + (osc2 * shape);
	env = EnvGen.kr(Env.new([0, 1, 0], [atk, rel], [1,-1]), doneAction:2);
	sig = Pan2.ar(sig, pan, amp);
	sig = sig * env * 0.4;
	Out.ar(out, sig);
}).add;



SynthDef(\shaker, {
	arg amp = 1, exc = 1, pan = 0.0, rel = 0.5, shape = 1, rev = 0.5, freq = 6000, out = 0;
	var sig, osc1, osc2, env;
	//osc1 = BrownNoise.ar(0.3) * exc;
	osc1 = WhiteNoise.ar(1) * exc;
	osc2 = MoogFF.ar(osc1, freq, 2);
	sig = (osc1 * (0.2 - shape.linlin(0.0, 1.0, 0.0, 0.2)) + (osc2 * shape.linlin(0.0, 1.0, 0.5, 1.0)));
	//sig = MoogFF.ar(sig, atk, 2);
	env = EnvGen.kr(Env.new([0, 1, 0], [0.01, rel.linlin(0.0, 1.0, 0.001, 0.2)], [1,-1]), doneAction:2);
	sig = sig * amp * 0.2;
	sig = Pan2.ar(sig, pan, amp);
	Out.ar(out, sig);
}).add;
)

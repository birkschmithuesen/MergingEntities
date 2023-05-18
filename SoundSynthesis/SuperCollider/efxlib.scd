(
SynthDef.new(\fx, {
	arg out=0, in, mix = 0.33;
	var sig;
	sig = In.ar(in, 2);
	Out.ar(out, sig);
}).add;

SynthDef.new(\reverb, {
	arg out=0, in, mix = 0.33;
	var sig;
	sig = In.ar(in, 2);

	chain = sig;    // Start with the original signal
	8.do {|i|       // Loop 8 times. For each loop, connect the signal through something.

		// A simple reverb
		chain = LeakDC.ar(AllpassL.ar(LPF.ar(chain*0.9,3000), 0.2, {0.19.rand+0.01}!2, 3));
	};

	sig = 0.5 * ((sig * mix) + (chain * (1-mix)));    // dry + wet

	//sig = FreeVerb.ar(sig, mix.linlin(0.0, 1.0, 0.0, 0.5), 0.8, 0.2);
	Out.ar(out, sig);
}).add;











)
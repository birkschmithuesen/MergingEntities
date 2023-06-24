(
SynthDef.new(\fx, {
	arg out=0, in, mix = 0.0;
	var sig;
	sig = In.ar(in, 2);
	Out.ar(out, sig);
}).add;

SynthDef.new(\delay, {
	arg out=0, in, mix = 0.0;
	var sig, dry, wet;
	dry = In.ar(in, 2);
	wet = CombN.ar(dry, ~tempo/4/4, ~tempo/4/4);
	sig = dry + (wet * mix.lincurve(0.0, 1.0, 0.0, 0.3, 1));
	Out.ar(out, sig);
}).add;

SynthDef.new(\reverb, {
	arg out=0, in, mix = 0.0;
	var sig, rev;
	sig = In.ar(in, 2);
	sig = FreeVerb.ar(sig, mix.lincurve(0.0, 1.0, 0.0, 0.5), 0.8, 0.2);
	Out.ar(out, sig);
}).add;

SynthDef.new(\reverb_endless, {
	arg out=0, in, mix = 0.0;
	var sig, chain;
	sig = In.ar(in, 2);

	chain = sig;    // Start with the original signal
	8.do {|i|       // Loop 8 times. For each loop, connect the signal through something.

		// A simple reverb
		chain = LeakDC.ar(AllpassL.ar(LPF.ar(chain*0.9,3000), 0.2, {0.19.rand+0.01}!2, 3));
	};

	sig = (sig * mix) + ((chain * (1-mix))*100);    // dry + wet
	Out.ar(out, sig);
}).add;






)
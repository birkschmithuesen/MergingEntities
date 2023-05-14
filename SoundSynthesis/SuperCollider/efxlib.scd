(
SynthDef.new(\reverb, {
	arg out=0, in, mix = 0.33;
	var sig;
	sig = In.ar(in, 2);
	sig = FreeVerb.ar(sig, mix.linlin(0.0, 1.0, 0.0, 0.4), 0.7, 0.2);
	Out.ar(out, sig);
}).add









)
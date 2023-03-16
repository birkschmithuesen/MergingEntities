




Synths_Sine {
	init {
		"Initalize Sinewave Synth".postln;
		SynthDef(\sine, {
			arg amp = 1, freq = 150, freq2_mult = 1.003714, out = 0;
			var sig;
			sig = SinOsc.ar([freq, freq * freq2_mult], 0, 0.2);
			sig = Mix(sig) * amp;
			Out.ar(out, [sig, sig]);
		}).add;
		~synth = Synth(\sine);
	}
	par1 {arg par;
		~synth.set(\amp, par);
	}
	par2 {arg par;
		~synth.set(\freq, par);
	}
	par3 {arg par;
		~synth.set(\freq2_mult, par);
	}
	par4 {arg par;
	}
}

Synths_Noise {
	init {
		"Initalize White Noise Synth".postln;
		SynthDef(\noise, {
			arg amp = 1, freq = 150, bw = 10, out = 0;
			var sig;
			sig = WhiteNoise.ar(0.25);
			sig = BBandPass.ar(sig, freq, bw);
			sig = sig * amp;
			Out.ar(out, [sig, sig]);
		}).add;
		~synth = Synth(\noise);
	}
	par1 {arg par;
		~synth.set(\amp, par);
	}
	par2 {arg par;
		~synth.set(\freq, par.linexp(0, 1, 80, 4000));
	}
	par3 {arg par;
		~synth.set(\bw, par.linlin(0, 1, 0.5, 5));
	}
	par4 {arg par;
	}
}


SawSynth {
	init {
		"Initalize Sawtooth Synth".postln;
		SynthDef(\saw, {
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
			sig = sig * 0.05 * amp;
			Out.ar(out, sig);
		}).add;
		~synth = Synth(\saw);
	}
	par1 {arg par;
		~synth.set(\amp, par);
	}
	par2 {arg par;
		//~synth.set(\freq, par);
	}
	par3 {arg par;
		~synth.set(\dev, par);
	}
	par4 {arg par;
	}
}

SawSynthTonechange : SawSynth {
	par1 {arg par;
		//par.postln;
		if ( par == 0.0,
			{~synth.set(\freq, round(rrand(60,1000),120), \amp, par)},
			{~synth.set(\amp, par)}
		)
	}
}


KLing{
	init {
		"Initialize Kling Synth".postln;
		SynthDef(\kling, {
			arg amp = 1, exc = 1, dens = 1, freq = 1000, dev = 1.01, lfodepth = 10, decay = 1.0, wide = 0.02, out = 0;
			var sig;
			sig = {arg count; Impulse.ar(dens, count * wide, exc * 0.2) * LFNoise0.kr(10, 0.5, 0.5)}!2;
			sig = Ringz.ar(sig, [
				LFSaw.kr(dens, 0.0).range(freq - lfodepth, freq),
				LFSaw.kr(dens, 1.0).range(freq, freq + lfodepth) * dev
			], decay);
			sig = Pan2.ar(sig[0], -0.3) + Pan2.ar(sig[1], 0.3);//Mix.new(sig * amp);
			Out.ar(out, sig);
		}).add;
		~synth = Synth(\kling);

		// some Control Specs to be able to control the synth with normalized OSC messages in a predefined range
		~ctrfreq = \freq.asSpec;
		~ctrfreq.minval = 100;
		~ctrfreq.maxval = 4000;
		~ctrlfo = \freq.asSpec;
		~ctrlfo.minval = 0.0;
		~ctrlfo.maxval = 800.0;
		~ctrdecay = [0.1, 1.0].asSpec;
	}

	// set the parameters
	par1 {arg par;
		~synth.set(\exc, par);
	}
	par2 {arg par;
		~synth.set(\freq, ~ctrfreq.map(par));
	}
	par3 {arg par;
		~synth.set(\dev, par * 2.0);
	}
	par4 {arg par;
		~synth.set(\lfodepth, ~ctrlfo.map(par));
	}
	par5 {arg par;
		~synth.set(\decay, ~ctrdecay.map(par));
	}
	par8 {arg par;
		~synth.set(\amp, par);
	}

}
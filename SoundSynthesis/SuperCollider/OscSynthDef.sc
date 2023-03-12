OscMoog{
	var <synth;

	classvar ctrcutoff, ctrres, ctrdepth, ctrspeed, ctrdetune;

	*new { arg synth;
		^super.newCopyArgs(synth).init;
	}

	init {
		ctrcutoff = [500, 20000, \exp].asSpec;
		ctrres = [0.16, 0.97, \lin].asSpec;
		ctrdepth = [0.0, 1.0, \lin].asSpec;
		ctrspeed = [2.0, 15.0, \lin].asSpec;
		ctrdetune = [1.0, 1.5, \lin].asSpec;
	}

	par1 {arg par;
		synth.set(\exc, par);
	}
	par2 {arg par;
		synth.set(\cutoff, ctrcutoff.map(par));
	}
	par3 {arg par;
		synth.set(\res, ctrres.map(par));
	}
	par4 {arg par;
		synth.set(\depth, ctrdepth.map(par));
	}
	par5 {arg par;
		synth.set(\speed, ctrspeed.map(par));
	}
	par6 {arg par;
		synth.set(\detune, ctrdetune.map(par));
	}
	par8 {arg par;
		synth.set(\amp, par);
	}
}
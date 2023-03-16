OscMoog{
	var <synth, address;

	classvar ctrcutoff, ctrres, ctrdepth, ctrspeed;

	*new { arg synth, address;
		^super.newCopyArgs(synth, address).init;
	}

	init {
		("Connect "++synth++" with OSC address "++address).postln;
		ctrcutoff = [500, 20000, \exp].asSpec;
		ctrres = [0.16, 0.97, \lin].asSpec;
		ctrdepth = [0.0, 1.0, \lin].asSpec;
		ctrspeed = [2.0, 15.0, \lin].asSpec;

		OSCFunc({
			arg msg;
			if (msg[1] != nil,
				{synth.set(\exc, msg[1]);},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[2] != nil,
				{synth.set(\cutoff, ctrcutoff.map(msg[2]));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[3] != nil,
				{synth.set(\res, ctrres.map(msg[3]));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[4] != nil,
				{synth.set(\depth, ctrdepth.map(msg[4]));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[5] != nil,
				{synth.set(\speed, ctrspeed.map(msg[5]));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);
	}
}

OscMoogGroup{
	var <group, address;

	classvar ctrdetune;

	*new {arg group, address;
		^super.newCopyArgs(group, address).init;
	}

	init {
		("Connect " ++ group ++ " with OSC address " ++ address).postln;
		ctrdetune = [0.0, 2.0, \lin].asSpec;

		OSCFunc({
			arg msg;
			if (msg[1] != nil,
				{group.set(\amp, msg[1]);},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[2] != nil,
				{group.set(\detune, ctrdetune.map(msg[2]));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);
	}
}

OscNoise{
	var <synth, address;

	*new {arg synth, address;
		^super.newCopyArgs(synth, address).init;
	}

	init {
		("Connect " ++ synth ++ " with OSC address " ++ address).postln;

		OSCFunc({
			arg msg;
			if (msg[1] != nil,
				{synth.set(\exc, msg[1]);},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[2] != nil,
				{synth.set(\freq, msg[2].linexp(0, 1, 80, 4000));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[3] != nil,
					{synth.set(\bw, msg[3].linlin(0, 1, 0.5, 5));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);
	}
}

OscNoiseGroup{
	var <group, address;

	classvar ctrdetune;

	*new {arg group, address;
		^super.newCopyArgs(group, address).init;
	}

	init {
		("Connect " ++ group ++ " with OSC address " ++ address).postln;

		OSCFunc({
			arg msg;
			if (msg[1] != nil,
				{group.set(\amp, msg[1]);},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);
	}
}
OscMoog{
	var <synth, address, detune_min, detune_max;

	classvar ctrcutoff, ctrres, ctrdepth, ctrspeed;

	*new { arg synth, address, detune_min, detune_max;
		^super.newCopyArgs(synth, address, detune_min, detune_max).init;
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
			if (msg[6] != nil,
				{synth.set(\detune, msg[6].linlin(0, 1, detune_min, detune_max));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[7] != nil,
				{synth.set(\pan, msg[7].linlin(0, 1, -1, 1));
				},
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
				{
					synth.set(\freq, msg[2].linexp(0, 1, 200, 10000));
					synth.set(\freq_gain, msg[2].linexp(0, 1, 10, 1));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[3] != nil,
				{synth.set(\moog_gain, msg[3].linlin(0, 1, 0.0, 3.9));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[4] != nil,
				{synth.set(\pan, msg[4].linlin(0, 1, -1.0, 1.0));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			/*
			if (msg[2] != nil,
			{synth.set(\freq, msg[2].linexp(0, 1, 1800, 4000));},
			{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[3] != nil,
			{synth.set(\bw, msg[3].linlin(0, 1, 0.01, 0.4));},
			{("received wrong OSC message length on address: " ++ address).postln}
			);
			*/
		},
		(address)
		);
	}
}


OscSaw{
	var <synth, address;

	*new {arg synth, address;
		^super.newCopyArgs(synth, address).init;
	}

	init {
		("Connect " ++ synth ++ " with OSC address " ++ address).postln;

		OSCFunc({
			arg msg;
			if (msg[1] != nil,
				{
					if ( msg[1] == 0.0,
						{synth.set(\degree, rrand(0,9), \exc, 0.0)},
						{synth.set(\exc, msg[1]);}
					)

				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[2] != nil,
				{synth.set(\dist, msg[2]);}//,{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);
	}
}


OscLead{
	var <synth, address, degree_min, degree_max, detune_min, detune_max;

	*new {arg synth, address, degree_min, degree_max, detune_min, detune_max;
		^super.newCopyArgs(synth, address, degree_min, degree_max, detune_min, detune_max).init;
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
				{synth.set(\degree, msg[2].linlin(0, 1, degree_min, degree_max).round);
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[3] != nil,
				{synth.set(\detune, msg[3].linlin(0, 1, detune_min, detune_max));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[4] != nil,
				{synth.set(\pan, msg[4].linlin(0, 1, -1.0, 1.0));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);

	}
}


OscKling{
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
				{synth.set(\freq, msg[2].linexp(0, 1, 100.0, 4000.0));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[3] != nil,
				{synth.set(\dev, msg[3].linlin(0, 1, 1.01, 2.0));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[4] != nil,
				{synth.set(\lfodepth, msg[4].linexp(0, 1, 5.0, 600.0));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[5] != nil,
				{synth.set(\decay, msg[5].linlin(0, 1, 0.03, 1.0));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[6] != nil,
				{synth.set(\dens, msg[6].linlin(0, 1, 5.0, 15.0));
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[7] != nil,
				{synth.set(\pan, msg[7]);
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);

	}
}


OscGrain{
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
				{synth.set(\dur, msg[2]);
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[3] != nil,
				{synth.set(\rate, msg[3]);
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[4] != nil,
				{synth.set(\pos, msg[4]);
				},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);

	}
}

// GROUPS

OscStandartGroup{
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


OscDetuneGroup{
	var <group, address, detune_min, detune_max;

	*new {arg group, address, detune_min, detune_max;
		^super.newCopyArgs(group, address, detune_min, detune_max).init;
	}

	init {
		("Connect " ++ group ++ " with OSC address " ++ address).postln;

		OSCFunc({
			arg msg;
			if (msg[1] != nil,
				{group.set(\amp, msg[1]);},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
			if (msg[2] != nil,
				{group.set(\detune, msg[2].linlin(0, 1, detune_min, detune_max));},
				{("received wrong OSC message length on address: " ++ address).postln}
			);
		},
		(address)
		);
	}
}

ServerOptions.devices; // all devices

Server.local.options.numOutputBusChannels = 4;
Server.default.options.outDevice_("Fireface UC Mac (23557042)");
s.reboot;

(
SynthDef(\drops, { |out, gate = 1, mask = 1, freq = 440, amp = 0.1, decay = 2, trigFreq = 10|
	var masterEg = EnvGen.kr(Env.asr(0.005, 1, decay), gate, doneAction: 2),
	trig = Impulse.kr(trigFreq),
	// note trig * mask: if mask == 0, then no triggers pass through
	dropEg = EnvGen.kr(Env.perc(0.01, 0.08), trig * mask),
	sig = SinOsc.ar(freq); //VarSaw.ar(freq);
	Out.ar(out, (sig * (amp * dropEg * masterEg)).dup);
}).add;
)



(

x = Synth.new(\drops, [\out, 2]);

///////////////////////////////////////////
// HUMAN
///////////////////////////////////////////

OSCdef.new(\Positionx0, {
	arg msg;
	x.set(
		\freq, msg[1].linlin(0,6.8,40,1000)
	);
}, '/Positionx0'
);



OSCdef.new(\Positiony0, {
	arg msg;
	x.set(
		\trigFreq, msg[1].linlin(0,3.6,1,20)
	);
}, '/Positiony0'
);
)


(
y = Synth.new(\drops, [\out, 2]);

///////////////////////////////////////////
// MACHINE
///////////////////////////////////////////

OSCdef.new(\Positionx1, {
	arg msg;
	y.set(
		\freq, msg[1].linlin(0,6.8,40,1000)
	);
}, '/Positionx1'
);



OSCdef.new(\Positiony1, {
	arg msg;
	y.set(
		\trigFreq, msg[1].linlin(0,3.6,1,20)
	);
}, '/Positiony1'
);
)


s.plotTree;
ServerOptions.devices; // all devices

Server.local.options.numOutputBusChannels = 2;
Server.default.options.outDevice_("Fireface UC Mac (23557042)");
s.reboot;

(
SynthDef(\drops, {
	arg out, freq = 440, amp = 0.5, decay = 2, trigFreq = 10, room = 0.5, dist = 1, lpf = 30000;
	var trig, dropEg, sig;
	trig = Impulse.kr(trigFreq);
	dropEg = EnvGen.kr(Env.perc(0.01, 0.08), trig);
	sig = SinOsc.ar(freq);
	sig = sig * dropEg;
	sig = (sig * dist).fold2(1);
	sig = RLPF.ar(sig, lpf, 0.2, 0.2);
	sig = FreeVerb.ar(sig, mix: 0.33, room:room)*amp;
	Out.ar(out, sig!2);
}).add;
)


distort
s.meter
s.plotTree;

s.scope;
(


x = Synth.new(\drops, [\out, 0]);

///////////////////////////////////////////
// HUMAN
///////////////////////////////////////////

OSCdef.new(\freq, {
	arg msg;
	x.set(
		\freq, msg[1].linlin(0,1,40,1000)
	);
}, '/freq'
);

OSCdef.new(\trigFreq, {
	arg msg;
	x.set(
		\trigFreq, msg[1].linlin(0,1,1,20)
	);
}, '/trigFreq'
);

OSCdef.new(\reverbRoom, {
	arg msg;
	x.set(
		\room, msg[1]
	);
}, '/reverbRoom'
);

OSCdef.new(\amp, {
	arg msg;
	x.set(
		\amp, msg[1]
	);
}, '/amp'
);

OSCdef.new(\dist, {
	arg msg;
	x.set(
		\dist, msg[1].linlin(0,1,1,30)
	);
}, '/dist'
);

OSCdef.new(\lpf, {
	arg msg;
	x.set(
		\lpf, msg[1].linexp(0,1,20,30000)
	);
}, '/lpf'
);

)

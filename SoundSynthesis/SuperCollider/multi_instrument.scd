ServerOptions.devices; // all devices

Server.local.options.numOutputBusChannels = 2;
Server.default.options.outDevice_("Fireface UC Mac (23557042)");
Server.default.options.outDevice_("External Headphones");
s.reboot;
s.meter;

///////////////////////////////////////////
// DROPS
///////////////////////////////////////////


(
SynthDef(\drops, {
	arg out, freq = 440, amp = 0.5, decay = 2, trigFreq = 10, room = 0.75, dist = 30, lpf = 30000;
	var trig, dropEg, sig;
	trig = Impulse.kr(trigFreq);
	dropEg = EnvGen.kr(Env.perc(0.01, 0.08), trig);
	sig = SinOsc.ar(freq);
	sig = sig * dropEg;
	sig = (sig * dist).fold2(1);
	sig = RLPF.ar(sig, lpf, 0.2);
	sig = FreeVerb.ar(sig, mix: 0.33, room:room)*amp*0.2;
	Out.ar(out, sig!2);
}).add;


~drops_1= Synth.new(\drops, [
	\out, 0,
	\room, 0.75,
	\trigFreq, 1,
	\dist, 1,
	\freq, 110,
]);

OSCdef.new(\drops1, {
	arg msg;
	~drops_1.set(
		\amp, msg[1]
	);
}, '/drops/1'
);

OSCdef.new(\lpf1, {
	arg msg;
	~drops_1.set(
		\lpf, msg[1].linexp(0,1,60,30000)
	);
}, '/lpf/1'
);

OSCdef.new(\dist1, {
	arg msg;
	~drops_1.set(
		\dist, msg[1].linexp(0,1,1,10)
	);
}, '/dist/1'
);

~drops_2= Synth.new(\drops, [
	\out, 0,
	\room, 0.75,
	\trigFreq, 1.75,
	\dist, 1,
	\freq, 220,
]);

OSCdef.new(\drops2, {
	arg msg;
	~drops_2.set(
		\amp, msg[1]
	);
}, '/drops/2'
);

OSCdef.new(\lpf2, {
	arg msg;
	~drops_2.set(
		\lpf, msg[1].linexp(0,1,60,30000)
	);
}, '/lpf/2'
);

OSCdef.new(\dist2, {
	arg msg;
	~drops_2.set(
		\dist, msg[1].linexp(0,1,1,10)
	);
}, '/dist/2'
);

~drops_3= Synth.new(\drops, [
	\out, 0,
	\room, 0.75,
	\trigFreq, 2.5,
	\dist, 1,
	\freq, 350,
]);

OSCdef.new(\drops3, {
	arg msg;
	~drops_3.set(
		\amp, msg[1]
	);
}, '/drops/3'
);

OSCdef.new(\lpf3, {
	arg msg;
	~drops_3.set(
		\lpf, msg[1].linexp(0,1,60,30000)
	);
}, '/lpf/3'
);

OSCdef.new(\dist3, {
	arg msg;
	~drops_3.set(
		\dist, msg[1].linexp(0,1,1,10)
	);
}, '/dist/3'
);

)



///////////////////////////////////////////
// TUBE
///////////////////////////////////////////

(
SynthDef(\tube, {
	arg exciter = 0.5, res_array = 0.5, res_delay = 0.5, out = 0, amp = 0.1;
	var sig;
	sig = NTube.ar(
		PinkNoise.ar * exciter,
		`[0.97,1.0,1.0,1.0,1.0,0.97],
		`[0.5,res_array,0.2,-0.4],
		`([0.01,0.02,0.01,0.005,0.05]*res_delay));
	sig = sig * amp*0.3;
	Out.ar(out, sig!2);
}).add;

~tube = Synth.new(\tube);


OSCdef.new(\exciter, {
	arg msg;
	~tube.set(\exciter, msg[1]);
}, '/exciter'
);

OSCdef.new(\res_array, {
	arg msg;
	~tube.set(\res_array, msg[1].linlin(0, 1, -1, 1));
}, '/res_array'
);

OSCdef.new(\res_delay, {
	arg msg;
	~tube.set(\res_delay, msg[1].linexp(0, 1, 0.001, 1));
}, '/res_delay'
);

OSCdef.new(\tube, {
	arg msg;
	~tube.set(\amp, msg[1].linexp(0, 1, 0.001, 1));
}, '/tube'
);
)



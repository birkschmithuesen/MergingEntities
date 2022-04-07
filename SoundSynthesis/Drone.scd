ServerOptions.devices; // all devices

Server.local.options.numOutputBusChannels = 4;
Server.default.options.outDevice_("Fireface UC Mac (23557042)");
s.reboot;


(
/*
A lush drone using a waveshaping algorithm for interpolating between multiple wavetables. This code is based off of a similar code example by totalgee:

http://sccode.org/1-4V1

It was only after coding this example that I realized the UGens VOsc and VOsc3 are specifically designed for this wavetable interpolating/morphing function. But this code was still very fun to build.

EF
*/

s.freeAll;
Window.closeAll;
Buffer.freeAll;

/*
Function for interpolating between two
values based on an interpolation value:

0 ==> old, 1 ==> new,
0.5 ==> halfway between old/new, etc
*/
~interpFn = {
	arg old=20, new=10, bal=0.5;
	old + ((new - old) * bal);
};

/*
three wacky envelopes, each converted to Signal format.
each has 12 level points. the first and last value are
always zero. the inner 10 points are random between -1
and +1. the internal functions normalize the levels so
the highest value is always +/-1.0
*/
~wt0 = Env(
	[0]++
	{
		var levs, peak;
		levs = {rrand(-1.0,1.0)}!10;
		peak = levs.abs.maxItem;
		levs = levs * peak.reciprocal;
	}.value ++
	[0],
	{exprand(0.01,1)}!11,
	{exprand(0.1,4)}!11
).asSignal(512);

~wt1 = Env(
	[0]++
	{
		var levs, peak;
		levs = {rrand(-1.0,1.0)}!10;
		peak = levs.abs.maxItem;
		levs = levs * peak.reciprocal;
	}.value ++
	[0],
	{exprand(0.01,1)}!11,
	{exprand(0.1,4)}!11
).asSignal(512);

~wt2 = Env(
	[0]++
	{
		var levs, peak;
		levs = {rrand(-1.0,1.0)}!10;
		peak = levs.abs.maxItem;
		levs = levs * peak.reciprocal;
	}.value ++
	[0],
	{exprand(0.01,1)}!11,
	{exprand(0.1,4)}!11
).asSignal(512);

/*
Signals that contain values representing interpolations between two wavetables.
~i0 interpolates between ~wt0 <==> ~wt1
~i1 interpolates between ~wt1 <==> ~wt2
*/
~i0 = ~wt0.copy;
~i1 = ~wt2.copy;

s.waitForBoot({

	//load signals to buffers in wavetable format
	~wt0Buf = Buffer.loadCollection(s, ~wt0.asWavetable);
	~wt1Buf = Buffer.loadCollection(s, ~wt1.asWavetable);
	~wt2Buf = Buffer.loadCollection(s, ~wt2.asWavetable);
	~i0Buf = Buffer.alloc(s, 1024, 1);
	~i1Buf = Buffer.alloc(s, 1024, 1);
	s.sync;
	~i0Buf.setn(0, ~i0.asWavetable);
	~i1Buf.setn(0, ~i1.asWavetable);

	s.sync;

	/*
	Create two Osc Synths, reading their wavetable
	data from ~i0Buf and ~i1Buf
	*/
	//SynthDef.new("drone"
	~synths = [~i0Buf, ~i1Buf].collect{
		arg buf, i;
		{
			/*
			detune=0 makes a much more boring sound, but also
			makes the wavetable interpolation more
			observable (evaluate s.scope and adjust the
			horizontal slider to watch the waveform)
			*/
			arg detune=0.2, freq=40, amp=0.1, filter=1000;
			var sig;
			sig = Osc.ar(
				buf.bufnum,

				//frequency with random moving detune value (in semitones)
				freq * LFNoise1.kr({Rand(0.08,0.15)}!8).bipolar(detune).midiratio,

				{Rand(0,2pi)}!8
			);

			//spread 8-channel detuned Osc texture across two channels
			sig = Splay.ar(sig);

			//avoid funky DC bias
			sig = LeakDC.ar(sig);

			sig = sig * amp;

			//sig = BLowPass.ar(sig, MouseY.kr(40, 10000,\exponential));
			sig = BBandPass.ar(sig, filter, 1.0, 5.0); //MouseY.kr(20, 10000, \exponential)
			//Out.ar(sig, 0)
		}.play(fadeTime:4);
	};

})
)

s.meter;

(
OSCdef.new(\Positionx0, {
	arg msg;
	~synths.do(_.set(\freq, msg[1].linlin(0,7,10,100)));
}, '/Positionx0'
);



OSCdef.new(\Positiony0, {
	arg msg;
	~synths.do(_.set(\filter, msg[1].linexp(0,3,20,8000)));
}, '/Positiony0'
);
)



//////////////////////////
// MACHINE
/////////////////////////

(
	/*
	Create two Osc Synths, reading their wavetable
	data from ~i0Buf and ~i1Buf
	*/
	~synths2 = [~i0Buf, ~i1Buf].collect{
		arg buf, i;
		{
			/*
			detune=0 makes a much more boring sound, but also
			makes the wavetable interpolation more
			observable (evaluate s.scope and adjust the
			horizontal slider to watch the waveform)
			*/
			arg detune=0.2, freq=40, amp=0.1, filter=1000;
			var sig;
			sig = Osc.ar(
				buf.bufnum,

				//frequency with random moving detune value (in semitones)
				freq * LFNoise1.kr({Rand(0.08,0.15)}!8).bipolar(detune).midiratio,

				{Rand(0,2pi)}!8
			);

			//spread 8-channel detuned Osc texture across two channels
			sig = Splay.ar(sig);

			//avoid funky DC bias
			sig = LeakDC.ar(sig);

			sig = sig * amp;

			//sig = BLowPass.ar(sig, MouseY.kr(40, 10000,\exponential));
			sig = BBandPass.ar(sig, filter, 1.0, 5.0); //MouseY.kr(20, 10000, \exponential)

		}.play(fadeTime:4);
	};

)


s.meter;

(
OSCdef.new(\Positionx1, {
	arg msg;
	~synths2.do(_.set(\freq, msg[1].linlin(0,7,10,100)));
}, '/Positionx1'
);



OSCdef.new(\Positiony1, {
	arg msg;
	~synths2.do(_.set(\filter, msg[1].linexp(0,3,20,8000)));
}, '/Positiony1'
);
)


//change frequency
~synths.do(_.set(\freq, 200));

//the two Synths can be changed independently
(
~synths[0].set(\freq, 55);
~synths[1].set(\freq, 108, \amp, 0.05);
)

//fade out
~synths.do(_.set(\gate, 0));
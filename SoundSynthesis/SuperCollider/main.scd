Server.killAll

ServerOptions.devices; // all devices
Server.default.options.inDevice_("Built-in Microph");
Server.default.options.outDevice_("BlackHole 64ch");
Server.default.options.outDevice_("MacBook Pro Speakers");
s.numOutputBusChannels = numChannels: 64;

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)

(
s.reboot{
	Server.default.options.outDevice_("BlackHole 64ch");
	s.options.numOutputBusChannels = 64;
}
)


(
// set global tempo
~tempo = 60/120 * 4; //60/90 * 4 //60sec / 128 BPM * 4 bars

// load Synth Defs and Pattern Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;
(~path++"efxlib.scd").load;

// define number of performers bodies
~numBodies = 4;
)


// SZENE 1 WHEEL
(~path++"wheel.scd").load;


(~path++"parts.scd").load;

(~path++"machine.scd").load;
(~path++"patternlib_2.scd").load;


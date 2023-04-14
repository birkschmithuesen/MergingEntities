Server.killAll

ServerOptions.devices; // all devices
Server.default.options.inDevice_("Built-in Microph");
Server.default.options.outDevice_("BlackHole 64ch");
s.numOutputBusChannels = numChannels: 64;

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)

(
s.reboot{
	s.options.numOutputBusChannels = 64;
}
)

(
// DOMINO -
// one instrument per body part
// each performer same instrument

var scale;

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;

// define number of performers bodies
~numBodies = 2;

// create groups
~moogGroup = Group.new;
~noiseGroup = Group.new;
//~sawGroup = Group.new;
~leadGroup = Group.new;

// create Synths
scale = FloatArray[48, 55, 60, 67];//some 5ths starting from c4
~moogs = {arg i; Synth.head(~moogGroup, \moog, [\out, i*2])}!~numBodies;
{arg i;
	~moogs[i].set(\freq, scale[i].midicps);
	("Created "++~moogs[i]++" with freq: "++(300+ (i*52))).postln;
}!~numBodies;

~noises = {arg i; Synth.head(~noiseGroup,\noise, [\out, i*2])}!~numBodies;
{arg i; ("Created "++~noises[i]).postln}!~numBodies;

/*
~saws = {arg i; Synth.head(~performerGroup[i],\saw)}!~numBodies;
{arg i; ("Created "++~saws[i]).postln}!~numBodies;
*/

~leads = {arg i; Synth.head(~leadGroup, \lead, [\out, i*2])}!~numBodies;
{arg i; ("Created "++~leads[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscMoog = {arg i; OscMoog.new(~moogs[i], ('/moog/' ++ i), 0.0, 1.3)}!~numBodies;
~oscMoogGroup = OscStandartGroup.new(~moogGroup, ('/moog/group'), 0.0, 2.0);

~oscNoise = {arg i; OscNoise.new(~noises[i], ('/noise/' ++ i))}!~numBodies;
~oscNoiseGroup = OscStandartGroup.new(~noiseGroup, ('/noise/group'));

//~oscSaw = {arg i; OscSaw.new(~saws[i], ('/saw/' ++ i))}!~numBodies;
//~oscSawGroup = OscDetuneGroup.new(~sawGroup, ('/saw/group'), 1.01, 1.5);

~oscLead = {arg i; OscLead.new(~leads[i], ('/lead/' ++ i), 0, 13, 0.04, 2.0)}!~numBodies;
~oscLeadGroup = OscStandartGroup.new(~leadGroup, ('/lead/group'));
)


(
// MOVEMENT -
// one body one instrument
// each performer has the same instrument

// create groups
~klingGroup = Group.new;

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;

// define number of performers bodies
~numBodies = 2;

// create Synths
~klings = {Synth.head(~klingGroup, \kling)}!~numBodies;
{arg i; ("Created "++~klings[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscKling = {arg i; OscKling.new(~klings[i], ('/kling/' ++ i))}!~numBodies;
~oscKlingGroup = OscStandartGroup.new(~klingGroup, ('/kling/group'));


)

s.meter;
~moogGroup.set(\amp, );
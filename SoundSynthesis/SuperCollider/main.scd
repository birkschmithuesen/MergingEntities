Server.killAll

ServerOptions.devices; // all devices
Server.default.options.inDevice_("Built-in Microph");
Server.default.options.outDevice_("External Headphones");

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)


(
// DOMINO -
// one instrument per body part
// each performer same instrument

var scale;

// create groups
~moogGroup = Group.new;
~noiseGroup = Group.new;
~sawGroup = Group.new;
~leadGroup = Group.new;

// load Synth Defs
~path = PathName(thisProcess.nowExecutingPath).parentPath;
(~path++"synthlib.scd").load;

// define number of performers bodies
~numBodies = 4;

// create Synths
scale = FloatArray[48, 55, 60, 67];//some 5ths starting from c4
~moogs = {Synth.head(~moogGroup,\moog)}!~numBodies;
{arg i;
	~moogs[i].set(\freq, scale[i].midicps);
	("Created "++~moogs[i]++" with freq: "++(300+ (i*52))).postln;
}!~numBodies;

~noises = {Synth.head(~noiseGroup,\noise)}!~numBodies;
{arg i; ("Created "++~noises[i]).postln}!~numBodies;

~saws = {Synth.head(~sawGroup,\saw)}!~numBodies;
{arg i; ("Created "++~saws[i]).postln}!~numBodies;

~leads = {Synth.head(~leadGroup, \lead)}!~numBodies;
{arg i; ("Created "++~leads[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscMoog = {arg i; OscMoog.new(~moogs[i], ('/moog/' ++ i))}!~numBodies;
~oscMoogGroup = OscDetuneGroup.new(~moogGroup, ('/moog/group'), 0.0, 2.0);

~oscNoise = {arg i; OscNoise.new(~noises[i], ('/noise/' ++ i))}!~numBodies;
~oscNoiseGroup = OscStandartGroup.new(~noiseGroup, ('/noise/group'));

~oscSaw = {arg i; OscSaw.new(~saws[i], ('/saw/' ++ i))}!~numBodies;
~oscSawGroup = OscDetuneGroup.new(~sawGroup, ('/saw/group'), 1.01, 1.5);

~oscLead = {arg i; OscLead.new(~leads[i], ('/lead/' ++ i), 0, 15)}!~numBodies;
~oscLeadGroup = OscDetuneGroup.new(~leadGroup, ('/lead/group'), 0.04, 1.0);
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
~numBodies = 4;

// create Synths
~klings = {Synth.head(~klingGroup, \kling)}!~numBodies;
{arg i; ("Created "++~klings[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscKling = {arg i; OscKling.new(~klings[i], ('/kling/' ++ i))}!~numBodies;
~oscKlingGroup = OscStandartGroup.new(~klingGroup, ('/kling/group'));


)
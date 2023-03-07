Server.killAll

(
s.boot;
s.plotTree;
s.meter;
s.freqscope;
)



// create a Synth
(
~numBodies = 3;

~moogs = {Synth(\moog)}!~numBodies;

{arg i; ~moogs[i].set(\freq, 200 + (i*400))}!~numBodies;

~oscDefMoog = {OscMoog.new}!~numBodies;
{arg i; ~oscDefMoog[i].init(~moogs[i])}!~numBodies;

)


(

// make the Synth controllable via OSC with four parameters
~oscRecMoog = {OscReceiver.new}!~numBodies;
{arg i; ~oscRecMoog[i].connect(~oscDefMoog[i], ('/' ++ i))}!~numBodies;

//{arg i; ('/' ++ i).postln}!3;
)

"0.4".lagw
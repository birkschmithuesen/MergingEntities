(
// WHEEL -
// one body one instrument
// each performer has the same instrument

// create groups
~klingGroup = Group.new;


// create Synths
~klings = {arg i; Synth.head(~klingGroup, \kling, [\out, i*2])}!~numBodies;
{arg i; ("Created "++~klings[i]).postln}!~numBodies;


// make the Synth controllable via OSC
~oscKling = {arg i; OscKling.new(~klings[i], ('/kling/' ++ i))}!~numBodies;
~oscKlingGroup = OscStandartGroup.new(~klingGroup, ('/kling/group'));
)
//////////////////////////////////////////////////////
// SENDER TO SEND OUT VALUES FROM THE SYNTHS e.g. envelopes
//////////////////////////////////////////////////////

OscSender{
	var addr_in, addr_out, target_ip, target_port;


	*new { arg addr_in, addr_out, target_ip, target_port;
		^super.newCopyArgs(addr_in, addr_out, target_ip, target_port).init;
	}

	init {
		("Receive values from "++addr_in++" and send it to "++addr_out).postln;
		~targetAddr = NetAddr(target_ip, target_port);
		OSCdef(addr_in, {
			arg msg;
			var data = msg[3];
			~targetAddr.sendMsg(addr_out, data);
		}, addr_in);
	}
}

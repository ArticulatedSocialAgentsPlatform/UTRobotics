package nl.utwente.hmi.mwdialogue.informationstate;

import hmi.flipper.informationstate.Record;

public interface Observer {

	public void hasChanged(Record record);
	
}

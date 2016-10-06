package nl.utwente.hmi.mwdialogue.persistence;

import java.io.Serializable;

import hmi.flipper.informationstate.Record;

/**
 * This simple wrapper class allows us to persistently store a subsection of the IS, while maintaining it's original path.
 * Using functions in PersistenceFunctions we can load and store records.
 * @author davisond
 *
 */
public class PersistentRecord implements Serializable {

	private static final long serialVersionUID = -1965938921275509708L;
	private String path;
	private Record rec;

	public PersistentRecord(String path, Record rec){
		this.setPath(path);
		this.setRec(rec);
	}

	public String getPath() {
		return path;
	}

	public void setPath(String path) {
		this.path = path;
	}

	public Record getRec() {
		return rec;
	}

	public void setRec(Record rec) {
		this.rec = rec;
	}
	
}

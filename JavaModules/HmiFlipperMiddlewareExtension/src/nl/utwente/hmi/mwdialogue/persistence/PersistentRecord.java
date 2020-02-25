/*******************************************************************************
 * Copyright (C) 2009-2020 Human Media Interaction, University of Twente, the Netherlands
 *
 * This file is part of the Articulated Social Agents Platform BML realizer (ASAPRealizer).
 *
 * ASAPRealizer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ASAPRealizer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ASAPRealizer.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
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

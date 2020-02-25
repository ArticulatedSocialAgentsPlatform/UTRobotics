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
package nl.utwente.hmi.communication;

import java.util.ArrayList;
import java.util.List;

public class Datatarget {

	private String name;
	private String middlewareLoaderClass;
	private String middlewareLoaderProperties;

	public Datatarget(String name, String middlewareLoaderClass, String middlewareLoaderProperties){
		this.setName(name);
		this.setMiddlewareLoaderClass(middlewareLoaderClass);
		this.setMiddlewareLoaderProperties(middlewareLoaderProperties);
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public String getMiddlewareLoaderClass() {
		return middlewareLoaderClass;
	}

	public void setMiddlewareLoaderClass(String internalPort) {
		this.middlewareLoaderClass = internalPort;
	}
	
	public String getMiddlewareLoaderProperties() {
		return middlewareLoaderProperties;
	}

	public void setMiddlewareLoaderProperties(String externalPort) {
		if(externalPort == null){
			externalPort = "";
		}
		this.middlewareLoaderProperties = externalPort;
	}
	
}

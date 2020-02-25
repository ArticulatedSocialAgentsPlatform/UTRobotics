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
package nl.utwente.hmi.middleware.yarp;

import java.util.Properties;
import java.util.Map.Entry;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.loader.MiddlewareLoader;

public class YARPMiddlewareLoader implements MiddlewareLoader {
	private static Logger logger = LoggerFactory.getLogger(YARPMiddlewareLoader.class.getName());

	/**
	 * This loads the STOMPMiddleware instance
	 */
	@Override
	public Middleware loadMiddleware(Properties ps) {
		Middleware m = null;
		String iPortName = "";
		String oPortName = "";
		
		for(Entry<Object, Object> entry : ps.entrySet()){
            System.out.println("propkey:"+(String)entry.getKey());
            System.out.println("propval:"+(String)entry.getValue());
            if(((String)entry.getKey()).equals("iPortName")){
				iPortName = (String)entry.getValue();
			}
            if(((String)entry.getKey()).equals("oPortName")){
				oPortName = (String)entry.getValue();
			}
		}
		
		if(iPortName.equals("") || oPortName.equals("")){
			logger.error("Could not load the YARPMiddleware, need at least properties: iPortName and oPortName");
		} else {
			m = new YARPMiddleware(iPortName, oPortName);
		}
		
		return m;
	}

}

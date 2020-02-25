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
package nl.utwente.hmi.mwdialogue.function;

import java.util.Arrays;

import nl.utwente.hmi.mwdialogue.ScenarioController;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class LoggerFunctions implements FunctionClass {
    private static Logger logger = LoggerFactory.getLogger("FlipperLogger");

	
	public void logMessage(Object... params){
		//this seems ugly.. but it was the only way I could get it to work with the reflection invoke...
		if(params instanceof String[]){
			String[] par = (String[])params;
			if(params.length >= 2){
				String level = par[0];
				String msg = par[1];
				String[] ps = Arrays.copyOfRange(par, 2, par.length);
				if("trace".equals(level)){
					logger.trace(msg, ps);
				} else if("debug".equals(level)){
					logger.debug(msg, ps);
				} else if("warn".equals(level)){
					logger.warn(msg, ps);
				} else if("error".equals(level)){
					logger.error(msg, ps);
				} else {
					logger.info(msg, ps);
				}
			} else if(params.length == 1){
				logger.info(par[0]);
			}
		}
	}
}

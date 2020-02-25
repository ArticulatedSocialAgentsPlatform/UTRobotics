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
package nl.utwente.hmi.mwdialogue;

import java.util.concurrent.ConcurrentHashMap;

/**
 * This is a thread-safe Singleton Configuration class, used to store the dialogues configuration information
 * This singleton uses the initialization on demand pattern: https://en.wikipedia.org/wiki/Initialization-on-demand_holder_idiom 
 * It is backed by a ConcurrentHashMap for storing the actual config information
 * @author davisond
 *
 */
public class Configuration {

	private ConcurrentHashMap<String, Object> configs = new ConcurrentHashMap<String, Object>();
	
	private static class Holder {
        static final Configuration INSTANCE = new Configuration();
    }

	/**
	 * Get the singleton instance of the config file
	 * @return
	 */
    public static Configuration getInstance() {
        return Holder.INSTANCE;
    }
    
    /**
     * Store a certain config value
     * @param name the name of the config
     * @param config the config value
     */
    public void storeConfig(String name, Object config){
    	configs.put(name, config);
    }
    
    /**
     * Retrieve a configuration value
     * @param name the value to retrieve
     * @return the config value, or null if not found
     */
    public Object getConfig(String name){
    	return configs.get(name);
    }
	
}

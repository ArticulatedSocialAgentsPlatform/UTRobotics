/*******************************************************************************
 * 
 * Copyright (C) 2009 Human Media Interaction, University of Twente, the Netherlands
 * 
 * This file is part of the Elckerlyc BML realizer.
 * 
 * Elckerlyc is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Elckerlyc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Elckerlyc.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
package asap.zeno.embodiment;

import hmi.environmentbase.Embodiment;
import hmi.environmentbase.EmbodimentLoader;
import hmi.environmentbase.Environment;
import hmi.environmentbase.Loader;
import hmi.xml.XMLScanException;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

import java.io.IOException;
import java.util.HashMap;
import java.util.Properties;

import lombok.Delegate;
import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.loader.GenericMiddlewareLoader;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.zeno.ZenoPlanner;
import asap.zeno.api.ZenoRobotController;
import asap.zeno.api.ZenoSpeechListener;
import asap.zeno.middlewareadapter.ZRCToMiddleware;
import asap.zeno.planunit.SpeakZU;

/**
*  This embodiment initiates an instance of a ZenoRobotController which might or might not use a Middleware such as YARP or Apollo to communicate to an actual robot controller
*/
public class ZenoEmbodiment implements Embodiment, EmbodimentLoader, ZenoSpeechListener
{
    private String id = "";
    private static Logger logger = LoggerFactory.getLogger(ZenoPlanner.class.getName());
	
    @Delegate
    private ZenoRobotController zenoRobot;
    
    HashMap<String,SpeakZU> speakidToZenounit = new HashMap<>();

    public void setId(String id)
    {
        this.id = id;
    }

    @Override
    public String getId()
    {
        return id;
    }
    
    /** This id is used to ensure a unique id when multiple bml blocks are executed simultanously with similar speech instructions*/
    private int uniqueIdCounter = 0;

    /**
     * Connects to and registers some producers and consumers for the various topics made available by the ROS Bridge
     */
    @Override
    public void readXML(XMLTokenizer theTokenizer, String loaderId, String vhId, String vhName, Environment[] environments, Loader ... requiredLoaders) 
    	throws IOException
    {
        setId(loaderId);
        
        if(!theTokenizer.atSTag("MiddlewareOptions"))
        {
            throw new XMLScanException("ZenoEmbodiment requires 1 inner MiddlewareOptions element that describes how it communicates with the Zeno robot controller");            
        }
        String middlewareLoader = null;
        Properties middlewareProperties = null;
        
        HashMap<String, String> attrMap = theTokenizer.getAttributes();
        XMLStructureAdapter adapter = new XMLStructureAdapter();
        middlewareLoader = adapter.getRequiredAttribute("loaderclass", attrMap, theTokenizer);

        theTokenizer.takeSTag("MiddlewareOptions");

        middlewareProperties = new Properties();
        while (theTokenizer.atSTag("MiddlewareProperty"))
        {
            HashMap<String, String> attrMap2 = theTokenizer.getAttributes();
            XMLStructureAdapter adapter2 = new XMLStructureAdapter();
            String name = adapter2.getRequiredAttribute("name", attrMap, theTokenizer);
            String value = adapter2.getRequiredAttribute("value", attrMap, theTokenizer);
            middlewareProperties.put(name, value);
            theTokenizer.takeSTag("MiddlewareProperty");
            theTokenizer.takeETag("MiddlewareProperty"); 
        }
        theTokenizer.takeETag("MiddlewareOptions");

        if (!theTokenizer.atETag("Loader")) throw new XMLScanException("ZenoEmbodiment can only have 1 MiddlewareOptions elements");
        
        GenericMiddlewareLoader gml = new GenericMiddlewareLoader(middlewareLoader, middlewareProperties);
        Middleware m = gml.load();

        zenoRobot = new ZRCToMiddleware(m);
        zenoRobot.addSpeechListener(this);
        return;
    }

    @Override
    public void unload()
    {
    	//TODO: unregister and stuff?
    	//con.disconnect();
    }

    @Override
    public Embodiment getEmbodiment()
    {
        return this;
    }

    public void speak(SpeakZU zu, String id, String text)
    {
    	speakidToZenounit.put(id, zu);
    	speak(id,text);
    }
	@Override
	public void speechStart(String id) {
		logger.debug("received speechStart for {}",id);
	}

	@Override
	public void speechEnd(String id) {
		logger.debug("received speechEnd for {}",id);
		SpeakZU zu = speakidToZenounit.get(id);
		if (zu==null)
		{
			logger.error("NULL ZU for ID {}",id);
		}
		else
		{
			zu.stopUnit();
		}
		speakidToZenounit.remove(id);
	}
    

}

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
package asap.eyepiengine.loader;
 
import hmi.environmentbase.CopyEmbodiment;
import hmi.environmentbase.CopyEnvironment;
import hmi.environmentbase.Embodiment;
import hmi.environmentbase.EmbodimentLoader;
import hmi.environmentbase.Environment;
import hmi.environmentbase.Loader;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import net.ser1.stomp.Client;
import net.ser1.stomp.Listener;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.eyepiengine.EyePiPlanner;

/**
*  This embodiment initiates the Stomp connection used to send messages to ROSridge running on the ROS side of the configuration. This bridge is part of the robot python packages developed at HMI 
*  The various control primitives are mapped to corresponding Stomp messages. The Engine depends on a ROS-Apollo bridge and on the packages for control of the EyePi like robot of Edwin and Jens
*  
* This embodiment is also a CopyEmbodiment: based on a ClockDrivenCopyEmbodiment, things such as the current saliency maps are streamed to RamSoc.
*
*  @author Dennis REidsma
*/
public class EyePiEmbodiment implements Embodiment, EmbodimentLoader, Listener, CopyEmbodiment
{
    private String id = "";
	private Client stompSendClient;
    private static Logger logger = LoggerFactory.getLogger(EyePiPlanner.class.getName());
    private XMLStructureAdapter adapter = new XMLStructureAdapter();
    private HashMap<String, String> attrMap = null;

    private String apolloHost = "127.0.0.1";
    private int    apolloPort = 61613;
    private String apolloUser = "admin";
    private String apolloPassword = "password";

    private String apolloBridgeId = "";

    private CopyEnvironment ce = null;
    
    
    //Command used to instruct the ROS-Apollo bridge to forward messages on a topic from one publisher to another
    private static final String BRIDGE_CONFIG = "<config>\r\n" + 
    		"    <relay publisher=\"%s\" name=\"%s\" />\r\n" + 
    		"</config>\r\n" + 
    		"";
    
    
    public void setId(String id)
    {
        this.id = id;
    }

    @Override
    public String getId()
    {
        return id;
    }

    /**
     * Connects to and registers some producers and consumers for the various topics made available by the ROS Bridge
     */
    @Override
    public void readXML(XMLTokenizer tokenizer, String loaderId, String vhId, String vhName, Environment[] environments, Loader ... requiredLoaders) 
    	throws IOException
    {
    	
    	setId(loaderId);

       	for (Environment e : environments)
        {
            if (e instanceof CopyEnvironment) ce = (CopyEnvironment) e;
        }
        if (ce == null)
        {
            throw new RuntimeException("EyePiEmbodiment requires an Environment of type CopyEnvironment");
        }

        attrMap = tokenizer.getAttributes();

        ArrayList<String> sourceNames = new ArrayList<String>();
        ArrayList<String> topicNames = new ArrayList<String>();
        
        while (!tokenizer.atETag("Loader"))
        {
        	//get map elements and apolloserver info and store them
        	if (tokenizer.atSTag("ApolloServer"))
        	{
                apolloHost = adapter.getRequiredAttribute("host", attrMap, tokenizer);
                apolloPort = adapter.getRequiredIntAttribute("port", attrMap, tokenizer);
        		apolloUser = adapter.getRequiredAttribute("user", attrMap, tokenizer);
        		apolloPassword  = adapter.getRequiredAttribute("password", attrMap, tokenizer);
                apolloBridgeId = adapter.getOptionalAttribute("bridgeId", attrMap, "");
                if (!apolloBridgeId.endsWith("_")) apolloBridgeId += "_"; //ROS Apollo bridge system prefixes topic names with "bridgeId_"
                tokenizer.takeSTag("ApolloServer");
                tokenizer.takeETag("ApolloServer");
        	}
        	if (tokenizer.atSTag("Map"))
        	{
                attrMap = tokenizer.getAttributes();
                String sourceName = adapter.getRequiredAttribute("source", attrMap, tokenizer);
                String topicName = adapter.getRequiredAttribute("topic", attrMap, tokenizer); 
                tokenizer.takeSTag("Map");
                tokenizer.takeETag("Map");
                sourceNames.add(sourceName);
                topicNames.add(topicName);
        	}
        }

        //connect to apollo with provided settings. Assumes the ROS-Apollo bridge is already running on the ROS side!
        try {
            
			stompSendClient = new Client(apolloHost, apolloPort, apolloUser, apolloPassword);
			
		} catch (Exception e) {
			logger.error("Error while initialising STOMP connection: "+e.getMessage() + ". Is Apollo already running? Also make sure that the bridge is already running on the ROS side.");
			e.printStackTrace();
		}
        
        //tell the apollo-ros bridge at the other end to relay certain messages from ros to appollo and vice versa, as defined in the Map elements
        //logger.error("currently the ros bridge will make duplicate subscriptions each time this config is called with same parameters, so the bridge must check for this or there will be an overwhelming increase in the amount of messages. Requires update of the bridge system");
        for (int i =0; i< sourceNames.size(); i++)
        {
        	String sourceName = sourceNames.get(i);
        	String topicName = topicNames.get(i);
            
        	// send route spec to Apollo-ROS bridge in order to setup relay
        	stompSendClient.send("/topic/"+apolloBridgeId+"bridge_config", String.format(BRIDGE_CONFIG,sourceName, topicName));  //don't include bridgename in topic name, you're already sending to the right bridge config topic!

        	// for ROS sourced messages, we also want to get notified when such a emssage arrives
            if (sourceName.equals("ros"))
            {
            	String channelForReceiving = apolloBridgeId+topicName.replace('/', '.'); //remember to replace slashes with a dot!!!
            	stompSendClient.subscribe("/topic/"+channelForReceiving, this); //cannot send and receive on same client? and: need to prefix "topic"thingy!!!
            	logger.debug("Receiving ROS messages on {}",channelForReceiving);
            }
        }
        
        ce.addCopyEmbodiment(this);
            
        return;
    }

    @Override
    public void unload()
    {
        ce.removeCopyEmbodiment(this);
    	stompSendClient.disconnect();
    }

    @Override
    public Embodiment getEmbodiment()
    {
        return this;
    }
	
	public void sendRosMessage(String message, String topic)
	{

    	stompSendClient.send("/topic/"+apolloBridgeId + topic.replace('/', '.'), message);  //remember to send to correct bridge (the one with the id...! 
    																						//also, remember that slashes should at this point be period instead of slash! (even though registration is with slash!!!) So the topic asap/jointStates is addressed as bridge_id+"_asap.jointStates"
	
	}


	//the feedback received here should be parsed and used to figure out and store things such as how long certain behaviors will run or whether a behavior completed.... Any planning error feedback (e.g. you sent an erroneous request to ROS) might also come in here! Of course, you may want to make more than one listener so you can e.e. separate a listener for errors from a listener for behavior completion, on different channels... that all depends on your agreement with the ros-programming-people who send stuff to you through ROS/Apollo channels
	public void message(Map headers, String body) {
		//TODO incomplete parsing, and we still need to process the message!
/*
 * COMPLETELY REWRITE BASED ON NEW MESSAGES
  		try

		{
			XMLTokenizer tok = new XMLTokenizer(body);
			tok.takeSTag("data");
			float duration = -1;
			String id = "";
	        attrMap = tok.getAttributes();
			while (!tok.atETag("data"))
			{
				if (tok.atSTag("duration"))
				{
					tok.takeSTag();
					String f = tok.takeCharData();
					if (!f.trim().equals(""))
					{
						duration = Float.parseFloat(f);
					}
					tok.takeETag("duration");
				}
				else if (tok.atSTag("identifier"))
				{
					tok.takeSTag();
					id = tok.takeCharData().trim();
					tok.takeETag("identifier");
				}
				else 
					tok.skipTag();
			}
			tok.takeETag("data");
		} catch (IOException ex)
		{
			logger.error("Cannot parse feedback from ROS. Body: {}", body);
		}
*/
	}
    
    /** probably called by a ClockDrivenCopyEnvironment.
    Will calculate the current saliency map based on gaze and gazeshift behaviors, then send this map to RamSoc
    */
    public void copy()
    {
        //send saliency map for gaze
    	if (hasGaze)
    	{
    		//send saliency map for gaze
    		logger.debug("Send gaze {},{}",gazeX,gazeY);
    		String msg = "<data><locationsX type=\"tuple\"><value type=\"float\">"+gazeX+"</value></locationsX><locationsY type=\"tuple\"><value type=\"float\">"+gazeY+"</value></locationsY><weights type=\"tuple\"><value type=\"int\">"+(int)gazeWeight+"</value></weights><deliberate type=\"int\">1</deliberate><inputId type=\"str\">1</inputId></data>";
    		stompSendClient.send("/topic/"+apolloBridgeId + gazeTopic.replace('/', '.'), msg);  //remember to send to correct bridge (the one with the id...! 
    	}
    	if (hasGazeShift)
    	{
            logger.debug("Always send gazeshift {},{}",gazeShiftX,gazeShiftY);
            String msg = "<data><locationsX type=\"tuple\"><value type=\"float\">"+gazeShiftX+"</value></locationsX><locationsY type=\"tuple\"><value type=\"float\">"+gazeShiftY+"</value></locationsY><weights type=\"tuple\"><value type=\"int\">"+(int)gazeShiftWeight+"</value></weights><deliberate type=\"int\">1</deliberate><inputId type=\"str\">2</inputId></data>";
            stompSendClient.send("/topic/"+apolloBridgeId + gazeShiftTopic.replace('/', '.'), msg);  //remember to send to correct bridge (the one with the id...! 
            //send saliency map for gazeshift with lower prio 
            //logger.debug("Send gazeshift {},{}",gazeShiftX,gazeShiftY);
        }
        //send current emotion values
    }
    private float arousal=0;
    private float valence=0;
    public void setEmotion(float arousal, float valence)
    {
        this.arousal = arousal;
        this.valence = valence;
    }
    private boolean hasGazeShift = false;
    private float gazeShiftX=0;
    private float gazeShiftY=0;
    private float gazeShiftWeight=1000; 
    private float gazeX=1; 
    private float gazeY=1;
    private float gazeWeight=10000; 
    private String gazeId="";
    private String gazeTopic="";
    private String gazeShiftId="";
    private String gazeShiftTopic="";
    private boolean hasGaze = false;
    /** id is the behavior id. the saliency map ID will be "2" */
    public void setGazeShiftCoords(float x, float y, float weight, String planUnitId, String topic)
    {
        gazeShiftX=x;
        gazeShiftY=y;
        gazeShiftWeight=weight;
        gazeShiftId = planUnitId;
        gazeShiftTopic = topic;
        hasGazeShift = true;
    }
    //wordt nooit aangeroepen, zodra je gaze shift gedaan hebt in BML blijft het namelijk staan tot je nieuwe gaze shift doet......
    public void clearGazeShiftCoords()
    {
        hasGazeShift = false;
    }
    /** id is the behavior id. the saliency map ID will be "1" (see copy function) */
    public void setGazeCoords(float x, float y, float weight, String planUnitId, String topic)
    {
        gazeX=x;
        gazeY=y;
        gazeWeight=weight;
        gazeId = planUnitId;
        gazeTopic = topic;
        hasGaze = true;
    }
    public void clearGazeCoords()
    {
        hasGaze = false;
    }
    /** call for a sequence to start immediately; send msg over specified apollotopic */
    public void callSequence(String planUnitID, int sequenceId, String topic)
    {
		logger.debug("Call sequence to start immediately, puID {}, seqID {}",planUnitID,sequenceId);
		String msg = "<data><sequence type=\"int\">"+sequenceId+"</sequence><id type=\"str\">"+planUnitID+"</id><requestType type=\"int\">1</requestType><requestCount type=\"int\">1</requestCount><request type=\"float\">0.1</request></data>";
		stompSendClient.send("/topic/"+apolloBridgeId + topic.replace('/', '.'), msg);  //remember to send to correct bridge (the one with the id...!     	
        logger.warn(msg);
    }
    /** call sequence to start delayed based on sync name and delay in milliseconds; send msg over specified apollotopic */
    public void callSequence(String planUnitID, int sequenceID, String topic, String syncId, int msDelay)
    {//TODO: this function does not yet work properly
		String msg = "<data><sequence type=\"int\">"+sequenceID+"</sequence><id type=\"str\">"+planUnitID+"</id></data>";
		stompSendClient.send("/topic/"+apolloBridgeId + topic.replace('/', '.'), msg);  //remember to send to correct bridge (the one with the id...!     	
        logger.debug(msg);
    }
    
}

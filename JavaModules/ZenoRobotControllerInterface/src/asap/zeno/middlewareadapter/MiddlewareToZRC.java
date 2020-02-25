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
package asap.zeno.middlewareadapter;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.MiddlewareListener;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static nl.utwente.hmi.middleware.helpers.JsonNodeBuilders.array;
import static nl.utwente.hmi.middleware.helpers.JsonNodeBuilders.object;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import asap.zeno.api.ZenoRobotController;
import asap.zeno.api.ZenoSpeechListener;

public class MiddlewareToZRC implements MiddlewareListener, ZenoSpeechListener {

    private Logger logger = LoggerFactory.getLogger(MiddlewareToZRC.class.getName());

	private Middleware middleware;
	private ZenoRobotController physicalRobot;


	private ObjectMapper om;

	public MiddlewareToZRC(Middleware m, ZenoRobotController zrc) {
		this.middleware = m;
		this.middleware.addListener(this);
		
		physicalRobot = zrc;
		physicalRobot.addSpeechListener(this);

		this.om = new ObjectMapper();
    }


	/**
	 * Here we are listening for actions
	 */
	@Override
	public void receiveData(JsonNode jn) {
		JsonNode request = jn.path("request");
		String action = request.path("action").asText();
		JsonNode params = request.path("params");
		if(action.equals("lookAt")){
			double x= params.path("x").asDouble();
			double y= params.path("y").asDouble();
			long duration = params.path("duration").asInt();
			physicalRobot.lookAt(x, y, duration);
		} else if(action.equals("moveJointsById")){
			Map<Integer, Double> positions = new HashMap<>();

			JsonNode ps = params.path("positions");
			Iterator<Entry<String,JsonNode>> it = ps.fields();
			while(it.hasNext())
			{
				Entry<String, JsonNode> e = it.next();
				JsonNode position = e.getValue();
				positions.put(position.path("id").asInt(), position.path("amount").asDouble());
			}
			long duration = params.path("duration").asInt();
			
			physicalRobot.moveJointsById(positions, duration);
		} else if(action.equals("moveJointsByName")){
			Map<String, Double> positions = new HashMap<>();

			JsonNode ps = params.path("positions");
			Iterator<Entry<String,JsonNode>> it = ps.fields();
			while(it.hasNext())
			{
				Entry<String, JsonNode> e = it.next();
				JsonNode position = e.getValue();
				positions.put(position.path("name").asText(), position.path("amount").asDouble());
			}
			long duration = params.path("duration").asInt();
			
			physicalRobot.moveJointsByName(positions, duration);
		} else if(action.equals("moveJointsByTDCName")){
			Map<String, Double> positions = new HashMap<>();

			JsonNode ps = params.path("positions");
			Iterator<Entry<String,JsonNode>> it = ps.fields();
			while(it.hasNext())
			{
				Entry<String, JsonNode> e = it.next();
				JsonNode position = e.getValue();
				positions.put(position.path("name").asText(), position.path("amount").asDouble());
			}
			long duration = params.path("duration").asInt();
			
			physicalRobot.moveJointsByTDCName(positions, duration);
		} else if(action.equals("speak")){
			physicalRobot.speak(params.path("id").asText(), params.path("text").asText());
		} else if(action.equals("getAnimationDurationByName")){
			double dur = physicalRobot.getAnimationDurationByName(params.path("name").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
		} else if(action.equals("playAnimationByName")){
			double dur = physicalRobot.playAnimationByName(params.path("name").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
		} else if(action.equals("playAnimationByFileName")){
			double dur = physicalRobot.playAnimationByFileName(params.path("fileName").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
		} else if(action.equals("playAnimationByContent")){
			double dur = physicalRobot.playAnimationByFileName(params.path("xmlContent").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
			
		} else if(action.equals("stopAnimation")) {
			physicalRobot.stopAnimation();
		}
	}

	/**
	 * Send the speech feedback back over the middleware
	 */
	@Override
	public void speechStart(String id) {
		JsonNode jn = object("feedback",
			       object()
					 .with("type","speechStart")
					 .with("speakId", id)
         ).end();

		logger.debug("sending feedback data: {}",jn.toString());
		middleware.sendData(jn);
	}


	@Override
	public void speechEnd(String id) {
		JsonNode jn = object("feedback",
			       object()
					 .with("type","speechEnd")
					 .with("speakId", id)
         ).end();

		logger.debug("sending feedback data: {}",jn.toString());
		middleware.sendData(jn);
	}
	

}

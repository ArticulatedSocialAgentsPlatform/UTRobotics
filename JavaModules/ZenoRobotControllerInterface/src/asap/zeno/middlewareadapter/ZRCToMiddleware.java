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

import static nl.utwente.hmi.middleware.helpers.JsonNodeBuilders.object;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import asap.zeno.api.ZenoRobotController;
import asap.zeno.api.ZenoSpeechListener;
import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.MiddlewareListener;

public class ZRCToMiddleware implements ZenoRobotController, MiddlewareListener {
	private static Logger logger = LoggerFactory.getLogger(ZRCToMiddleware.class.getName());

	private Middleware middleware;

	/** used to store the duration response for an animation request */
	private LinkedBlockingQueue<Double> animDurationQueue = new LinkedBlockingQueue<Double>();
	
    private final List<ZenoSpeechListener> zenoSpeechListeners = Collections.synchronizedList(new ArrayList<ZenoSpeechListener>());

	private ObjectMapper om;

	public ZRCToMiddleware(Middleware middleware) {
		this.middleware = middleware;
		middleware.addListener(this);

		this.om = new ObjectMapper();
	}

	@Override
	public void speak(String id, String text) {
		JsonNode jn = object("request",
							       object()
									 .with("action","speak")
									 .with("params",
						                   object().with("text",text).with("id", id)
						                  )
				            ).end();
		
		logger.debug("sending data: {}",jn.toString());
		middleware.sendData(jn);
	}

	@Override
	public double getAnimationDurationByName(String name) {
		JsonNode jn = object("request",
								object()
								    .with("action","getAnimationDurationByName")
				                    .with("params",
						                  object("name",name)
				                         )
				            ).end();
		
		logger.debug("sending data: {}",jn.toString());
		animDurationQueue.clear();
		middleware.sendData(jn);
		Double dur;
		try {
			dur = animDurationQueue.poll(1000, TimeUnit.MILLISECONDS);
		} catch (InterruptedException e) {
			logger.error("exception while waiting for animDuration.take() {}",e);
			return 0.0d;
		}
		if (dur==null)
		{
			logger.error("Did not get duration for animation from middleware");
			return 0.0d;
		}
		else
		{
			return dur.doubleValue();
		}
	}

	@Override
	public double playAnimationByName(String name) {
		JsonNode jn = object("request",
								object()
								    .with("action","playAnimationByName")
				                    .with("params",
						                  object("name",name)
				                         )
				            ).end();
		
		logger.debug("sending data: {}",jn.toString());
		animDurationQueue.clear();
		middleware.sendData(jn);
		Double dur;
		try {
			dur = animDurationQueue.poll(1000, TimeUnit.MILLISECONDS);
		} catch (InterruptedException e) {
			logger.error("exception while waiting for animDuration.take() {}",e);
			return 0.0d;
		}
		if (dur==null)
		{
			logger.error("Did not get duration for animation from middleware");
			return 0.0d;
		}
		else
		{
			return dur.doubleValue();
		}
	}
	
	@Override
	public double playAnimationByFileName(String fileName) {
		JsonNode jn = object("request",
								object ()
								     .with("action","playAnimationByFileName")
				                     .with("params",
						                   object("fileName",fileName)
				                          )
				            ).end();

		logger.debug("sending data: {}",jn.toString());
		animDurationQueue.clear();
		middleware.sendData(jn);
		Double dur;
		try {
			dur = animDurationQueue.poll(1000, TimeUnit.MILLISECONDS);
		} catch (InterruptedException e) {
			logger.error("exception while waiting for animDuration.take() {}",e);
			return 0.0d;
		}
		if (dur==null)
		{
			logger.error("Did not get duration for animation from middleware");
			return 0.0d;
		}
		else
		{
			return dur.doubleValue();
		}
	}

	@Override
	public double playAnimationByContent(String xmlContent) {
		JsonNode jn = object("request",
								object()
										.with("action","playAnimationByContent")
				                        .with("params",
						                        object("xmlContent",xmlContent)
						                     )
				            ).end();
			
			logger.debug("sending data: {}",jn.toString());
			animDurationQueue.clear();
			middleware.sendData(jn);
			Double dur;
			try {
				dur = animDurationQueue.poll(1000, TimeUnit.MILLISECONDS);
			} catch (InterruptedException e) {
				logger.error("exception while waiting for animDuration.take() {}",e);
				return 0.0d;
			}
			if (dur==null)
			{
				logger.error("Did not get duration for animation from middleware");
				return 0.0d;
			}
			else
			{
				return dur.doubleValue();
			}
	}
	
	@Override
	public void stopAnimation() {
		JsonNode jn = object("request",
				       object()
						 .with("action","stopAnimation")
					).end();

		logger.debug("sending data: {}",jn.toString());
		middleware.sendData(jn);
	}
	
	@Override
	public void moveJointsById(Map<Integer, Double> positions, long duration)
	{
		ObjectNode ps = om.createObjectNode();
		int i = 1;
		for (Entry<Integer,Double> e: positions.entrySet())
		{
			ObjectNode p = om.createObjectNode();
			p.put("id", e.getKey());
			p.put("amount", e.getValue());
			
			ps.set("e"+(i++), p);
		}
		
		ObjectNode params = om.createObjectNode();
		params.set("positions", ps);
		params.put("duration", (int)duration);
		
		ObjectNode on = om.createObjectNode();
		on.set("request", object().with("action", "moveJointsById")
				                  .with("params", params).end());
		
		logger.debug("sending data: {}",on.toString());
		middleware.sendData(on);
		
	}
	@Override
	public void moveJointsByName(Map<String, Double> positions, long duration)
	{

		ObjectNode ps = om.createObjectNode();
		int i = 1;
		for (Entry<String,Double> e: positions.entrySet())
		{
			ObjectNode p = om.createObjectNode();
			p.put("name", e.getKey());
			p.put("amount", e.getValue());
			
			ps.set("e"+(i++), p);
		}
		
		ObjectNode params = om.createObjectNode();
		params.set("positions", ps);
		params.put("duration", (int)duration);
		
		ObjectNode on = om.createObjectNode();
		on.set("request", object().with("action", "moveJointsByName")
				                  .with("params", params).end());
		
		
		logger.debug("sending data: {}",on.toString());
		middleware.sendData(on);
	}
	@Override
	public void moveJointsByTDCName(Map<String, Double> positions, long duration)
	{

		ObjectNode ps = om.createObjectNode();
		int i = 1;
		for (Entry<String,Double> e: positions.entrySet())
		{
			ObjectNode p = om.createObjectNode();
			p.put("name", e.getKey());
			p.put("amount", e.getValue());
			
			ps.set("e"+(i++), p);
		}
		
		ObjectNode params = om.createObjectNode();
		params.set("positions", ps);
		params.put("duration", (int)duration);
		
		ObjectNode on = om.createObjectNode();
		on.set("request", object().with("action", "moveJointsByTDCName")
				                  .with("params", params).end());
		
		
		logger.debug("sending data: {}",on.toString());
		middleware.sendData(on);
	}

	
	@Override
	public void lookAt(double x, double y) {
		lookAt(x,y,100);
	}
	

	@Override
	public void lookAt(double x, double y, long duration) {
		JsonNode jn = object("request",
			       object()
					 .with("action","lookAt")
					 .with("params",
		                   object().with("x",x).with("y", y).with("duration", duration)
		                  )
         ).end();

		logger.debug("sending data: {}",jn.toString());
		middleware.sendData(jn);
	}
	@Override
	public void receiveData(JsonNode jn) {
		logger.debug("receiving data: {}",jn.toString());
		if (jn.get("feedback").get("type").asText().equals("speechStart")){
			sendSpeechStart(jn.get("feedback").get("speakId").asText());
		} else if (jn.get("feedback").get("type").asText().equals("speechEnd")){
			sendSpeechEnd(jn.get("feedback").get("speakId").asText());
		} else if (jn.get("feedback").get("type").asText().equals("animDuration")) {
			try {
				animDurationQueue.put(jn.get("feedback").get("duration").asDouble());
			} catch (InterruptedException e) {
				logger.warn("exception waiting for animDurationQueue.put: {}",e);
			}
		}	
	}


	@Override
	public void addSpeechListener(ZenoSpeechListener l) {
		zenoSpeechListeners.add(l);
	}

	@Override
	public void removeSpeechListener(ZenoSpeechListener l) {
		zenoSpeechListeners.remove(l);
	}

	@Override
	public void removeAllSpeechListeners() {
		zenoSpeechListeners.clear();		
	}

	private void sendSpeechStart(String speakId) {
        synchronized (zenoSpeechListeners)
        {
            for (ZenoSpeechListener l : zenoSpeechListeners)
            {
                try
                {
                    l.speechStart(speakId);
                }
                catch (Exception ex)
                {
                    logger.warn("Exception in ZenoSpeechListener: {}, start feedback", ex);
                }
            }
        }		
	}

	private void sendSpeechEnd(String speakId) {
        synchronized (zenoSpeechListeners)
        {
            for (ZenoSpeechListener l : zenoSpeechListeners)
            {
                try
                {
                    l.speechEnd(speakId);
                }
                catch (Exception ex)
                {
                    logger.warn("Exception in ZenoSpeechListener: {}, start feedback", ex);
                }
            }
        }		
	}
}

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
		JsonNode request = jn.get("request");
		String action = request.get("action").asText();
		JsonNode params = request.get("params");
		if(action.equals("lookAt")){
			double x= params.get("x").asDouble();
			double y= params.get("y").asDouble();
			long duration = params.get("duration").asInt();
			physicalRobot.lookAt(x, y, duration);
		} else if(action.equals("moveJointsById")){
			Map<Integer, Double> positions = new HashMap<>();

			JsonNode ps = params.get("positions");
			Iterator<Entry<String,JsonNode>> it = ps.fields();
			while(it.hasNext())
			{
				Entry<String, JsonNode> e = it.next();
				JsonNode position = e.getValue();
				positions.put(position.get("id").asInt(), position.get("amount").asDouble());
			}
			long duration = params.get("duration").asInt();
			
			physicalRobot.moveJointsById(positions, duration);
		} else if(action.equals("moveJointsByName")){
			Map<String, Double> positions = new HashMap<>();

			JsonNode ps = params.get("positions");
			Iterator<Entry<String,JsonNode>> it = ps.fields();
			while(it.hasNext())
			{
				Entry<String, JsonNode> e = it.next();
				JsonNode position = e.getValue();
				positions.put(position.get("name").asText(), position.get("amount").asDouble());
			}
			long duration = params.get("duration").asInt();
			
			physicalRobot.moveJointsByName(positions, duration);
		} else if(action.equals("moveJointsByTDCName")){
			Map<String, Double> positions = new HashMap<>();

			JsonNode ps = params.get("positions");
			Iterator<Entry<String,JsonNode>> it = ps.fields();
			while(it.hasNext())
			{
				Entry<String, JsonNode> e = it.next();
				JsonNode position = e.getValue();
				positions.put(position.get("name").asText(), position.get("amount").asDouble());
			}
			long duration = params.get("duration").asInt();
			
			physicalRobot.moveJointsByTDCName(positions, duration);
		} else if(action.equals("speak")){
			physicalRobot.speak(params.get("id").asText(), params.get("text").asText());
		} else if(action.equals("getAnimationDurationByName")){
			double dur = physicalRobot.getAnimationDurationByName(params.get("name").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
		} else if(action.equals("playAnimationByName")){
			double dur = physicalRobot.playAnimationByName(params.get("name").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
		} else if(action.equals("playAnimationByFileName")){
			double dur = physicalRobot.playAnimationByFileName(params.get("fileName").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
		} else if(action.equals("playAnimationByContent")){
			double dur = physicalRobot.playAnimationByFileName(params.get("xmlContent").asText());
			JsonNode jndur = object("feedback",
				       object()
						 .with("type","animDuration")
						 .with("duration", new Double(dur))
	         ).end();
			middleware.sendData(jndur);
			
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

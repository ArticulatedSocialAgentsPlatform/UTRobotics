package nl.utwente.hmi.worker;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;

import nl.utwente.hmi.middleware.Middleware;
import hmi.flipper.defaultInformationstate.DefaultItem;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Record;
import nl.utwente.hmi.mwdialogue.informationstate.Observer;
import nl.utwente.hmi.mwdialogue.informationstate.helper.RecordHelper;

public class InformationStateToMiddlewareWorker implements Observer {
	private static Logger logger = LoggerFactory.getLogger(InformationStateToMiddlewareWorker.class.getName());

	private RecordHelper rh;
	private Middleware middleware;

	public InformationStateToMiddlewareWorker(Middleware middleware){
		this.middleware = middleware;

		this.rh = new RecordHelper();
	}
	
	@Override
	public void hasChanged(Record record) {
		if(record.getItems().size() > 0){
			logger.debug("IS has changed: {}",record.toString());
			
			JsonNode jn = rh.convertISToJSON(new DefaultItem(record));
			logger.debug("Transforming to JSON: {}", jn.toString());
	
			logger.debug("Sending to middleware ");
			middleware.sendData(jn);
		} else {
			logger.warn("The specified record is empty!");
		}
	}

}

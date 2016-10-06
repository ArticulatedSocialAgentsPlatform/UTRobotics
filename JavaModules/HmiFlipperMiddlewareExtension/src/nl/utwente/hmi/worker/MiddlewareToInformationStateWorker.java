package nl.utwente.hmi.worker;

import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;

import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Record;
import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.MiddlewareListener;
import nl.utwente.hmi.middleware.helpers.JSONHelper;
import nl.utwente.hmi.middleware.worker.AbstractWorker;
import nl.utwente.hmi.mwdialogue.ScenarioController;
import nl.utwente.hmi.mwdialogue.ScenarioConfigLoader;
import nl.utwente.hmi.mwdialogue.ScenarioController.SimpleInformationStateUpdateTask;
import nl.utwente.hmi.mwdialogue.informationstate.helper.RecordHelper;

/**
 * This worker class processes JSON data instances, and updates the information state accordingly.
 * It is possible to either select only specific keys from the JSON, or update the entire IS hierarchy based on the entire bottle.
 * @author davisond
 *
 */
public class MiddlewareToInformationStateWorker extends AbstractWorker implements MiddlewareListener {
	private static Logger logger = LoggerFactory.getLogger(MiddlewareToInformationStateWorker.class.getName());

	private ScenarioController sc;
	private String isName;
	private List<String> filterKeys;
	private RecordHelper rh;
	private JSONHelper jh;

	private boolean lossless;

	/**
	 * Starts a worker withou any filters. If the SC is null, there will be no actual updates to the scenario, but this will run in "dummy" mode.
	 * @param sc the scenario controller on which to publish IS updates
	 * @param isName the name of this IS worker
	 */
	public MiddlewareToInformationStateWorker(ScenarioController sc, String isName){
		this.sc = sc;
		this.isName = isName;
		this.lossless = true;
		this.rh = new RecordHelper();
		this.jh = new JSONHelper();
		this.filterKeys = new ArrayList<String>();
	}
	
	/**
	 * Constructs a new worker which monitors a stream of data for certain key/values.
	 * @param is the information state which is updated
	 * @param sc the scenario loader (controller)
	 * @param isName the name of this information state worker (this is the root of the path where the updates are stored in the information state)
	 * @param filterKeys a list of Data keys which to filter for.
	 * @param lossless flag to indicate whether the incoming data should be treated as a lossless or lossy data
	 */
	public MiddlewareToInformationStateWorker(ScenarioController sc, String isName, List<String> filterKeys, boolean lossless){
		this(sc, isName);
		
		this.lossless = lossless;
		
		if(filterKeys != null && filterKeys.size() > 0){
			this.filterKeys = filterKeys;
		}
	}
	
	/**
	 * Here we disassemble the data to search for the specific key/value relevant for updating this information state
	 */
	@Override
	public void processData(JsonNode jn) {
		logger.info("Got JSON data: {}",jn.toString());
	    Record rec = new DefaultRecord();
	    
	    if(jn != null && !jn.isMissingNode() && jn.size() > 0){
	    	
	    	//should we filter anything?	    	
	    	if(filterKeys.size()>0){
		    	for(String filter : filterKeys){
					JsonNode found = jh.searchKey(jn, filter);
					
					if(!found.isMissingNode()){
						//add the found key and value to the IS record
						rec.set(filter, rh.convertJSONToIS(found.get(filter)).getRecord());
					}
				}
	    	} else {
		    	//ok nothing specific to check for, so we want to just recursively add the whole data
	    		rec = rh.convertJSONToIS(jn).getRecord();
	    	}
	    	
	    	logger.debug("Adding to IS: \r\n{}",rec.toString());
	    	
	    	if(sc != null && rec.getItems().size() > 0){
	    		sc.updateInformationState(new ScenarioController.SimpleInformationStateUpdateTask(isName, rec, lossless));
	    	}
	    }
	}

	@Override
	public void receiveData(JsonNode jn) {
		addDataToQueue(jn);
	}

	public boolean isLossless() {
		return lossless;
	}
}

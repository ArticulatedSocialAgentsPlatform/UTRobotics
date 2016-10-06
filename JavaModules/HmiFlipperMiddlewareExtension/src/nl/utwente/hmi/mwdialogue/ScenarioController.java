package nl.utwente.hmi.mwdialogue;

import hmi.flipper.behaviourselection.TemplateController;
import hmi.flipper.defaultInformationstate.DefaultItem;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Item;
import hmi.flipper.informationstate.Record;
import hmi.util.ClockListener;
import hmi.util.SystemClock;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Properties;
import java.util.Set;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import nl.utwente.hmi.communication.Datasource;
import nl.utwente.hmi.communication.Datatarget;
import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.loader.GenericMiddlewareLoader;
import nl.utwente.hmi.mwdialogue.function.LoggerFunctions;
import nl.utwente.hmi.mwdialogue.function.PersistenceFunctions;
import nl.utwente.hmi.mwdialogue.function.TaskGenerationFunctions;
import nl.utwente.hmi.mwdialogue.informationstate.ObservableInformationState;
import nl.utwente.hmi.mwdialogue.informationstate.Observer;
import nl.utwente.hmi.mwdialogue.informationstate.helper.RecordHelper;
import nl.utwente.hmi.worker.InformationStateToMiddlewareWorker;
import nl.utwente.hmi.worker.MiddlewareToInformationStateWorker;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;

import saiba.bml.feedback.BMLBlockProgressFeedback;
import saiba.bml.feedback.BMLFeedback;
import saiba.bml.feedback.BMLFeedbackParser;
import saiba.bml.feedback.BMLPredictionFeedback;
import saiba.bml.feedback.BMLSyncPointProgressFeedback;
import saiba.bml.feedback.BMLWarningFeedback;
import asap.middlewareadapters.BMLRealizerToMiddlewareAdapter;
import asap.realizerport.BMLFeedbackListener;
import asap.realizerport.RealizerPort;

/**
 * This class has control over the various YARP listener threads and associated worker threads. It contains an internal reference to the template engine and the global information state store.
 * To prevent concurrency issues each worker thread may not manually modify the global IS. Instead, they can create an InformationStateUpdateTask and instruct the ScenarioController to process this task by calling the function updateInformationState()
 * @author davisond
 *
 */
public class ScenarioController implements Observer, Runnable, ClockListener, BMLFeedbackListener {

    @SuppressWarnings("unused")
    private static Logger logger = LoggerFactory.getLogger(ScenarioController.class.getName());

	private List<Datasource> datasources;
	private List<Datatarget> datatargets;
	private List<String> templateFileNames;

	private ObservableInformationState is;
	private TemplateController tc;

	private RecordHelper recordHelper;

	private String bmlFeedbackVar;
	private BlockingQueue<InformationStateUpdateTask> losslessInformationStateUpdateQueue;
	private ConcurrentMap<String, InformationStateUpdateTask> lossyInformationStateUpdateMap;
	
	private RealizerPort realizerPort = null;

	//this is where we're dumping IS updates
	private Middleware isDumper;

	private boolean lossyMapChanged;


	/** The number of times we will call Flipper.checkTemplates() after an information state update has taken place. **/
	private static final int NR_OF_TEMPLATE_CHECKS = 30;

	/** The number of milliseconds we want to sleep between starting the YARP listening and writhing threads **/
	private static final int STAGGERED_STARTUP_DELAY = 250;
	
	public ScenarioController(List<Datasource> datasources, List<Datatarget> datatargets, List<String> templateFileNames){
		this.datasources = datasources;
		this.datatargets = datatargets;
		this.templateFileNames = templateFileNames;
		this.recordHelper = new RecordHelper();
		this.losslessInformationStateUpdateQueue = new LinkedBlockingQueue<InformationStateUpdateTask>();
		this.lossyInformationStateUpdateMap = new ConcurrentHashMap<String, InformationStateUpdateTask>();
	}
	
	
	
	public void initScenario(){
		initFlipper();
		initMiddlewares();
		
		//start listening for IS update tasks
		new Thread(this).start();
		//..and start (clock time to IS)
		SystemClock clock = new SystemClock(1000 , "is_clock");  
		clock.start();
		clock.addClockListener(this);
	}
	private void initBMLCommands()
	{
		DefaultRecord r = new DefaultRecord();
		r.set("bmlcommand","$lt$speech id=\"$speechid$\" start=\"$speechstart$\"/$gt$$lt$text$gt$$speechcontent$$lt$/text$gt$$lt$/speech$gt$");
		updateInformationState(new ScenarioController.SimpleInformationStateUpdateTask("speechcommandmary", r));
		r = new DefaultRecord();
		r.set("bmlcommand","$lt$sze:speak id=\"$speechid$\" start=\"$speechstart$\" text=\"$speechcontent$\"/$gt$");
		updateInformationState(new ScenarioController.SimpleInformationStateUpdateTask("speechcommandzeno", r));

	}
	
	
	/**
	 * Loads the flipper templatecontroller and initiates the information state
	 */
	private void initFlipper() {
		//create the informationstate
		is = new ObservableInformationState();
		is.addObserver(this);
		
		//init the templatecontroller
		tc = new TemplateController();
		
		for(String templateFileName : templateFileNames){
			if(tc.processTemplateFile(templateFileName)){
				logger.info("Successfully processed dialogue template file [{}]...", templateFileName);
			} else {
				logger.error("Something went wrong when loading the dialogue file [{}]... Does it exist and is it formatted correctly?", templateFileName);
				System.exit(0);
			}
		}
		
		//add the collection of functions that we can use in Effects
		//TODO: DISCUSS we should probably load this automatically using reflection.. for instance everything under nl.utwente.hmi.mwdialogue.function
		tc.addFunction(new LoggerFunctions());
		tc.addFunction(new PersistenceFunctions(is));
		tc.addFunction(new TaskGenerationFunctions(is));
	}

	/**
	 * Here we init all the specific middlewares using the GenericMiddlewareLoader
	 */
	private void initMiddlewares(){
		//first, start the BML request and feedback, for this we use a RealizerPort
		String mwBMLLoaderClass = (String)Configuration.getInstance().getConfig("mw_bml_loaderclass");
		Properties mwBMLProperties = new Properties();
		
		//parse the properties for this middleware
		List<String> ps = new ArrayList<String>(Arrays.asList(((String)Configuration.getInstance().getConfig("mw_bml_properties")).split(",")));
		for(String p : ps){
			String[] prop = p.split(":");
			if(prop.length == 2){
				mwBMLProperties.put(prop[0], prop[1]);
			}
		}
		
		realizerPort = new BMLRealizerToMiddlewareAdapter(mwBMLLoaderClass, mwBMLProperties);
        realizerPort.addListeners(this);

		//then load the informationstate dump middleware (to which we stream IS updates)
		GenericMiddlewareLoader isDumperLoader = new GenericMiddlewareLoader((String)Configuration.getInstance().getConfig("mw_is_dump_loaderclass"),parseProperties((String)Configuration.getInstance().getConfig("mw_is_dump_properties")));
		isDumper = isDumperLoader.load();
		
		//now process the datasources
		for(Datasource ds : datasources){
			//load the actual middleware instance for this data source
			GenericMiddlewareLoader dsLoader = new GenericMiddlewareLoader(ds.getMiddlewareLoaderClass(), parseProperties(ds.getMiddlewareLoaderProperties()));
			Middleware dsMW = dsLoader.load();
			
			//init the worker which will process new Data from the middleware
			MiddlewareToInformationStateWorker disWorker = new MiddlewareToInformationStateWorker(this, ds.getName(), ds.getFilterKeys(), ds.getLossless());
			new Thread(disWorker).start();
			
			//finally, tell the middleware to send all data on to our worker thread
			dsMW.addListener(disWorker);
			try {
				//sleep a bit to give each middleware loader some time to get connected
				Thread.sleep(STAGGERED_STARTUP_DELAY);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		
		//and finally the datatargets
		for(Datatarget dt : datatargets){
			//load the actual middleware instance for this datatarget
			GenericMiddlewareLoader dtLoader = new GenericMiddlewareLoader(dt.getMiddlewareLoaderClass(), parseProperties(dt.getMiddlewareLoaderProperties()));
			Middleware dtMW = dtLoader.load();
			
			//create the worker required for transforming and sending data
			InformationStateToMiddlewareWorker ismWorker = new InformationStateToMiddlewareWorker(dtMW);

			//create new top-level information state record, which will be observed
			ObservableInformationState ois = new ObservableInformationState();
			ois.addObserver(ismWorker);
			
			//now add this observable information state to top level of global information state
    		this.updateInformationState(new ScenarioController.SimpleInformationStateUpdateTask(dt.getName(), ois));
			
			try {
				//sleep a bit to give each middleware some time to get connected
				Thread.sleep(STAGGERED_STARTUP_DELAY);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	/**
	 * Takes a string in the form "key1:val1,key2:val2" and transforms it into Properties with the specified key and vals..
	 * This can be used to parse properties for Middleware loaders
	 * @param properties a string in the form of "key1:val1,key2:val2"
	 * @return Properties
	 */
	private Properties parseProperties(String properties){
		
		List<String> ps = new ArrayList<String>(Arrays.asList(properties.split(",")));
		Properties returnProperties = new Properties();
		
		for(int i = 0; i < ps.size(); i++){
			String[] prop = ps.get(i).split(":");
			if(prop.length == 2){
				returnProperties.put(prop[0], prop[1]);
			}
		}
		
		return returnProperties;
	}
	

	@Override
	public void hasChanged(Record record) {
		
		logger.debug("\n!! IS update !! -----------------------\n{}",((DefaultRecord)record).toString());
		JsonNode jn = recordHelper.convertISToJSON(new DefaultItem(record));
		isDumper.sendData(jn);
		
	}

	/**
	 * Add an information state update task to the processing queue.
	 * @param t the task to add
	 */
	public void updateInformationState(InformationStateUpdateTask t){
		try {
			if(t.isLossless()){
				this.losslessInformationStateUpdateQueue.put(t);
			} else {
				//TODO: this might not be thread-safe when we check for lossyMapChanged before reading the map.values()
				//but I assume that lossy information is streamed frequently so it will be all good in the next iteration
				lossyMapChanged = true;
				this.lossyInformationStateUpdateMap.put(t.getID(),t);
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static interface InformationStateUpdateTask{
		/**
		 * Perform an update task on the given informationstate
		 * @param is the information state to be updated
		 */
		public void run(ObservableInformationState is);
		
		/**
		 * Should this update task be lossless or lossy
		 * @return true if the update task is lossless or false if it could be lossy
		 */
		public boolean isLossless();
		
		/**
		 * An id for this update task, used to distinguish similar tasks and filter duplicates when lossless==false
		 * @return an identifier for this task
		 */
		public String getID();
	}
	/**
	 * Simple class that represents anupdate task to the information state.
	 * This is used to ensure a thread-safe way to update the IS.
	 * A worker thread can pass an instance of this object to the ScenarioController.updateInformationState() which will then process the task.
	 * By default, an update to the information state is lossless, ensuring it is always executed. 
	 * However, if a source is streaming data it may set the lossless=false flag so that only the most recent update tasks is executed
	 * @author davisond
	 *
	 */
	public static class SimpleInformationStateUpdateTask implements InformationStateUpdateTask{
		private String name;
		private Record r;
		private boolean lossless;

		public String getName() {
			return name;
		}

		public void setName(String name) {
			this.name = name;
		}

		public Record getR() {
			return r;
		}

		public void setR(Record r) {
			this.r = r;
		}
		
		public void run(ObservableInformationState is)
		{
			logger.debug("Running IS update task: {}",getName());
			is.set(getName(), getR());
		}

		/**
		 * Create a new task using the specified name and record.
		 * The record will be added to the global IS using the specified name as root level object.
		 * @param name the root-level name of this record
		 * @param r the record to add
		 */
		public SimpleInformationStateUpdateTask(String name, Record r){
			this.name = name;
			this.r = r;
			this.lossless = true;
		}

		/**
		 * Create a new task using the specified name and record and indication whether it should be treated as lossless source or not
		 * The record will be added to the global IS using the specified name as root level object.
		 * @param name the root-level name of this record
		 * @param r the record to add
		 * @param lossless flag to indicate whether this update task should be lossless or not
		 */
		public SimpleInformationStateUpdateTask(String name, Record r, boolean lossless){
			this(name, r);
			this.lossless = lossless;
		}
		
		public String toString()
		{
			return name+","+r;
		}

		@Override
		public boolean isLossless() {
			return lossless;
		}

		@Override
		public String getID() {
			return this.getName();
		}
	}
	
	@Override
	public void run() {
		while(true){

			try {
				//IMPORTANT: all current items in the lossless queue should be processed in this run loop in the order in which they arrived
				//items marked as lossy (i.e. lossless==false) only run the most recent value
				
				//this list contains all lossless update tasks to be executed
				List<InformationStateUpdateTask> losslessTasks = new ArrayList<InformationStateUpdateTask>();
				
				//add polling to bring down the cpu load when nothing is coming in
				InformationStateUpdateTask firstTask = losslessInformationStateUpdateQueue.poll(10L, TimeUnit.MILLISECONDS);
				
				//is there a new lossless task?
				if(firstTask != null){
					losslessTasks.add(firstTask);
				}
				
				//take all current items in the lossless processing queue
				losslessInformationStateUpdateQueue.drainTo(losslessTasks);
				
				//iterate over each individual lossless update task
				int totalTasks = losslessTasks.size();
				for(int taskNr = 0; taskNr < totalTasks; taskNr++){
					InformationStateUpdateTask task = losslessTasks.get(taskNr);
					logger.info("Processing lossless update task {} of {}: {}", new Object[] { taskNr+1, totalTasks, task.getID()});
					
					task.run(is);
	
					//execute the checktemplates a few times in a row (hardcoded for now). 
					//This allows any dependant effect-chains to fully execute.
					//TODO: build a tc.checkPendingUpdates() to check whether there are any more templates waiting to be executed.
					for(int i = 0; i < NR_OF_TEMPLATE_CHECKS; i++){
						tc.checkTemplates(is);
					}
				}
				
				//now finish with the lossy tasks
				//map.values() returns an iterable collection, containing a snapshot of the map at this moment (I think)
				if(lossyMapChanged){
					for(InformationStateUpdateTask task : lossyInformationStateUpdateMap.values()){
						logger.info("Processing lossy update task: {}", task.getID());
						task.run(is);
						
						//execute the checktemplates a few times in a row (hardcoded for now). 
						//This allows any dependant effect-chains to fully execute.
						//TODO: build a tc.checkPendingUpdates() to check whether there are any more templates waiting to be executed.
						for(int i = 0; i < NR_OF_TEMPLATE_CHECKS; i++){
							tc.checkTemplates(is);
						}
					}
					//TODO: if we decide we need/want to add extra synchronisation here, we should also do a map.clear() to remove all old/processed values
					lossyMapChanged = false;
				}
			
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	@Override
	public void initTime(double initTime) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void time(double currentTime) {
		// TODO Auto-generated method stub
		DefaultRecord r = new DefaultRecord();
		r.set("value",new Double(currentTime));
		updateInformationState(new ScenarioController.SimpleInformationStateUpdateTask("TIME", r));
	}

	@Override
	public void feedback(String feedback) {
		// received BML feedback. Parse and store in is?
        try
        {
            BMLFeedback fb = BMLFeedbackParser.parseFeedback(feedback);
            if(fb instanceof BMLPredictionFeedback)
            {
                //nothing
            }
            else if (fb instanceof BMLBlockProgressFeedback)
            {
            
            	//store block end if requested in is
            	BMLBlockProgressFeedback bfb = (BMLBlockProgressFeedback)fb;
            	if (bfb.getSyncId()!=null && !bfb.getSyncId().equals(""))
            	{
	            	final String selectorpath = "$bmlfeedback.watches._select[watchid="+bfb.getBmlId()+"_"+bfb.getSyncId()+"]"; 
	            	final String addpath = "$bmlfeedback.watches._addlast"; 
	        		final DefaultRecord r = new DefaultRecord();
	        		r.set("status","RECEIVED");
	        		r.set("watchid",bfb.getBmlId()+"_"+bfb.getSyncId());
	            	updateInformationState(new ScenarioController.InformationStateUpdateTask()
	            	{
	            		public void run(ObservableInformationState is)
	            		{
	            			//check whether we are actually waiting for this update:
	            			Item item = is.getValueOfPath(selectorpath+".status");
	            			if (item==null) return;
	            			if (item.getType()!=Item.Type.String)return;
	            			if (item.getString().equals("WAITING"))
	            			{
	            				//if yes, set in is
	            				is.remove(selectorpath);
	            				is.set(addpath,r);
	            			}
	            		}

						@Override
						public boolean isLossless() {
							return true;
						}

						@Override
						public String getID() {
							return "BMLBlockProgressFeedback";
						}
	            	});
            	}
            }
            else if (fb instanceof BMLSyncPointProgressFeedback)
            {
            	
            	//store syncpointprogress if requested in is
            	BMLSyncPointProgressFeedback sfb = (BMLSyncPointProgressFeedback)fb;
            	if (sfb.getSyncId()!=null && !sfb.getSyncId().equals("") && sfb.getBehaviourId()!=null && !sfb.getBehaviourId().equals("") )
            	{
	            	final String selectorpath = "$bmlfeedback.watches._select[watchid="+sfb.getBMLId()+"_"+sfb.getBehaviourId()+"_"+sfb.getSyncId()+"]"; 
	            	final String addpath = "$bmlfeedback.watches._addlast"; 
	        		final DefaultRecord r = new DefaultRecord();
	        		r.set("status","RECEIVED");
	        		r.set("watchid",sfb.getBMLId()+"_"+sfb.getBehaviourId()+"_"+sfb.getSyncId());
	            	updateInformationState(new ScenarioController.InformationStateUpdateTask()
	            	{
	            		public void run(ObservableInformationState is)
	            		{
	            			//check whether we are actually waiting for this update:
	            			Item item = is.getValueOfPath(selectorpath+".status");
	            			if (item==null) return;
	            			if (item.getType()!=Item.Type.String)return;
	            			if (item.getString().equals("WAITING"))
	            			{
	            				//if yes, set in is
	            				is.remove(selectorpath);
	            				is.set(addpath,r);
	            			}
	            		}

						@Override
						public boolean isLossless() {
							return true;
						}

						@Override
						public String getID() {
							return "BMLSyncPointProgressFeedback";
						}
	            	});
            	}
                
            }
            else if (fb instanceof BMLWarningFeedback)
            {
                //nothing
            }
        }
        catch (IOException e)
        {
            logger.warn("Could not parse feedback {}", feedback);
        }
		
		
	}
	
}

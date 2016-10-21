package nl.utwente.hmi.mwbehaviour;

import static nl.utwente.hmi.middleware.helpers.JsonNodeBuilders.array;
import static nl.utwente.hmi.middleware.helpers.JsonNodeBuilders.object;
import hmi.flipper.behaviourselection.behaviours.BehaviourClass;
import hmi.flipper.behaviourselection.template.value.Value;
import hmi.flipper.defaultInformationstate.DefaultItem;
import hmi.flipper.defaultInformationstate.DefaultRecord;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URLEncoder;
import java.util.ArrayList;
import java.util.Properties;
import java.util.Random;

import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.middleware.loader.GenericMiddlewareLoader;
import nl.utwente.hmi.mwdialogue.Configuration;
import nl.utwente.hmi.mwdialogue.informationstate.helper.RecordHelper;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import asap.realizerport.RealizerPort;

public class BMLTemplateBehaviour implements BehaviourClass{
	private static Logger logger = LoggerFactory.getLogger(BMLTemplateBehaviour.class.getName());

	//this is the realizerport we should use for sending BML to ASAP
	private RealizerPort realizerPort;
	
	private String bml = "";
	private boolean behaviourPrepared = false;
	
	private Middleware middleware;
	private String middlewareLoaderClass = "";
    private Properties ps;
	private Random random;
	
	private RecordHelper recordHelper;

	public String TEMPLATE_DIR = "behaviours";
	public static final String TEMPLATE_EXTENSION = "xml";
	public static final String TEMPLATE_PLACEHOLDER_CHAR = "$";
	public static final int TEMPLATE_ITERATION_DEPTH = 10;
	
	public BMLTemplateBehaviour(){
		super();
		
		random = new Random();
		
		this.recordHelper = new RecordHelper();
	}
	
	public void setRealizerPort(RealizerPort realizerPort){
		this.realizerPort = realizerPort;
	}
	
	@Override
	public void execute(ArrayList<String> argNames, ArrayList<Value> argValues) {
		if(!behaviourPrepared){
			prepare(argNames, argValues);
		}
		
		if(behaviourPrepared){
			logger.debug("Sending the BML: \n{}", bml);

            try {
				realizerPort.performBML(bml);
				behaviourPrepared = false;
            } catch (Exception e)
            {
                e.printStackTrace();
                System.exit(0);
            }
		}
	}

	/**
	 * Generates a random id we can use for BML blocks in ASAP
	 * This is a workaround for the repeated-speech bug when ids are recycled
	 * @return a random generated id: "random_id_<long>"
	 */
	private String getRandomID(){
		long randomLong = random.nextLong();
		//we want a positive long, otherwise ASAP flips....
		while(randomLong <= 0){
			randomLong = random.nextLong();
		}
		
		return "random_id_"+randomLong;
	}
	
	@Override
	public void prepare(ArrayList<String> argNames, ArrayList<Value> argValues) {
		if(argNames.size() == argValues.size()){
			int filenameI = argNames.indexOf("templateFilename");
			if(filenameI < 0){
				logger.error("You must provide argument [templateFilename]");
				return;
			}
			
			//if no id given, we generate random id. Assumption: the id parameter contains the bml block id :)

  			int idI = argNames.indexOf("id");
 

			if(idI < 0){
				argNames.add("id");
				argValues.add(new Value(getRandomID()));
			} 
			
			try {
				String filename = TEMPLATE_DIR+"/"+argValues.get(filenameI)+"."+TEMPLATE_EXTENSION;
				//logger.debug(filename);
				String fileContents = readFile(filename);
				
				ArrayList<String> processedValues = processValues(argValues);
				fileContents = fillPlaceholders(fileContents, argNames, processedValues);
				fileContents = fileContents.replace(TEMPLATE_PLACEHOLDER_CHAR+"lt"+TEMPLATE_PLACEHOLDER_CHAR, "<");
				fileContents = fileContents.replace(TEMPLATE_PLACEHOLDER_CHAR+"gt"+TEMPLATE_PLACEHOLDER_CHAR, ">");
				if(fileContents.contains(TEMPLATE_PLACEHOLDER_CHAR)){
					logger.warn("Warning: not all placeholders in template [{}] have been filled!", filename);
				}
				
				this.bml = fileContents;
				this.behaviourPrepared = true;
			} catch (IOException e) {
				logger.error("Something went wrong while reading the file [{}]... Does the file exist? Specify only the filename, do not include path or file extension.", argValues.get(filenameI));
				e.printStackTrace();
			}
			
		}
	}

	private ArrayList<String> processValues(ArrayList<Value> values){
		ArrayList<String> processedValues = new ArrayList<String>();

		for(Value v : values){
			if(v == null){
				processedValues.add("null");
			} else if(v.getExportType() == Value.ExportType.JSON && v.getType() == Value.Type.Record){
				//TODO: DANIEL this still needs to be fixed in the AsapMiddlewareEngine
				processedValues.add(recordHelper.convertISToJSON(new DefaultItem(v.getRecordValue())).toString());
			} else {
				processedValues.add(v.toString());
			}
		}
		
		return processedValues;
	}
	
	/**
	 * Helper function for filling the placeholders in a certain content with specific values. (Using a bruteforce approach)
	 * This function continues iteratively replacing the placeholders untill there are no more changes in the content (or maximum depth of TEMPLATE_ITERATION_DEPTH is reached), making it possible to nest placeholders in a template.
	 * TODO: we could add a check here to see if there is a potential infinite recursion in the placeholder matching.. i.e. in the form of "$PLACE$ = text $PLACE$ text"
	 * @param content the template contents
	 * @param placeholders the placeholder names
	 * @param values the actual values to plug into the placeholder
	 * @return the contents with as many placeholders filled as possible
	 */
	private String fillPlaceholders(String content, ArrayList<String> placeholders, ArrayList<String> values){
		String oldContent = "";
		String newContent = content;
		
		int it = 0;
		while(it < TEMPLATE_ITERATION_DEPTH && newContent.contains(TEMPLATE_PLACEHOLDER_CHAR)){
			oldContent = newContent;
			
			//now replace the template placeholders with the arguments provided to this function
			for(int i = 0; i< placeholders.size(); i++){
				String placeholder = placeholders.get(i);
				String value = values.get(i);
				newContent = newContent.replace(TEMPLATE_PLACEHOLDER_CHAR+placeholder+TEMPLATE_PLACEHOLDER_CHAR, value);
			}
			
			//if there are no more changes we are done!
			if(oldContent.equals(newContent)){
				break;
			}
			
			it++;
		}
		
		return newContent;
	}
	
	/**
	 * Returns contents from file specified in filename
	 * @param filename the file to read
	 * @return the contents of the file
	 * @throws IOException if file is not found
	 */
	private String readFile(String filename) throws IOException {
        BufferedReader br = null;
        try
        {
             br = new BufferedReader(new InputStreamReader(this.getClass().getClassLoader().getResourceAsStream(filename)));
        } catch (Exception e)
        {
        	e.printStackTrace();
            throw new RuntimeException("Cannot read file " +filename);
        }
	    try {
	        StringBuilder sb = new StringBuilder();
	        String line = br.readLine();

	        while (line != null) {
	            sb.append(line);
	            sb.append("\n");
	            line = br.readLine();
	        }
	        return sb.toString();
	    } finally {
	        br.close();
	    }
	}
	
	
}

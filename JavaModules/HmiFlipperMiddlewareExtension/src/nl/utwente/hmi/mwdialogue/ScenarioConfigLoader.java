package nl.utwente.hmi.mwdialogue;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import nl.utwente.hmi.communication.Datasource;
import nl.utwente.hmi.communication.Datatarget;
import nl.utwente.hmi.worker.MiddlewareToInformationStateWorker;
import nl.utwente.hmi.middleware.MiddlewareListener;
import nl.utwente.hmi.middleware.worker.Worker;
import nl.utwente.hmi.middleware.Middleware;
import nl.utwente.hmi.mwdialogue.informationstate.ObservableInformationState;
import nl.utwente.hmi.mwdialogue.informationstate.Observer;
import hmi.flipper.behaviourselection.TemplateController;
import hmi.flipper.defaultInformationstate.DefaultRecord;
import hmi.flipper.informationstate.Record;

//TODO: refactor this class a bit, so all the config loading and init stuff happens here, but the actual scenario instance is separate
/**
 * Loads the specified scenario based on the various properties- and config-files...
 * @author davisond
 *
 */
 
public class ScenarioConfigLoader {
	private static Logger logger = LoggerFactory.getLogger(ScenarioConfigLoader.class.getName());

	private String propFile;
	private Properties prop;

	private ScenarioController sc;

	private List<Datasource> datasources;

	private List<Datatarget> datatargets;

	private String bmlFeedbackVar;
	
	private String bmlTemplateDir;

	private String mwBMLProperties;

	private String mwBMLLoaderClass;

	private String mwISDumpProperties;

	private String mwISDumpLoaderClass;

	private String datasourceFileName;

	private String datatargetFileName;

	private ArrayList<String> templateFileNames;
	
	public ScenarioConfigLoader(String propFile){
		this.propFile = propFile;
	}
	
	/**
	 * First loads all the properties and settings from the various config files
	 * Then creates and inits the ScenarioController
	 */
	public void loadScenario(){
		loadProperties();
		loadDatasources();
		loadDatatargets();
		sc = new ScenarioController(datasources, datatargets, templateFileNames);
		sc.initScenario();
	}
	
	/**
	 * Loads the default properties and any additional properties defined by the propFile
	 */
	private void loadProperties() {
		//first set the default values
		
		//flipper-specific properties
		Properties defaultProp = new Properties();
		defaultProp.put("template_file_name", "scenarios/test_scenario.xml");
		defaultProp.put("datasource_file_name", "datasources/default_datasource.xml");
		defaultProp.put("datatarget_file_name", "datatargets/default_datatarget.xml");
		
		//by default, we use STOMP middleware.. override this to use a different middleware
		
		//a stream of IS dumps
		defaultProp.put("mw_is_dump_loaderclass", "nl.utwente.hmi.middleware.stomp.STOMPMiddlewareLoader");
		defaultProp.put("mw_is_dump_properties", "apolloIP:127.0.0.1,apolloPort:61613,iTopic:/topic/dummy,oTopic:/topic/isDump");
		
		//where to write BML and receive BML feedback
		defaultProp.put("mw_bml_loaderclass", "nl.utwente.hmi.middleware.stomp.STOMPMiddlewareLoader");
		defaultProp.put("mw_bml_properties", "apolloIP:127.0.0.1,apolloPort:61613,iTopic:/topic/bmlFeedback,oTopic:/topic/bmlRequests");
		
		//where to store BML feedback in informationstate
		defaultProp.put("bml_is_feedback_var", "$bmlrealizer.feedback");
		defaultProp.put("bml_template_dir", "$behaviours");
		
		//now load the user-defined values (if any)
		prop = new Properties(defaultProp);
		InputStream input = null;
	 
		try {
	 
			input = getClass().getClassLoader().getResourceAsStream(propFile);
			if (input == null) {
				logger.error("Sorry, unable to find properties file: {}", propFile);
			} else {
				//load the actual properties
				logger.info("Loading properties: {}",propFile);
				prop.load(input);
			}

			templateFileNames = new ArrayList<String>(Arrays.asList(prop.getProperty("template_file_name").split(",")));
			datasourceFileName = prop.getProperty("datasource_file_name");
			datatargetFileName = prop.getProperty("datatarget_file_name");

			mwISDumpLoaderClass = prop.getProperty("mw_is_dump_loaderclass");
			mwISDumpProperties = prop.getProperty("mw_is_dump_properties");

			mwBMLLoaderClass = prop.getProperty("mw_bml_loaderclass");
			mwBMLProperties = prop.getProperty("mw_bml_properties");
			
			bmlFeedbackVar = prop.getProperty("bml_feedback_var");
			bmlTemplateDir = prop.getProperty("bml_template_dir");
			
			Configuration.getInstance().storeConfig("template_file_names", templateFileNames);
			Configuration.getInstance().storeConfig("datasource_file_name", datasourceFileName);
			Configuration.getInstance().storeConfig("datatarget_file_name", datatargetFileName);
			Configuration.getInstance().storeConfig("mw_is_dump_loaderclass", mwISDumpLoaderClass);
			Configuration.getInstance().storeConfig("mw_is_dump_properties", mwISDumpProperties);
			Configuration.getInstance().storeConfig("mw_bml_loaderclass", mwBMLLoaderClass);
			Configuration.getInstance().storeConfig("mw_bml_properties", mwBMLProperties);
			Configuration.getInstance().storeConfig("bml_feedback_var", bmlFeedbackVar);
			Configuration.getInstance().storeConfig("bml_template_dir", bmlTemplateDir);
		} catch (IOException ex) {
			ex.printStackTrace();
		} finally {
			if (input != null) {
				try {
					input.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
	}
	
	/**
	 * Loads the defined datasources from the config xml file
	 * The actual initialisation of each listener and worker thread is performed in initYARP()
	 */
	private void loadDatasources(){
		datasources = new ArrayList<Datasource>();
		if(datasourceFileName != null && !datasourceFileName.equals("") && getClass().getClassLoader().getResource(datasourceFileName) != null){
			//load the specified file from the resource folder
			File xmlFile = new File(getClass().getClassLoader().getResource(datasourceFileName).getFile());
			try {
				//construct the XML DOM parser
				DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
				DocumentBuilder dBuilder;
				dBuilder = dbFactory.newDocumentBuilder();
				Document doc = dBuilder.parse(xmlFile);
				
				logger.info("Parsing datasource config file [{}]....",datasourceFileName);
				
				//now iterate over the datasource config elements we need
				Element root = doc.getDocumentElement();
				NodeList sources = root.getElementsByTagName("datasource");
				
				if(sources.getLength() == 0){
					logger.warn("No sources have been defined :-(");
				}
				
				for(int d = 0; d < sources.getLength(); d++){
					if(sources.item(d).getNodeType() == Node.ELEMENT_NODE){
						Element source = (Element)sources.item(d);
						Node nameNode = source.getElementsByTagName("name").item(0);
						Node losslessNode = source.getElementsByTagName("lossless").item(0);
						Node middlewareLoaderClassNode = source.getElementsByTagName("middlewareLoaderClass").item(0);
						Node middlewareLoaderPropertiesNode = source.getElementsByTagName("middlewareLoaderProperties").item(0);
						
						ArrayList<String> filterKeys = new ArrayList<String>();

						NodeList filters = source.getElementsByTagName("filter");
						for(int f = 0; f < filters.getLength(); f++){
							filterKeys.add(filters.item(f).getTextContent().trim());
						}
						
						
						if(nameNode != null && middlewareLoaderClassNode != null && middlewareLoaderPropertiesNode != null){
							//sources are lossless by default, can only be overridden by setting <lossless>FALSE</lossless>
							boolean lossless = !(losslessNode != null && "FALSE".equals(losslessNode.getTextContent().trim()));

							logger.info("Parsing the datasource [{}] as lossless={}....",nameNode.getTextContent().trim(), lossless);
							logger.info("Found the following filters: {}", filterKeys);
							
							String name = nameNode.getTextContent().trim();
							String middlewareLoaderClass = middlewareLoaderClassNode.getTextContent().trim();
							String middlewareLoaderProperties = middlewareLoaderPropertiesNode.getTextContent().trim();

							//all is well, we can now create our datasource and add it to the list
							Datasource ds = new Datasource(name, middlewareLoaderClass, middlewareLoaderProperties, filterKeys, lossless);
							
							datasources.add(ds);
						} else {
							logger.error("Some error while parsing datasource nr [{}].. Perhaps your formatting is not correct, I expect at least the following elements for each datasource: name, middlewareLoaderClass, middlewareLoaderProperties", d);
						}
					}
				}
				
				Configuration.getInstance().storeConfig("datasources", datasources);
			} catch (ParserConfigurationException | IOException | SAXException e) {
				logger.error("Something went wrong when reading datasource file [{}].. Is it valid XML?", datasourceFileName);
				e.printStackTrace();
			}
		} else {
			logger.error("Datasource config file not defined or doesnt exist!");
		}
	}


	/**
	 * Loads the defined datatargets from the config xml file
	 */
	private void loadDatatargets(){
		datatargets = new ArrayList<Datatarget>();
		if(datatargetFileName != null && !datatargetFileName.equals("") && getClass().getClassLoader().getResource(datatargetFileName) != null){
			//load the specified file from the resource folder
			File xmlFile = new File(getClass().getClassLoader().getResource(datatargetFileName).getFile());
			try {
				//construct the XML DOM parser
				DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
				DocumentBuilder dBuilder;
				dBuilder = dbFactory.newDocumentBuilder();
				Document doc = dBuilder.parse(xmlFile);
				
				logger.info("Parsing datatarget config file [{}]....", datatargetFileName);
				
				//now iterate over the datatarget config elements we need
				Element root = doc.getDocumentElement();
				NodeList targets = root.getElementsByTagName("datatarget");
				
				if(targets.getLength() == 0){
					logger.warn("No targets have been defined :-(");
				}
				
				for(int d = 0; d < targets.getLength(); d++){
					if(targets.item(d).getNodeType() == Node.ELEMENT_NODE){
						Element target = (Element)targets.item(d);
						Node nameNode = target.getElementsByTagName("name").item(0);
						Node middlewareLoaderClassNode = target.getElementsByTagName("middlewareLoaderClass").item(0);
						Node middlewareLoaderPropertiesNode = target.getElementsByTagName("middlewareLoaderProperties").item(0);
						
						if(nameNode != null && middlewareLoaderClassNode != null && middlewareLoaderPropertiesNode != null){
							logger.info("Parsing the datatarget [{}]....", nameNode.getTextContent().trim());
							
							String name = nameNode.getTextContent().trim();
							String middlewareLoaderClass = middlewareLoaderClassNode.getTextContent().trim();
							String middlewareLoaderProperties = middlewareLoaderPropertiesNode.getTextContent().trim();
							
							//all is well, we can now create our datatarget and add it to the list
							Datatarget dt = new Datatarget(name, middlewareLoaderClass, middlewareLoaderProperties);
							datatargets.add(dt);
						} else {
							logger.error("Some error while parsing datatarget nr [{}].. Perhaps your formatting is not correct, I expect at least the following elements for each datatarget: name, middlewareLoaderClass, middlewareLoaderProperties", d);
						}
					}
				}
				
				Configuration.getInstance().storeConfig("datatargets", datatargets);
			} catch (ParserConfigurationException | IOException | SAXException e) {
				logger.error("Something went wrong when reading datatarget file [{}].. Is it valid XML?", datatargetFileName);
				e.printStackTrace();
			}
		} else {
			logger.error("Datatarget config file not defined or doesnt exist!");
		}
	}
	

	public static void main(String[] args){
		String propFile = "config.properties";
		if(args.length == 1){
			propFile = args[0];
		}
		
		ScenarioConfigLoader loader = new ScenarioConfigLoader(propFile);
		loader.loadScenario();
	}

}

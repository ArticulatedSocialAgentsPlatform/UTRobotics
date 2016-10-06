package nl.utwente.hmi.mwdialogue;

import java.util.concurrent.ConcurrentHashMap;

/**
 * This is a thread-safe Singleton Configuration class, used to store the dialogues configuration information
 * This singleton uses the initialization on demand pattern: https://en.wikipedia.org/wiki/Initialization-on-demand_holder_idiom 
 * It is backed by a ConcurrentHashMap for storing the actual config information
 * @author davisond
 *
 */
public class Configuration {

	private ConcurrentHashMap<String, Object> configs = new ConcurrentHashMap<String, Object>();
	
	private static class Holder {
        static final Configuration INSTANCE = new Configuration();
    }

	/**
	 * Get the singleton instance of the config file
	 * @return
	 */
    public static Configuration getInstance() {
        return Holder.INSTANCE;
    }
    
    /**
     * Store a certain config value
     * @param name the name of the config
     * @param config the config value
     */
    public void storeConfig(String name, Object config){
    	configs.put(name, config);
    }
    
    /**
     * Retrieve a configuration value
     * @param name the value to retrieve
     * @return the config value, or null if not found
     */
    public Object getConfig(String name){
    	return configs.get(name);
    }
	
}

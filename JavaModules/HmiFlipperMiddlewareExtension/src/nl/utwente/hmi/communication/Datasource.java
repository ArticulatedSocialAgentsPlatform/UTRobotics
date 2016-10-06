package nl.utwente.hmi.communication;

import java.util.ArrayList;
import java.util.List;

public class Datasource {

	private String name;
	private String middlewareLoaderClass;
	private String middlewareLoaderProperties;
	private List<String> filterKeys;
	private boolean lossless;

	public Datasource(String name, String middlewareLoaderClass, String middlewareLoaderProperties, List<String> filterKeys, boolean lossless){
		this.setName(name);
		this.setMiddlewareLoaderClass(middlewareLoaderClass);
		this.setMiddlewareLoaderProperties(middlewareLoaderProperties);
		this.setFilterKeys(filterKeys);
		this.setLossless(lossless);
	}

	public void setLossless(boolean lossless) {
		this.lossless=lossless;
	}
	
	public boolean getLossless(){
		return lossless;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public String getMiddlewareLoaderClass() {
		return middlewareLoaderClass;
	}

	public void setMiddlewareLoaderClass(String middlewareLoaderClass) {
		if(middlewareLoaderClass == null){
			middlewareLoaderClass = "";
		}
		this.middlewareLoaderClass = middlewareLoaderClass;
	}

	public String getMiddlewareLoaderProperties() {
		return middlewareLoaderProperties;
	}

	public void setMiddlewareLoaderProperties(String middlewareLoaderProperties) {
		this.middlewareLoaderProperties = middlewareLoaderProperties;
	}

	public List<String> getFilterKeys() {
		return filterKeys;
	}

	public void setFilterKeys(List<String> filterKeys) {
		this.filterKeys = filterKeys;
	}
	
}

package nl.utwente.hmi.communication;

import java.util.ArrayList;
import java.util.List;

public class Datatarget {

	private String name;
	private String middlewareLoaderClass;
	private String middlewareLoaderProperties;

	public Datatarget(String name, String middlewareLoaderClass, String middlewareLoaderProperties){
		this.setName(name);
		this.setMiddlewareLoaderClass(middlewareLoaderClass);
		this.setMiddlewareLoaderProperties(middlewareLoaderProperties);
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

	public void setMiddlewareLoaderClass(String internalPort) {
		this.middlewareLoaderClass = internalPort;
	}
	
	public String getMiddlewareLoaderProperties() {
		return middlewareLoaderProperties;
	}

	public void setMiddlewareLoaderProperties(String externalPort) {
		if(externalPort == null){
			externalPort = "";
		}
		this.middlewareLoaderProperties = externalPort;
	}
	
}

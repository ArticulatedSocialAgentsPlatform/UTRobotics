package nl.utwente.hmi.mwdialogue.informationstate;

import java.util.ArrayList;
import java.util.List;

import hmi.flipper.defaultInformationstate.DefaultRecord;

public class ObservableInformationState extends DefaultRecord {

	private List<Observer> observers;

	public ObservableInformationState(){
		super();
		
		this.observers = new ArrayList<Observer>();
	}
	
	public void addObserver(Observer observer){
		this.observers.add(observer);
	}
	
	private void notifyObservers(){
		for(Observer ob : observers){
			ob.hasChanged(this);
		}
	}
	
	@Override
	public void set( String path, Object value ){
		super.set(path, value);
		notifyObservers();
	}
	
	@Override
    public void remove( String path ){
		super.remove(path);
		notifyObservers();
	}
	
}

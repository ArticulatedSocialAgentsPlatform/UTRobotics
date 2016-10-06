package nl.utwente.hmi.mwbehaviour;

import hmi.flipper.behaviourselection.behaviours.BehaviourClass;
import hmi.flipper.behaviourselection.template.value.Value;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;


public class EmptyBehaviour implements BehaviourClass{

	

	public EmptyBehaviour(){
		super();
	}

	
	@Override
	public void execute(ArrayList<String> argNames, ArrayList<Value> argValues) {

	}

	@Override
	public void prepare(ArrayList<String> argNames, ArrayList<Value> argValues) {
		
	}

	
}

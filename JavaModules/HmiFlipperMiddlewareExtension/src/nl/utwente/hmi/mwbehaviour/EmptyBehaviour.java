/*******************************************************************************
 * Copyright (C) 2009-2020 Human Media Interaction, University of Twente, the Netherlands
 *
 * This file is part of the Articulated Social Agents Platform BML realizer (ASAPRealizer).
 *
 * ASAPRealizer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ASAPRealizer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ASAPRealizer.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
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

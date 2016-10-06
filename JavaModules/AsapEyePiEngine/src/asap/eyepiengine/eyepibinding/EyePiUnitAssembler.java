/*******************************************************************************
 * Copyright (C) 2009 Human Media Interaction, University of Twente, the Netherlands
 * 
 * This file is part of the Elckerlyc BML realizer.
 * 
 * Elckerlyc is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Elckerlyc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Elckerlyc.  If not, see http://www.gnu.org/licenses/.
 ******************************************************************************/
package asap.eyepiengine.eyepibinding;

import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

import java.util.HashMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.eyepiengine.planunit.CallSequenceEPU;
import asap.eyepiengine.planunit.DirectRosMessageEPU;
import asap.eyepiengine.planunit.GazeShiftEPU;
import asap.eyepiengine.planunit.LookAtEPU;
import asap.eyepiengine.planunit.ParametrizedRosMessageEPU;
import asap.eyepiengine.planunit.EyePiUnit;

public class EyePiUnitAssembler extends XMLStructureAdapter
{
    private static Logger logger = LoggerFactory
            .getLogger(EyePiUnitAssembler.class.getName());

    private EyePiUnit eyepiUnit;

    public EyePiUnitAssembler()
    {
    }

    @Override
    public void decodeAttributes(HashMap<String, String> attrMap, XMLTokenizer tokenizer)
    {
        String type = getRequiredAttribute("type", attrMap, tokenizer);
        //TODO extend with all units that you want to build using the binding
        if (type.equals("DirectRosMessageEPU"))
        {
        	logger.debug("Make EyePi planunit in binding: {}", type);
            eyepiUnit = new DirectRosMessageEPU();
        } 
        else if (type.equals("ParametrizedRosMessageEPU"))
        {
        	logger.debug("Make EyePi planunit in binding: {}", type);
            eyepiUnit = new ParametrizedRosMessageEPU();
        } 
        else if (type.equals("CallSequenceEPU"))
        {
        	logger.debug("Make EyePi planunit in binding: {}", type);
            eyepiUnit = new CallSequenceEPU();
        } 
        else if (type.equals("LookAtEPU"))
        {
        	logger.debug("Make EyePi planunit in binding: {}", type);
            eyepiUnit = new LookAtEPU();
        } 
        else if (type.equals("GazeShiftEPU"))
        {
        	logger.debug("Make EyePi planunit in binding: {}", type);
            eyepiUnit = new GazeShiftEPU();
        } 
        else
        {
          logger.warn("Cannot read EyePiUnit type \"{}\" in EyePiBinding; omitting this EyePiUnit", type);
        }
        
    }

    /**
     * @return the planunit
     */
    public EyePiUnit getEyePiUnit()
    {
        return eyepiUnit;
    }

   /*
    * The XML Stag for XML encoding
    */
   private static final String XMLTAG = "EyePiUnit";
 
   /**
    * The XML Stag for XML encoding -- use this static method when you want to see if a given String equals
    * the xml tag for this class
    */
   public static String xmlTag() { return XMLTAG; }
 
   /**
    * The XML Stag for XML encoding -- use this method to find out the run-time xml tag of an object
    */
   @Override
   public String getXMLTag() {
      return XMLTAG;
   }
}

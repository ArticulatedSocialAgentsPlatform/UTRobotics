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

import hmi.environmentbase.Embodiment;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import saiba.bml.BMLInfo;
import saiba.bml.core.Behaviour;
import asap.binding.SpecParameterDefault;
import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.planunit.ParameterException;
import asap.eyepiengine.loader.EyePiEmbodiment;
import asap.eyepiengine.planunit.EyePiUnit;
import asap.eyepiengine.planunit.TimedEyePiUnit;

public class EyePiBinding extends XMLStructureAdapter
{
    private ArrayList<EyePiUnitSpec> specs = new ArrayList<EyePiUnitSpec>();
    private Logger logger = LoggerFactory.getLogger(EyePiBinding.class.getName());
    private EyePiEmbodiment eyepiEmbodiment = null;

    public EyePiBinding(Embodiment embodiment)
    {
    	if (embodiment instanceof EyePiEmbodiment)
        {
            this.eyepiEmbodiment = (EyePiEmbodiment) embodiment;
        } else 
        {
            throw new RuntimeException("EyePiBinding requires an embodiment of type EyePiEmbodiment");	
        }
    }

    public List<TimedEyePiUnit> getEyePiUnit(FeedbackManager fbManager, BMLBlockPeg bbPeg, Behaviour b)
    {
        ArrayList<TimedEyePiUnit> twus = new ArrayList<TimedEyePiUnit>();
        for (EyePiUnitSpec s : specs)
        {
            if (s.getType().equals(b.getXMLTag())
                    && hasEqualNameSpace(b,s.getSpecnamespace()) )
            {
                if (!s.satisfiesConstraints(b))
                {
                     //System.out.println("Constraint mismatch: "+b.getNamespace()+","+s.getSpecnamespace()+","+b.getXMLTag()+","+s.getType());
                }
                else
                {
                    //System.out.println("Found type and constraint match");
                    EyePiUnit wuCopy = s.eyepiUnit.copy(eyepiEmbodiment);
                    TimedEyePiUnit twu = wuCopy.createTEPU(fbManager, bbPeg, b.getBmlId(), b.id);
                    twus.add(twu);
                    
                    // System.out.println("set def params");
                    // set default parameter values
                    for (SpecParameterDefault spd : s.getParameterDefaults())
                    {
                        try
                        {
                            wuCopy.setParameterValue(spd.name, spd.value);
                        }
                        catch (ParameterException e)
                        {
                            logger.warn("Error in setting default value in getEyePiUnit, parameter " + spd.name, e);
                        }
                        logger.debug("Setting parameter {} to default {}", spd.name, spd.value);
                    }

                    // System.out.println("Map params");
                    // map parameters
                    for (String param : s.getParameters())
                    {
                        if (b.specifiesParameter(param))
                        {
                            String value = b.getStringParameterValue(param);
                            try
                            {
                                wuCopy.setParameterValue(s.getParameter(param), value);
                            }
                            catch (ParameterException e)
                            {
                                logger.warn("Error in parameter mapping in getEyePiUnit, parameter " + param, e);
                            }
                            logger.debug("Setting parameter {} mapped to  {}", param, s.getParameter(param));
                        }
                    }
                    
                    wuCopy.prepareUnit();
                }
            }
        }
        return twus;

    }

    @Override
    public void decodeContent(XMLTokenizer tokenizer) throws IOException
    {
        while (tokenizer.atSTag())
        {
            String tag = tokenizer.getTagName();
            if (tag.equals(EyePiUnitSpec.xmlTag()))
            {
                EyePiUnitSpec wuSpec = new EyePiUnitSpec();
                wuSpec.readXML(tokenizer);
                if (wuSpec.eyepiUnit != null) specs.add(wuSpec); // don't add failed planunits to the binding
                else logger.warn("Dropped EyePi planunit spec because we could not construct the EyePiUnit");
                // println(null) causes error in Android
                // System.out.println(puSpec.getSpecnamespace());
            }
            else 
            {
            	throw new RuntimeException("EyePiBinding only allows EyePiUnitSpec entries!");	
            }
        }
    }

    /*
     * The XML Stag for XML encoding
     */
    private static final String XMLTAG = "eyepibinding";

    /**
     * The XML Stag for XML encoding -- use this static method when you want to see if a given String equals
     * the xml tag for this class
     */
    public static String xmlTag()
    {
        return XMLTAG;
    }

    /**
     * The XML Stag for XML encoding -- use this method to find out the run-time xml tag of an object
     */
    @Override
    public String getXMLTag()
    {
        return XMLTAG;
    }
    private boolean hasEqualNameSpace(Behaviour b, String ns)
    {
        if(b.getNamespace() == null && ns == null) return true;
        if(ns==null && b.getNamespace().equals(BMLInfo.BMLNAMESPACE))return true;
        if(ns==null)return false;
        if(ns.equals(b.getNamespace()))return true;
        return false;
    }
 
}

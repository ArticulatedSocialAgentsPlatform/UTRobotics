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
package asap.zeno.zenobinding;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.binding.SpecParameterDefault;
import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.planunit.ParameterException;
import asap.zeno.embodiment.ZenoEmbodiment;
import asap.zeno.planunit.TimedZenoUnit;
import asap.zeno.planunit.ZenoUnit;
import hmi.environmentbase.Embodiment;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;
import saiba.bml.BMLInfo;
import saiba.bml.core.Behaviour;

public class ZenoBinding extends XMLStructureAdapter
{
    private ArrayList<ZenoUnitSpec> specs = new ArrayList<ZenoUnitSpec>();
    private Logger logger = LoggerFactory.getLogger(ZenoBinding.class.getName());
    private ZenoEmbodiment zenoEmbodiment = null;

    public ZenoBinding(Embodiment embodiment)
    {
    	if (embodiment instanceof ZenoEmbodiment)
        {
            this.zenoEmbodiment = (ZenoEmbodiment) embodiment;
        } else 
        {
            throw new RuntimeException("ZenoBinding requires an embodiment of type ZenoEmbodiment");	
        }
    }

    public List<TimedZenoUnit> getZenoUnit(FeedbackManager fbManager, BMLBlockPeg bbPeg, Behaviour b)
    {
        ArrayList<TimedZenoUnit> tzus = new ArrayList<TimedZenoUnit>();
        for (ZenoUnitSpec s : specs)
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
                    ZenoUnit zuCopy = s.zenoUnit.copy(zenoEmbodiment);
                    TimedZenoUnit tzu = zuCopy.createTZU(fbManager, bbPeg, b.getBmlId(), b.id);
                    tzus.add(tzu);

                    // System.out.println("set def params");
                    // set default parameter values
                    for (SpecParameterDefault zupc : s.getParameterDefaults())
                    {
                        try
                        {
                            zuCopy.setParameterValue(zupc.name, zupc.value);
                        }
                        catch (ParameterException e)
                        {
                            logger.warn("Error in setting default value in getZenoUnit, parameter " + zupc.name, e);
                        }
                        logger.debug("Setting parameter {} to default {}", zupc.name, zupc.value);
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
                                zuCopy.setParameterValue(s.getParameter(param), value);
                            }
                            catch (ParameterException e)
                            {
                                logger.warn("Error in parameter mapping in getZenoUnit, parameter " + param, e);
                            }
                            logger.debug("Setting parameter {} mapped to  {}", param, s.getParameter(param));
                        }
                    }

                }
            }
        }
        return tzus;

    }

    @Override
    public void decodeContent(XMLTokenizer tokenizer) throws IOException
    {
        while (tokenizer.atSTag())
        {
            String tag = tokenizer.getTagName();
            if (tag.equals(ZenoUnitSpec.xmlTag()))
            {
                ZenoUnitSpec nuSpec = new ZenoUnitSpec();
                nuSpec.readXML(tokenizer);
                if (nuSpec.zenoUnit != null) specs.add(nuSpec); // don't add failed nao units to the binding
                else logger.warn("Dropped zeno unit spec because we could not construct the zeno unit");
                // println(null) causes error in Android
                // System.out.println(puSpec.getSpecnamespace());
            }
        }
    }

    /*
     * The XML Stag for XML encoding
     */
    private static final String XMLTAG = "zenobinding";

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

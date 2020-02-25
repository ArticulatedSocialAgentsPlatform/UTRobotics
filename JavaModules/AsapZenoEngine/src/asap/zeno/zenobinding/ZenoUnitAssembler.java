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
package asap.zeno.zenobinding;

import java.util.HashMap;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import asap.zeno.planunit.AnimationZU;
import asap.zeno.planunit.DummySpeakZU;
import asap.zeno.planunit.LookAtZU;
import asap.zeno.planunit.PoseZU;
import asap.zeno.planunit.SpeakZU;
import asap.zeno.planunit.StopAnimationZU;
import asap.zeno.planunit.ZenoUnit;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

public class ZenoUnitAssembler extends XMLStructureAdapter
{
    private static Logger logger = LoggerFactory
            .getLogger(ZenoUnitAssembler.class.getName());

    private ZenoUnit zenoUnit;

    public ZenoUnitAssembler()
    {
    }

    @Override
    public void decodeAttributes(HashMap<String, String> attrMap, XMLTokenizer tokenizer)
    {
        String type = getRequiredAttribute("type", attrMap, tokenizer);
        if (type.equals("SpeakZU"))
        {
            zenoUnit = new SpeakZU();
        }
        else if (type.equals("DummySpeakZU"))
        {
            zenoUnit = new DummySpeakZU();
        }
        else if (type.equals("LookAtZU"))
        {
            zenoUnit = new LookAtZU();
        }
        else if (type.equals("AnimationZU"))
        {
            zenoUnit = new AnimationZU();
        }
        else if (type.equals("StopAnimationZU"))
        {
            zenoUnit = new StopAnimationZU();
        }
        else if (type.equals("PoseZU"))
        {
            zenoUnit = new PoseZU();
        }
        else
        {
          logger.warn("Cannot read ZenoUnit type \"{}\" in ZenoBinding; omitting this ZenoUnit", type);
        }
        
    }

    /**
     * @return the naoUnit
     */
    public ZenoUnit getZenoUnit()
    {
        return zenoUnit;
    }

   /*
    * The XML Stag for XML encoding
    */
   private static final String XMLTAG = "ZenoUnit";
 
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

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
package asap.eyepiengine.bml;

import hmi.xml.XMLFormatting;
import hmi.xml.XMLTokenizer;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import saiba.bml.core.OffsetDirection;
import saiba.bml.parser.SyncPoint;

import com.google.common.collect.ImmutableList;

/**
 * This class represents gaze on EyePi; adapted from the BML default gaze behavior
 * @author dennisr
 */
public class EyePiGazeBehaviour extends EyePiBehaviour
{
    @Override
    public void addDefaultSyncPoints()
    {
        for(String s:getDefaultSyncPoints())
        {
            addSyncPoint(new SyncPoint(bmlId, id, s));
        }        
    }
    private float x;
    private float y;

    private float offsetAngle; // In degrees.

    // private enum OffsetDirection { RIGHT, LEFT, UP, DOWN, UPRIGHT, UPLEFT, DOWNLEFT, DOWNRIGHT,
    // POLAR }
    private OffsetDirection offsetDirection = OffsetDirection.NONE;

    private String influence = ""; // Empty string is unknown, TODO: Model this as a set.

    private static final List<String> DEFAULT_SYNCS = ImmutableList.of("start","ready","relax","end");
    
    public static List<String> getDefaultSyncPoints()
    {
        return DEFAULT_SYNCS;
    }

    @Override
    public boolean satisfiesConstraint(String name, String value)
    {
        if (name.equals("influence")) return influence.equals(value);
        if (name.equals("offsetDirection")) return offsetDirection.equals(value);
        return super.satisfiesConstraint(name, value);
    }

    @Override
    public float getFloatParameterValue(String name)
    {
        if (name.equals("x")) return x;
        if (name.equals("y")) return y;
        return super.getFloatParameterValue(name);
    }

    @Override
    public String getStringParameterValue(String name)
    {
        if (name.equals("offsetAngle")) return "" + offsetAngle;
        else if (name.equals("offsetDirection")) return offsetDirection.toString();
        else if (name.equals("influence")) return influence;
        return super.getStringParameterValue(name);
    }

    @Override
    public boolean specifiesParameter(String name)
    {
        if (name.equals("x")) return true;
        else if (name.equals("y")) return true;
        else if (name.equals("offsetAngle")) return true;
        else if (name.equals("offsetDirection")) return true;
        else if (name.equals("influence")) return true;
        return super.specifiesParameter(name);
    }

    public EyePiGazeBehaviour(String bmlId,XMLTokenizer tokenizer) throws IOException
    {
        super(bmlId);
        readXML(tokenizer);
    }

    @Override
    public StringBuilder appendAttributeString(StringBuilder buf, XMLFormatting fmt)
    {
        appendAttribute(buf, "x", x);
        appendAttribute(buf, "y", y);
        appendAttribute(buf, "offsetAngle", offsetAngle);
        appendAttribute(buf, "offsetDirection", offsetDirection.toString());
        appendAttribute(buf, "influence", influence);        
        return super.appendAttributeString(buf, fmt);
    }

    @Override
    public void decodeAttributes(HashMap<String, String> attrMap, XMLTokenizer tokenizer)
    {
        x = getOptionalFloatAttribute("x", attrMap,0.0f);
        y = getOptionalFloatAttribute("y", attrMap,0.0f);
        offsetAngle = getOptionalFloatAttribute("offsetAngle", attrMap, 0.0f);
        offsetDirection = OffsetDirection.valueOf(getOptionalAttribute("offsetDirection", attrMap,
                OffsetDirection.NONE.toString()));
        influence = getOptionalAttribute("influence", attrMap, "");
        super.decodeAttributes(attrMap, tokenizer);
    }

    /*
     * The XML Stag for XML encoding
     */
    private static final String XMLTAG = "eyePiGaze";

    /**
     * The XML Stag for XML encoding -- use this static method when you want to see if a given
     * String equals the xml tag for this class
     */
    public static String xmlTag()
    {
        return XMLTAG;
    }

    /**
     * The XML Stag for XML encoding -- use this method to find out the run-time xml tag of an
     * object
     */
    @Override
    public String getXMLTag()
    {
        return XMLTAG;
    }

}

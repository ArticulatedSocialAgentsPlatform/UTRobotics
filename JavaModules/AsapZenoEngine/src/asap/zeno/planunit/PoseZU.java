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
package asap.zeno.planunit;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.primitives.Floats;

import asap.realizer.feedback.FeedbackManager;
import asap.realizer.pegboard.BMLBlockPeg;
import asap.realizer.pegboard.TimePeg;
import asap.realizer.planunit.InvalidParameterException;
import asap.realizer.planunit.KeyPosition;
import asap.realizer.planunit.KeyPositionManager;
import asap.realizer.planunit.KeyPositionManagerImpl;
import asap.realizer.planunit.ParameterException;
import asap.zeno.ZenoPlanner;
import asap.zeno.embodiment.ZenoEmbodiment;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

//TODO: DENNIS proper interpolation timing for this planunit (instead of just "in 100 msec")
/**
 * set a new pose (body and/or face). Timing: just acquire the pose "as soon as possible"
 *
 * @author Dennis Reidsma
 *
 */
public class PoseZU extends ZenoUnit
{
    private final KeyPositionManager keyPositionManager = new KeyPositionManagerImpl();

    private XMLStructureAdapter adapter = new XMLStructureAdapter();
    private static Logger logger = LoggerFactory.getLogger(PoseZU.class.getName());

    
    // the unique id of this ZU as specified in the BML
    private String zuId;

	private ZenoEmbodiment sze;
	
	private String pose = "";
	private float intensity = 1;

	Map<String, Double> minPoseMap= new HashMap<>();
	Map<String, Double> maxPoseMap= new HashMap<>();
	Map<String, Double> targetPoseMap= new HashMap<>();
	

    public PoseZU()
    {
        KeyPosition start = new KeyPosition("start", 0d, 1d);
        KeyPosition end = new KeyPosition("end", 1d, 1d);
        addKeyPosition(start);
        addKeyPosition(end);
    }

    public void setEmbodiment(ZenoEmbodiment sze)
    {
    	this.sze = sze;
    }
    
    @Override
    public void setFloatParameterValue(String name, float value) throws ParameterException
    {
    	if (name.equals("intensity"))
        {
            intensity = value;
            if ((intensity < 0) || (intensity >1)) throw new InvalidParameterException(name,""+value);
            for (Entry <String,Double> minPose: minPoseMap.entrySet())
            {
            	Double maxPoseValue = maxPoseMap.get(minPose.getKey());
            	targetPoseMap.put(minPose.getKey(), minPose.getValue().doubleValue()+intensity*(maxPoseValue.doubleValue()-minPose.getValue().doubleValue()));
            }
        }
        else
        {
        	throw new InvalidParameterException(name, ""+value);
        }
    	
    }

    @Override
    public void setParameterValue(String name, String value) throws ParameterException
    {
    	if (name.equals("pose"))
        {
    		minPoseMap= new HashMap<>();
    		maxPoseMap= new HashMap<>();
    		targetPoseMap= new HashMap<>();
            pose = value;
            try 
            {
            	//TODO: make this configurable from the loader
	            XMLTokenizer tok = XMLTokenizer.forResource("ZENO/poses", pose+".xml");
	            tok.takeSTag("Pose");
	            while (!tok.atETag("Pose"))
	            {
	            	if (!tok.atSTag("Value"))
	            		throw new RuntimeException("Error reading pose file: expected Value tag");
	                HashMap<String, String> attrMap = null;
                    attrMap = tok.getAttributes();
	                String jointname = adapter.getRequiredAttribute("name", attrMap, tok);
	                float increment = adapter.getRequiredFloatAttribute("increment", attrMap, tok);
	                float baseoffset = adapter.getRequiredFloatAttribute("baseoffset", attrMap, tok);
                    tok.takeEmptyElement("Value");
                    minPoseMap.put(jointname,new Double(baseoffset+0.5));
                    maxPoseMap.put(jointname,new Double(baseoffset+0.5+increment));
					//here, also take intensity into account, in case it was set before the pose config was loaded
                    targetPoseMap.put(jointname,new Double(baseoffset+0.5+intensity*increment));
	            }
	            tok.takeETag("Pose");
            }
            catch (Exception e)
            {
            	throw new InvalidParameterException(name,value);
            }
        }
        else
        {
            Float f = Floats.tryParse(value);
            if (f!=null)
            {
                setFloatParameterValue(name, f);
            }
            else
            {
                throw new InvalidParameterException(name, value);
            }
        }
    }

    @Override
    public String getParameterValue(String name) throws ParameterException
    {
        if (name.equals("pose"))
        {
            return pose;
        }
        return "" + getFloatParameterValue(name);
    }

    @Override
    public float getFloatParameterValue(String name) throws ParameterException
    {
    	if (name.equals("intensity"))
        {
            return intensity;
        }
   		return 0;
    }

    @Override
    public boolean hasValidParameters()
    {
    	 return (intensity >= 0) && (intensity <=1);
    }

    /** start the unit. Duration */
    public void startUnit(double time) throws ZUPlayException
    {
        if (expectedDuration == TimePeg.VALUE_UNKNOWN)
        {
            sze.moveJointsByName(targetPoseMap, 100);
            //logger.debug("setting pose {} for default duration 100",targetPoseMap);
        }
        else
        {
            sze.moveJointsByName(targetPoseMap,(int)(expectedDuration*1000)); 
            //logger.debug("setting pose {} for duration {}",targetPoseMap,(int)(expectedDuration*1000));
        }
    }

    /**
     * 
     * @param t
     *            execution time, 0 &lt t &lt 1
     * @throws ZUPlayException
     *             if the play fails for some reason
     */
    public void play(double t) throws ZUPlayException
    {
    }

	public void cleanup()
    {
    }

    /**
     * Creates the TimedZenoUnit
     * 
     * @param bmlId
     *            BML block id
     * @param id
     *            behaviour id
     * 
     * @return the TPU
     */
    @Override
    public TimedZenoUnit createTZU(FeedbackManager bfm, BMLBlockPeg bbPeg, String bmlId, String id)
    {
        this.zuId = id;
        return new TimedZenoUnit(bfm, bbPeg, bmlId, id, this);
    }


    /**
     * Create a copy of this zeno unit and link it to the display
     */
    @Override
    public ZenoUnit copy(ZenoEmbodiment zenoEmbodiment)
    {
        PoseZU result = new PoseZU();
        result.setEmbodiment(zenoEmbodiment);
       
        for (KeyPosition keypos : getKeyPositions())
        {
            result.addKeyPosition(keypos.deepCopy());
        }
        result.minPoseMap= new HashMap<String,Double>(minPoseMap); 
        result.maxPoseMap= new HashMap<String,Double>(maxPoseMap); 
        result.targetPoseMap= new HashMap<String,Double>(targetPoseMap); 
        return result;
    }

    @Override
    public void addKeyPosition(KeyPosition kp)
    {
        keyPositionManager.addKeyPosition(kp);
    }

    @Override
    public KeyPosition getKeyPosition(String name)
    {
        return keyPositionManager.getKeyPosition(name);
    }

    @Override
    public List<KeyPosition> getKeyPositions()
    {
        return keyPositionManager.getKeyPositions();
    }

    @Override
    public void setKeyPositions(List<KeyPosition> p)
    {
        keyPositionManager.setKeyPositions(p);
    }

    @Override
    public void removeKeyPosition(String id)
    {
        keyPositionManager.removeKeyPosition(id);
    }


    /**
     * @return Preferred duration (in seconds) of this zeno unit, 0 means not determined/infinite
     */
    @Override
    public double getPreferredDuration()
    {
		return 0.1;
	}

    public double getRigidity()
    {
    	// can stretch if you want
    	return 0.0d;
    }
}

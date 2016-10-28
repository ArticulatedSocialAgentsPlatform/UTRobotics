/*******************************************************************************
 * 
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
package asap.zeno.loader;

import hmi.environmentbase.EmbodimentLoader;
import hmi.environmentbase.Environment;
import hmi.environmentbase.Loader;
import hmi.util.ArrayUtils;
import hmi.util.Resources;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

import java.io.IOException;
import java.util.HashMap;

import asap.realizer.DefaultEngine;
import asap.realizer.DefaultPlayer;
import asap.realizer.Engine;
import asap.realizer.Player;
import asap.realizer.planunit.PlanManager;
import asap.realizer.planunit.PlanPlayer;
import asap.realizer.planunit.SingleThreadedPlanPlayer;
import asap.realizerembodiments.AsapRealizerEmbodiment;
import asap.realizerembodiments.EngineLoader;
import asap.zeno.ZenoPlanner;
import asap.zeno.embodiment.ZenoEmbodiment;
import asap.zeno.planunit.TimedZenoUnit;
import asap.zeno.zenobinding.ZenoBinding;

/**

*/
public class ZenoEngineLoader implements EngineLoader
{

    private XMLStructureAdapter adapter = new XMLStructureAdapter();
    private ZenoEmbodiment ze = null;

    private Engine engine = null;
    private PlanManager<TimedZenoUnit> planManager = null;
    private Player player = null;
    private String id = "";
    // some variables cached during loading
    private ZenoBinding zenoBinding = null;
    private AsapRealizerEmbodiment are = null;

    @Override
    public void readXML(XMLTokenizer tokenizer, String loaderId, String vhId, String vhName, Environment[] environments,
            Loader... requiredLoaders) throws IOException
    {
        id = loaderId;
        for (EmbodimentLoader e : ArrayUtils.getClassesOfType(requiredLoaders, EmbodimentLoader.class))
        {
            if (e.getEmbodiment() instanceof ZenoEmbodiment)
            {
                ze = (ZenoEmbodiment) e.getEmbodiment();
            }
            if (e.getEmbodiment() instanceof AsapRealizerEmbodiment)
            {
                are = (AsapRealizerEmbodiment) e.getEmbodiment();
            }
        }
        if (ze == null)
        {
            throw new RuntimeException("ZenoEngineLoader requires an EmbodimentLoader containing a ZenoEmbodiment");
        }
        if (are == null)
        {
            throw new RuntimeException("ZenoEngineLoader requires an EmbodimentLoader containing a ZenoEmbodiment");
        }
        while (!tokenizer.atETag("Loader"))
        {
            readSection(tokenizer);
        }
        constructEngine(tokenizer);
    }

    @Override
    public void unload()
    {
        // engine.shutdown(); already done by realizer
    }

    protected void readSection(XMLTokenizer tokenizer) throws IOException
    {
        HashMap<String, String> attrMap = null;
        if (tokenizer.atSTag("ZenoBinding"))
        {
            attrMap = tokenizer.getAttributes();
            zenoBinding = new ZenoBinding(ze.getEmbodiment());
            try
            {
                zenoBinding.readXML(new Resources(adapter.getOptionalAttribute("resources", attrMap, "")).getReader(adapter
                        .getRequiredAttribute("filename", attrMap, tokenizer)));
            }
            catch (Exception e)
            {
                e.printStackTrace();
                throw new RuntimeException("Cannnot load ZenoBinding: " + e);
            }
            tokenizer.takeEmptyElement("ZenoBinding");
        }
        else
        {
            throw tokenizer.getXMLScanException("Unknown tag in Loader content");
        }
    }

    private void constructEngine(XMLTokenizer tokenizer)
    {
        if (zenoBinding == null) throw tokenizer.getXMLScanException("zenobinding is null, cannot build zenoplanner ");
        planManager = new PlanManager<TimedZenoUnit>();
        PlanPlayer planPlayer = new SingleThreadedPlanPlayer<TimedZenoUnit>(are.getFeedbackManager(), planManager);
        player = new DefaultPlayer(planPlayer);
        ZenoPlanner planner = new ZenoPlanner(are.getFeedbackManager(), zenoBinding, planManager);
        engine = new DefaultEngine<TimedZenoUnit>(planner, player, planManager);
        engine.setId(id);

        // add engine to realizer;
        are.addEngine(engine);

    }

    /** Return the Engine that was constructed from the XML specification */
    public Engine getEngine()
    {
        return engine;
    }

    public PlanManager<TimedZenoUnit> getPlanManager()
    {
        return planManager;
    }

    public ZenoEmbodiment getZenoEmbodiment()
    {
        return ze;
    }
    
    public String getId()
    {
        return id;
    }

    public void setId(String newId)
    {
        id = newId;
    }
}

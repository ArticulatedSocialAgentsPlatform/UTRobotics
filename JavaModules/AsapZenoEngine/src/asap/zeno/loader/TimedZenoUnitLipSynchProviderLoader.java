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
package asap.zeno.loader;

import java.io.IOException;
import java.util.HashMap;

import asap.realizer.lipsync.LipSynchProvider;
import asap.realizerembodiments.AsapRealizerEmbodiment;
import asap.realizerembodiments.LipSynchProviderLoader;
import asap.zeno.lipsync.TimedZenoUnitLipSynchProvider;
import asap.zeno.viseme.VisemeToZenoPoseMapping;
import asap.zeno.viseme.ZenoVisemeBinding;
import hmi.environmentbase.Environment;
import hmi.environmentbase.Loader;
import hmi.util.ArrayUtils;
import hmi.util.Resources;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

//TODO: document format of this loader's XML
/**
 * Loads a TimedZenoUnitLipSynchProvider
 * @author Dennis Reidsma
 */
public class TimedZenoUnitLipSynchProviderLoader implements LipSynchProviderLoader
{
    private String id;
    private LipSynchProvider lipSyncProvider;

    public void setId(String newId)
    {
        id = newId;
    }

    @Override
    public String getId()
    {
        return id;
    }

    @Override
    public void readXML(XMLTokenizer tokenizer, String loaderId, String vhId, String vhName, Environment[] environments,
            Loader... requiredLoaders) throws IOException
    {
        setId(loaderId);

        ZenoEngineLoader zel = ArrayUtils.getFirstClassOfType(requiredLoaders, ZenoEngineLoader.class);
        if (zel == null)
        {
            throw tokenizer.getXMLScanException("TimedZenoUnitLipSynchProviderLoader requires ZenoEngineLoader.");
        }

        AsapRealizerEmbodiment are = ArrayUtils.getFirstClassOfType(requiredLoaders, AsapRealizerEmbodiment.class);
        if (are == null)
        {
            throw new RuntimeException(
                    "TimedZenoUnitLipSynchProviderLoader requires an EmbodimentLoader containing a AsapRealizerEmbodiment");
        }
        if (!tokenizer.atSTag("ZenoVisemeBinding"))
        {
            throw new RuntimeException(
                    "TimedZenoUnitLipSynchProviderLoader requires a child element of type ZenoVisemeBinding with attribute filename (and optionally attribute resources)");
        }
        ZenoVisemeBinding visBinding;
        VisemeToZenoPoseMapping mapping = new VisemeToZenoPoseMapping();
        XMLStructureAdapter adapter = new XMLStructureAdapter();
        HashMap<String, String> attrMap = tokenizer.getAttributes();
        mapping.readXML(new Resources(adapter.getOptionalAttribute("resources", attrMap, "")).getReader(adapter.getRequiredAttribute("filename", attrMap, tokenizer)));
        visBinding = new ZenoVisemeBinding(mapping);
        tokenizer.takeEmptyElement("ZenoVisemeBinding");


        lipSyncProvider = new TimedZenoUnitLipSynchProvider(visBinding, zel.getZenoEmbodiment(), zel.getPlanManager(), are.getPegBoard());
    }       

    @Override
    public void unload()
    {

    }

    @Override
    public LipSynchProvider getLipSyncProvider()
    {
        return lipSyncProvider;
    }
}

/*******************************************************************************
 *******************************************************************************/
package asap.zeno.viseme;

import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import com.google.common.collect.ImmutableSet;

import hmi.xml.XMLScanException;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

//TODO: DENNIS this solution actually loads the pose from file every time a viseme is requested :( caching is better (less loading time), but that requires changes to PoseZU as well
/**
 * given a viseme number, return the appropriate zeno pose name. 
 * 
 * The mapping is read from a resource file. Note: meaning of viseme number dependent on chose viseme set, e.g., Disney 13, or IKP. See corresponding faceengine classes for the details of the visemes
 *
 * @author Dennis Reidsma
 */
public class VisemeToZenoPoseMapping extends XMLStructureAdapter
{
    /** key: tring representation of the viseme number. Value: the pose name (see PoseZU for how this is processed) */
    private Map<String,String> mapping = new HashMap<String,String>();

    /**
     * Get the set of poses used in the mapping (what do we use this for?)
     */
    public Set<String> getUsedPoses()
    {
        Set<String> poses = new HashSet<String>();
        for(Entry<String,String> entry:mapping.entrySet())
        {
            poses.add(entry.getValue());
        }
        return ImmutableSet.copyOf(poses);
    }
    
    /**
     * Get the pose name for viseme vis. Returns null if not found.
     */
    public String getPoseForViseme(int vis)
    {
      return mapping.get(String.valueOf(vis));
    }

    /**
     * Get the pose name for viseme vis. Returns null if not found.
     */
    public String getPoseForViseme(String vis)
    {
      return mapping.get(vis);
    }
    
    @Override
    public void decodeContent(XMLTokenizer tokenizer) throws IOException
    {
        while (tokenizer.atSTag())
        {
            String tag = tokenizer.getTagName();
            if (!tag.equals("Mapping")) throw new XMLScanException("Unknown element in VisemeToZenoPoseMapping: "+tag);
            HashMap<String, String> attrMap = tokenizer.getAttributes();
            String viseme = getRequiredAttribute("viseme", attrMap, tokenizer);
            String pose = getRequiredAttribute("pose", attrMap, tokenizer);
            mapping.put(viseme,pose);
            tokenizer.takeSTag("Mapping");
            tokenizer.takeETag("Mapping");
        }
    }

    /*
     * The XML Stag for XML encoding
     */
    private static final String XMLTAG = "VisemeToZenoPoseMapping";
 
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
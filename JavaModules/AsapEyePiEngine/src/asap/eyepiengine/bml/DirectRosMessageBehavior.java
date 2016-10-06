/*******************************************************************************
 *******************************************************************************/
package asap.eyepiengine.bml;

import hmi.xml.XMLFormatting;
import hmi.xml.XMLStructureAdapter;
import hmi.xml.XMLTokenizer;

import java.io.IOException;
import java.util.HashMap;

/**
 * Sends the contents of this behavior XML directly to ROS via the RosEmbodiment
 */
public class DirectRosMessageBehavior extends EyePiBehaviour
{
    private String rosMsgContent;
    private String topic;

    @Override
    public boolean satisfiesConstraint(String name, String value)
    {
        return super.satisfiesConstraint(name, value);
    }

    public DirectRosMessageBehavior(String bmlId, XMLTokenizer tokenizer) throws IOException
    {
        super(bmlId);
        readXML(tokenizer);
    }

    @Override
    public StringBuilder appendAttributeString(StringBuilder buf, XMLFormatting fmt)
    {
        appendAttribute(buf, "topic", topic);
        return super.appendAttributeString(buf, fmt);
    }

    @Override
    public void decodeAttributes(HashMap<String, String> attrMap, XMLTokenizer tokenizer)
    {
        topic = getRequiredAttribute("topic", attrMap, tokenizer);
        super.decodeAttributes(attrMap, tokenizer);
    }


    @Override
    public StringBuilder appendContent(StringBuilder buf, XMLFormatting fmt)
    {
        buf.append(rosMsgContent);
        return buf;
    }

    @Override
    public void decodeContent(XMLTokenizer tokenizer) throws IOException
    {
        rosMsgContent = tokenizer.getXMLSection();
    }

    /*
     * The XML Stag for XML encoding
     */
    private static final String XMLTAG = "directRosMessage";

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

    @Override
    public String getStringParameterValue(String name)
    {
    	if (name.equals("content")) return rosMsgContent;
    	if (name.equals("topic")) return topic;
        return super.getStringParameterValue(name);
    }

    @Override
    public float getFloatParameterValue(String name)
    {
        return super.getFloatParameterValue(name);
    }

    @Override
    public boolean specifiesParameter(String name)
    {
    	if (name.equals("content")) return true;
    	if (name.equals("topic")) return true;
        return super.specifiesParameter(name);
    }
}

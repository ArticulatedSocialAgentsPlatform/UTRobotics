using System;
using System.IO;
using System.Text;
using System.Xml.Serialization;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;

namespace KinectSkeleton {

  [XmlRoot("scene")]
  [DataContract]
  public class SceneUpdate {
    [XmlElement]
    [DataMember]
    public ScenePerson person1;

    [XmlElement]
    [DataMember]
    public ScenePerson person2;

    [IgnoreDataMember]
    private XmlSerializer xml;
    [IgnoreDataMember]
    private DataContractJsonSerializer json;

    private bool useJSON;

    public SceneUpdate() {
      this.person1 = new ScenePerson();
      this.person2 = new ScenePerson();

      this.xml = new System.Xml.Serialization.XmlSerializer(this.GetType());
      this.json = new DataContractJsonSerializer(typeof(SceneUpdate));

      this.updateSettings();
    }

    public void updateSettings() {
      this.useJSON = KinectSkeleton.Properties.Settings.Default.useJSON;
    }

    public void clear() {
      this.person1.clear();
      this.person2.clear();
    }

    public ScenePerson getScenePerson(int index) {
      if (index == 1) return this.person1;
      if (index == 2) return this.person2;
      throw new Exception("Invalid index");
    }

    public String serialize() {
      if (this.useJSON) {
        using (MemoryStream stream = new MemoryStream()) {
          this.json.WriteObject(stream, this);
          return Encoding.Default.GetString(stream.ToArray()); ;
        }
      } else {
        using (StringWriter textWriter = new StringWriter()) {
          this.xml.Serialize(textWriter, this);
          return textWriter.ToString();
        }
      }
    }
  }

  public class ScenePerson {

    public ScenePerson() {
      this.clear();
    }

    public void clear() {
      this.SkeletonHeadX = "";
      this.SkeletonHeadY = "";
      this.FaceHeadX = "";
      this.FaceHeadY = "";
      this.SkeletonDepth = "";
      this.AudioLikeliness = 0;
    }

    [XmlElement]
    [DataMember]
    public String TrackingId;

    [XmlElement]
    [DataMember]
    public String SkeletonHeadX;
    [XmlElement]
    [DataMember]
    public String SkeletonHeadY;

    [XmlElement]
    [DataMember]
    public String FaceHeadX;
    [XmlElement]
    [DataMember]
    public String FaceHeadY;

    [XmlElement]
    [DataMember]
    public String SkeletonDepth;

    [XmlElement]
    [DataMember]
    public Double AudioLikeliness;
  }
}

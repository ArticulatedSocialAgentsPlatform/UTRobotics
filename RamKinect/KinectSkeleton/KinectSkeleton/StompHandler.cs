using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Apache.NMS;
using Apache.NMS.Stomp;
using System.Windows.Forms;

namespace KinectSkeleton {

  class StompHandler {

    // Connection parameters
    public String user { get; set; }
    public String password { get; set; }
    public String host { get; set; }
    public String topic { get; set; }
    public int port { get; set; }

    // Connection data
    private String brokerUri;
    private ConnectionFactory factory;
    private IConnection connection;
    private ISession session;
    private IDestination destination;
    private IMessageProducer producer;

    // Check last message count
    private decimal updateRate;
    private DateTime lastSend;

    public StompHandler() {
      this.lastSend = DateTime.Now;
      this.initializeParameter();
      this.openConnection();
      this.openTopic();
    }

    private void initializeParameter() {
      this.host = KinectSkeleton.Properties.Settings.Default.apolloHost;
      this.port = Int32.Parse(KinectSkeleton.Properties.Settings.Default.apolloPort);
      this.user = KinectSkeleton.Properties.Settings.Default.apolloUser;
      this.password = KinectSkeleton.Properties.Settings.Default.apolloPassword;
      this.topic = KinectSkeleton.Properties.Settings.Default.apolloTopic;
      this.updateRate = KinectSkeleton.Properties.Settings.Default.maxUpdateRate;
    }

    private void openConnection() {
      var openForm = new OpeningApollo();
      openForm.Show();
      openForm.Invalidate();
      openForm.Update();
      openForm.Refresh();
      try {
        this.brokerUri = "stomp:tcp://" + host + ":" + port;
        this.factory = new ConnectionFactory(this.brokerUri);
        this.connection = this.factory.CreateConnection(this.user, this.password);
        this.connection.Start();
      } catch (Exception e){
        MessageBox.Show("Connection to Apollo failed!" + Environment.NewLine + e.Message, "Apollo error");
        this.connection = null;
      }
      openForm.Close();
      openForm.Dispose();
    }

    private void closeConnection() {
      if (this.connection != null) {
        this.connection.Close();
        this.connection.Dispose();

        this.session.Dispose();
        this.session = null;
        this.destination = null;
        this.producer.Dispose();
        this.producer = null;
      }
    }

    public void reconnect() {
      this.closeConnection();
      this.initializeParameter();
      this.openConnection();
      this.openTopic();
    }

    private void openTopic() {
      if (this.connection != null && this.connection.IsStarted) {
        this.session = this.connection.CreateSession(AcknowledgementMode.AutoAcknowledge);
        this.destination = this.session.GetTopic(this.topic);
        this.producer = this.session.CreateProducer(this.destination);
        this.producer.DeliveryMode = MsgDeliveryMode.NonPersistent;
      }
    }

    public void sendTextMessage(String text) {
      try {
        DateTime sendAt = this.lastSend.AddSeconds(1 / System.Convert.ToDouble(this.updateRate));
        if (sendAt < DateTime.Now && 
          this.connection != null && this.connection.IsStarted && this.producer != null) {
          this.producer.Send(this.session.CreateTextMessage(text));
          this.lastSend = DateTime.Now;
        }
      } catch {
        // Catch all
      }
    }
    
  }

}

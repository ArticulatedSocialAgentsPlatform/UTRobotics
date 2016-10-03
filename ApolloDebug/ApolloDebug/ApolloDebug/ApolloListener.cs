using Apache.NMS;
using Apache.NMS.Stomp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace ApolloDebug {
  class ApolloListener {

    private volatile bool stopRequested;

    public void StopListen() {
      this.stopRequested = true;
    }

    public void DoListen() {
      // Reset stop request
      this.stopRequested = false;

      // Retrieve parameters
      String host = ApolloDebug.Properties.Settings.Default.apolloHost;
      int port = Int32.Parse(ApolloDebug.Properties.Settings.Default.apolloPort);
      String user = ApolloDebug.Properties.Settings.Default.apolloUser;
      String password = ApolloDebug.Properties.Settings.Default.apolloPassword;
      String channel = ApolloDebug.Properties.Settings.Default.apolloChannel;
      int interval = Int32.Parse(ApolloDebug.Properties.Settings.Default.apolloInterval);
      long count = 0;

      // Allow main thread to continue creating the form
      Thread.Sleep(100);

      // Output debug information
      Console.WriteLine(String.Format(
        "Starting Apollo listener on {0}:{1}, credentials {2}:{3}, opening channel {4}...",
        host, port, user, password, channel
      ));

      String brokerUri = "stomp:tcp://" + host + ":" + port;
      ConnectionFactory factory = new ConnectionFactory(brokerUri);

      try {
        IConnection connection = factory.CreateConnection(user, password);
        connection.Start();
        ISession session = connection.CreateSession(AcknowledgementMode.ClientAcknowledge);
        IDestination dest = session.GetTopic(channel);

        IMessageConsumer consumer = session.CreateConsumer(dest);
        Console.WriteLine(String.Format(
          "Waiting for messages (messages printed with interval {0})...",
          interval
        ));

        while (!this.stopRequested) {
          IMessage msg = consumer.ReceiveNoWait();
          if (msg is ITextMessage) {
            ITextMessage txtMsg = msg as ITextMessage;
            String body = txtMsg.Text;

            count++;
            if (count % interval == 0) {
              Console.WriteLine();
              Console.WriteLine(String.Format("Received {0} messages.", count));
              Console.WriteLine(body);
            }

          } else {
            if (msg == null) {
              Thread.Sleep(10);
              continue;
            }
            Console.WriteLine();
            Console.WriteLine("Unexpected message type: " + msg.GetType().Name);
          }
        }

        Console.WriteLine();
        Console.WriteLine("Stopping Apollo listener...");
        connection.Close();
      } catch (Exception e) {
        Console.WriteLine();
        Console.WriteLine("Exception during Apollo listener execution");
        Console.WriteLine(e.Message);
      }
    }

  }
}

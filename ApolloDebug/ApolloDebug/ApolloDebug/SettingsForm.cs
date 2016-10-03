
using System;
using System.Windows.Forms;
using System.Threading;

namespace ApolloDebug {

  public partial class SettingsForm : Form {

    private ApolloListener apolloListener;
    private Thread apolloThread;

    public SettingsForm() {
      InitializeComponent();
      this.apolloHostTextbox.Text = ApolloDebug.Properties.Settings.Default.apolloHost;
      this.apolloPortTextbox.Text = ApolloDebug.Properties.Settings.Default.apolloPort;
      this.apolloChannelTextbox.Text = ApolloDebug.Properties.Settings.Default.apolloChannel;
      this.apolloIntervalTextbox.Text = ApolloDebug.Properties.Settings.Default.apolloInterval;
      this.apolloUserTextbox.Text = ApolloDebug.Properties.Settings.Default.apolloUser;
      this.apolloPasswordTextbox.Text = ApolloDebug.Properties.Settings.Default.apolloPassword;
    }

    private void updateSettings() {
      ApolloDebug.Properties.Settings.Default.apolloHost = this.apolloHostTextbox.Text;
      ApolloDebug.Properties.Settings.Default.apolloPort = this.apolloPortTextbox.Text;
      ApolloDebug.Properties.Settings.Default.apolloChannel = this.apolloChannelTextbox.Text;
      ApolloDebug.Properties.Settings.Default.apolloInterval = this.apolloIntervalTextbox.Text;
      ApolloDebug.Properties.Settings.Default.apolloUser = this.apolloUserTextbox.Text;
      ApolloDebug.Properties.Settings.Default.apolloPassword = this.apolloPasswordTextbox.Text;
      ApolloDebug.Properties.Settings.Default.Save();
    }

    public void restartApollo(object sender, EventArgs e) {
      // Close existing connection
      this.stopApollo(sender, e);

      // Open apollo
      this.startApollo();
    }

    private void startApollo() {
      // Always update the settings on open
      this.updateSettings();

      // Check if the listener already exists, otherwise create them
      if (this.apolloListener == null) this.apolloListener = new ApolloListener();

      // Start the thread
      this.apolloThread = new Thread(this.apolloListener.DoListen);
      this.apolloThread.IsBackground = true;
      this.apolloThread.Start();

      // Enable the stop button
      this.stopButton.Enabled = true;
    }

    public void stopApollo(object sender, EventArgs e) {
      // Disable the stop button
      this.stopButton.Enabled = false;

      // Check if the thread exists and is alive
      if (this.apolloThread != null && this.apolloThread.IsAlive) {
        // Then request the stop
        this.apolloListener.StopListen();
        var success = this.apolloThread.Join(new TimeSpan(0, 0, 2));

        // If not successful, abort the thread
        if (!success) {
          this.apolloThread.Abort();
        }
        Console.WriteLine("Stopped Apollo listener!");
      }
    }

    public void quitApplication(object sender, EventArgs e) {
      // Stop the Apollo thread
      this.stopApollo(sender, e);

      // Exit application
      Application.Exit();
    }

    public void threadCheck(object sender, EventArgs e) {
      if (this.apolloThread != null && !this.apolloThread.IsAlive) {
        this.stopButton.Enabled = false;
      }
    }

    private void SettingsForm_Load(object sender, EventArgs e) {
      // Start the listener on load
      this.startApollo();

      // Create a ticker to check the background thread
      System.Windows.Forms.Timer t1 = new System.Windows.Forms.Timer();
      t1.Interval = 500;
      t1.Tick += new EventHandler(this.threadCheck);
      t1.Start();
    }

    private void SettingsForm_FormClosing(object sender, FormClosingEventArgs e) {
      // Also execute the quit on the form close button click
      this.quitApplication(sender, e);
    }
  }
}

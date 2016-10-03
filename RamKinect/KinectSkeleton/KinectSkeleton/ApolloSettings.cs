using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace KinectSkeleton {
  public partial class ApolloSettings : Form {
    public ApolloSettings() {
      InitializeComponent();
    }

    private void ApolloSettings_Load(object sender, EventArgs e) {
      this.apolloHostTextbox.Text = KinectSkeleton.Properties.Settings.Default.apolloHost;
      this.apolloPortTextbox.Text = KinectSkeleton.Properties.Settings.Default.apolloPort;
      this.apolloUserTextbox.Text = KinectSkeleton.Properties.Settings.Default.apolloUser;
      this.apolloPasswordTextbox.Text = KinectSkeleton.Properties.Settings.Default.apolloPassword;
      this.apolloChannelTextbox.Text = KinectSkeleton.Properties.Settings.Default.apolloTopic;
      this.maxUpdateRateNumeric.Value = KinectSkeleton.Properties.Settings.Default.maxUpdateRate;
      this.jsonCheckbox.Checked = KinectSkeleton.Properties.Settings.Default.useJSON;
    }

    private void updateButton_Click(object sender, EventArgs e) {
      KinectSkeleton.Properties.Settings.Default.apolloHost = this.apolloHostTextbox.Text;
      KinectSkeleton.Properties.Settings.Default.apolloPort = this.apolloPortTextbox.Text;
      KinectSkeleton.Properties.Settings.Default.apolloUser = this.apolloUserTextbox.Text;
      KinectSkeleton.Properties.Settings.Default.apolloPassword = this.apolloPasswordTextbox.Text;
      KinectSkeleton.Properties.Settings.Default.apolloTopic = this.apolloChannelTextbox.Text;
      KinectSkeleton.Properties.Settings.Default.maxUpdateRate = this.maxUpdateRateNumeric.Value;
      KinectSkeleton.Properties.Settings.Default.useJSON = this.jsonCheckbox.Checked;
      KinectSkeleton.Properties.Settings.Default.Save();
      this.Close();
    }
  }
}

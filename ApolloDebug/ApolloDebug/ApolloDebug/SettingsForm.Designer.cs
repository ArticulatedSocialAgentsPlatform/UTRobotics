namespace ApolloDebug {
  partial class SettingsForm {
    /// <summary>
    /// Required designer variable.
    /// </summary>
    private System.ComponentModel.IContainer components = null;

    /// <summary>
    /// Clean up any resources being used.
    /// </summary>
    /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
    protected override void Dispose(bool disposing) {
      if (disposing && (components != null)) {
        components.Dispose();
      }
      base.Dispose(disposing);
    }

    #region Windows Form Designer generated code

    /// <summary>
    /// Required method for Designer support - do not modify
    /// the contents of this method with the code editor.
    /// </summary>
    private void InitializeComponent() {
      System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(SettingsForm));
      this.apolloHostTextbox = new System.Windows.Forms.TextBox();
      this.label1 = new System.Windows.Forms.Label();
      this.groupBox1 = new System.Windows.Forms.GroupBox();
      this.apolloPasswordTextbox = new System.Windows.Forms.TextBox();
      this.label6 = new System.Windows.Forms.Label();
      this.apolloUserTextbox = new System.Windows.Forms.TextBox();
      this.label5 = new System.Windows.Forms.Label();
      this.apolloPortTextbox = new System.Windows.Forms.TextBox();
      this.label2 = new System.Windows.Forms.Label();
      this.updateButton = new System.Windows.Forms.Button();
      this.quitButton = new System.Windows.Forms.Button();
      this.groupBox2 = new System.Windows.Forms.GroupBox();
      this.apolloIntervalTextbox = new System.Windows.Forms.TextBox();
      this.label3 = new System.Windows.Forms.Label();
      this.apolloChannelTextbox = new System.Windows.Forms.TextBox();
      this.label4 = new System.Windows.Forms.Label();
      this.pictureBox1 = new System.Windows.Forms.PictureBox();
      this.pictureBox2 = new System.Windows.Forms.PictureBox();
      this.pictureBox3 = new System.Windows.Forms.PictureBox();
      this.stopButton = new System.Windows.Forms.Button();
      this.groupBox1.SuspendLayout();
      this.groupBox2.SuspendLayout();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).BeginInit();
      this.SuspendLayout();
      // 
      // apolloHostTextbox
      // 
      this.apolloHostTextbox.Location = new System.Drawing.Point(9, 33);
      this.apolloHostTextbox.Name = "apolloHostTextbox";
      this.apolloHostTextbox.Size = new System.Drawing.Size(154, 20);
      this.apolloHostTextbox.TabIndex = 0;
      // 
      // label1
      // 
      this.label1.AutoSize = true;
      this.label1.Location = new System.Drawing.Point(6, 17);
      this.label1.Name = "label1";
      this.label1.Size = new System.Drawing.Size(29, 13);
      this.label1.TabIndex = 2;
      this.label1.Text = "Host";
      // 
      // groupBox1
      // 
      this.groupBox1.Controls.Add(this.apolloPasswordTextbox);
      this.groupBox1.Controls.Add(this.label6);
      this.groupBox1.Controls.Add(this.apolloUserTextbox);
      this.groupBox1.Controls.Add(this.label5);
      this.groupBox1.Controls.Add(this.apolloPortTextbox);
      this.groupBox1.Controls.Add(this.label2);
      this.groupBox1.Controls.Add(this.apolloHostTextbox);
      this.groupBox1.Controls.Add(this.label1);
      this.groupBox1.Location = new System.Drawing.Point(12, 12);
      this.groupBox1.Name = "groupBox1";
      this.groupBox1.Size = new System.Drawing.Size(170, 172);
      this.groupBox1.TabIndex = 3;
      this.groupBox1.TabStop = false;
      this.groupBox1.Text = "Apollo properties";
      // 
      // apolloPasswordTextbox
      // 
      this.apolloPasswordTextbox.Location = new System.Drawing.Point(9, 144);
      this.apolloPasswordTextbox.Name = "apolloPasswordTextbox";
      this.apolloPasswordTextbox.Size = new System.Drawing.Size(154, 20);
      this.apolloPasswordTextbox.TabIndex = 3;
      // 
      // label6
      // 
      this.label6.AutoSize = true;
      this.label6.Location = new System.Drawing.Point(6, 130);
      this.label6.Name = "label6";
      this.label6.Size = new System.Drawing.Size(53, 13);
      this.label6.TabIndex = 7;
      this.label6.Text = "Password";
      // 
      // apolloUserTextbox
      // 
      this.apolloUserTextbox.Location = new System.Drawing.Point(9, 107);
      this.apolloUserTextbox.Name = "apolloUserTextbox";
      this.apolloUserTextbox.Size = new System.Drawing.Size(154, 20);
      this.apolloUserTextbox.TabIndex = 2;
      // 
      // label5
      // 
      this.label5.AutoSize = true;
      this.label5.Location = new System.Drawing.Point(6, 93);
      this.label5.Name = "label5";
      this.label5.Size = new System.Drawing.Size(29, 13);
      this.label5.TabIndex = 5;
      this.label5.Text = "User";
      // 
      // apolloPortTextbox
      // 
      this.apolloPortTextbox.Location = new System.Drawing.Point(9, 70);
      this.apolloPortTextbox.Name = "apolloPortTextbox";
      this.apolloPortTextbox.Size = new System.Drawing.Size(154, 20);
      this.apolloPortTextbox.TabIndex = 1;
      // 
      // label2
      // 
      this.label2.AutoSize = true;
      this.label2.Location = new System.Drawing.Point(6, 56);
      this.label2.Name = "label2";
      this.label2.Size = new System.Drawing.Size(26, 13);
      this.label2.TabIndex = 3;
      this.label2.Text = "Port";
      // 
      // updateButton
      // 
      this.updateButton.Location = new System.Drawing.Point(204, 112);
      this.updateButton.Name = "updateButton";
      this.updateButton.Size = new System.Drawing.Size(170, 23);
      this.updateButton.TabIndex = 6;
      this.updateButton.Text = "Update settings and restart";
      this.updateButton.UseVisualStyleBackColor = true;
      this.updateButton.Click += new System.EventHandler(this.restartApollo);
      // 
      // quitButton
      // 
      this.quitButton.Location = new System.Drawing.Point(204, 160);
      this.quitButton.Name = "quitButton";
      this.quitButton.Size = new System.Drawing.Size(170, 23);
      this.quitButton.TabIndex = 8;
      this.quitButton.Text = "Quit application";
      this.quitButton.UseVisualStyleBackColor = true;
      this.quitButton.Click += new System.EventHandler(this.quitApplication);
      // 
      // groupBox2
      // 
      this.groupBox2.Controls.Add(this.apolloIntervalTextbox);
      this.groupBox2.Controls.Add(this.label3);
      this.groupBox2.Controls.Add(this.apolloChannelTextbox);
      this.groupBox2.Controls.Add(this.label4);
      this.groupBox2.Location = new System.Drawing.Point(204, 12);
      this.groupBox2.Name = "groupBox2";
      this.groupBox2.Size = new System.Drawing.Size(170, 98);
      this.groupBox2.TabIndex = 5;
      this.groupBox2.TabStop = false;
      this.groupBox2.Text = "Channel properties";
      // 
      // apolloIntervalTextbox
      // 
      this.apolloIntervalTextbox.Location = new System.Drawing.Point(9, 70);
      this.apolloIntervalTextbox.Name = "apolloIntervalTextbox";
      this.apolloIntervalTextbox.Size = new System.Drawing.Size(154, 20);
      this.apolloIntervalTextbox.TabIndex = 5;
      // 
      // label3
      // 
      this.label3.AutoSize = true;
      this.label3.Location = new System.Drawing.Point(6, 56);
      this.label3.Name = "label3";
      this.label3.Size = new System.Drawing.Size(65, 13);
      this.label3.TabIndex = 3;
      this.label3.Text = "Print interval";
      // 
      // apolloChannelTextbox
      // 
      this.apolloChannelTextbox.Location = new System.Drawing.Point(9, 33);
      this.apolloChannelTextbox.Name = "apolloChannelTextbox";
      this.apolloChannelTextbox.Size = new System.Drawing.Size(154, 20);
      this.apolloChannelTextbox.TabIndex = 4;
      // 
      // label4
      // 
      this.label4.AutoSize = true;
      this.label4.Location = new System.Drawing.Point(6, 17);
      this.label4.Name = "label4";
      this.label4.Size = new System.Drawing.Size(75, 13);
      this.label4.TabIndex = 2;
      this.label4.Text = "Channel name";
      // 
      // pictureBox1
      // 
      this.pictureBox1.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox1.Image")));
      this.pictureBox1.Location = new System.Drawing.Point(12, 190);
      this.pictureBox1.Name = "pictureBox1";
      this.pictureBox1.Size = new System.Drawing.Size(170, 67);
      this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
      this.pictureBox1.TabIndex = 7;
      this.pictureBox1.TabStop = false;
      // 
      // pictureBox2
      // 
      this.pictureBox2.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox2.Image")));
      this.pictureBox2.Location = new System.Drawing.Point(204, 190);
      this.pictureBox2.Name = "pictureBox2";
      this.pictureBox2.Size = new System.Drawing.Size(170, 67);
      this.pictureBox2.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
      this.pictureBox2.TabIndex = 8;
      this.pictureBox2.TabStop = false;
      // 
      // pictureBox3
      // 
      this.pictureBox3.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox3.Image")));
      this.pictureBox3.Location = new System.Drawing.Point(12, 263);
      this.pictureBox3.Name = "pictureBox3";
      this.pictureBox3.Size = new System.Drawing.Size(365, 20);
      this.pictureBox3.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
      this.pictureBox3.TabIndex = 9;
      this.pictureBox3.TabStop = false;
      // 
      // stopButton
      // 
      this.stopButton.Enabled = false;
      this.stopButton.Location = new System.Drawing.Point(204, 136);
      this.stopButton.Name = "stopButton";
      this.stopButton.Size = new System.Drawing.Size(170, 23);
      this.stopButton.TabIndex = 7;
      this.stopButton.Text = "Stop listener";
      this.stopButton.UseVisualStyleBackColor = true;
      this.stopButton.Click += new System.EventHandler(this.stopApollo);
      // 
      // SettingsForm
      // 
      this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
      this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
      this.ClientSize = new System.Drawing.Size(389, 292);
      this.Controls.Add(this.stopButton);
      this.Controls.Add(this.pictureBox3);
      this.Controls.Add(this.pictureBox2);
      this.Controls.Add(this.pictureBox1);
      this.Controls.Add(this.groupBox2);
      this.Controls.Add(this.quitButton);
      this.Controls.Add(this.updateButton);
      this.Controls.Add(this.groupBox1);
      this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
      this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
      this.MaximizeBox = false;
      this.Name = "SettingsForm";
      this.Text = "Apollo Debug";
      this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.SettingsForm_FormClosing);
      this.Load += new System.EventHandler(this.SettingsForm_Load);
      this.groupBox1.ResumeLayout(false);
      this.groupBox1.PerformLayout();
      this.groupBox2.ResumeLayout(false);
      this.groupBox2.PerformLayout();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit();
      this.ResumeLayout(false);

    }

    #endregion

    private System.Windows.Forms.TextBox apolloHostTextbox;
    private System.Windows.Forms.Label label1;
    private System.Windows.Forms.GroupBox groupBox1;
    private System.Windows.Forms.Label label2;
    private System.Windows.Forms.TextBox apolloPortTextbox;
    private System.Windows.Forms.Button updateButton;
    private System.Windows.Forms.Button quitButton;
    private System.Windows.Forms.GroupBox groupBox2;
    private System.Windows.Forms.TextBox apolloIntervalTextbox;
    private System.Windows.Forms.Label label3;
    private System.Windows.Forms.TextBox apolloChannelTextbox;
    private System.Windows.Forms.Label label4;
    private System.Windows.Forms.PictureBox pictureBox1;
    private System.Windows.Forms.PictureBox pictureBox2;
    private System.Windows.Forms.PictureBox pictureBox3;
    private System.Windows.Forms.TextBox apolloPasswordTextbox;
    private System.Windows.Forms.Label label6;
    private System.Windows.Forms.TextBox apolloUserTextbox;
    private System.Windows.Forms.Label label5;
    private System.Windows.Forms.Button stopButton;
  }
}


namespace KinectSkeleton
{
    partial class KinectTracker
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
      System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(KinectTracker));
      this.kinectVideoBox = new System.Windows.Forms.PictureBox();
      this.skeletonTrackingCheckbox = new System.Windows.Forms.CheckBox();
      this.faceTrackingCheckbox = new System.Windows.Forms.CheckBox();
      this.headTrackingCheckbox = new System.Windows.Forms.CheckBox();
      this.audioTrackingCheckbox = new System.Windows.Forms.CheckBox();
      this.groupBox1 = new System.Windows.Forms.GroupBox();
      this.depth1 = new System.Windows.Forms.Label();
      this.label8 = new System.Windows.Forms.Label();
      this.speech1 = new System.Windows.Forms.Label();
      this.label6 = new System.Windows.Forms.Label();
      this.faceHeadY1 = new System.Windows.Forms.Label();
      this.faceHeadX1 = new System.Windows.Forms.Label();
      this.skeletonHeadY1 = new System.Windows.Forms.Label();
      this.skeletonHeadX1 = new System.Windows.Forms.Label();
      this.label4 = new System.Windows.Forms.Label();
      this.label3 = new System.Windows.Forms.Label();
      this.label2 = new System.Windows.Forms.Label();
      this.label1 = new System.Windows.Forms.Label();
      this.groupBox2 = new System.Windows.Forms.GroupBox();
      this.depth2 = new System.Windows.Forms.Label();
      this.label13 = new System.Windows.Forms.Label();
      this.speech2 = new System.Windows.Forms.Label();
      this.label7 = new System.Windows.Forms.Label();
      this.faceHeadY2 = new System.Windows.Forms.Label();
      this.faceHeadX2 = new System.Windows.Forms.Label();
      this.skeletonHeadY2 = new System.Windows.Forms.Label();
      this.skeletonHeadX2 = new System.Windows.Forms.Label();
      this.label9 = new System.Windows.Forms.Label();
      this.label10 = new System.Windows.Forms.Label();
      this.label11 = new System.Windows.Forms.Label();
      this.label12 = new System.Windows.Forms.Label();
      this.debugOutputCheckbox = new System.Windows.Forms.CheckBox();
      this.pictureBox1 = new System.Windows.Forms.PictureBox();
      this.pictureBox2 = new System.Windows.Forms.PictureBox();
      this.pictureBox3 = new System.Windows.Forms.PictureBox();
      this.updateApollo = new System.Windows.Forms.Button();
      ((System.ComponentModel.ISupportInitialize)(this.kinectVideoBox)).BeginInit();
      this.groupBox1.SuspendLayout();
      this.groupBox2.SuspendLayout();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).BeginInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).BeginInit();
      this.SuspendLayout();
      // 
      // kinectVideoBox
      // 
      this.kinectVideoBox.Location = new System.Drawing.Point(12, 9);
      this.kinectVideoBox.Name = "kinectVideoBox";
      this.kinectVideoBox.Size = new System.Drawing.Size(640, 480);
      this.kinectVideoBox.TabIndex = 0;
      this.kinectVideoBox.TabStop = false;
      // 
      // skeletonTrackingCheckbox
      // 
      this.skeletonTrackingCheckbox.AutoSize = true;
      this.skeletonTrackingCheckbox.Checked = true;
      this.skeletonTrackingCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.skeletonTrackingCheckbox.Location = new System.Drawing.Point(12, 502);
      this.skeletonTrackingCheckbox.Name = "skeletonTrackingCheckbox";
      this.skeletonTrackingCheckbox.Size = new System.Drawing.Size(149, 17);
      this.skeletonTrackingCheckbox.TabIndex = 1;
      this.skeletonTrackingCheckbox.Text = "Enable Skeleton Tracking";
      this.skeletonTrackingCheckbox.UseVisualStyleBackColor = true;
      // 
      // faceTrackingCheckbox
      // 
      this.faceTrackingCheckbox.AutoSize = true;
      this.faceTrackingCheckbox.Checked = true;
      this.faceTrackingCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.faceTrackingCheckbox.Location = new System.Drawing.Point(356, 502);
      this.faceTrackingCheckbox.Name = "faceTrackingCheckbox";
      this.faceTrackingCheckbox.Size = new System.Drawing.Size(131, 17);
      this.faceTrackingCheckbox.TabIndex = 2;
      this.faceTrackingCheckbox.Text = "Enable Face Tracking";
      this.faceTrackingCheckbox.UseVisualStyleBackColor = true;
      // 
      // headTrackingCheckbox
      // 
      this.headTrackingCheckbox.AutoSize = true;
      this.headTrackingCheckbox.Checked = true;
      this.headTrackingCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.headTrackingCheckbox.Location = new System.Drawing.Point(192, 502);
      this.headTrackingCheckbox.Name = "headTrackingCheckbox";
      this.headTrackingCheckbox.Size = new System.Drawing.Size(133, 17);
      this.headTrackingCheckbox.TabIndex = 3;
      this.headTrackingCheckbox.Text = "Enable Head Tracking";
      this.headTrackingCheckbox.UseVisualStyleBackColor = true;
      // 
      // audioTrackingCheckbox
      // 
      this.audioTrackingCheckbox.AutoSize = true;
      this.audioTrackingCheckbox.Checked = true;
      this.audioTrackingCheckbox.CheckState = System.Windows.Forms.CheckState.Checked;
      this.audioTrackingCheckbox.Location = new System.Drawing.Point(518, 502);
      this.audioTrackingCheckbox.Name = "audioTrackingCheckbox";
      this.audioTrackingCheckbox.Size = new System.Drawing.Size(134, 17);
      this.audioTrackingCheckbox.TabIndex = 4;
      this.audioTrackingCheckbox.Text = "Enable Audio Tracking";
      this.audioTrackingCheckbox.UseVisualStyleBackColor = true;
      // 
      // groupBox1
      // 
      this.groupBox1.Controls.Add(this.depth1);
      this.groupBox1.Controls.Add(this.label8);
      this.groupBox1.Controls.Add(this.speech1);
      this.groupBox1.Controls.Add(this.label6);
      this.groupBox1.Controls.Add(this.faceHeadY1);
      this.groupBox1.Controls.Add(this.faceHeadX1);
      this.groupBox1.Controls.Add(this.skeletonHeadY1);
      this.groupBox1.Controls.Add(this.skeletonHeadX1);
      this.groupBox1.Controls.Add(this.label4);
      this.groupBox1.Controls.Add(this.label3);
      this.groupBox1.Controls.Add(this.label2);
      this.groupBox1.Controls.Add(this.label1);
      this.groupBox1.Location = new System.Drawing.Point(658, 9);
      this.groupBox1.Name = "groupBox1";
      this.groupBox1.Size = new System.Drawing.Size(254, 121);
      this.groupBox1.TabIndex = 5;
      this.groupBox1.TabStop = false;
      this.groupBox1.Text = "Person 1";
      // 
      // depth1
      // 
      this.depth1.AutoSize = true;
      this.depth1.Location = new System.Drawing.Point(184, 103);
      this.depth1.Name = "depth1";
      this.depth1.Size = new System.Drawing.Size(10, 13);
      this.depth1.TabIndex = 11;
      this.depth1.Text = "-";
      this.depth1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label8
      // 
      this.label8.AutoSize = true;
      this.label8.Location = new System.Drawing.Point(6, 102);
      this.label8.Name = "label8";
      this.label8.Size = new System.Drawing.Size(90, 13);
      this.label8.TabIndex = 10;
      this.label8.Text = "Depth information";
      // 
      // speech1
      // 
      this.speech1.AutoSize = true;
      this.speech1.Location = new System.Drawing.Point(184, 82);
      this.speech1.Name = "speech1";
      this.speech1.Size = new System.Drawing.Size(10, 13);
      this.speech1.TabIndex = 9;
      this.speech1.Text = "-";
      this.speech1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label6
      // 
      this.label6.AutoSize = true;
      this.label6.Location = new System.Drawing.Point(6, 81);
      this.label6.Name = "label6";
      this.label6.Size = new System.Drawing.Size(94, 13);
      this.label6.TabIndex = 8;
      this.label6.Text = "Speech probability";
      // 
      // faceHeadY1
      // 
      this.faceHeadY1.AutoSize = true;
      this.faceHeadY1.Location = new System.Drawing.Point(213, 59);
      this.faceHeadY1.Name = "faceHeadY1";
      this.faceHeadY1.Size = new System.Drawing.Size(10, 13);
      this.faceHeadY1.TabIndex = 7;
      this.faceHeadY1.Text = "-";
      this.faceHeadY1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // faceHeadX1
      // 
      this.faceHeadX1.AutoSize = true;
      this.faceHeadX1.Location = new System.Drawing.Point(155, 59);
      this.faceHeadX1.Name = "faceHeadX1";
      this.faceHeadX1.Size = new System.Drawing.Size(10, 13);
      this.faceHeadX1.TabIndex = 6;
      this.faceHeadX1.Text = "-";
      this.faceHeadX1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // skeletonHeadY1
      // 
      this.skeletonHeadY1.AutoSize = true;
      this.skeletonHeadY1.Location = new System.Drawing.Point(213, 37);
      this.skeletonHeadY1.Name = "skeletonHeadY1";
      this.skeletonHeadY1.Size = new System.Drawing.Size(10, 13);
      this.skeletonHeadY1.TabIndex = 5;
      this.skeletonHeadY1.Text = "-";
      this.skeletonHeadY1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // skeletonHeadX1
      // 
      this.skeletonHeadX1.AutoSize = true;
      this.skeletonHeadX1.Location = new System.Drawing.Point(155, 37);
      this.skeletonHeadX1.Name = "skeletonHeadX1";
      this.skeletonHeadX1.Size = new System.Drawing.Size(10, 13);
      this.skeletonHeadX1.TabIndex = 4;
      this.skeletonHeadX1.Text = "-";
      this.skeletonHeadX1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label4
      // 
      this.label4.AutoSize = true;
      this.label4.Location = new System.Drawing.Point(211, 16);
      this.label4.Name = "label4";
      this.label4.Size = new System.Drawing.Size(14, 13);
      this.label4.TabIndex = 3;
      this.label4.Text = "Y";
      this.label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label3
      // 
      this.label3.AutoSize = true;
      this.label3.Location = new System.Drawing.Point(153, 16);
      this.label3.Name = "label3";
      this.label3.Size = new System.Drawing.Size(14, 13);
      this.label3.TabIndex = 2;
      this.label3.Text = "X";
      this.label3.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label2
      // 
      this.label2.AutoSize = true;
      this.label2.Location = new System.Drawing.Point(6, 59);
      this.label2.Name = "label2";
      this.label2.Size = new System.Drawing.Size(99, 13);
      this.label2.TabIndex = 1;
      this.label2.Text = "Face tracking head";
      // 
      // label1
      // 
      this.label1.AutoSize = true;
      this.label1.Location = new System.Drawing.Point(6, 37);
      this.label1.Name = "label1";
      this.label1.Size = new System.Drawing.Size(117, 13);
      this.label1.TabIndex = 0;
      this.label1.Text = "Skeleton tracking head";
      // 
      // groupBox2
      // 
      this.groupBox2.Controls.Add(this.depth2);
      this.groupBox2.Controls.Add(this.label13);
      this.groupBox2.Controls.Add(this.speech2);
      this.groupBox2.Controls.Add(this.label7);
      this.groupBox2.Controls.Add(this.faceHeadY2);
      this.groupBox2.Controls.Add(this.faceHeadX2);
      this.groupBox2.Controls.Add(this.skeletonHeadY2);
      this.groupBox2.Controls.Add(this.skeletonHeadX2);
      this.groupBox2.Controls.Add(this.label9);
      this.groupBox2.Controls.Add(this.label10);
      this.groupBox2.Controls.Add(this.label11);
      this.groupBox2.Controls.Add(this.label12);
      this.groupBox2.Location = new System.Drawing.Point(658, 136);
      this.groupBox2.Name = "groupBox2";
      this.groupBox2.Size = new System.Drawing.Size(254, 121);
      this.groupBox2.TabIndex = 8;
      this.groupBox2.TabStop = false;
      this.groupBox2.Text = "Person 2";
      // 
      // depth2
      // 
      this.depth2.AutoSize = true;
      this.depth2.Location = new System.Drawing.Point(184, 102);
      this.depth2.Name = "depth2";
      this.depth2.Size = new System.Drawing.Size(10, 13);
      this.depth2.TabIndex = 11;
      this.depth2.Text = "-";
      this.depth2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label13
      // 
      this.label13.AutoSize = true;
      this.label13.Location = new System.Drawing.Point(6, 102);
      this.label13.Name = "label13";
      this.label13.Size = new System.Drawing.Size(90, 13);
      this.label13.TabIndex = 12;
      this.label13.Text = "Depth information";
      // 
      // speech2
      // 
      this.speech2.AutoSize = true;
      this.speech2.Location = new System.Drawing.Point(184, 81);
      this.speech2.Name = "speech2";
      this.speech2.Size = new System.Drawing.Size(10, 13);
      this.speech2.TabIndex = 10;
      this.speech2.Text = "-";
      this.speech2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label7
      // 
      this.label7.AutoSize = true;
      this.label7.Location = new System.Drawing.Point(6, 81);
      this.label7.Name = "label7";
      this.label7.Size = new System.Drawing.Size(94, 13);
      this.label7.TabIndex = 10;
      this.label7.Text = "Speech probability";
      // 
      // faceHeadY2
      // 
      this.faceHeadY2.AutoSize = true;
      this.faceHeadY2.Location = new System.Drawing.Point(213, 58);
      this.faceHeadY2.Name = "faceHeadY2";
      this.faceHeadY2.Size = new System.Drawing.Size(10, 13);
      this.faceHeadY2.TabIndex = 7;
      this.faceHeadY2.Text = "-";
      this.faceHeadY2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // faceHeadX2
      // 
      this.faceHeadX2.AutoSize = true;
      this.faceHeadX2.Location = new System.Drawing.Point(155, 58);
      this.faceHeadX2.Name = "faceHeadX2";
      this.faceHeadX2.Size = new System.Drawing.Size(10, 13);
      this.faceHeadX2.TabIndex = 6;
      this.faceHeadX2.Text = "-";
      this.faceHeadX2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // skeletonHeadY2
      // 
      this.skeletonHeadY2.AutoSize = true;
      this.skeletonHeadY2.Location = new System.Drawing.Point(213, 35);
      this.skeletonHeadY2.Name = "skeletonHeadY2";
      this.skeletonHeadY2.Size = new System.Drawing.Size(10, 13);
      this.skeletonHeadY2.TabIndex = 5;
      this.skeletonHeadY2.Text = "-";
      this.skeletonHeadY2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // skeletonHeadX2
      // 
      this.skeletonHeadX2.AutoSize = true;
      this.skeletonHeadX2.Location = new System.Drawing.Point(155, 35);
      this.skeletonHeadX2.Name = "skeletonHeadX2";
      this.skeletonHeadX2.Size = new System.Drawing.Size(10, 13);
      this.skeletonHeadX2.TabIndex = 4;
      this.skeletonHeadX2.Text = "-";
      this.skeletonHeadX2.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label9
      // 
      this.label9.AutoSize = true;
      this.label9.Location = new System.Drawing.Point(211, 16);
      this.label9.Name = "label9";
      this.label9.Size = new System.Drawing.Size(14, 13);
      this.label9.TabIndex = 3;
      this.label9.Text = "Y";
      this.label9.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label10
      // 
      this.label10.AutoSize = true;
      this.label10.Location = new System.Drawing.Point(153, 16);
      this.label10.Name = "label10";
      this.label10.Size = new System.Drawing.Size(14, 13);
      this.label10.TabIndex = 2;
      this.label10.Text = "X";
      this.label10.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
      // 
      // label11
      // 
      this.label11.AutoSize = true;
      this.label11.Location = new System.Drawing.Point(6, 58);
      this.label11.Name = "label11";
      this.label11.Size = new System.Drawing.Size(99, 13);
      this.label11.TabIndex = 1;
      this.label11.Text = "Face tracking head";
      // 
      // label12
      // 
      this.label12.AutoSize = true;
      this.label12.Location = new System.Drawing.Point(6, 35);
      this.label12.Name = "label12";
      this.label12.Size = new System.Drawing.Size(117, 13);
      this.label12.TabIndex = 0;
      this.label12.Text = "Skeleton tracking head";
      // 
      // debugOutputCheckbox
      // 
      this.debugOutputCheckbox.AutoSize = true;
      this.debugOutputCheckbox.Location = new System.Drawing.Point(726, 263);
      this.debugOutputCheckbox.Name = "debugOutputCheckbox";
      this.debugOutputCheckbox.Size = new System.Drawing.Size(126, 17);
      this.debugOutputCheckbox.TabIndex = 9;
      this.debugOutputCheckbox.Text = "Pause Debug Output";
      this.debugOutputCheckbox.UseVisualStyleBackColor = true;
      // 
      // pictureBox1
      // 
      this.pictureBox1.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox1.Image")));
      this.pictureBox1.Location = new System.Drawing.Point(658, 414);
      this.pictureBox1.Name = "pictureBox1";
      this.pictureBox1.Size = new System.Drawing.Size(254, 75);
      this.pictureBox1.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
      this.pictureBox1.TabIndex = 10;
      this.pictureBox1.TabStop = false;
      // 
      // pictureBox2
      // 
      this.pictureBox2.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox2.Image")));
      this.pictureBox2.Location = new System.Drawing.Point(658, 319);
      this.pictureBox2.Name = "pictureBox2";
      this.pictureBox2.Size = new System.Drawing.Size(254, 89);
      this.pictureBox2.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
      this.pictureBox2.TabIndex = 11;
      this.pictureBox2.TabStop = false;
      // 
      // pictureBox3
      // 
      this.pictureBox3.Image = ((System.Drawing.Image)(resources.GetObject("pictureBox3.Image")));
      this.pictureBox3.Location = new System.Drawing.Point(658, 498);
      this.pictureBox3.Name = "pictureBox3";
      this.pictureBox3.Size = new System.Drawing.Size(254, 24);
      this.pictureBox3.SizeMode = System.Windows.Forms.PictureBoxSizeMode.Zoom;
      this.pictureBox3.TabIndex = 12;
      this.pictureBox3.TabStop = false;
      // 
      // updateApollo
      // 
      this.updateApollo.Location = new System.Drawing.Point(658, 286);
      this.updateApollo.Name = "updateApollo";
      this.updateApollo.Size = new System.Drawing.Size(254, 23);
      this.updateApollo.TabIndex = 13;
      this.updateApollo.Text = "Update Apollo settings";
      this.updateApollo.UseVisualStyleBackColor = true;
      this.updateApollo.Click += new System.EventHandler(this.updateApollo_Click);
      // 
      // KinectTracker
      // 
      this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
      this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
      this.ClientSize = new System.Drawing.Size(924, 531);
      this.Controls.Add(this.updateApollo);
      this.Controls.Add(this.pictureBox3);
      this.Controls.Add(this.pictureBox2);
      this.Controls.Add(this.pictureBox1);
      this.Controls.Add(this.debugOutputCheckbox);
      this.Controls.Add(this.groupBox2);
      this.Controls.Add(this.groupBox1);
      this.Controls.Add(this.audioTrackingCheckbox);
      this.Controls.Add(this.headTrackingCheckbox);
      this.Controls.Add(this.faceTrackingCheckbox);
      this.Controls.Add(this.skeletonTrackingCheckbox);
      this.Controls.Add(this.kinectVideoBox);
      this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
      this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
      this.MaximizeBox = false;
      this.Name = "KinectTracker";
      this.Text = "Kinect Skeleton Tracker";
      this.Load += new System.EventHandler(this.KinectTracker_Load);
      ((System.ComponentModel.ISupportInitialize)(this.kinectVideoBox)).EndInit();
      this.groupBox1.ResumeLayout(false);
      this.groupBox1.PerformLayout();
      this.groupBox2.ResumeLayout(false);
      this.groupBox2.PerformLayout();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox2)).EndInit();
      ((System.ComponentModel.ISupportInitialize)(this.pictureBox3)).EndInit();
      this.ResumeLayout(false);
      this.PerformLayout();

        }

    #endregion

    private System.Windows.Forms.PictureBox kinectVideoBox;
    private System.Windows.Forms.CheckBox skeletonTrackingCheckbox;
    private System.Windows.Forms.CheckBox faceTrackingCheckbox;
    private System.Windows.Forms.CheckBox headTrackingCheckbox;
    private System.Windows.Forms.CheckBox audioTrackingCheckbox;
    private System.Windows.Forms.GroupBox groupBox1;
    private System.Windows.Forms.Label faceHeadY1;
    private System.Windows.Forms.Label faceHeadX1;
    private System.Windows.Forms.Label skeletonHeadY1;
    private System.Windows.Forms.Label skeletonHeadX1;
    private System.Windows.Forms.Label label4;
    private System.Windows.Forms.Label label3;
    private System.Windows.Forms.Label label1;
    private System.Windows.Forms.Label label2;
    private System.Windows.Forms.GroupBox groupBox2;
    private System.Windows.Forms.Label faceHeadY2;
    private System.Windows.Forms.Label faceHeadX2;
    private System.Windows.Forms.Label skeletonHeadY2;
    private System.Windows.Forms.Label skeletonHeadX2;
    private System.Windows.Forms.Label label9;
    private System.Windows.Forms.Label label10;
    private System.Windows.Forms.Label label11;
    private System.Windows.Forms.Label label12;
    private System.Windows.Forms.CheckBox debugOutputCheckbox;
    private System.Windows.Forms.Label speech1;
    private System.Windows.Forms.Label label6;
    private System.Windows.Forms.Label speech2;
    private System.Windows.Forms.Label label7;
    private System.Windows.Forms.Label depth1;
    private System.Windows.Forms.Label label8;
    private System.Windows.Forms.Label depth2;
    private System.Windows.Forms.Label label13;
    private System.Windows.Forms.PictureBox pictureBox1;
    private System.Windows.Forms.PictureBox pictureBox2;
    private System.Windows.Forms.PictureBox pictureBox3;
    private System.Windows.Forms.Button updateApollo;
  }
}


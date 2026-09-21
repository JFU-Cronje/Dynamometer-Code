using System;
using System.IO;
using System.Windows;
using System.IO.Ports;
using System.Windows.Threading;

namespace DynamometerGUI
{
    public partial class MainWindow : Window
    {
        SerialPort arduinoPort;
        DispatcherTimer testTimer;
        DateTime startTime;
        bool isRecording = false;
        StreamWriter csvWriter;

        public MainWindow()
        {
            InitializeComponent();
            
            // Timer Setup
            testTimer = new DispatcherTimer();
            testTimer.Interval = TimeSpan.FromMilliseconds(100);
            testTimer.Tick += TestTimer_Tick;

            // Single Stand Setup
            PlotFrontThrustSingle.Plot.Title("Front Thrust");
            PlotFrontThrustSingle.Plot.YLabel("Thrust (kgf)");
            PlotFrontTorqueSingle.Plot.Title("Front Torque");
            PlotFrontTorqueSingle.Plot.YLabel("Torque (Nm)");
            PlotFrontRPMSingle.Plot.Title("Front RPM");
            PlotFrontRPMSingle.Plot.YLabel("RPM");

            // CRP Setup
            PlotFrontThrustCRP.Plot.Title("Front Thrust");
            PlotFrontThrustCRP.Plot.YLabel("Thrust (kgf)");
            PlotFrontTorqueCRP.Plot.Title("Front Torque");
            PlotFrontTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotFrontRPMCRP.Plot.Title("Front RPM");
            PlotFrontRPMCRP.Plot.YLabel("RPM");
            
            PlotBackThrustCRP.Plot.Title("Back Thrust");
            PlotBackThrustCRP.Plot.YLabel("Thrust (kgf)");
            PlotBackTorqueCRP.Plot.Title("Back Torque");
            PlotBackTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotBackRPMCRP.Plot.Title("Back RPM");
            PlotBackRPMCRP.Plot.YLabel("RPM");

            RefreshAllPlots();
            
            arduinoPort = new SerialPort("COM3", 115200);
            try { arduinoPort.Open(); }
            catch { MessageBox.Show("Could not open COM3. Verify the Arduino Uno is plugged in."); }
        }

        private void TestTimer_Tick(object sender, EventArgs e)
        {
            TimeSpan elapsed = DateTime.Now - startTime;
            TxtTimer.Text = $"Time: {elapsed:mm\\:ss\\.f}";
        }

        private void BtnStart_Click(object sender, RoutedEventArgs e)
        {
            startTime = DateTime.Now;
            testTimer.Start();
            SendCommand("START"); // Adjust this command to match your Arduino code
        }

        private void BtnStop_Click(object sender, RoutedEventArgs e)
        {
            testTimer.Stop();
            if (isRecording)
            {
                BtnRecord_Click(null, null); // Stop recording if active
            }
            SendCommand("STOP"); // Adjust this command to match your Arduino code
        }

        private void BtnRecord_Click(object sender, RoutedEventArgs e)
        {
            if (!isRecording)
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                csvWriter = new StreamWriter($"DynamometerData_{timestamp}.csv");
                csvWriter.WriteLine("Time,Thrust1,Torque1,RPM1,Thrust2,Torque2,RPM2");
                
                isRecording = true;
                BtnRecord.Content = "Stop Rec";
            }
            else
            {
                isRecording = false;
                BtnRecord.Content = "Record";
                csvWriter?.Close();
            }
        }

        private void CmbThrustUnit_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (PlotFrontThrustSingle == null) return; 

            string unitLabel = CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)";
            
            PlotFrontThrustSingle.Plot.YLabel(unitLabel);
            PlotFrontThrustCRP.Plot.YLabel(unitLabel);
            PlotBackThrustCRP.Plot.YLabel(unitLabel);

            RefreshAllPlots();
        }

        private void RefreshAllPlots()
        {
            PlotFrontThrustSingle.Refresh();
            PlotFrontTorqueSingle.Refresh();
            PlotFrontRPMSingle.Refresh();
            
            PlotFrontThrustCRP.Refresh();
            PlotFrontTorqueCRP.Refresh();
            PlotFrontRPMCRP.Refresh();
            
            PlotBackThrustCRP.Refresh();
            PlotBackTorqueCRP.Refresh();
            PlotBackRPMCRP.Refresh();
        }

        public void BtnTareTorque_Click(object sender, RoutedEventArgs e) { SendCommand("A"); }
        public void BtnCalTorque_Click(object sender, RoutedEventArgs e) { SendCommand("B"); }
        public void BtnTareThrust_Click(object sender, RoutedEventArgs e) { SendCommand("C"); }
        public void BtnCalThrust_Click(object sender, RoutedEventArgs e) { SendCommand("D"); }
        public void BtnTareTorqueBack_Click(object sender, RoutedEventArgs e) { SendCommand("E"); }
        public void BtnCalTorqueBack_Click(object sender, RoutedEventArgs e) { SendCommand("F"); }
        public void BtnTareThrustBack_Click(object sender, RoutedEventArgs e) { SendCommand("G"); }
        public void BtnCalThrustBack_Click(object sender, RoutedEventArgs e) { SendCommand("H"); }

        public void SendCommand(string command)
        {
            if (arduinoPort != null && arduinoPort.IsOpen)
            {
                arduinoPort.Write(command);
            }
        }
    }
}
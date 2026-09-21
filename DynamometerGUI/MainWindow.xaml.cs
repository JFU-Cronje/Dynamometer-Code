using System;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Controls;
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
        StreamWriter? csvWriter = null;

        public MainWindow()
        {
            InitializeComponent();
            
            this.SetResourceReference(Window.BackgroundProperty, "MainBg");
            this.SetResourceReference(Window.ForegroundProperty, "MainFg");
            
            testTimer = new DispatcherTimer();
            testTimer.Interval = TimeSpan.FromMilliseconds(10);
            testTimer.Tick += TestTimer_Tick;

            PlotFrontThrustSingle.Plot.Title("Front Thrust");
            PlotFrontThrustSingle.Plot.YLabel("Thrust (kgf)");
            PlotFrontTorqueSingle.Plot.Title("Front Torque");
            PlotFrontTorqueSingle.Plot.YLabel("Torque (Nm)");
            PlotFrontRPMSingle.Plot.Title("Front RPM");
            PlotFrontRPMSingle.Plot.YLabel("RPM");

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

        private void TestTimer_Tick(object? sender, EventArgs e)
        {
            TimeSpan elapsed = DateTime.Now - startTime;
            string timeFormat = $"Time: {elapsed:hh\\:mm\\:ss\\.fff}";
            TxtTimerSingle.Text = timeFormat;
            TxtTimerCRP.Text = timeFormat;
        }

        private void ChkActivateSingle_Checked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = true;
            SendCommand("UNLOCK_SINGLE");
        }

        private void ChkActivateSingle_Unchecked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = false;
            SldThrottleSingle.Value = 0; 
            SendCommand("LOCK_SINGLE");
        }

        private void ChkActivateCRP_Checked(object sender, RoutedEventArgs e)
        {
            SldThrottleFrontCRP.IsEnabled = true;
            SldThrottleBackCRP.IsEnabled = true;
            SendCommand("UNLOCK_CRP");
        }

        private void ChkActivateCRP_Unchecked(object sender, RoutedEventArgs e)
        {
            SldThrottleFrontCRP.IsEnabled = false;
            SldThrottleBackCRP.IsEnabled = false;
            SldThrottleFrontCRP.Value = 0; 
            SldThrottleBackCRP.Value = 0;
            SendCommand("LOCK_CRP");
        }

        private void SldThrottleSingle_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueSingle != null) TxtThrottleValueSingle.Text = e.NewValue.ToString("F4");
            if (TxtInputThrottle1 != null) TxtInputThrottle1.Text = $"Front ESC Throttle: {e.NewValue:F4}";
            SendCommand($"T1_{e.NewValue:F4}");
        }

        private void SldThrottleFrontCRP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueFrontCRP != null) TxtThrottleValueFrontCRP.Text = e.NewValue.ToString("F4");
            if (TxtInputThrottle1 != null) TxtInputThrottle1.Text = $"Front ESC Throttle: {e.NewValue:F4}";
            SendCommand($"T1_{e.NewValue:F4}");
        }

        private void SldThrottleBackCRP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueBackCRP != null) TxtThrottleValueBackCRP.Text = e.NewValue.ToString("F4");
            if (TxtInputThrottle2 != null) TxtInputThrottle2.Text = $"Back ESC Throttle: {e.NewValue:F4}";
            SendCommand($"T2_{e.NewValue:F4}");
        }

        private void BtnRecord_Click(object sender, RoutedEventArgs e)
        {
            if (!isRecording)
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                csvWriter = new StreamWriter($"DynamometerData_{timestamp}.csv");
                csvWriter.WriteLine("Time,Thrust1,Torque1,RPM1,Thrust2,Torque2,RPM2,Voltage,Current,Throttle1,Throttle2,Power");
                
                startTime = DateTime.Now;
                testTimer.Start();
                isRecording = true;
                BtnRecordSingle.Content = "Stop Rec";
                BtnRecordCRP.Content = "Stop Rec";
            }
            else
            {
                testTimer.Stop();
                isRecording = false;
                BtnRecordSingle.Content = "Record";
                BtnRecordCRP.Content = "Record";
                csvWriter?.Close();
            }
        }

        private void CmbTheme_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (PlotFrontThrustSingle == null) return;
            
            bool isDark = CmbTheme.SelectedIndex == 1;

            if (isDark)
            {
                this.Resources["MainBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#1E1E1E"));
                this.Resources["MainFg"] = new SolidColorBrush(Colors.White);
                this.Resources["BtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#2D2D30"));
            }
            else
            {
                this.Resources["MainBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#edf0f4"));
                this.Resources["MainFg"] = new SolidColorBrush(Colors.Black);
                this.Resources["BtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#D0D9E4"));
            }

            SetPlotTheme(isDark);
        }

        private void SetPlotTheme(bool isDark)
        {
            var plots = new[] 
            { 
                PlotFrontThrustSingle, PlotFrontTorqueSingle, PlotFrontRPMSingle, 
                PlotFrontThrustCRP, PlotFrontTorqueCRP, PlotFrontRPMCRP, 
                PlotBackThrustCRP, PlotBackTorqueCRP, PlotBackRPMCRP 
            };

            foreach (var plotControl in plots)
            {
                if (isDark)
                {
                    plotControl.Plot.FigureBackground.Color = ScottPlot.Color.FromHex("#1E1E1E");
                    plotControl.Plot.DataBackground.Color = ScottPlot.Color.FromHex("#1E1E1E");
                    plotControl.Plot.Axes.Color(ScottPlot.Color.FromHex("#FFFFFF"));
                }
                else
                {
                    plotControl.Plot.FigureBackground.Color = ScottPlot.Color.FromHex("#edf0f4");
                    plotControl.Plot.DataBackground.Color = ScottPlot.Colors.White;
                    plotControl.Plot.Axes.Color(ScottPlot.Colors.Black);
                }
            }
            
            RefreshAllPlots();
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
                arduinoPort.Write(command + "\n");
            }
        }
    }
}
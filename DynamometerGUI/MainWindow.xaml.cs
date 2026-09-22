using System;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Controls;
using System.IO.Ports;
using System.Windows.Threading;
using System.Windows.Input;
using System.Linq;
using System.Data;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Windows.Interop;
using System.Windows.Media.Imaging;

namespace DynamometerGUI
{
    public partial class MainWindow : Window
    {
        [DllImport("dwmapi.dll")]
        private static extern int DwmSetWindowAttribute(IntPtr hwnd, int attr, ref int attrValue, int attrSize);

        SerialPort arduinoPort;
        DispatcherTimer testTimer;
        DispatcherTimer stopHoldTimer;
        DateTime startTime;
        DateTime stopPressTime;
        double nextLogInterval;
        bool isRecording = false;
        bool isStopping = false;
        StreamWriter? csvWriter = null;

        // Custom Directories
        string logDir = @"S:\Documents\1_Education\BEng (Mechatronics) @ SU\Y4S2\4_Mechatronic Project 478 - Skripsie\1_26894874_Cronje\10_Software\Dynamometer Code\DynamometerGUI\bin\DynoLogs";
        string calDir = @"S:\Documents\1_Education\BEng (Mechatronics) @ SU\Y4S2\4_Mechatronic Project 478 - Skripsie\1_26894874_Cronje\10_Software\Dynamometer Code\DynamometerGUI\bin\DynoCalibrations";
        
        // Calibration State
        bool isCalibrating = false;
        string currentCalFile = "";
        string tempCalFile = "";

        public MainWindow()
        {
            InitializeComponent();
            
            this.SetResourceReference(Window.BackgroundProperty, "MainBg");
            this.SetResourceReference(Window.ForegroundProperty, "MainFg");
            
            Directory.CreateDirectory(logDir);
            Directory.CreateDirectory(calDir);

            testTimer = new DispatcherTimer();
            testTimer.Interval = TimeSpan.FromMilliseconds(10);
            testTimer.Tick += TestTimer_Tick;

            stopHoldTimer = new DispatcherTimer();
            stopHoldTimer.Interval = TimeSpan.FromMilliseconds(50);
            stopHoldTimer.Tick += StopHoldTimer_Tick;

            InitializeGraphAxes();
            RefreshAllPlots();
            LoadLogFiles();
            
            arduinoPort = new SerialPort("COM3", 115200);
            try { arduinoPort.Open(); }
            catch { MessageBox.Show("Could not open COM3. Verify the Arduino Uno is plugged in."); }
        }

        private void InitializeGraphAxes()
        {
            var plots = new[] 
            { 
                PlotFrontThrustSingle, PlotFrontTorqueSingle, PlotFrontRPMSingle, 
                PlotFrontThrustCRP, PlotFrontTorqueCRP, PlotFrontRPMCRP, 
                PlotBackThrustCRP, PlotBackTorqueCRP, PlotBackRPMCRP 
            };

            foreach (var p in plots)
            {
                p.Plot.Axes.SetLimitsX(0, 20);
                
                var anno = p.Plot.Add.Annotation("Time (s)", ScottPlot.Alignment.LowerRight);
                anno.LabelBorderColor = ScottPlot.Colors.Transparent;
                anno.LabelBackgroundColor = ScottPlot.Colors.Transparent;
                anno.LabelShadowColor = ScottPlot.Colors.Transparent;
                anno.LabelFontColor = ScottPlot.Colors.Gray;
            }

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
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            ApplyTitleBarTheme(CmbTheme.SelectedIndex == 1);
        }

        private void ApplyTitleBarTheme(bool isDark)
        {
            try
            {
                var hwnd = new WindowInteropHelper(this).Handle;
                int useImmersiveDarkMode = isDark ? 1 : 0;
                DwmSetWindowAttribute(hwnd, 20, ref useImmersiveDarkMode, sizeof(int));
            }
            catch { }
        }

        private void TestTimer_Tick(object? sender, EventArgs e)
        {
            TimeSpan elapsed = DateTime.Now - startTime;
            string timeFormat = $"Time: {elapsed:hh\\:mm\\:ss\\.fff}";
            TxtTimerSingle.Text = timeFormat;
            TxtTimerCRP.Text = timeFormat;

            if (isRecording && csvWriter != null)
            {
                if (elapsed.TotalSeconds >= nextLogInterval)
                {
                    TimeSpan exactLogTime = TimeSpan.FromSeconds(nextLogInterval);
                    string elapsedLogTime = exactLogTime.ToString(@"hh\:mm\:ss\.fff");
                    
                    double throttle1 = TabSingle.IsSelected ? SldThrottleSingle.Value : SldThrottleFrontCRP.Value;
                    double throttle2 = TabSingle.IsSelected ? 0 : SldThrottleBackCRP.Value;
                    
                    string t1 = throttle1.ToString("F4", CultureInfo.InvariantCulture);
                    string t2 = throttle2.ToString("F4", CultureInfo.InvariantCulture);
                    
                    csvWriter.WriteLine($"{elapsedLogTime},0,0,0,0,0,{t1},0,0,0,0,0,0,0,0,0,0,{t2},0,0,0,0,0");
                    nextLogInterval += 0.5;
                }
            }
        }

        private void StopRecording()
        {
            if (isRecording)
            {
                testTimer.Stop();
                isRecording = false;
                BtnRecordSingle.Content = "Record";
                BtnRecordCRP.Content = "Record";
                csvWriter?.Close();
                LoadLogFiles();
            }
        }

        // --- Emergency Stop Logic ---
        
        private void StopHoldTimer_Tick(object? sender, EventArgs e)
        {
            TimeSpan elapsed = DateTime.Now - stopPressTime;
            double remaining = 3.0 - elapsed.TotalSeconds;

            if (remaining <= 0)
            {
                CancelStopPress();
                ExecuteEmergencyStop();
            }
            else
            {
                TxtStopCountdown.Text = remaining.ToString("F1", CultureInfo.InvariantCulture);
            }
        }

        private void StartStopPress()
        {
            if (!isStopping)
            {
                isStopping = true;
                StopBtnTransform.Y = 4;
                TxtStopCountdown.Visibility = Visibility.Visible;
                stopPressTime = DateTime.Now;
                TxtStopCountdown.Text = "3.0";
                stopHoldTimer.Start();
            }
        }

        private void CancelStopPress()
        {
            if (isStopping)
            {
                isStopping = false;
                stopHoldTimer.Stop();
                TxtStopCountdown.Visibility = Visibility.Hidden;
                StopBtnTransform.Y = 0;
            }
        }

        private void ExecuteEmergencyStop()
        {
            ChkActivateSingle.IsChecked = false;
            ChkActivateCRP.IsChecked = false;
            StopRecording();
            SendCommand("STOP_ALL");
        }

        private void BtnStop_MouseDown(object sender, MouseButtonEventArgs e)
        {
            StartStopPress();
        }

        private void BtnStop_MouseUp(object sender, MouseButtonEventArgs e)
        {
            CancelStopPress();
        }

        private void BtnStop_MouseLeave(object sender, MouseEventArgs e)
        {
            CancelStopPress();
        }

        private void Window_PreviewKeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Space && !e.IsRepeat)
            {
                StartStopPress();
                e.Handled = true; 
            }
        }

        private void Window_PreviewKeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Space)
            {
                CancelStopPress();
                e.Handled = true;
            }
        }

        // --- Tab, Validation, and UI Isolation Logic ---

        private void MainTabControl_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (e.Source is TabControl && PanelLiveDataInputsRoot != null && PanelLiveDataDerivedRoot != null && TabCRP != null && TabLogs != null)
            {
                if (TabLogs.IsSelected)
                {
                    PanelLiveDataInputsRoot.Visibility = Visibility.Collapsed;
                    PanelLiveDataDerivedRoot.Visibility = Visibility.Collapsed;
                }
                else
                {
                    PanelLiveDataInputsRoot.Visibility = Visibility.Visible;
                    PanelLiveDataDerivedRoot.Visibility = Visibility.Visible;
                    
                    bool isCRP = TabCRP.IsSelected;
                    PanelBackDataInputs.Visibility = isCRP ? Visibility.Visible : Visibility.Collapsed;
                    PanelBackDataDerived.Visibility = isCRP ? Visibility.Visible : Visibility.Collapsed;
                }
            }
        }

        private void LoadLogFiles()
        {
            if (ListLogFiles == null) return;
            ListLogFiles.Items.Clear();
            
            var files = Directory.GetFiles(logDir, "DynamometerData_*.csv").OrderByDescending(f => f).ToList();
            
            foreach (var f in files)
            {
                ListLogFiles.Items.Add(Path.GetFileName(f));
            }
        }

        private void ListLogFiles_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (ListLogFiles.SelectedItem == null) return;
            
            string file = Path.Combine(logDir, ListLogFiles.SelectedItem.ToString() ?? "");
            if (!File.Exists(file)) return;

            string[] lines = File.ReadAllLines(file);
            if (lines.Length == 0) return;

            var dt = new DataTable();
            string[] headers = lines[0].Split(',');
            foreach (string header in headers)
            {
                dt.Columns.Add(header);
            }

            for (int i = 1; i < lines.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(lines[i])) continue;
                
                string[] parts = lines[i].Split(',');
                
                if (parts.Length > dt.Columns.Count)
                {
                    dt.Rows.Add(parts.Take(dt.Columns.Count).ToArray());
                }
                else if (parts.Length < dt.Columns.Count)
                {
                    var padded = parts.ToList();
                    while (padded.Count < dt.Columns.Count) padded.Add("");
                    dt.Rows.Add(padded.ToArray());
                }
                else
                {
                    dt.Rows.Add(parts);
                }
            }
            
            DataGridLogs.ItemsSource = dt.DefaultView;
        }

        private void BtnOpenLogsFolder_Click(object sender, RoutedEventArgs e)
        {
            Process.Start(new ProcessStartInfo
            {
                FileName = logDir,
                UseShellExecute = true
            });
        }

        // --- Calibration Logic ---

        private void SetCalibrationButtonsState(bool enabled)
        {
            BtnTareTorqueFront1.IsEnabled = enabled;
            BtnCalTorqueFront1.IsEnabled = enabled;
            BtnTareThrustFront1.IsEnabled = enabled;
            BtnCalThrustFront1.IsEnabled = enabled;
            
            BtnTareTorqueFront2.IsEnabled = enabled;
            BtnCalTorqueFront2.IsEnabled = enabled;
            BtnTareThrustFront2.IsEnabled = enabled;
            BtnCalThrustFront2.IsEnabled = enabled;
            
            BtnTareTorqueBack.IsEnabled = enabled;
            BtnCalTorqueBack.IsEnabled = enabled;
            BtnTareThrustBack.IsEnabled = enabled;
            BtnCalThrustBack.IsEnabled = enabled;

            BtnStartCalSingle.IsEnabled = !enabled;
            BtnFinishCalSingle.IsEnabled = enabled;
            BtnStartCalCRP.IsEnabled = !enabled;
            BtnFinishCalCRP.IsEnabled = enabled;

            // Lock out tabs so user cannot switch mid-calibration
            TabSingle.IsEnabled = !enabled || TabSingle.IsSelected;
            TabCRP.IsEnabled = !enabled || TabCRP.IsSelected;
            TabLogs.IsEnabled = !enabled;
        }

        private void BtnStartCal_Click(object sender, RoutedEventArgs e)
        {
            isCalibrating = true;
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            tempCalFile = Path.Combine(calDir, $"Calibration_{timestamp}.csv");
            
            File.WriteAllText(tempCalFile, "ActionLog,Thrust1,Torque1,Thrust2,Torque2,ScaleX,ScaleY\nInit,1,1,1,1,1,1\n");
            SetCalibrationButtonsState(true);
        }

        private void BtnFinishCal_Click(object sender, RoutedEventArgs e)
        {
            isCalibrating = false;
            currentCalFile = tempCalFile;
            SetCalibrationButtonsState(false);
            MessageBox.Show("Calibration saved to:\n" + currentCalFile, "Calibration Complete");
        }

        private void LogCalibrationAction(string action)
        {
            if (isCalibrating && !string.IsNullOrEmpty(tempCalFile))
            {
                File.AppendAllText(tempCalFile, $"{action},1,1,1,1,1,1\n");
            }
        }

        public void BtnTareTorque_Click(object sender, RoutedEventArgs e) { SendCommand("A"); LogCalibrationAction("TareFrontTorque"); }
        public void BtnCalTorque_Click(object sender, RoutedEventArgs e) { SendCommand("B"); LogCalibrationAction("CalFrontTorque"); }
        public void BtnTareThrust_Click(object sender, RoutedEventArgs e) { SendCommand("C"); LogCalibrationAction("TareFrontThrust"); }
        public void BtnCalThrust_Click(object sender, RoutedEventArgs e) { SendCommand("D"); LogCalibrationAction("CalFrontThrust"); }
        public void BtnTareTorqueBack_Click(object sender, RoutedEventArgs e) { SendCommand("E"); LogCalibrationAction("TareBackTorque"); }
        public void BtnCalTorqueBack_Click(object sender, RoutedEventArgs e) { SendCommand("F"); LogCalibrationAction("CalBackTorque"); }
        public void BtnTareThrustBack_Click(object sender, RoutedEventArgs e) { SendCommand("G"); LogCalibrationAction("TareBackThrust"); }
        public void BtnCalThrustBack_Click(object sender, RoutedEventArgs e) { SendCommand("H"); LogCalibrationAction("CalBackThrust"); }

        // --- Controls Logic ---

        private void ChkActivateSingle_Checked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = true;
            BtnRecordSingle.IsEnabled = true;
            TabCRP.IsEnabled = false; 
            TabLogs.IsEnabled = false;
            SendCommand("UNLOCK_SINGLE");
        }

        private void ChkActivateSingle_Unchecked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = false;
            SldThrottleSingle.Value = 0; 
            BtnRecordSingle.IsEnabled = false;
            TabCRP.IsEnabled = true; 
            TabLogs.IsEnabled = true;
            StopRecording();
            SendCommand("LOCK_SINGLE");
        }

        private void ChkActivateCRP_Checked(object sender, RoutedEventArgs e)
        {
            SldThrottleFrontCRP.IsEnabled = true;
            SldThrottleBackCRP.IsEnabled = true;
            BtnRecordCRP.IsEnabled = true;
            TabSingle.IsEnabled = false;
            TabLogs.IsEnabled = false;
            SendCommand("UNLOCK_CRP");
        }

        private void ChkActivateCRP_Unchecked(object sender, RoutedEventArgs e)
        {
            SldThrottleFrontCRP.IsEnabled = false;
            SldThrottleBackCRP.IsEnabled = false;
            SldThrottleFrontCRP.Value = 0; 
            SldThrottleBackCRP.Value = 0;
            BtnRecordCRP.IsEnabled = false;
            TabSingle.IsEnabled = true; 
            TabLogs.IsEnabled = true;
            StopRecording();
            SendCommand("LOCK_CRP");
        }

        private void SldThrottleSingle_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueSingle != null) TxtThrottleValueSingle.Text = e.NewValue.ToString("F4");
            if (TxtInputThrottle1 != null) TxtInputThrottle1.Text = $"ESC Throttle: {e.NewValue:F4}";
            SendCommand($"T1_{e.NewValue.ToString("F4", CultureInfo.InvariantCulture)}");
        }

        private void SldThrottleFrontCRP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueFrontCRP != null) TxtThrottleValueFrontCRP.Text = e.NewValue.ToString("F4");
            if (TxtInputThrottle1 != null) TxtInputThrottle1.Text = $"ESC Throttle: {e.NewValue:F4}";
            SendCommand($"T1_{e.NewValue.ToString("F4", CultureInfo.InvariantCulture)}");
        }

        private void SldThrottleBackCRP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueBackCRP != null) TxtThrottleValueBackCRP.Text = e.NewValue.ToString("F4");
            if (TxtInputThrottle2 != null) TxtInputThrottle2.Text = $"ESC Throttle: {e.NewValue:F4}";
            SendCommand($"T2_{e.NewValue.ToString("F4", CultureInfo.InvariantCulture)}");
        }

        private void BtnRecord_Click(object sender, RoutedEventArgs e)
        {
            if (!isRecording)
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                csvWriter = new StreamWriter(Path.Combine(logDir, $"DynamometerData_{timestamp}.csv"));
                csvWriter.WriteLine("Time,Thrust1,Torque1,RPM1,Voltage1,Current1,Throttle1,P_BAT-ESC1,P_ESC-MOT1,ElecEff1,P_Mech1,MechEff1,Thrust2,Torque2,RPM2,Voltage2,Current2,Throttle2,P_BAT-ESC2,P_ESC-MOT2,ElecEff2,P_Mech2,MechEff2");
                
                startTime = DateTime.Now;
                nextLogInterval = 0.5;
                testTimer.Start();
                isRecording = true;
                BtnRecordSingle.Content = "Stop Rec";
                BtnRecordCRP.Content = "Stop Rec";
            }
            else
            {
                StopRecording();
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
                this.Resources["BtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#3E3E42"));
                this.Resources["HoverBtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#555555"));
                this.Resources["DisabledBtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#2D2D30"));
                this.Resources["ControlBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#2D2D30"));
                
                if (ImgLogo != null)
                {
                    try { ImgLogo.Source = new BitmapImage(new Uri("pack://application:,,,/Dynamometer Logo NoText DarkMode NoBackground.png")); } catch { }
                }
            }
            else
            {
                this.Resources["MainBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#edf0f4"));
                this.Resources["MainFg"] = new SolidColorBrush(Colors.Black);
                this.Resources["BtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#D0D9E4"));
                this.Resources["HoverBtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#B8C5D1"));
                this.Resources["DisabledBtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#cccccc"));
                this.Resources["ControlBg"] = new SolidColorBrush(Colors.White);

                if (ImgLogo != null)
                {
                    try { ImgLogo.Source = new BitmapImage(new Uri("pack://application:,,,/Dynamometer Logo NoText NoBackground.png")); } catch { }
                }
            }

            SetPlotTheme(isDark);
            ApplyTitleBarTheme(isDark);
        }

        private void SetPlotTheme(bool isDark)
        {
            var plots = new[] 
            { 
                PlotFrontThrustSingle, PlotFrontTorqueSingle, PlotFrontRPMSingle, 
                PlotFrontThrustCRP, PlotFrontTorqueCRP, PlotFrontRPMCRP, 
                PlotBackThrustCRP, PlotBackTorqueCRP, PlotBackRPMCRP 
            };

            foreach (var p in plots)
            {
                p.Plot.Axes.AutoScaleY();
                
                if (isDark)
                {
                    p.Plot.FigureBackground.Color = ScottPlot.Color.FromHex("#1E1E1E");
                    p.Plot.DataBackground.Color = ScottPlot.Color.FromHex("#1E1E1E");
                    p.Plot.Axes.Color(ScottPlot.Color.FromHex("#FFFFFF"));
                    p.Plot.Grid.MajorLineColor = ScottPlot.Color.FromHex("#444444");
                }
                else
                {
                    p.Plot.FigureBackground.Color = ScottPlot.Color.FromHex("#edf0f4");
                    p.Plot.DataBackground.Color = ScottPlot.Colors.White;
                    p.Plot.Axes.Color(ScottPlot.Colors.Black);
                    p.Plot.Grid.MajorLineColor = ScottPlot.Color.FromHex("#DDDDDD");
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

        public void SendCommand(string command)
        {
            if (arduinoPort != null && arduinoPort.IsOpen)
            {
                arduinoPort.Write(command + "\n");
            }
        }
    }
}
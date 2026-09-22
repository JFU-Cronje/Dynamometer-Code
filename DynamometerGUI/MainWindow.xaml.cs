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
        double thrust1, torque1, rpm1, volt1, curr1;
        double thrust2, torque2, rpm2, volt2, curr2;
        double pBatEsc1, pMech1, mechEff1;
        double pBatEsc2, pMech2, mechEff2;

        // Graph Loggers
        ScottPlot.Plottables.DataLogger dlFrontThrustSingle = null!, dlFrontTorqueSingle = null!, dlFrontRPMSingle = null!;
        ScottPlot.Plottables.DataLogger dlFrontThrustCRP = null!, dlFrontTorqueCRP = null!, dlFrontRPMCRP = null!;
        ScottPlot.Plottables.DataLogger dlBackThrustCRP = null!, dlBackTorqueCRP = null!, dlBackRPMCRP = null!;
        double liveGraphTime = 0;

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
            testTimer.Interval = TimeSpan.FromMilliseconds(50);
            testTimer.Tick += TestTimer_Tick;
            testTimer.Start(); // Run continuously for live graphing

            stopHoldTimer = new DispatcherTimer();
            stopHoldTimer.Interval = TimeSpan.FromMilliseconds(50);
            stopHoldTimer.Tick += StopHoldTimer_Tick;

            InitializeGraphAxes();
            RefreshAllPlots();
            LoadLogFiles();
            
            arduinoPort = new SerialPort("COM3", 115200);
            try 
            {
                 arduinoPort.DtrEnable = true;
                 arduinoPort.Open();
                 arduinoPort.DataReceived += ArduinoPort_DataReceived;
            }
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
                p.Plot.Axes.SetLimitsX(0, 60);
                
                // Force X-axis to display 5-second intervals (approx 12 ticks over a 60s span)
                var tickGen = new ScottPlot.TickGenerators.NumericAutomatic();
                tickGen.TargetTickCount = 12;
                p.Plot.Axes.Bottom.TickGenerator = tickGen;
                
                var anno = p.Plot.Add.Annotation("Time (s)", ScottPlot.Alignment.LowerRight);
                anno.LabelBorderColor = ScottPlot.Colors.Transparent;
                anno.LabelBackgroundColor = ScottPlot.Colors.Transparent;
                anno.LabelShadowColor = ScottPlot.Colors.Transparent;
                anno.LabelFontColor = ScottPlot.Colors.Gray;
            }

            dlFrontThrustSingle = PlotFrontThrustSingle.Plot.Add.DataLogger();
            dlFrontTorqueSingle = PlotFrontTorqueSingle.Plot.Add.DataLogger();
            dlFrontRPMSingle = PlotFrontRPMSingle.Plot.Add.DataLogger();
            
            dlFrontThrustCRP = PlotFrontThrustCRP.Plot.Add.DataLogger();
            dlFrontTorqueCRP = PlotFrontTorqueCRP.Plot.Add.DataLogger();
            dlFrontRPMCRP = PlotFrontRPMCRP.Plot.Add.DataLogger();
            
            dlBackThrustCRP = PlotBackThrustCRP.Plot.Add.DataLogger();
            dlBackTorqueCRP = PlotBackTorqueCRP.Plot.Add.DataLogger();
            dlBackRPMCRP = PlotBackRPMCRP.Plot.Add.DataLogger();

            var loggers = new[] { dlFrontThrustSingle, dlFrontTorqueSingle, dlFrontRPMSingle, 
                                  dlFrontThrustCRP, dlFrontTorqueCRP, dlFrontRPMCRP, 
                                  dlBackThrustCRP, dlBackTorqueCRP, dlBackRPMCRP };
            
            foreach (var l in loggers)
            {
                l.ManageAxisLimits = false; 
            }

            PlotFrontThrustSingle.Plot.Title("Front Thrust");
            PlotFrontThrustSingle.Plot.YLabel("Thrust (kgf)");
            PlotFrontThrustSingle.Plot.Axes.SetLimitsY(-1, 5); 

            PlotFrontTorqueSingle.Plot.Title("Front Torque");
            PlotFrontTorqueSingle.Plot.YLabel("Torque (Nm)");
            PlotFrontTorqueSingle.Plot.Axes.SetLimitsY(-0.1, 1);

            PlotFrontRPMSingle.Plot.Title("Front RPM");
            PlotFrontRPMSingle.Plot.YLabel("RPM");
            PlotFrontRPMSingle.Plot.Axes.SetLimitsY(-500, 10000);

            PlotFrontThrustCRP.Plot.Title("Front Thrust");
            PlotFrontThrustCRP.Plot.YLabel("Thrust (kgf)");
            PlotFrontThrustCRP.Plot.Axes.SetLimitsY(-1, 5);

            PlotFrontTorqueCRP.Plot.Title("Front Torque");
            PlotFrontTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotFrontTorqueCRP.Plot.Axes.SetLimitsY(-0.1, 1);

            PlotFrontRPMCRP.Plot.Title("Front RPM");
            PlotFrontRPMCRP.Plot.YLabel("RPM");
            PlotFrontRPMCRP.Plot.Axes.SetLimitsY(-500, 10000);
            
            PlotBackThrustCRP.Plot.Title("Back Thrust");
            PlotBackThrustCRP.Plot.YLabel("Thrust (kgf)");
            PlotBackThrustCRP.Plot.Axes.SetLimitsY(-1, 5);

            PlotBackTorqueCRP.Plot.Title("Back Torque");
            PlotBackTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotBackTorqueCRP.Plot.Axes.SetLimitsY(-0.1, 1);

            PlotBackRPMCRP.Plot.Title("Back RPM");
            PlotBackRPMCRP.Plot.YLabel("RPM");
            PlotBackRPMCRP.Plot.Axes.SetLimitsY(-500, 10000);
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            ApplyTitleBarTheme(CmbTheme.SelectedIndex == 1);
        }

        private void ArduinoPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string line = arduinoPort.ReadLine().Trim();
                string[] parts = line.Split(',');

                if (parts.Length == 10)
                {
                    thrust1 = double.Parse(parts[0], CultureInfo.InvariantCulture);
                    torque1 = double.Parse(parts[1], CultureInfo.InvariantCulture);
                    rpm1 = double.Parse(parts[2], CultureInfo.InvariantCulture);
                    volt1 = double.Parse(parts[3], CultureInfo.InvariantCulture);
                    curr1 = double.Parse(parts[4], CultureInfo.InvariantCulture);

                    thrust2 = double.Parse(parts[5], CultureInfo.InvariantCulture);
                    torque2 = double.Parse(parts[6], CultureInfo.InvariantCulture);
                    rpm2 = double.Parse(parts[7], CultureInfo.InvariantCulture);
                    volt2 = double.Parse(parts[8], CultureInfo.InvariantCulture);
                    curr2 = double.Parse(parts[9], CultureInfo.InvariantCulture);

                    // Derived calculations
                    pBatEsc1 = volt1 * curr1;
                    pMech1 = torque1 * rpm1 * (2.0 * Math.PI / 60.0);
                    mechEff1 = pBatEsc1 > 0 ? (pMech1 / pBatEsc1) * 100.0 : 0;

                    pBatEsc2 = volt2 * curr2;
                    pMech2 = torque2 * rpm2 * (2.0 * Math.PI / 60.0);
                    mechEff2 = pBatEsc2 > 0 ? (pMech2 / pBatEsc2) * 100.0 : 0;

                    // Push updates to the UI thread
                    Dispatcher.Invoke(() =>
                    {
                        liveGraphTime += 0.1; 

                        double displayThrust1 = CmbThrustUnit.SelectedIndex == 0 ? thrust1 / 9.80665 : thrust1;
                        double displayThrust2 = CmbThrustUnit.SelectedIndex == 0 ? thrust2 / 9.80665 : thrust2;

                        dlFrontThrustSingle?.Add(liveGraphTime, displayThrust1);
                        dlFrontTorqueSingle?.Add(liveGraphTime, torque1);
                        dlFrontRPMSingle?.Add(liveGraphTime, rpm1);
                        
                        dlFrontThrustCRP?.Add(liveGraphTime, displayThrust1);
                        dlFrontTorqueCRP?.Add(liveGraphTime, torque1);
                        dlFrontRPMCRP?.Add(liveGraphTime, rpm1);
                        
                        dlBackThrustCRP?.Add(liveGraphTime, displayThrust2);
                        dlBackTorqueCRP?.Add(liveGraphTime, torque2);
                        dlBackRPMCRP?.Add(liveGraphTime, rpm2);

                        // Center the X axis after 30s
                        double minX = liveGraphTime < 30 ? 0 : liveGraphTime - 30;
                        double maxX = liveGraphTime < 30 ? 60 : liveGraphTime + 30;

                        var plots = new[] { PlotFrontThrustSingle, PlotFrontTorqueSingle, PlotFrontRPMSingle, PlotFrontThrustCRP, PlotFrontTorqueCRP, PlotFrontRPMCRP, PlotBackThrustCRP, PlotBackTorqueCRP, PlotBackRPMCRP };
                        var currentVals = new[] { displayThrust1, torque1, rpm1, displayThrust1, torque1, rpm1, displayThrust2, torque2, rpm2 };

                        for (int i = 0; i < plots.Length; i++)
                        {
                            var p = plots[i];
                            if (p != null)
                            {
                                p.Plot.Axes.SetLimitsX(minX, maxX);
                                
                                // Auto-expand Y-axis if data breaches the current visible boundaries
                                var yRange = p.Plot.Axes.GetLimits().Rect.YRange;
                                if (currentVals[i] >= yRange.Max || currentVals[i] <= yRange.Min)
                                {
                                    p.Plot.Axes.AutoScaleY();
                                }
                            }
                        }

                        TxtFrontVoltage.Text = $"Voltage: {volt1:F2} V";
                        TxtFrontCurrent.Text = $"Current: {curr1:F2} A";
                        TxtFrontRPM.Text = $"RPM: {rpm1:F0}";
                        TxtFrontPBatEsc.Text = $"P_BAT-ESC: {pBatEsc1:F2} W";
                        TxtFrontMechPower.Text = $"Mech Power: {pMech1:F2} W";
                        TxtFrontMechEff.Text = $"Mech Eff: {mechEff1:F1} %";

                        TxtBackVoltage.Text = $"Voltage: {volt2:F2} V";
                        TxtBackCurrent.Text = $"Current: {curr2:F2} A";
                        TxtBackRPM.Text = $"RPM: {rpm2:F0}";
                        TxtBackPBatEsc.Text = $"P_BAT-ESC: {pBatEsc2:F2} W";
                        TxtBackMechPower.Text = $"Mech Power: {pMech2:F2} W";
                        TxtBackMechEff.Text = $"Mech Eff: {mechEff2:F1} %";
                    });
                }
            }
            catch { }
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
            if (isRecording)
            {
                TimeSpan elapsed = DateTime.Now - startTime;
                string timeFormat = $"Time: {elapsed:hh\\:mm\\:ss\\.fff}";
                TxtTimerSingle.Text = timeFormat;
                TxtTimerCRP.Text = timeFormat;

                if (csvWriter != null && elapsed.TotalSeconds >= nextLogInterval)
                {
                    TimeSpan exactLogTime = TimeSpan.FromSeconds(nextLogInterval);
                    string elapsedLogTime = exactLogTime.ToString(@"hh\:mm\:ss\.fff");
                    
                    double throttle1 = TabSingle.IsSelected ? SldThrottleSingle.Value : SldThrottleFrontCRP.Value;
                    double throttle2 = TabCRP.IsSelected ? SldThrottleBackCRP.Value : 0;
                    
                    string t1 = throttle1.ToString("F4", CultureInfo.InvariantCulture);
                    string t2 = throttle2.ToString("F4", CultureInfo.InvariantCulture);

                    bool isCRP = TabCRP.IsSelected;
                    double logThrust2 = isCRP ? thrust2 : 0;
                    double logTorque2 = isCRP ? torque2 : 0;
                    double logRpm2 = isCRP ? rpm2 : 0;
                    double logVolt2 = isCRP ? volt2 : 0;
                    double logCurr2 = isCRP ? curr2 : 0;
                    double logPBatEsc2 = isCRP ? pBatEsc2 : 0;
                    double logPMech2 = isCRP ? pMech2 : 0;
                    double logMechEff2 = isCRP ? mechEff2 : 0;
                    
                    string csvLine = string.Format(CultureInfo.InvariantCulture, 
                        "{0},{1:F3},{2:F3},{3:F0},{4:F2},{5:F2},{6},{7:F2},0.00,0.0,{8:F2},{9:F1},{10:F3},{11:F3},{12:F0},{13:F2},{14:F2},{15},{16:F2},0.00,0.0,{17:F2},{18:F1}",
                        elapsedLogTime, 
                        thrust1, torque1, rpm1, volt1, curr1, t1, pBatEsc1, pMech1, mechEff1,
                        logThrust2, logTorque2, logRpm2, logVolt2, logCurr2, t2, logPBatEsc2, logPMech2, logMechEff2);

                    csvWriter.WriteLine(csvLine);
                    nextLogInterval += 0.5;
                }
            }

            RefreshAllPlots();
        }

        private void StopRecording()
        {
            if (isRecording)
            {
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
                    // Use Hidden instead of Collapsed to reserve the layout space and prevent vertical jumping
                    PanelLiveDataInputsRoot.Visibility = Visibility.Hidden;
                    PanelLiveDataDerivedRoot.Visibility = Visibility.Hidden;
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
                csvWriter.AutoFlush = true;
                csvWriter.WriteLine("Time,Thrust1,Torque1,RPM1,Voltage1,Current1,Throttle1,P_BAT-ESC1,P_ESC-MOT1,ElecEff1,P_Mech1,MechEff1,Thrust2,Torque2,RPM2,Voltage2,Current2,Throttle2,P_BAT-ESC2,P_ESC-MOT2,ElecEff2,P_Mech2,MechEff2");
                
                startTime = DateTime.Now;
                nextLogInterval = 0.5;
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

            var loggers = new[] 
            { 
                dlFrontThrustSingle, dlFrontTorqueSingle, dlFrontRPMSingle, 
                dlFrontThrustCRP, dlFrontTorqueCRP, dlFrontRPMCRP, 
                dlBackThrustCRP, dlBackTorqueCRP, dlBackRPMCRP 
            };

            for (int i = 0; i < plots.Length; i++)
            {
                var p = plots[i];
                var l = loggers[i];
                
                if (isDark)
                {
                    p.Plot.FigureBackground.Color = ScottPlot.Color.FromHex("#1E1E1E");
                    p.Plot.DataBackground.Color = ScottPlot.Color.FromHex("#1E1E1E");
                    p.Plot.Axes.Color(ScottPlot.Color.FromHex("#FFFFFF"));
                    p.Plot.Grid.MajorLineColor = ScottPlot.Color.FromHex("#444444");
                    
                    if (l != null) l.Color = ScottPlot.Color.FromHex("#FFC107"); // High-contrast amber
                }
                else
                {
                    p.Plot.FigureBackground.Color = ScottPlot.Color.FromHex("#edf0f4");
                    p.Plot.DataBackground.Color = ScottPlot.Colors.White;
                    p.Plot.Axes.Color(ScottPlot.Colors.Black);
                    p.Plot.Grid.MajorLineColor = ScottPlot.Color.FromHex("#DDDDDD");
                    
                    if (l != null) l.Color = ScottPlot.Colors.Blue;
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
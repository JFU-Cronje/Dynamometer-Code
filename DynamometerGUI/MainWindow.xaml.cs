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
        
        // Data variables
        double thrust1, torque1, rpm1, volt1, curr1;
        double thrust2, torque2, rpm2, volt2, curr2;
        double elecPower1, mechPower1, motorEff1;
        double elecPower2, mechPower2, motorEff2;

        // Throttle variables
        double targetThrottle1 = 0;
        double actualThrottle1 = 0;
        double targetThrottle2 = 0;
        double actualThrottle2 = 0;
        const double RAMP_RATE = 1.0; 

        // Automatic testing state
        bool isAutoRunning = false;
        int autoPhase = 0;
        double autoTimer = 0;
        int autoTotalSteps = 5;
        double autoStepDuration = 2.0;
        double[] autoTargetsFront = new double[6];
        double[] autoTargetsBack = new double[6];

        // Graph Loggers
        ScottPlot.Plottables.DataLogger dlFrontThrustSingle = null!, dlFrontTorqueSingle = null!, dlFrontRPMSingle = null!;
        ScottPlot.Plottables.DataLogger dlFrontThrustCRP = null!, dlFrontTorqueCRP = null!, dlFrontRPMCRP = null!;
        ScottPlot.Plottables.DataLogger dlBackThrustCRP = null!, dlBackTorqueCRP = null!, dlBackRPMCRP = null!;
        double liveGraphTime = 0;

        // Application Local Directories
        string logDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoLogs");
        string calSingleDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoCalibrationsSingle");
        string calCRPDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoCalibrationsCRP");
        
        // Calibration State
        bool isCalibrating = false;
        double scaleT1 = 1, scaleQ1 = 1, scaleT2 = 1, scaleQ2 = 1;

        public MainWindow()
        {
            InitializeComponent();
            
            this.SetResourceReference(Window.BackgroundProperty, "MainBg");
            this.SetResourceReference(Window.ForegroundProperty, "MainFg");
            
            Directory.CreateDirectory(logDir);
            Directory.CreateDirectory(calSingleDir);
            Directory.CreateDirectory(calCRPDir);

            testTimer = new DispatcherTimer();
            testTimer.Interval = TimeSpan.FromMilliseconds(50);
            testTimer.Tick += TestTimer_Tick;
            testTimer.Start();

            stopHoldTimer = new DispatcherTimer();
            stopHoldTimer.Interval = TimeSpan.FromMilliseconds(50);
            stopHoldTimer.Tick += StopHoldTimer_Tick;

            InitializeGraphAxes();
            LoadLogFiles();
            
            AutoConnectArduino(); 
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            ApplyTitleBarTheme(CmbTheme.SelectedIndex == 1);
            testTimer.Start();
        }

        private void AutoConnectArduino()
        {
            foreach (string portName in SerialPort.GetPortNames().Reverse())
            {
                try
                {
                    arduinoPort = new SerialPort(portName, 115200);
                    arduinoPort.WriteTimeout = 50; 
                    arduinoPort.ReadTimeout = 500;
                    arduinoPort.DtrEnable = true;
                    arduinoPort.Open();
                    
                    arduinoPort.DataReceived += ArduinoPort_DataReceived;
                    LoadLatestCalibration(); 
                    return; 
                }
                catch { }
            }
            MessageBox.Show("Could not automatically find the Arduino. Please check your USB connection.");
        }

        private void LoadLatestCalibration()
        {
            string targetDir = TabSingle.IsSelected ? calSingleDir : calCRPDir;
            var latestFile = Directory.GetFiles(targetDir, "Calibration_*.csv").OrderByDescending(f => f).FirstOrDefault();

            if (latestFile != null)
            {
                string[] lines = File.ReadAllLines(latestFile);
                if (lines.Length > 1)
                {
                    string[] parts = lines[1].Split(',');
                    if (parts.Length >= 4)
                    {
                        scaleT1 = double.Parse(parts[0], CultureInfo.InvariantCulture);
                        scaleQ1 = double.Parse(parts[1], CultureInfo.InvariantCulture);
                        scaleT2 = double.Parse(parts[2], CultureInfo.InvariantCulture);
                        scaleQ2 = double.Parse(parts[3], CultureInfo.InvariantCulture);

                        SendCommand($"S_T1_{scaleT1.ToString("F4", CultureInfo.InvariantCulture)}");
                        SendCommand($"S_Q1_{scaleQ1.ToString("F4", CultureInfo.InvariantCulture)}");
                        SendCommand($"S_T2_{scaleT2.ToString("F4", CultureInfo.InvariantCulture)}");
                        SendCommand($"S_Q2_{scaleQ2.ToString("F4", CultureInfo.InvariantCulture)}");
                    }
                }
            }
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

            foreach (var l in new[] { dlFrontThrustSingle, dlFrontTorqueSingle, dlFrontRPMSingle, dlFrontThrustCRP, dlFrontTorqueCRP, dlFrontRPMCRP, dlBackThrustCRP, dlBackTorqueCRP, dlBackRPMCRP })
            {
                l.ManageAxisLimits = false; 
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

        private void ArduinoPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string line = arduinoPort.ReadLine().Trim();

                if (line.StartsWith("CAL:")) return;

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

                    elecPower1 = volt1 * curr1;
                    mechPower1 = torque1 * rpm1 * (2.0 * Math.PI / 60.0);
                    motorEff1 = elecPower1 > 0 ? (mechPower1 / elecPower1) * 100.0 : 0;

                    elecPower2 = volt2 * curr2;
                    mechPower2 = torque2 * rpm2 * (2.0 * Math.PI / 60.0);
                    motorEff2 = elecPower2 > 0 ? (mechPower2 / elecPower2) * 100.0 : 0;

                    Dispatcher.BeginInvoke(new Action(() =>
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

                        // 30-second window centered on the live data point
                        double minX = liveGraphTime < 15.0 ? 0 : liveGraphTime - 15.0;
                        double maxX = liveGraphTime < 15.0 ? 30.0 : liveGraphTime + 15.0;

                        if (TabSingle.IsSelected)
                        {
                            PlotFrontThrustSingle.Plot.Axes.SetLimitsX(minX, maxX);
                            PlotFrontTorqueSingle.Plot.Axes.SetLimitsX(minX, maxX);
                            PlotFrontRPMSingle.Plot.Axes.SetLimitsX(minX, maxX);

                            PlotFrontThrustSingle.Refresh();
                            PlotFrontTorqueSingle.Refresh();
                            PlotFrontRPMSingle.Refresh();
                        }
                        else if (TabCRP.IsSelected)
                        {
                            PlotFrontThrustCRP.Plot.Axes.SetLimitsX(minX, maxX);
                            PlotFrontTorqueCRP.Plot.Axes.SetLimitsX(minX, maxX);
                            PlotFrontRPMCRP.Plot.Axes.SetLimitsX(minX, maxX);
                            PlotBackThrustCRP.Plot.Axes.SetLimitsX(minX, maxX);
                            PlotBackTorqueCRP.Plot.Axes.SetLimitsX(minX, maxX);
                            PlotBackRPMCRP.Plot.Axes.SetLimitsX(minX, maxX);

                            PlotFrontThrustCRP.Refresh();
                            PlotFrontTorqueCRP.Refresh();
                            PlotFrontRPMCRP.Refresh();
                            PlotBackThrustCRP.Refresh();
                            PlotBackTorqueCRP.Refresh();
                            PlotBackRPMCRP.Refresh();
                        }
                    }));
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
            TxtFrontVoltage.Text = $"Voltage: {volt1:F2} V";
            TxtFrontCurrent.Text = $"Current: {curr1:F2} A";
            TxtFrontRPM.Text = $"RPM: {rpm1:F0}";
            TxtFrontElecPower.Text = $"Elec Power: {elecPower1:F2} W";
            TxtFrontMechPower.Text = $"Mech Power: {mechPower1:F2} W";
            TxtFrontMotorEff.Text = $"Motor Eff: {motorEff1:F2} %";

            TxtBackVoltage.Text = $"Voltage: {volt2:F2} V";
            TxtBackCurrent.Text = $"Current: {curr2:F2} A";
            TxtBackRPM.Text = $"RPM: {rpm2:F0}";
            TxtBackElecPower.Text = $"Elec Power: {elecPower2:F2} W";
            TxtBackMechPower.Text = $"Mech Power: {mechPower2:F2} W";
            TxtBackMotorEff.Text = $"Motor Eff: {motorEff2:F2} %";
            
            if (isAutoRunning)
            {
                autoTimer -= 0.05;
                if (autoTimer <= 0)
                {
                    autoPhase++;
                    if (autoPhase == autoTotalSteps + 1)
                    {
                        // Phase N+1: The 2-second cooldown buffer at the end of the test
                        targetThrottle1 = 0;
                        targetThrottle2 = 0;
                        autoTimer = 2.0; 
                    }
                    else if (autoPhase > autoTotalSteps + 1)
                    {
                        // Phase N+2: Wait for actual throttle to ramp down to 0% before unlocking UI
                        if (actualThrottle1 <= 0.1 && actualThrottle2 <= 0.1)
                        {
                            StopAutoTest(); 
                        }
                    }
                    else
                    {
                        // Active test phases (Phases 1 to N)
                        targetThrottle1 = autoTargetsFront[autoPhase - 1];
                        if (TabCRP.IsSelected) targetThrottle2 = autoTargetsBack[autoPhase - 1];
                        autoTimer = autoStepDuration; 
                    }
                }
            }

            // Smooth Ramping Logic
            if (actualThrottle1 < targetThrottle1)
                actualThrottle1 = Math.Min(actualThrottle1 + RAMP_RATE, targetThrottle1);
            else if (actualThrottle1 > targetThrottle1)
                actualThrottle1 = Math.Max(actualThrottle1 - RAMP_RATE, targetThrottle1);

            if (actualThrottle2 < targetThrottle2)
                actualThrottle2 = Math.Min(actualThrottle2 + RAMP_RATE, targetThrottle2);
            else if (actualThrottle2 > targetThrottle2)
                actualThrottle2 = Math.Max(actualThrottle2 - RAMP_RATE, targetThrottle2);

            TxtInputThrottle1.Text = $"ESC Throttle: {actualThrottle1:F0} %";
            TxtInputThrottle2.Text = $"ESC Throttle: {actualThrottle2:F0} %";

            SendCommand($"T1_{actualThrottle1.ToString("F0", CultureInfo.InvariantCulture)}");
            if (TabCRP.IsSelected) SendCommand($"T2_{actualThrottle2.ToString("F0", CultureInfo.InvariantCulture)}");

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
                    
                    string t1 = actualThrottle1.ToString("F0", CultureInfo.InvariantCulture);
                    string t2 = actualThrottle2.ToString("F0", CultureInfo.InvariantCulture);

                    bool isCRP = TabCRP.IsSelected;
                    double logThrust2 = isCRP ? thrust2 : 0;
                    double logTorque2 = isCRP ? torque2 : 0;
                    double logRpm2 = isCRP ? rpm2 : 0;
                    double logVolt2 = isCRP ? volt2 : 0;
                    double logCurr2 = isCRP ? curr2 : 0;
                    double logElecPower2 = isCRP ? elecPower2 : 0;
                    double logMechPower2 = isCRP ? mechPower2 : 0;
                    double logMotorEff2 = isCRP ? motorEff2 : 0;
                    
                    string csvLine = string.Format(CultureInfo.InvariantCulture, 
                        "{0},{1:F3},{2:F3},{3:F0},{4:F2},{5:F2},{6},{7:F2},{8:F2},{9:F2},{10:F3},{11:F3},{12:F0},{13:F2},{14:F2},{15},{16:F2},{17:F2},{18:F2}",
                        elapsedLogTime, 
                        thrust1, torque1, rpm1, volt1, curr1, t1, elecPower1, mechPower1, motorEff1,
                        logThrust2, logTorque2, logRpm2, logVolt2, logCurr2, t2, logElecPower2, logMechPower2, logMotorEff2);

                    csvWriter.WriteLine(csvLine);
                    nextLogInterval += 0.1; 
                }
            }

            // Manage Auto Test button state (Disable if manual throttle is non-zero)
            if (!isAutoRunning)
            {
                if (ChkActivateSingle.IsChecked == true)
                {
                    BtnStartAutoSingle.IsEnabled = (SldThrottleSingle.Value == 0 && actualThrottle1 < 0.1);
                }
                if (ChkActivateCRP.IsChecked == true)
                {
                    BtnStartAutoCRP.IsEnabled = (SldThrottleFrontCRP.Value == 0 && SldThrottleBackCRP.Value == 0 && actualThrottle1 < 0.1 && actualThrottle2 < 0.1);
                }
            }
        }

        private void StopRecording()
        {
            if (isRecording)
            {
                isRecording = false;
                BtnRecordSingle.Content = "Record Manual Test";
                BtnRecordCRP.Content = "Record Manual Test";
                csvWriter?.Close();
                LoadLogFiles();
            }
        }

        private void StopAutoTest()
        {
            isAutoRunning = false;
            targetThrottle1 = 0;
            targetThrottle2 = 0;
            
            BtnStartAutoSingle.Content = "Start Auto Test";
            BtnStartAutoCRP.Content = "Start Auto Test";
            
            if (LblRecordingAutoSingle != null) LblRecordingAutoSingle.Visibility = Visibility.Hidden;
            if (LblRecordingAutoCRP != null) LblRecordingAutoCRP.Visibility = Visibility.Hidden;
            
            // Re-enable manual sliders and record buttons if the system is still active
            if (ChkActivateSingle.IsChecked == true) 
            {
                SldThrottleSingle.IsEnabled = true;
                BtnRecordSingle.IsEnabled = true;
            }
            if (ChkActivateCRP.IsChecked == true) 
            {
                SldThrottleFrontCRP.IsEnabled = true;
                SldThrottleBackCRP.IsEnabled = true;
                BtnRecordCRP.IsEnabled = true;
            }
            
            StopRecording();
        }

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
            if (isAutoRunning) StopAutoTest();
            targetThrottle1 = 0;
            targetThrottle2 = 0;
            StopRecording();
            SendCommand("STOP_ALL");
        }   

        private void BtnStop_MouseDown(object sender, MouseButtonEventArgs e) { StartStopPress(); }
        private void BtnStop_MouseUp(object sender, MouseButtonEventArgs e) { CancelStopPress(); }
        private void BtnStop_MouseLeave(object sender, MouseEventArgs e) { CancelStopPress(); }
        private void Window_PreviewKeyDown(object sender, KeyEventArgs e) { if (e.Key == Key.Space && !e.IsRepeat) { StartStopPress(); e.Handled = true; } }
        private void Window_PreviewKeyUp(object sender, KeyEventArgs e) { if (e.Key == Key.Space) { CancelStopPress(); e.Handled = true; } }

        private void MainTabControl_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (e.Source is TabControl && PanelLiveDataInputsRoot != null)
            {
                if (TabLogs.IsSelected)
                {
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

                    if (!isCalibrating && !isRecording) LoadLatestCalibration();
                }
            }
        }

        private void LoadLogFiles()
        {
            if (ListLogFiles == null) return;
            ListLogFiles.Items.Clear();
            var files = Directory.GetFiles(logDir, "DynamometerData_*.csv").OrderByDescending(f => f).ToList();
            foreach (var f in files) ListLogFiles.Items.Add(Path.GetFileName(f));
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
            foreach (string header in headers) dt.Columns.Add(header);
            for (int i = 1; i < lines.Length; i++)
            {
                if (string.IsNullOrWhiteSpace(lines[i])) continue;
                string[] parts = lines[i].Split(',');
                
                if (parts.Length > dt.Columns.Count) dt.Rows.Add(parts.Take(dt.Columns.Count).ToArray());
                else if (parts.Length < dt.Columns.Count)
                {
                    var padded = parts.ToList();
                    while (padded.Count < dt.Columns.Count) padded.Add("");
                    dt.Rows.Add(padded.ToArray());
                }
                else dt.Rows.Add(parts);
            }
            DataGridLogs.ItemsSource = dt.DefaultView;
        }

        private void BtnOpenLogsFolder_Click(object sender, RoutedEventArgs e)
        {
            Process.Start(new ProcessStartInfo { FileName = logDir, UseShellExecute = true });
        }

        private void SetCalibrationButtonsState(bool enabled)
        {
            BtnTareThrustFront1.IsEnabled = enabled;
            BtnCalThrustFront1.IsEnabled = enabled;
            BtnTareTorqueFront1.IsEnabled = enabled;
            BtnCalTorqueFront1.IsEnabled = enabled;
            
            BtnTareThrustFront2.IsEnabled = enabled;
            BtnCalThrustFront2.IsEnabled = enabled;
            BtnTareThrustBack.IsEnabled = enabled;
            BtnCalThrustBack.IsEnabled = enabled;
            BtnTareTorqueFront2.IsEnabled = enabled;
            BtnCalTorqueFront2.IsEnabled = enabled;
            BtnTareTorqueBack.IsEnabled = enabled;
            BtnCalTorqueBack.IsEnabled = enabled;

            BtnStartCalSingle.IsEnabled = !enabled;
            BtnFinishCalSingle.IsEnabled = enabled;
            BtnStartCalCRP.IsEnabled = !enabled;
            BtnFinishCalCRP.IsEnabled = enabled;

            TabSingle.IsEnabled = !enabled || TabSingle.IsSelected;
            TabCRP.IsEnabled = !enabled || TabCRP.IsSelected;
            TabLogs.IsEnabled = !enabled;
        }

        private void BtnStartCal_Click(object sender, RoutedEventArgs e)
        {
            isCalibrating = true;
            SetCalibrationButtonsState(true);
        }

        private void BtnFinishCal_Click(object sender, RoutedEventArgs e)
        {
            isCalibrating = false;
            SetCalibrationButtonsState(false);

            string targetDir = TabSingle.IsSelected ? calSingleDir : calCRPDir;
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string calFile = Path.Combine(targetDir, $"Calibration_{timestamp}.csv");
            
            File.WriteAllText(calFile, "ScaleT1,ScaleQ1,ScaleT2,ScaleQ2\n");
            File.AppendAllText(calFile, string.Format(CultureInfo.InvariantCulture, "{0:F4},{1:F4},{2:F4},{3:F4}\n", scaleT1, scaleQ1, scaleT2, scaleQ2));
            
            MessageBox.Show("Calibration saved to:\n" + calFile, "Calibration Complete");
        }

        public void BtnTareThrust_Click(object sender, RoutedEventArgs e) { SendCommand("A"); }
        public void BtnCalThrust_Click(object sender, RoutedEventArgs e) { SendCommand("B"); }
        public void BtnTareTorque_Click(object sender, RoutedEventArgs e) { SendCommand("C"); }
        public void BtnCalTorque_Click(object sender, RoutedEventArgs e) { SendCommand("D"); }
        public void BtnTareThrustBack_Click(object sender, RoutedEventArgs e) { SendCommand("E"); }
        public void BtnCalThrustBack_Click(object sender, RoutedEventArgs e) { SendCommand("F"); }
        public void BtnTareTorqueBack_Click(object sender, RoutedEventArgs e) { SendCommand("G"); }
        public void BtnCalTorqueBack_Click(object sender, RoutedEventArgs e) { SendCommand("H"); }

        private void ChkActivateSingle_Checked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = true;
            BtnRecordSingle.IsEnabled = true;
            BtnStartAutoSingle.IsEnabled = true;
            TabCRP.IsEnabled = false; 
            TabLogs.IsEnabled = false;
            SendCommand("UNLOCK_SINGLE");
        }

        private void ChkActivateSingle_Unchecked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = false;
            SldThrottleSingle.Value = 0; 
            BtnRecordSingle.IsEnabled = false;
            BtnStartAutoSingle.IsEnabled = false;
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
            BtnStartAutoCRP.IsEnabled = true;
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
            BtnStartAutoCRP.IsEnabled = false;
            TabSingle.IsEnabled = true; 
            TabLogs.IsEnabled = true;
            StopRecording();
            SendCommand("LOCK_CRP");
        }

        private void SldThrottleSingle_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueSingle != null) TxtThrottleValueSingle.Text = e.NewValue.ToString("F0");
            targetThrottle1 = e.NewValue;
        }

        private void SldThrottleFrontCRP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueFrontCRP != null) TxtThrottleValueFrontCRP.Text = e.NewValue.ToString("F0");
            targetThrottle1 = e.NewValue;
        }

        private void SldThrottleBackCRP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtThrottleValueBackCRP != null) TxtThrottleValueBackCRP.Text = e.NewValue.ToString("F0");
            targetThrottle2 = e.NewValue;
        }

        private void BtnRecord_Click(object sender, RoutedEventArgs e)
        {
            if (!isRecording)
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string mode = isAutoRunning ? "Auto" : "Manual";
                csvWriter = new StreamWriter(Path.Combine(logDir, $"DynamometerData_{mode}_{timestamp}.csv"));
                csvWriter.AutoFlush = true;
                csvWriter.WriteLine("Time,Thrust1,Torque1,RPM1,Voltage1,Current1,Throttle1,ElecPower1,MechPower1,MotorEff1,Thrust2,Torque2,RPM2,Voltage2,Current2,Throttle2,ElecPower2,MechPower2,MotorEff2");
                
                startTime = DateTime.Now;
                nextLogInterval = 0.1;
                isRecording = true;
                
                if (isAutoRunning)
                {
                    BtnRecordSingle.IsEnabled = false;
                    BtnRecordCRP.IsEnabled = false;
                    if (TabSingle.IsSelected && LblRecordingAutoSingle != null) LblRecordingAutoSingle.Visibility = Visibility.Visible;
                    if (TabCRP.IsSelected && LblRecordingAutoCRP != null) LblRecordingAutoCRP.Visibility = Visibility.Visible;
                }
                else
                {
                    BtnRecordSingle.Content = "Stop Recording";
                    BtnRecordCRP.Content = "Stop Recording";
                }
            }
            else StopRecording();
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
                if (ImgLogo != null) try { ImgLogo.Source = new BitmapImage(new Uri("pack://application:,,,/Dynamometer Logo NoText DarkMode NoBackground.png")); } catch { }
            }
            else
            {
                this.Resources["MainBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#edf0f4"));
                this.Resources["MainFg"] = new SolidColorBrush(Colors.Black);
                this.Resources["BtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#D0D9E4"));
                this.Resources["HoverBtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#B8C5D1"));
                this.Resources["DisabledBtnBg"] = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#cccccc"));
                this.Resources["ControlBg"] = new SolidColorBrush(Colors.White);
                if (ImgLogo != null) try { ImgLogo.Source = new BitmapImage(new Uri("pack://application:,,,/Dynamometer Logo NoText NoBackground.png")); } catch { }
            }

            SetPlotTheme(isDark);
            ApplyTitleBarTheme(isDark);
        }

        private void SetPlotTheme(bool isDark)
        {
            var plots = new[] { PlotFrontThrustSingle, PlotFrontTorqueSingle, PlotFrontRPMSingle, PlotFrontThrustCRP, PlotFrontTorqueCRP, PlotFrontRPMCRP, PlotBackThrustCRP, PlotBackTorqueCRP, PlotBackRPMCRP };
            var loggers = new[] { dlFrontThrustSingle, dlFrontTorqueSingle, dlFrontRPMSingle, dlFrontThrustCRP, dlFrontTorqueCRP, dlFrontRPMCRP, dlBackThrustCRP, dlBackTorqueCRP, dlBackRPMCRP };

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
                    if (l != null) l.Color = ScottPlot.Color.FromHex("#FFC107"); 
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
            RefreshVisiblePlots();
        }

        private void CmbThrustUnit_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (PlotFrontThrustSingle == null) return; 
            string unitLabel = CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)";
            PlotFrontThrustSingle.Plot.YLabel(unitLabel);
            PlotFrontThrustCRP.Plot.YLabel(unitLabel);
            PlotBackThrustCRP.Plot.YLabel(unitLabel);
            RefreshVisiblePlots();
        }

        private void RefreshVisiblePlots()
        {
            // Massive performance gain: Only forces WPF to redraw the graphs that are actually on screen
            if (TabSingle.IsSelected)
            {
                PlotFrontThrustSingle.Refresh();
                PlotFrontTorqueSingle.Refresh();
                PlotFrontRPMSingle.Refresh();
            }
            else if (TabCRP.IsSelected)
            {
                PlotFrontThrustCRP.Refresh();
                PlotFrontTorqueCRP.Refresh();
                PlotFrontRPMCRP.Refresh();
                PlotBackThrustCRP.Refresh();
                PlotBackTorqueCRP.Refresh();
                PlotBackRPMCRP.Refresh();
            }
        }

        public void SendCommand(string command)
        {
            try
            {
                if (arduinoPort != null && arduinoPort.IsOpen)
                {
                    arduinoPort.Write(command + "\n");
                }
            }
            catch
            {
                // General catch to prevent unhandled TimeoutExceptions from crashing the app
                if (arduinoPort != null && arduinoPort.IsOpen)
                {
                    try { arduinoPort.Close(); } catch { }
                }
                
                Dispatcher.Invoke(() => 
                {
                    ChkActivateSingle.IsChecked = false;
                    ChkActivateCRP.IsChecked = false;
                    if (isAutoRunning) StopAutoTest();
                    targetThrottle1 = 0;
                    targetThrottle2 = 0;
                    StopRecording();
                });
            }
        }

        private void SldAutoStepsSingle_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtAutoStepsSingle == null) return;
            int steps = (int)e.NewValue;
            TxtAutoStepsSingle.Text = steps.ToString();
            
            TextBox[] boxes = { TxtAutoSingle1, TxtAutoSingle2, TxtAutoSingle3, TxtAutoSingle4, TxtAutoSingle5, TxtAutoSingle6 };
            for (int i = 0; i < 6; i++) boxes[i].Visibility = i < steps ? Visibility.Visible : Visibility.Collapsed;
        }

        private void SldAutoStepsCRP_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (TxtAutoStepsCRP == null) return;
            int steps = (int)e.NewValue;
            TxtAutoStepsCRP.Text = steps.ToString();
            
            TextBox[] fBoxes = { TxtAutoFrontCRP1, TxtAutoFrontCRP2, TxtAutoFrontCRP3, TxtAutoFrontCRP4, TxtAutoFrontCRP5, TxtAutoFrontCRP6 };
            TextBox[] bBoxes = { TxtAutoBackCRP1, TxtAutoBackCRP2, TxtAutoBackCRP3, TxtAutoBackCRP4, TxtAutoBackCRP5, TxtAutoBackCRP6 };
            
            for (int i = 0; i < 6; i++)
            {
                fBoxes[i].Visibility = i < steps ? Visibility.Visible : Visibility.Collapsed;
                bBoxes[i].Visibility = i < steps ? Visibility.Visible : Visibility.Collapsed;
            }
        }

        private void BtnResetAutoSingle_Click(object sender, RoutedEventArgs e)
        {
            SldAutoStepsSingle.Value = 5;
            TxtStepDurationSingle.Text = "2.0";
            TxtAutoSingle1.Text = "10"; TxtAutoSingle2.Text = "20"; TxtAutoSingle3.Text = "30"; 
            TxtAutoSingle4.Text = "40"; TxtAutoSingle5.Text = "50"; TxtAutoSingle6.Text = "60";
        }

        private void BtnResetAutoCRP_Click(object sender, RoutedEventArgs e)
        {
            SldAutoStepsCRP.Value = 5;
            TxtStepDurationCRP.Text = "2.0";
            TxtAutoFrontCRP1.Text = "10"; TxtAutoFrontCRP2.Text = "20"; TxtAutoFrontCRP3.Text = "30"; 
            TxtAutoFrontCRP4.Text = "40"; TxtAutoFrontCRP5.Text = "50"; TxtAutoFrontCRP6.Text = "60";
            
            TxtAutoBackCRP1.Text = "10"; TxtAutoBackCRP2.Text = "20"; TxtAutoBackCRP3.Text = "30"; 
            TxtAutoBackCRP4.Text = "40"; TxtAutoBackCRP5.Text = "50"; TxtAutoBackCRP6.Text = "60";
        }

        private void TxtStepDuration_LostFocus(object sender, RoutedEventArgs e)
        {
            if (sender is TextBox txt)
            {
                if (double.TryParse(txt.Text, NumberStyles.Any, CultureInfo.InvariantCulture, out double duration))
                {
                    if (duration > 99.0) duration = 99.0;
                    if (duration < 0.1) duration = 0.1; // Prevent negative or zero time intervals
                    txt.Text = duration.ToString("F1", CultureInfo.InvariantCulture);
                }
                else
                {
                    txt.Text = "2.0"; // Fallback if the user enters letters or invalid text
                }
            }
        }

        private void BtnStartAutoSingle_Click(object sender, RoutedEventArgs e)
        {
            if (isAutoRunning)
            {
                StopAutoTest();
                return;
            }

            autoTotalSteps = (int)SldAutoStepsSingle.Value;
            TextBox[] boxes = { TxtAutoSingle1, TxtAutoSingle2, TxtAutoSingle3, TxtAutoSingle4, TxtAutoSingle5, TxtAutoSingle6 };
            
            try
            {
                TxtStepDuration_LostFocus(TxtStepDurationSingle, null!); // Force clamp/format if clicked directly
                autoStepDuration = double.Parse(TxtStepDurationSingle.Text, CultureInfo.InvariantCulture);
                
                for (int i = 0; i < autoTotalSteps; i++) autoTargetsFront[i] = double.Parse(boxes[i].Text, CultureInfo.InvariantCulture);
                
                autoPhase = 0;
                autoTimer = 2.0; // 2 seconds of 0 throttle hold
                targetThrottle1 = 0;
                targetThrottle2 = 0;
                isAutoRunning = true;

                // Update UI state
                BtnStartAutoSingle.Content = "Stop Auto Test";
                SldThrottleSingle.IsEnabled = false;

                BtnRecord_Click(sender, e);
            }
            catch { MessageBox.Show("Please enter valid numbers for the step duration and targets."); }
        }

        private void BtnStartAutoCRP_Click(object sender, RoutedEventArgs e)
        {
            if (isAutoRunning)
            {
                StopAutoTest();
                return;
            }

            autoTotalSteps = (int)SldAutoStepsCRP.Value;
            TextBox[] fBoxes = { TxtAutoFrontCRP1, TxtAutoFrontCRP2, TxtAutoFrontCRP3, TxtAutoFrontCRP4, TxtAutoFrontCRP5, TxtAutoFrontCRP6 };
            TextBox[] bBoxes = { TxtAutoBackCRP1, TxtAutoBackCRP2, TxtAutoBackCRP3, TxtAutoBackCRP4, TxtAutoBackCRP5, TxtAutoBackCRP6 };

            try
            {
                TxtStepDuration_LostFocus(TxtStepDurationCRP, null!); // Force clamp/format if clicked directly
                autoStepDuration = double.Parse(TxtStepDurationCRP.Text, CultureInfo.InvariantCulture);
                
                for (int i = 0; i < autoTotalSteps; i++)
                {
                    autoTargetsFront[i] = double.Parse(fBoxes[i].Text, CultureInfo.InvariantCulture);
                    autoTargetsBack[i] = double.Parse(bBoxes[i].Text, CultureInfo.InvariantCulture);
                }
                
                autoPhase = 0;
                autoTimer = 2.0; 
                targetThrottle1 = 0;
                targetThrottle2 = 0;
                isAutoRunning = true;

                // Update UI state
                BtnStartAutoCRP.Content = "Stop Auto Test";
                SldThrottleFrontCRP.IsEnabled = false;
                SldThrottleBackCRP.IsEnabled = false;

                BtnRecord_Click(sender, e);
            }
            catch { MessageBox.Show("Please enter valid numbers for the step duration and targets."); }
        }
    }
}
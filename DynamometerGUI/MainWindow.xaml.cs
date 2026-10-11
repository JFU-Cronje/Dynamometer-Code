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
using System.Threading;
using System.Collections.Concurrent;

namespace DynamometerGUI
{
    public struct TelemetryPacket
    {
        public double Thrust1, Torque1, RPM1, Volt1, Curr1;
        public double Thrust2, Torque2, RPM2, Volt2, Curr2;
        public double ElecPower1, MechPower1, MotorEff1;
        public double ElecPower2, MechPower2, MotorEff2;
    }

    public partial class MainWindow : Window
    {
        [DllImport("dwmapi.dll")]
        private static extern int DwmSetWindowAttribute(IntPtr hwnd, int attr, ref int attrValue, int attrSize);

        SerialPort? arduinoPort;
        Thread? serialWorkerThread;
        private volatile bool isSerialWorkerRunning = false;
        private readonly ConcurrentQueue<TelemetryPacket> telemetryQueue = new();
        private readonly object csvLock = new object();
        private volatile bool isCRPSelected = false;

        DispatcherTimer masterLoopTimer;

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
        int lastSentThrottle1 = -1;
        int lastSentThrottle2 = -1;
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

        // State Flags
        private bool isSetupComplete = false;
        private int startupCountdownTicks = 200; // 200 ticks * 50ms = 10.0s
        private int textThrottleCounter = 0;

        // Known Calibration Constants (176g mass, 150mm arm)
        const double TARGET_THRUST_N = 0.176 * 9.80665;        // 1.72597 N
        const double TARGET_TORQUE_NM = TARGET_THRUST_N * 0.150; // 0.25890 Nm

        // Application Local Directories
        string logDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoLogs");
        string calSingleDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoCalibrationsSingle");
        string calCRPDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoCalibrationsCRP");
        
        // Calibration State
        bool isCalibrating = false;
        double scaleT1 = 1.0, scaleQ1 = 1.0, scaleT2 = 1.0, scaleQ2 = 1.0;

        public MainWindow()
        {
            InitializeComponent();
            
            this.SetResourceReference(Window.BackgroundProperty, "MainBg");
            this.SetResourceReference(Window.ForegroundProperty, "MainFg");
            
            Directory.CreateDirectory(logDir);
            Directory.CreateDirectory(calSingleDir);
            Directory.CreateDirectory(calCRPDir);

            // Single unified timer running at 20 Hz (50 ms)
            masterLoopTimer = new DispatcherTimer(DispatcherPriority.Normal);
            masterLoopTimer.Interval = TimeSpan.FromMilliseconds(50);
            masterLoopTimer.Tick += MasterLoopTimer_Tick;

            // Lock controls during the startup sequence
            ChkActivateSingle.IsEnabled = false;
            ChkActivateCRP.IsEnabled = false;
            BtnStartCalSingle.IsEnabled = false;
            BtnStartCalCRP.IsEnabled = false;

            InitializeGraphAxes();
            LoadLogFiles();
            LoadLatestCalibration();
            AutoConnectArduino(); 
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            ApplyTitleBarTheme(CmbTheme.SelectedIndex == 1);
            masterLoopTimer.Start();
        }

        protected override void OnClosed(EventArgs e)
        {
            base.OnClosed(e);
            isSerialWorkerRunning = false;
            if (serialWorkerThread != null && serialWorkerThread.IsAlive)
            {
                serialWorkerThread.Join(200);
            }
            if (arduinoPort != null && arduinoPort.IsOpen)
            {
                try { arduinoPort.Close(); } catch { }
            }
        }

        private void AutoConnectArduino()
        {
            foreach (string portName in SerialPort.GetPortNames().Reverse())
            {
                try
                {
                    var port = new SerialPort(portName, 115200);
                    port.NewLine = "\n";
                    port.WriteTimeout = 100; 
                    port.ReadTimeout = 300;
                    port.DtrEnable = true;
                    port.Open();

                    arduinoPort = port;
                    isSerialWorkerRunning = true;
                    
                    // Dedicated thread eliminates serial reader lock contention
                    serialWorkerThread = new Thread(SerialWorkerLoop);
                    serialWorkerThread.IsBackground = true;
                    serialWorkerThread.Priority = ThreadPriority.AboveNormal;
                    serialWorkerThread.Start();
                    return; 
                }
                catch { }
            }
            MessageBox.Show("Could not automatically find the Arduino. Please check your USB connection.");
        }

        private void SerialWorkerLoop()
        {
            while (isSerialWorkerRunning)
            {
                try
                {
                    if (arduinoPort == null || !arduinoPort.IsOpen)
                    {
                        Thread.Sleep(50);
                        continue;
                    }

                    string line = arduinoPort.ReadLine().Trim();

                    if (!isSetupComplete || line.StartsWith("CAL:") || line.StartsWith("---"))
                    {
                        continue;
                    }

                    string[] parts = line.Split(',');
                    if (parts.Length == 10 &&
                        double.TryParse(parts[0], NumberStyles.Any, CultureInfo.InvariantCulture, out double t1) &&
                        double.TryParse(parts[1], NumberStyles.Any, CultureInfo.InvariantCulture, out double q1) &&
                        double.TryParse(parts[2], NumberStyles.Any, CultureInfo.InvariantCulture, out double r1) &&
                        double.TryParse(parts[3], NumberStyles.Any, CultureInfo.InvariantCulture, out double v1) &&
                        double.TryParse(parts[4], NumberStyles.Any, CultureInfo.InvariantCulture, out double c1) &&
                        double.TryParse(parts[5], NumberStyles.Any, CultureInfo.InvariantCulture, out double t2) &&
                        double.TryParse(parts[6], NumberStyles.Any, CultureInfo.InvariantCulture, out double q2) &&
                        double.TryParse(parts[7], NumberStyles.Any, CultureInfo.InvariantCulture, out double r2) &&
                        double.TryParse(parts[8], NumberStyles.Any, CultureInfo.InvariantCulture, out double v2) &&
                        double.TryParse(parts[9], NumberStyles.Any, CultureInfo.InvariantCulture, out double c2))
                    {
                        double ep1 = v1 * c1;
                        double mp1 = q1 * r1 * (2.0 * Math.PI / 60.0);
                        double me1 = ep1 > 0 ? (mp1 / ep1) * 100.0 : 0;

                        double ep2 = v2 * c2;
                        double mp2 = q2 * r2 * (2.0 * Math.PI / 60.0);
                        double me2 = ep2 > 0 ? (mp2 / ep2) * 100.0 : 0;

                        // 1. Enqueue telemetry FIRST so graphs NEVER freeze
                        telemetryQueue.Enqueue(new TelemetryPacket
                        {
                            Thrust1 = t1, Torque1 = q1, RPM1 = r1, Volt1 = v1, Curr1 = c1,
                            Thrust2 = t2, Torque2 = q2, RPM2 = r2, Volt2 = v2, Curr2 = c2,
                            ElecPower1 = ep1, MechPower1 = mp1, MotorEff1 = me1,
                            ElecPower2 = ep2, MechPower2 = mp2, MotorEff2 = me2
                        });

                        // 2. Thread-safe disk logging
                        if (isRecording)
                        {
                            lock (csvLock)
                            {
                                if (isRecording && csvWriter != null)
                                {
                                    TimeSpan elapsed = DateTime.Now - startTime;
                                    if (elapsed.TotalSeconds >= nextLogInterval)
                                    {
                                        TimeSpan exactLogTime = TimeSpan.FromSeconds(nextLogInterval);
                                        string elapsedLogTime = exactLogTime.ToString(@"hh\:mm\:ss\.fff");
                                        string actT1 = actualThrottle1.ToString("F0", CultureInfo.InvariantCulture);
                                        string actT2 = actualThrottle2.ToString("F0", CultureInfo.InvariantCulture);
                                        bool isCrp = isCRPSelected; // Uses thread-safe cached boolean

                                        string csvLine = string.Format(CultureInfo.InvariantCulture,
                                            "{0},{1:F3},{2:F3},{3:F0},{4:F2},{5:F2},{6},{7:F2},{8:F2},{9:F2}," +
                                            "{10:F3},{11:F3},{12:F0},{13:F2},{14:F2},{15},{16:F2},{17:F2},{18:F2}",
                                            elapsedLogTime,
                                            t1, q1, r1, v1, c1, actT1, ep1, mp1, me1,
                                            isCrp ? t2 : 0, isCrp ? q2 : 0, isCrp ? r2 : 0, isCrp ? v2 : 0, isCrp ? c2 : 0,
                                            actT2, isCrp ? ep2 : 0, isCrp ? mp2 : 0, isCrp ? me2 : 0);

                                        csvWriter.WriteLine(csvLine);
                                        nextLogInterval += 0.1;
                                    }
                                }
                            }
                        }
                    }
                }
                catch (TimeoutException) { }
                catch (Exception)
                {
                    Thread.Sleep(20);
                }
            }
        }

        private void MasterLoopTimer_Tick(object? sender, EventArgs e)
        {
            // 1. Startup Sequence Management
            if (!isSetupComplete)
            {
                startupCountdownTicks--;
                if (startupCountdownTicks % 20 == 0) // Once every second
                {
                    int secondsLeft = startupCountdownTicks / 20;
                    TxtStartupCountdown.Text = $"Hardware Initializing... Please wait ({secondsLeft}s)";
                }

                if (startupCountdownTicks <= 0)
                {
                    isSetupComplete = true;
                    BorderStartupNotice.Visibility = Visibility.Collapsed;

                    SendCalibrationToArduino();

                    ChkActivateSingle.IsEnabled = true;
                    ChkActivateCRP.IsEnabled = true;
                    BtnStartCalSingle.IsEnabled = true;
                    BtnStartCalCRP.IsEnabled = true;

                    ResetGraphAxesY();
                }
                return;
            }

            // 2. Telemetry Processing & Data Logging
            bool hasNewData = false;
            while (telemetryQueue.TryDequeue(out var packet))
            {
                thrust1 = packet.Thrust1; torque1 = packet.Torque1; rpm1 = packet.RPM1; volt1 = packet.Volt1; curr1 = packet.Curr1;
                thrust2 = packet.Thrust2; torque2 = packet.Torque2; rpm2 = packet.RPM2; volt2 = packet.Volt2; curr2 = packet.Curr2;
                elecPower1 = packet.ElecPower1; mechPower1 = packet.MechPower1; motorEff1 = packet.MotorEff1;
                elecPower2 = packet.ElecPower2; mechPower2 = packet.MechPower2; motorEff2 = packet.MotorEff2;

                liveGraphTime += 0.1;
                hasNewData = true;

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
            }

            // 3. Render Graphs (Only redrawn when new data arrives to save CPU load)
            if (hasNewData)
            {
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
            }

            // 4. Update UI Text Labels (Throttled to 5 Hz to eliminate layout invalidation stalls)
            textThrottleCounter++;
            if (textThrottleCounter >= 4)
            {
                textThrottleCounter = 0;
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

                if (isRecording)
                {
                    TimeSpan elapsed = DateTime.Now - startTime;
                    string timeFormat = $"Time: {elapsed:hh\\:mm\\:ss\\.fff}";
                    TxtTimerSingle.Text = timeFormat;
                    TxtTimerCRP.Text = timeFormat;
                }
            }

            // 5. Emergency Stop Button Countdown
            if (isStopping)
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

            // 6. Automatic Test Sequence
            if (isAutoRunning)
            {
                autoTimer -= 0.05;
                if (autoTimer <= 0)
                {
                    autoPhase++;
                    if (autoPhase == autoTotalSteps + 1)
                    {
                        targetThrottle1 = 0;
                        targetThrottle2 = 0;
                        autoTimer = 2.0; 
                    }
                    else if (autoPhase > autoTotalSteps + 1)
                    {
                        if (actualThrottle1 <= 0.1 && actualThrottle2 <= 0.1)
                        {
                            StopAutoTest(); 
                        }
                    }
                    else
                    {
                        targetThrottle1 = autoTargetsFront[autoPhase - 1];
                        if (TabCRP.IsSelected) targetThrottle2 = autoTargetsBack[autoPhase - 1];
                        autoTimer = autoStepDuration; 
                    }
                }
            }

            // 7. Smooth Throttle Ramping Logic
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

            int currentT1Int = (int)Math.Round(actualThrottle1);
            int currentT2Int = (int)Math.Round(actualThrottle2);

            if (currentT1Int != lastSentThrottle1)
            {
                SendCommand($"T1_{currentT1Int}");
                lastSentThrottle1 = currentT1Int;
            }

            if (TabCRP.IsSelected && currentT2Int != lastSentThrottle2)
            {
                SendCommand($"T2_{currentT2Int}");
                lastSentThrottle2 = currentT2Int;
            }

            // 8. Auto Test Interlock
            if (!isAutoRunning && isSetupComplete)
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
                if (arduinoPort != null && arduinoPort.IsOpen)
                {
                    try { arduinoPort.Close(); } catch { }
                }
                
                ChkActivateSingle.IsChecked = false;
                ChkActivateCRP.IsChecked = false;
                if (isAutoRunning) StopAutoTest();
                targetThrottle1 = 0;
                targetThrottle2 = 0;
                StopRecording();
            }
        }

        private void LoadLatestCalibration()
        {
            try
            {
                string targetDir = TabSingle.IsSelected ? calSingleDir : calCRPDir;
                var latestFile = Directory.GetFiles(targetDir, "Calibration_*.csv")
                                          .OrderByDescending(f => f)
                                          .FirstOrDefault();

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
                        }
                    }
                }
                else
                {
                    scaleT1 = 1.0; scaleQ1 = 1.0; scaleT2 = 1.0; scaleQ2 = 1.0;
                }

                if (isSetupComplete)
                {
                    SendCalibrationToArduino();
                }
            }
            catch { }
        }

        private void SaveCalibration()
        {
            try
            {
                string targetDir = TabSingle.IsSelected ? calSingleDir : calCRPDir;
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string calFile = Path.Combine(targetDir, $"Calibration_{timestamp}.csv");
                
                File.WriteAllText(calFile, "ScaleT1,ScaleQ1,ScaleT2,ScaleQ2\n");
                File.AppendAllText(calFile, string.Format(CultureInfo.InvariantCulture, 
                    "{0:G8},{1:G8},{2:G8},{3:G8}\n", scaleT1, scaleQ1, scaleT2, scaleQ2));
            }
            catch { }
        }

        private void SendCalibrationToArduino()
        {
            SendCommand($"S_T1_{scaleT1.ToString("G8", CultureInfo.InvariantCulture)}");
            SendCommand($"S_Q1_{scaleQ1.ToString("G8", CultureInfo.InvariantCulture)}");
            SendCommand($"S_T2_{scaleT2.ToString("G8", CultureInfo.InvariantCulture)}");
            SendCommand($"S_Q2_{scaleQ2.ToString("G8", CultureInfo.InvariantCulture)}");
        }

        private void ResetGraphAxesY()
        {
            double thrustMin = CmbThrustUnit.SelectedIndex == 0 ? -2.0 : -2.0 * 9.80665;
            double thrustMax = CmbThrustUnit.SelectedIndex == 0 ? 5.0 : 5.0 * 9.80665;

            PlotFrontThrustSingle.Plot.Title("Front Thrust");
            PlotFrontThrustSingle.Plot.YLabel(CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)");
            PlotFrontThrustSingle.Plot.Axes.SetLimitsY(thrustMin, thrustMax);

            PlotFrontTorqueSingle.Plot.Title("Front Torque");
            PlotFrontTorqueSingle.Plot.YLabel("Torque (Nm)");
            PlotFrontTorqueSingle.Plot.Axes.SetLimitsY(-0.2, 1.7);

            PlotFrontRPMSingle.Plot.Title("Front RPM");
            PlotFrontRPMSingle.Plot.YLabel("RPM");
            PlotFrontRPMSingle.Plot.Axes.SetLimitsY(-1000, 10000);

            PlotFrontThrustCRP.Plot.Title("Front Thrust");
            PlotFrontThrustCRP.Plot.YLabel(CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)");
            PlotFrontThrustCRP.Plot.Axes.SetLimitsY(thrustMin, thrustMax);

            PlotFrontTorqueCRP.Plot.Title("Front Torque");
            PlotFrontTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotFrontTorqueCRP.Plot.Axes.SetLimitsY(-0.2, 1.7);

            PlotFrontRPMCRP.Plot.Title("Front RPM");
            PlotFrontRPMCRP.Plot.YLabel("RPM");
            PlotFrontRPMCRP.Plot.Axes.SetLimitsY(-1000, 10000);

            PlotBackThrustCRP.Plot.Title("Back Thrust");
            PlotBackThrustCRP.Plot.YLabel(CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)");
            PlotBackThrustCRP.Plot.Axes.SetLimitsY(thrustMin, thrustMax);

            PlotBackTorqueCRP.Plot.Title("Back Torque");
            PlotBackTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotBackTorqueCRP.Plot.Axes.SetLimitsY(-0.2, 1.7);

            PlotBackRPMCRP.Plot.Title("Back RPM");
            PlotBackRPMCRP.Plot.YLabel("RPM");
            PlotBackRPMCRP.Plot.Axes.SetLimitsY(-1000, 10000);

            RefreshVisiblePlots();
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
                p.Plot.Axes.Bottom.TickGenerator = new ScottPlot.TickGenerators.NumericFixedInterval(5.0);
                p.Plot.Axes.SetLimitsX(0, 30);
                
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

            ResetGraphAxesY();
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

        private void StopRecording()
        {
            if (isRecording)
            {
                lock (csvLock)
                {
                    isRecording = false;
                    csvWriter?.Flush();
                    csvWriter?.Close();
                    csvWriter = null;
                }
                BtnRecordSingle.Content = "Record Manual Test";
                BtnRecordCRP.Content = "Record Manual Test";
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

        private void StartStopPress()
        {
            if (!isStopping)
            {
                isStopping = true;
                StopBtnTransform.Y = 4;
                TxtStopCountdown.Visibility = Visibility.Visible;
                stopPressTime = DateTime.Now;
                TxtStopCountdown.Text = "3.0";
            }
        }

        private void CancelStopPress()
        {
            if (isStopping)
            {
                isStopping = false;
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
            isCRPSelected = TabCRP.IsSelected; // Thread-safe state caching

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

                    if (!isCalibrating && !isRecording && isSetupComplete) LoadLatestCalibration();
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

            bool isSystemActive = (ChkActivateSingle.IsChecked == true || ChkActivateCRP.IsChecked == true);

            BtnStartCalSingle.IsEnabled = !enabled && !isSystemActive && isSetupComplete;
            BtnFinishCalSingle.IsEnabled = enabled;
            BtnStartCalCRP.IsEnabled = !enabled && !isSystemActive && isSetupComplete;
            BtnFinishCalCRP.IsEnabled = enabled;

            // Two-Way Interlock: Lock system activation while calibration is in progress
            ChkActivateSingle.IsEnabled = !enabled && isSetupComplete;
            ChkActivateCRP.IsEnabled = !enabled && isSetupComplete;

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
            SaveCalibration();
            ResetGraphAxesY();
            MessageBox.Show("Calibration complete. Axis limits restored.", "Calibration Saved");
        }

        public void BtnTareThrust_Click(object sender, RoutedEventArgs e) 
        { 
            SendCommand("A"); 
        }

        public void BtnCalThrust_Click(object sender, RoutedEventArgs e) 
        { 
            if (Math.Abs(thrust1) < 1e-5)
            {
                MessageBox.Show("No load detected. Place the 176g weight on the motor mount before calibrating.", "Calibration Warning");
                return;
            }
            scaleT1 = (TARGET_THRUST_N * scaleT1) / thrust1;
            SendCommand($"S_T1_{scaleT1.ToString("G8", CultureInfo.InvariantCulture)}");
            SaveCalibration();
            MessageBox.Show($"Front Thrust calibrated to {TARGET_THRUST_N:F3} N ({0.176:F3} kgf).\nScale Factor: {scaleT1:G6}", "Calibration Saved");
        }

        public void BtnTareTorque_Click(object sender, RoutedEventArgs e) 
        { 
            SendCommand("C"); 
        }

        public void BtnCalTorque_Click(object sender, RoutedEventArgs e) 
        { 
            if (Math.Abs(torque1) < 1e-5)
            {
                MessageBox.Show("No load detected. Place the 176g weight on the torque arm before calibrating.", "Calibration Warning");
                return;
            }
            scaleQ1 = (TARGET_TORQUE_NM * scaleQ1) / torque1;
            SendCommand($"S_Q1_{scaleQ1.ToString("G8", CultureInfo.InvariantCulture)}");
            SaveCalibration();
            MessageBox.Show($"Front Torque calibrated to {TARGET_TORQUE_NM:F4} Nm.\nScale Factor: {scaleQ1:G6}", "Calibration Saved");
        }

        public void BtnTareThrustBack_Click(object sender, RoutedEventArgs e) 
        { 
            SendCommand("E"); 
        }

        public void BtnCalThrustBack_Click(object sender, RoutedEventArgs e) 
        { 
            if (Math.Abs(thrust2) < 1e-5)
            {
                MessageBox.Show("No load detected. Place the 176g weight on the back mount before calibrating.", "Calibration Warning");
                return;
            }
            scaleT2 = (TARGET_THRUST_N * scaleT2) / thrust2;
            SendCommand($"S_T2_{scaleT2.ToString("G8", CultureInfo.InvariantCulture)}");
            SaveCalibration();
            MessageBox.Show($"Back Thrust calibrated to {TARGET_THRUST_N:F3} N.\nScale Factor: {scaleT2:G6}", "Calibration Saved");
        }

        public void BtnTareTorqueBack_Click(object sender, RoutedEventArgs e) 
        { 
            SendCommand("G"); 
        }

        public void BtnCalTorqueBack_Click(object sender, RoutedEventArgs e) 
        { 
            if (Math.Abs(torque2) < 1e-5)
            {
                MessageBox.Show("No load detected. Place the 176g weight on the back torque arm before calibrating.", "Calibration Warning");
                return;
            }
            scaleQ2 = (TARGET_TORQUE_NM * scaleQ2) / torque2;
            SendCommand($"S_Q2_{scaleQ2.ToString("G8", CultureInfo.InvariantCulture)}");
            SaveCalibration();
            MessageBox.Show($"Back Torque calibrated to {TARGET_TORQUE_NM:F4} Nm.\nScale Factor: {scaleQ2:G6}", "Calibration Saved");
        }

        private void ChkActivateSingle_Checked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = true;
            BtnRecordSingle.IsEnabled = true;
            BtnStartAutoSingle.IsEnabled = true;
            TabCRP.IsEnabled = false; 
            TabLogs.IsEnabled = false;

            // Two-Way Interlock: Lock calibration buttons while system is active
            BtnStartCalSingle.IsEnabled = false;
            BtnStartCalCRP.IsEnabled = false;

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

            if (!isCalibrating && isSetupComplete)
            {
                BtnStartCalSingle.IsEnabled = true;
                BtnStartCalCRP.IsEnabled = true;
            }

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

            // Two-Way Interlock: Lock calibration buttons while system is active
            BtnStartCalSingle.IsEnabled = false;
            BtnStartCalCRP.IsEnabled = false;

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

            if (!isCalibrating && isSetupComplete)
            {
                BtnStartCalSingle.IsEnabled = true;
                BtnStartCalCRP.IsEnabled = true;
            }

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
                lock (csvLock)
                {
                    string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                    string mode = isAutoRunning ? "Auto" : "Manual";
                    csvWriter = new StreamWriter(Path.Combine(logDir, $"DynamometerData_{mode}_{timestamp}.csv"));
                    csvWriter.AutoFlush = true;
                    csvWriter.WriteLine("Time,Thrust1,Torque1,RPM1,Voltage1,Current1,Throttle1,ElecPower1,MechPower1,MotorEff1,Thrust2,Torque2,RPM2,Voltage2,Current2,Throttle2,ElecPower2,MechPower2,MotorEff2");

                    startTime = DateTime.Now;
                    nextLogInterval = 0.1;
                    isRecording = true;
                }
                
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
                    if (duration < 0.1) duration = 0.1;
                    txt.Text = duration.ToString("F1", CultureInfo.InvariantCulture);
                }
                else
                {
                    txt.Text = "2.0";
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
                TxtStepDuration_LostFocus(TxtStepDurationSingle, null!);
                autoStepDuration = double.Parse(TxtStepDurationSingle.Text, CultureInfo.InvariantCulture);
                
                for (int i = 0; i < autoTotalSteps; i++) autoTargetsFront[i] = double.Parse(boxes[i].Text, CultureInfo.InvariantCulture);
                
                autoPhase = 0;
                autoTimer = 2.0;
                targetThrottle1 = 0;
                targetThrottle2 = 0;
                isAutoRunning = true;

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
                TxtStepDuration_LostFocus(TxtStepDurationCRP, null!);
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

                BtnStartAutoCRP.Content = "Stop Auto Test";
                SldThrottleFrontCRP.IsEnabled = false;
                SldThrottleBackCRP.IsEnabled = false;

                BtnRecord_Click(sender, e);
            }
            catch { MessageBox.Show("Please enter valid numbers for the step duration and targets."); }
        }
    }
}
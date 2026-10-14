using System;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Controls;
using System.IO.Ports;
using System.Windows.Threading;
using System.Windows.Input;
using System.Linq;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Windows.Interop;
using System.Windows.Media.Imaging;
using System.Threading;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace DynamometerGUI
{
    public struct TelemetryPacket
    {
        public double RawT1, RawQ1, RawT2, RawQ2;
        public double Thrust1, Torque1, RPM1, Volt1, Curr1;
        public double Thrust2, Torque2, RPM2, Volt2, Curr2;
        public double ElecPower1, MechPower1, MotorEff1, PropEff1;
        public double ElecPower2, MechPower2, MotorEff2, PropEff2;
    }

    public sealed class LogRow
    {
        public string Time { get; set; } = "";
        public string Thrust1 { get; set; } = "";
        public string Torque1 { get; set; } = "";
        public string RPM1 { get; set; } = "";
        public string Voltage1 { get; set; } = "";
        public string Current1 { get; set; } = "";
        public string Throttle1 { get; set; } = "";
        public string ElecPower1 { get; set; } = "";
        public string MechPower1 { get; set; } = "";
        public string MotorEff1 { get; set; } = "";
        public string PropEff1 { get; set; } = "";
        public string Thrust2 { get; set; } = "";
        public string Torque2 { get; set; } = "";
        public string RPM2 { get; set; } = "";
        public string Voltage2 { get; set; } = "";
        public string Current2 { get; set; } = "";
        public string Throttle2 { get; set; } = "";
        public string ElecPower2 { get; set; } = "";
        public string MechPower2 { get; set; } = "";
        public string MotorEff2 { get; set; } = "";
        public string PropEff2 { get; set; } = "";
    }

    // 3-Point Monotone Cubic Hermite Spline Calibrator anchored at (0,0)
    public class SplineCalibrator
    {
        public double[] WeightsGrams { get; private set; } = { 176.0, 402.06, 1000.0 };
        public double[] RawCounts { get; private set; } = { 1725.97, 3942.86, 9806.65 };
        public double[] TargetValues { get; private set; } = { 1.72597, 3.94286, 9.80665 };
        public bool[] PointCaptured { get; private set; } = { false, false, false };

        private double[] uKnots = new double[4];
        private double[] yKnots = new double[4];
        private double[] tangents = new double[4];
        private double sign = 1.0;
        private bool isValid = false;

        public void ResetCaptureFlags()
        {
            PointCaptured[0] = false;
            PointCaptured[1] = false;
            PointCaptured[2] = false;
        }

        public void SyncWeightsAndTargets(double[] weightsGrams, bool isTorque)
        {
            double armFactor = isTorque ? 0.150 : 1.0;
            for (int i = 0; i < 3; i++)
            {
                WeightsGrams[i] = weightsGrams[i];
                TargetValues[i] = (weightsGrams[i] / 1000.0) * 9.80665 * armFactor;
            }
        }

        public void SetPoint(int index, double weightG, double rawVal, double targetVal)
        {
            if (index < 0 || index > 2 || Math.Abs(rawVal) < 1e-6) return;

            WeightsGrams[index] = weightG;
            RawCounts[index] = rawVal;
            TargetValues[index] = targetVal;
            PointCaptured[index] = true;

            // Fill uncaptured points proportionally so live readout updates immediately after 1st or 2nd weight
            double currentRatio = rawVal / targetVal;
            for (int i = 0; i < 3; i++)
            {
                if (!PointCaptured[i])
                {
                    RawCounts[i] = TargetValues[i] * currentRatio;
                }
            }
            RebuildSpline();
        }

        public void LoadFromCsvRow(double w1, double w2, double w3, double r1, double r2, double r3, bool isTorque)
        {
            WeightsGrams[0] = w1; WeightsGrams[1] = w2; WeightsGrams[2] = w3;
            RawCounts[0] = r1; RawCounts[1] = r2; RawCounts[2] = r3;

            double armFactor = isTorque ? 0.150 : 1.0;
            TargetValues[0] = (w1 / 1000.0) * 9.80665 * armFactor;
            TargetValues[1] = (w2 / 1000.0) * 9.80665 * armFactor;
            TargetValues[2] = (w3 / 1000.0) * 9.80665 * armFactor;

            PointCaptured[0] = true; PointCaptured[1] = true; PointCaptured[2] = true;
            RebuildSpline();
        }

        public void LoadLegacySingleScale(double scale, bool isTorque)
        {
            if (Math.Abs(scale) < 1e-12) scale = 1.0;
            double armFactor = isTorque ? 0.150 : 1.0;
            for (int i = 0; i < 3; i++)
            {
                TargetValues[i] = (WeightsGrams[i] / 1000.0) * 9.80665 * armFactor;
                RawCounts[i] = TargetValues[i] / scale;
                PointCaptured[i] = true;
            }
            RebuildSpline();
        }

        public void RebuildSpline()
        {
            sign = RawCounts[2] >= 0 ? 1.0 : -1.0;

            // Sort knots by magnitude to guarantee strictly increasing x coordinates
            var pairs = new (double u, double y)[]
            {
                (Math.Abs(RawCounts[0]), TargetValues[0]),
                (Math.Abs(RawCounts[1]), TargetValues[1]),
                (Math.Abs(RawCounts[2]), TargetValues[2])
            }.OrderBy(p => p.u).ToArray();

            uKnots[0] = 0.0;
            yKnots[0] = 0.0;
            for (int i = 0; i < 3; i++)
            {
                uKnots[i + 1] = pairs[i].u;
                yKnots[i + 1] = pairs[i].y;
            }

            // Ensure strictly positive intervals
            for (int i = 1; i < 4; i++)
            {
                if (uKnots[i] <= uKnots[i - 1] + 1e-6)
                {
                    uKnots[i] = uKnots[i - 1] + 1.0;
                }
            }

            double[] d = new double[3];
            for (int i = 0; i < 3; i++)
            {
                d[i] = (yKnots[i + 1] - yKnots[i]) / (uKnots[i + 1] - uKnots[i]);
            }

            tangents[0] = d[0];
            tangents[1] = 0.5 * (d[0] + d[1]);
            tangents[2] = 0.5 * (d[1] + d[2]);
            tangents[3] = d[2];

            // Fritsch-Carlson monotonicity constraint
            for (int i = 0; i < 3; i++)
            {
                if (Math.Abs(d[i]) < 1e-12)
                {
                    tangents[i] = 0.0;
                    tangents[i + 1] = 0.0;
                }
                else
                {
                    double alpha = tangents[i] / d[i];
                    double beta = tangents[i + 1] / d[i];
                    double hypot = Math.Sqrt(alpha * alpha + beta * beta);
                    if (hypot > 3.0)
                    {
                        double tau = 3.0 / hypot;
                        tangents[i] = tau * alpha * d[i];
                        tangents[i + 1] = tau * beta * d[i];
                    }
                }
            }

            isValid = true;
        }

        public double Evaluate(double rawReading)
        {
            if (!isValid) return rawReading;

            double u = rawReading * sign;
            if (u < 0.0)
            {
                return u * tangents[0];
            }
            if (u >= uKnots[3])
            {
                return yKnots[3] + (u - uKnots[3]) * tangents[3];
            }

            int k = 0;
            if (u >= uKnots[2]) k = 2;
            else if (u >= uKnots[1]) k = 1;

            double h = uKnots[k + 1] - uKnots[k];
            double t = (u - uKnots[k]) / h;
            double t2 = t * t;
            double t3 = t2 * t;

            double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
            double h10 = t3 - 2.0 * t2 + t;
            double h01 = -2.0 * t3 + 3.0 * t2;
            double h11 = t3 - t2;

            return h00 * yKnots[k] + h10 * h * tangents[k] + h01 * yKnots[k + 1] + h11 * h * tangents[k + 1];
        }

        public string GetSummaryText()
        {
            return $"Knots: ({RawCounts[0]:F0} -> {TargetValues[0]:F2}), ({RawCounts[1]:F0} -> {TargetValues[1]:F2}), ({RawCounts[2]:F0} -> {TargetValues[2]:F2})";
        }

        public SplineCalibrator Clone()
        {
            var copy = new SplineCalibrator
            {
                sign = this.sign,
                isValid = this.isValid
            };
            Array.Copy(this.WeightsGrams, copy.WeightsGrams, 3);
            Array.Copy(this.RawCounts, copy.RawCounts, 3);
            Array.Copy(this.TargetValues, copy.TargetValues, 3);
            Array.Copy(this.PointCaptured, copy.PointCaptured, 3);
            Array.Copy(this.uKnots, copy.uKnots, 4);
            Array.Copy(this.yKnots, copy.yKnots, 4);
            Array.Copy(this.tangents, copy.tangents, 4);
            return copy;
        }
    }

    public partial class MainWindow : Window
    {
        [DllImport("dwmapi.dll")]
        private static extern int DwmSetWindowAttribute(IntPtr hwnd, int attr, ref int attrValue, int attrSize);

        private const double GRAVITY = 9.80665;
        private const double TORQUE_ARM_M = 0.150;

        // Pre-cached frozen brushes and logos for stutter-free theme switching
        private static readonly SolidColorBrush BrushLightMainBg = CreateFrozenBrush("#edf0f4");
        private static readonly SolidColorBrush BrushLightMainFg = CreateFrozenBrush("#000000");
        private static readonly SolidColorBrush BrushLightBtnBg = CreateFrozenBrush("#D0D9E4");
        private static readonly SolidColorBrush BrushLightHoverBtnBg = CreateFrozenBrush("#B8C5D1");
        private static readonly SolidColorBrush BrushLightDisabledBtnBg = CreateFrozenBrush("#cccccc");
        private static readonly SolidColorBrush BrushLightControlBg = CreateFrozenBrush("#FFFFFF");

        private static readonly SolidColorBrush BrushDarkMainBg = CreateFrozenBrush("#1E1E1E");
        private static readonly SolidColorBrush BrushDarkMainFg = CreateFrozenBrush("#FFFFFF");
        private static readonly SolidColorBrush BrushDarkBtnBg = CreateFrozenBrush("#3E3E42");
        private static readonly SolidColorBrush BrushDarkHoverBtnBg = CreateFrozenBrush("#555555");
        private static readonly SolidColorBrush BrushDarkDisabledBtnBg = CreateFrozenBrush("#2D2D30");
        private static readonly SolidColorBrush BrushDarkControlBg = CreateFrozenBrush("#2D2D30");

        private static readonly ScottPlot.Color SpDarkBg = ScottPlot.Color.FromHex("#1E1E1E");
        private static readonly ScottPlot.Color SpDarkFg = ScottPlot.Color.FromHex("#FFFFFF");
        private static readonly ScottPlot.Color SpDarkGrid = ScottPlot.Color.FromHex("#444444");
        private static readonly ScottPlot.Color SpDarkLine = ScottPlot.Color.FromHex("#FFC107");

        private static readonly ScottPlot.Color SpLightBg = ScottPlot.Color.FromHex("#edf0f4");
        private static readonly ScottPlot.Color SpLightGrid = ScottPlot.Color.FromHex("#DDDDDD");

        private BitmapImage? logoLight;
        private BitmapImage? logoDark;

        SerialPort? arduinoPort;
        Thread? serialWorkerThread;
        private volatile bool isSerialWorkerRunning = false;
        private readonly ConcurrentQueue<TelemetryPacket> telemetryQueue = new();
        private readonly ConcurrentQueue<string> serialCommandQueue = new();
        private readonly object csvLock = new object();
        private volatile bool isCRPSelected = false;
        private bool lastActiveTestTabIsCRP = false;

        DispatcherTimer masterLoopTimer;

        DateTime startTime;
        DateTime stopPressTime;
        double nextLogInterval;
        bool isRecording = false;
        bool isStopping = false;
        StreamWriter? csvWriter = null;
        
        // Live data variables
        double rawThrust1, rawTorque1, rawThrust2, rawTorque2;
        double thrust1, torque1, rpm1, volt1, curr1;
        double thrust2, torque2, rpm2, volt2, curr2;
        double elecPower1, mechPower1, motorEff1, propEff1;
        double elecPower2, mechPower2, motorEff2, propEff2;

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
        int autoTotalSteps = 6;
        double autoStepDuration = 10.0;
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
        private int logLoadVersion = 0;

        // Application Local Directories
        string logDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoLogs");
        string calSingleDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoCalibrationsSingle");
        string calCRPDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "DynoCalibrationsCRP");
        
        // Calibration State & Splines
        bool isCalibrating = false;
        double[] calWeightsGrams = new double[] { 176.0, 402.06, 1000.0 };

        // Active splines accessed by background thread via volatile reference swap
        private volatile SplineCalibrator activeSplineT1 = new();
        private volatile SplineCalibrator activeSplineQ1 = new();
        private volatile SplineCalibrator activeSplineT2 = new();
        private volatile SplineCalibrator activeSplineQ2 = new();

        // Cached Single and CRP Splines
        private SplineCalibrator singleSplineT1 = new();
        private SplineCalibrator singleSplineQ1 = new();
        private SplineCalibrator crpSplineT1 = new();
        private SplineCalibrator crpSplineQ1 = new();
        private SplineCalibrator crpSplineT2 = new();
        private SplineCalibrator crpSplineQ2 = new();

        public MainWindow()
        {
            InitializeComponent();
            
            this.SetResourceReference(Window.BackgroundProperty, "MainBg");
            this.SetResourceReference(Window.ForegroundProperty, "MainFg");
            
            Directory.CreateDirectory(logDir);
            Directory.CreateDirectory(calSingleDir);
            Directory.CreateDirectory(calCRPDir);

            PreloadLogos();

            // Pre-initialize DataGrid item container generator
            DataGridLogs.ItemsSource = Array.Empty<LogRow>();

            masterLoopTimer = new DispatcherTimer(DispatcherPriority.Normal);
            masterLoopTimer.Interval = TimeSpan.FromMilliseconds(50);
            masterLoopTimer.Tick += MasterLoopTimer_Tick;

            // Lock controls during startup
            ChkActivateSingle.IsEnabled = false;
            ChkActivateCRP.IsEnabled = false;
            BtnTareSingleTab.IsEnabled = false;
            BtnTareCRPTab.IsEnabled = false;
            BtnStartCal.IsEnabled = false;
            BtnPostCalTareAll.IsEnabled = false;

            UpdateTargetWeightLabels(formatBoxes: true);
            InitializeGraphAxes();
            LoadLogFiles();
            LoadLatestCalibrationFromDisk();
            AutoConnectArduino(); 
        }

        private void PreloadLogos()
        {
            try
            {
                logoLight = new BitmapImage();
                logoLight.BeginInit();
                logoLight.UriSource = new Uri("pack://application:,,,/Dynamometer Logo NoText NoBackground.png");
                logoLight.CacheOption = BitmapCacheOption.OnLoad;
                logoLight.EndInit();
                logoLight.Freeze();
            }
            catch { }

            try
            {
                logoDark = new BitmapImage();
                logoDark.BeginInit();
                logoDark.UriSource = new Uri("pack://application:,,,/Dynamometer Logo NoText DarkMode NoBackground.png");
                logoDark.CacheOption = BitmapCacheOption.OnLoad;
                logoDark.EndInit();
                logoDark.Freeze();
            }
            catch { }
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            ApplyTitleBarTheme(CmbTheme.SelectedIndex == 1);
            UpdateHeaderCenterAlignment();
            masterLoopTimer.Start();
        }

        private void HeaderGrid_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            UpdateHeaderCenterAlignment();
        }

        private void UpdateHeaderCenterAlignment()
        {
            if (HeaderGrid == null || PanelLogoTitle == null || PanelEmergencyStop == null || PanelLiveDataInputsRoot == null) return;
            if (PanelLiveDataInputsRoot.Visibility != Visibility.Visible) return;

            double totalWidth = HeaderGrid.ActualWidth;
            if (totalWidth <= 0) return;

            double leftFixedWidth = PanelLogoTitle.ActualWidth + PanelLogoTitle.Margin.Left + PanelLogoTitle.Margin.Right
                                  + PanelEmergencyStop.ActualWidth + PanelEmergencyStop.Margin.Left + PanelEmergencyStop.Margin.Right;

            double inputsColumnWidth = PanelBackDataInputs.Visibility == Visibility.Visible ? 300.0 : 150.0;
            double inputsTotalWidth = inputsColumnWidth + PanelLiveDataInputsRoot.Margin.Left + PanelLiveDataInputsRoot.Margin.Right;

            double targetSpacer = (totalWidth / 2.0) - leftFixedWidth - inputsTotalWidth;
            ColLeftCenterSpacer.Width = new GridLength(Math.Max(0.0, targetSpacer));
        }

        private void Window_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (Keyboard.FocusedElement is TextBox focusedBox)
            {
                DependencyObject? clicked = e.OriginalSource as DependencyObject;
                while (clicked != null && clicked != focusedBox)
                {
                    clicked = VisualTreeHelper.GetParent(clicked);
                }
                if (clicked != focusedBox)
                {
                    Keyboard.ClearFocus();
                    FocusManager.SetFocusedElement(this, this);
                }
            }
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
                    port.ReadTimeout = 200;
                    port.DtrEnable = true;
                    port.Open();

                    arduinoPort = port;
                    isSerialWorkerRunning = true;
                    
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

                    // Flush queued outgoing commands on the worker thread so the UI thread never blocks
                    while (serialCommandQueue.TryDequeue(out string? cmd))
                    {
                        if (!string.IsNullOrEmpty(cmd))
                        {
                            arduinoPort.Write(cmd + "\n");
                        }
                    }

                    string line = arduinoPort.ReadLine().Trim();

                    if (!isSetupComplete || line.StartsWith("CAL:") || line.StartsWith("---"))
                    {
                        continue;
                    }

                    string[] parts = line.Split(',');
                    if (parts.Length == 10 &&
                        double.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out double rawT1) &&
                        double.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double rawQ1) &&
                        double.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out double r1) &&
                        double.TryParse(parts[3], NumberStyles.Float, CultureInfo.InvariantCulture, out double v1) &&
                        double.TryParse(parts[4], NumberStyles.Float, CultureInfo.InvariantCulture, out double c1) &&
                        double.TryParse(parts[5], NumberStyles.Float, CultureInfo.InvariantCulture, out double rawT2) &&
                        double.TryParse(parts[6], NumberStyles.Float, CultureInfo.InvariantCulture, out double rawQ2) &&
                        double.TryParse(parts[7], NumberStyles.Float, CultureInfo.InvariantCulture, out double r2) &&
                        double.TryParse(parts[8], NumberStyles.Float, CultureInfo.InvariantCulture, out double v2) &&
                        double.TryParse(parts[9], NumberStyles.Float, CultureInfo.InvariantCulture, out double c2))
                    {
                        double t1 = activeSplineT1.Evaluate(rawT1);
                        double q1 = activeSplineQ1.Evaluate(rawQ1);
                        double t2 = activeSplineT2.Evaluate(rawT2);
                        double q2 = activeSplineQ2.Evaluate(rawQ2);

                        double ep1 = v1 * c1;
                        double mp1 = q1 * r1 * (2.0 * Math.PI / 60.0);
                        double me1 = ep1 > 0 ? (mp1 / ep1) * 100.0 : 0;
                        double pe1 = mp1 > 0 ? (t1 / mp1) : 0;

                        double ep2 = v2 * c2;
                        double mp2 = q2 * r2 * (2.0 * Math.PI / 60.0);
                        double me2 = ep2 > 0 ? (mp2 / ep2) * 100.0 : 0;
                        double pe2 = mp2 > 0 ? (t2 / mp2) : 0;

                        telemetryQueue.Enqueue(new TelemetryPacket
                        {
                            RawT1 = rawT1, RawQ1 = rawQ1, RawT2 = rawT2, RawQ2 = rawQ2,
                            Thrust1 = t1, Torque1 = q1, RPM1 = r1, Volt1 = v1, Curr1 = c1,
                            Thrust2 = t2, Torque2 = q2, RPM2 = r2, Volt2 = v2, Curr2 = c2,
                            ElecPower1 = ep1, MechPower1 = mp1, MotorEff1 = me1, PropEff1 = pe1,
                            ElecPower2 = ep2, MechPower2 = mp2, MotorEff2 = me2, PropEff2 = pe2
                        });

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
                                        bool isCrp = isCRPSelected;

                                        string csvLine = string.Format(CultureInfo.InvariantCulture,
                                            "{0},{1:F3},{2:F3},{3:F0},{4:F2},{5:F2},{6},{7:F2},{8:F2},{9:F2},{10:F3}," +
                                            "{11:F3},{12:F3},{13:F0},{14:F2},{15:F2},{16},{17:F2},{18:F2},{19:F2},{20:F3}",
                                            elapsedLogTime,
                                            t1, q1, r1, v1, c1, actT1, ep1, mp1, me1, pe1,
                                            isCrp ? t2 : 0, isCrp ? q2 : 0, isCrp ? r2 : 0, isCrp ? v2 : 0, isCrp ? c2 : 0,
                                            actT2, isCrp ? ep2 : 0, isCrp ? mp2 : 0, isCrp ? me2 : 0, isCrp ? pe2 : 0);

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
                if (startupCountdownTicks % 20 == 0)
                {
                    int secondsLeft = startupCountdownTicks / 20;
                    TxtStartupCountdown.Text = $"Hardware Initializing... Please wait ({secondsLeft}s)";
                }

                if (startupCountdownTicks <= 0)
                {
                    isSetupComplete = true;
                    BorderStartupNotice.Visibility = Visibility.Collapsed;

                    EnsureArduinoUnityScale();
                    ApplyActiveCalibrationForCurrentTab();

                    ChkActivateSingle.IsEnabled = true;
                    ChkActivateCRP.IsEnabled = true;
                    BtnTareSingleTab.IsEnabled = true;
                    BtnTareCRPTab.IsEnabled = true;
                    BtnStartCal.IsEnabled = true;
                    BtnPostCalTareAll.IsEnabled = true;

                    ResetGraphAxesY();
                }
                return;
            }

            // 2. Process Telemetry Packets
            bool hasNewData = false;
            while (telemetryQueue.TryDequeue(out var packet))
            {
                rawThrust1 = packet.RawT1; rawTorque1 = packet.RawQ1;
                rawThrust2 = packet.RawT2; rawTorque2 = packet.RawQ2;
                thrust1 = packet.Thrust1; torque1 = packet.Torque1; rpm1 = packet.RPM1; volt1 = packet.Volt1; curr1 = packet.Curr1;
                thrust2 = packet.Thrust2; torque2 = packet.Torque2; rpm2 = packet.RPM2; volt2 = packet.Volt2; curr2 = packet.Curr2;
                elecPower1 = packet.ElecPower1; mechPower1 = packet.MechPower1; motorEff1 = packet.MotorEff1; propEff1 = packet.PropEff1;
                elecPower2 = packet.ElecPower2; mechPower2 = packet.MechPower2; motorEff2 = packet.MotorEff2; propEff2 = packet.PropEff2;

                liveGraphTime += 0.1;
                hasNewData = true;

                double displayThrust1 = CmbThrustUnit.SelectedIndex == 0 ? thrust1 / GRAVITY : thrust1;
                double displayThrust2 = CmbThrustUnit.SelectedIndex == 0 ? thrust2 / GRAVITY : thrust2;

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

            // 3. Render Active Charts
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

            // 4. Update Recording Stopwatch at 20 Hz
            if (isRecording)
            {
                TimeSpan elapsed = DateTime.Now - startTime;
                string timeFormat = $"Time: {elapsed:hh\\:mm\\:ss\\.fff}";
                TxtTimerSingle.Text = timeFormat;
                TxtTimerCRP.Text = timeFormat;
            }

            // 5. Update UI Text Labels at 5 Hz
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
                TxtFrontPropEff.Text = $"Prop Eff: {propEff1:F3} N/W";

                TxtBackVoltage.Text = $"Voltage: {volt2:F2} V";
                TxtBackCurrent.Text = $"Current: {curr2:F2} A";
                TxtBackRPM.Text = $"RPM: {rpm2:F0}";
                TxtBackElecPower.Text = $"Elec Power: {elecPower2:F2} W";
                TxtBackMechPower.Text = $"Mech Power: {mechPower2:F2} W";
                TxtBackMotorEff.Text = $"Motor Eff: {motorEff2:F2} %";
                TxtBackPropEff.Text = $"Prop Eff: {propEff2:F3} N/W";

                if (TabCalibration.IsSelected)
                {
                    TxtCalLiveThrust1.Text = $"Front Thrust: {thrust1:F3} N ({(thrust1 / GRAVITY):F3} kgf)";
                    TxtCalLiveTorque1.Text = $"Front Torque: {torque1:F4} Nm";
                    TxtCalScaleT1.Text = activeSplineT1.GetSummaryText();
                    TxtCalScaleQ1.Text = activeSplineQ1.GetSummaryText();

                    TxtCalLiveThrust2.Text = $"Back Thrust: {thrust2:F3} N ({(thrust2 / GRAVITY):F3} kgf)";
                    TxtCalLiveTorque2.Text = $"Back Torque: {torque2:F4} Nm";
                    TxtCalScaleT2.Text = activeSplineT2.GetSummaryText();
                    TxtCalScaleQ2.Text = activeSplineQ2.GetSummaryText();
                }
            }

            // 6. Emergency Stop Countdown
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

            // 7. Automatic Test Sequencing
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

            // 8. Throttle Ramping
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

            // 9. Auto Test Interlock
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
            serialCommandQueue.Enqueue(command);
        }

        private void EnsureArduinoUnityScale()
        {
            SendCommand("S_T1_1.0");
            SendCommand("S_Q1_1.0");
            SendCommand("S_T2_1.0");
            SendCommand("S_Q2_1.0");
        }

        private void ParseCalWeightsFromUI(bool formatBoxes)
        {
            if (TxtCalWeight1 == null || TxtCalWeight2 == null || TxtCalWeight3 == null) return;

            double[] defaults = { 176.0, 402.06, 1000.0 };
            TextBox[] boxes = { TxtCalWeight1, TxtCalWeight2, TxtCalWeight3 };

            for (int i = 0; i < 3; i++)
            {
                string rawText = boxes[i].Text.Trim().Replace(',', '.');
                if (double.TryParse(rawText, NumberStyles.Float, CultureInfo.InvariantCulture, out double val) && val > 0.1 && val <= 50000.0)
                {
                    calWeightsGrams[i] = val;
                    if (formatBoxes)
                    {
                        boxes[i].Text = val.ToString("G6", CultureInfo.InvariantCulture);
                    }
                }
                else if (formatBoxes)
                {
                    calWeightsGrams[i] = defaults[i];
                    boxes[i].Text = defaults[i].ToString("G6", CultureInfo.InvariantCulture);
                }
            }
        }

        private void UpdateTargetWeightLabels(bool formatBoxes)
        {
            if (TxtCalTargetThrust == null || TxtCalTargetTorque == null) return;
            ParseCalWeightsFromUI(formatBoxes);

            double t1 = (calWeightsGrams[0] / 1000.0) * GRAVITY;
            double t2 = (calWeightsGrams[1] / 1000.0) * GRAVITY;
            double t3 = (calWeightsGrams[2] / 1000.0) * GRAVITY;

            double q1 = t1 * TORQUE_ARM_M;
            double q2 = t2 * TORQUE_ARM_M;
            double q3 = t3 * TORQUE_ARM_M;

            TxtCalTargetThrust.Text = $"Target Thrusts: {t1:F3} N / {t2:F3} N / {t3:F3} N";
            TxtCalTargetTorque.Text = $"Target Torques: {q1:F4} Nm / {q2:F4} Nm / {q3:F4} Nm";
        }

        private void TxtCalWeight_TextChanged(object sender, TextChangedEventArgs e)
        {
            UpdateTargetWeightLabels(formatBoxes: false);
        }

        private void TxtCalWeight_LostFocus(object sender, RoutedEventArgs e)
        {
            UpdateTargetWeightLabels(formatBoxes: true);
        }

        private void TxtCalWeight_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter || e.Key == Key.Return)
            {
                UpdateTargetWeightLabels(formatBoxes: true);
                Keyboard.ClearFocus();
                FocusManager.SetFocusedElement(this, this);
                e.Handled = true;
            }
        }

        private void CmbCalMode_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (BtnStartCal == null || BtnFinishCal == null) return;
            bool isCrpMode = CmbCalMode.SelectedIndex == 1;

            BtnStartCal.Content = isCrpMode ? "Start CRP Calibration" : "Start Single Calibration";
            BtnFinishCal.Content = isCrpMode ? "Finish & Save CRP Calibration" : "Finish & Save Single Calibration";

            if (PanelCalBackThrust != null) PanelCalBackThrust.Visibility = isCrpMode ? Visibility.Visible : Visibility.Collapsed;
            if (PanelCalBackTorque != null) PanelCalBackTorque.Visibility = isCrpMode ? Visibility.Visible : Visibility.Collapsed;
            if (PanelGuideCRP != null) PanelGuideCRP.Visibility = isCrpMode ? Visibility.Visible : Visibility.Collapsed;
            if (PanelGuideSingle != null) PanelGuideSingle.Visibility = isCrpMode ? Visibility.Collapsed : Visibility.Visible;

            if (TabCalibration != null && TabCalibration.IsSelected)
            {
                PanelBackDataInputs.Visibility = isCrpMode ? Visibility.Visible : Visibility.Collapsed;
                PanelBackDataDerived.Visibility = isCrpMode ? Visibility.Visible : Visibility.Collapsed;
                UpdateHeaderCenterAlignment();
            }

            if (!isCalibrating) ApplyActiveCalibrationForCurrentTab();
        }

        private void ApplyActiveCalibrationForCurrentTab()
        {
            if (TabSingle.IsSelected)
            {
                activeSplineT1 = singleSplineT1;
                activeSplineQ1 = singleSplineQ1;
            }
            else if (TabCRP.IsSelected)
            {
                activeSplineT1 = crpSplineT1;
                activeSplineQ1 = crpSplineQ1;
                activeSplineT2 = crpSplineT2;
                activeSplineQ2 = crpSplineQ2;
            }
            else if (TabCalibration.IsSelected && !isCalibrating)
            {
                bool isCrpMode = CmbCalMode != null && CmbCalMode.SelectedIndex == 1;
                activeSplineT1 = isCrpMode ? crpSplineT1 : singleSplineT1;
                activeSplineQ1 = isCrpMode ? crpSplineQ1 : singleSplineQ1;
                activeSplineT2 = crpSplineT2;
                activeSplineQ2 = crpSplineQ2;
            }
        }

        private void LoadLatestCalibrationFromDisk()
        {
            try
            {
                var latestSingle = Directory.GetFiles(calSingleDir, "Calibration_*.csv")
                                            .OrderByDescending(f => new FileInfo(f).LastWriteTime)
                                            .FirstOrDefault();
                if (latestSingle != null)
                {
                    string[] lines = File.ReadAllLines(latestSingle);
                    if (lines.Length > 1)
                    {
                        string[] p = lines[1].Split(',');
                        if (p.Length >= 9)
                        {
                            double w1 = double.Parse(p[0], CultureInfo.InvariantCulture);
                            double w2 = double.Parse(p[1], CultureInfo.InvariantCulture);
                            double w3 = double.Parse(p[2], CultureInfo.InvariantCulture);

                            singleSplineT1.LoadFromCsvRow(
                                w1, w2, w3,
                                double.Parse(p[3], CultureInfo.InvariantCulture),
                                double.Parse(p[4], CultureInfo.InvariantCulture),
                                double.Parse(p[5], CultureInfo.InvariantCulture),
                                isTorque: false);

                            singleSplineQ1.LoadFromCsvRow(
                                w1, w2, w3,
                                double.Parse(p[6], CultureInfo.InvariantCulture),
                                double.Parse(p[7], CultureInfo.InvariantCulture),
                                double.Parse(p[8], CultureInfo.InvariantCulture),
                                isTorque: true);
                        }
                        else if (p.Length >= 2)
                        {
                            singleSplineT1.LoadLegacySingleScale(double.Parse(p[0], CultureInfo.InvariantCulture), isTorque: false);
                            singleSplineQ1.LoadLegacySingleScale(double.Parse(p[1], CultureInfo.InvariantCulture), isTorque: true);
                        }
                    }
                }

                var latestCRP = Directory.GetFiles(calCRPDir, "Calibration_*.csv")
                                         .OrderByDescending(f => new FileInfo(f).LastWriteTime)
                                         .FirstOrDefault();
                if (latestCRP != null)
                {
                    string[] lines = File.ReadAllLines(latestCRP);
                    if (lines.Length > 1)
                    {
                        string[] p = lines[1].Split(',');
                        if (p.Length >= 15)
                        {
                            double w1 = double.Parse(p[0], CultureInfo.InvariantCulture);
                            double w2 = double.Parse(p[1], CultureInfo.InvariantCulture);
                            double w3 = double.Parse(p[2], CultureInfo.InvariantCulture);

                            calWeightsGrams[0] = w1;
                            calWeightsGrams[1] = w2;
                            calWeightsGrams[2] = w3;

                            if (TxtCalWeight1 != null)
                            {
                                TxtCalWeight1.Text = w1.ToString("G6", CultureInfo.InvariantCulture);
                                TxtCalWeight2.Text = w2.ToString("G6", CultureInfo.InvariantCulture);
                                TxtCalWeight3.Text = w3.ToString("G6", CultureInfo.InvariantCulture);
                                UpdateTargetWeightLabels(formatBoxes: false);
                            }

                            crpSplineT1.LoadFromCsvRow(
                                w1, w2, w3,
                                double.Parse(p[3], CultureInfo.InvariantCulture),
                                double.Parse(p[4], CultureInfo.InvariantCulture),
                                double.Parse(p[5], CultureInfo.InvariantCulture),
                                isTorque: false);

                            crpSplineQ1.LoadFromCsvRow(
                                w1, w2, w3,
                                double.Parse(p[6], CultureInfo.InvariantCulture),
                                double.Parse(p[7], CultureInfo.InvariantCulture),
                                double.Parse(p[8], CultureInfo.InvariantCulture),
                                isTorque: true);

                            crpSplineT2.LoadFromCsvRow(
                                w1, w2, w3,
                                double.Parse(p[9], CultureInfo.InvariantCulture),
                                double.Parse(p[10], CultureInfo.InvariantCulture),
                                double.Parse(p[11], CultureInfo.InvariantCulture),
                                isTorque: false);

                            crpSplineQ2.LoadFromCsvRow(
                                w1, w2, w3,
                                double.Parse(p[12], CultureInfo.InvariantCulture),
                                double.Parse(p[13], CultureInfo.InvariantCulture),
                                double.Parse(p[14], CultureInfo.InvariantCulture),
                                isTorque: true);
                        }
                        else if (p.Length >= 4)
                        {
                            crpSplineT1.LoadLegacySingleScale(double.Parse(p[0], CultureInfo.InvariantCulture), isTorque: false);
                            crpSplineQ1.LoadLegacySingleScale(double.Parse(p[1], CultureInfo.InvariantCulture), isTorque: true);
                            crpSplineT2.LoadLegacySingleScale(double.Parse(p[2], CultureInfo.InvariantCulture), isTorque: false);
                            crpSplineQ2.LoadLegacySingleScale(double.Parse(p[3], CultureInfo.InvariantCulture), isTorque: true);
                        }
                    }
                }

                ApplyActiveCalibrationForCurrentTab();
            }
            catch { }
        }

        private void SaveSingleCalibrationToDisk()
        {
            try
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string calFile = Path.Combine(calSingleDir, $"Calibration_{timestamp}_Single.csv");
                
                string header = "W1_g,W2_g,W3_g,RawT1_1,RawT1_2,RawT1_3,RawQ1_1,RawQ1_2,RawQ1_3\n";
                string data = string.Format(CultureInfo.InvariantCulture,
                    "{0:G6},{1:G6},{2:G6},{3:G8},{4:G8},{5:G8},{6:G8},{7:G8},{8:G8}\n",
                    calWeightsGrams[0], calWeightsGrams[1], calWeightsGrams[2],
                    singleSplineT1.RawCounts[0], singleSplineT1.RawCounts[1], singleSplineT1.RawCounts[2],
                    singleSplineQ1.RawCounts[0], singleSplineQ1.RawCounts[1], singleSplineQ1.RawCounts[2]);

                File.WriteAllText(calFile, header + data);
            }
            catch { }
        }

        private void SaveCRPCalibrationToDisk()
        {
            try
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string calFile = Path.Combine(calCRPDir, $"Calibration_{timestamp}_CRP.csv");
                
                string header = "W1_g,W2_g,W3_g,RawT1_1,RawT1_2,RawT1_3,RawQ1_1,RawQ1_2,RawQ1_3,RawT2_1,RawT2_2,RawT2_3,RawQ2_1,RawQ2_2,RawQ2_3\n";
                string data = string.Format(CultureInfo.InvariantCulture,
                    "{0:G6},{1:G6},{2:G6},{3:G8},{4:G8},{5:G8},{6:G8},{7:G8},{8:G8},{9:G8},{10:G8},{11:G8},{12:G8},{13:G8},{14:G8}\n",
                    calWeightsGrams[0], calWeightsGrams[1], calWeightsGrams[2],
                    crpSplineT1.RawCounts[0], crpSplineT1.RawCounts[1], crpSplineT1.RawCounts[2],
                    crpSplineQ1.RawCounts[0], crpSplineQ1.RawCounts[1], crpSplineQ1.RawCounts[2],
                    crpSplineT2.RawCounts[0], crpSplineT2.RawCounts[1], crpSplineT2.RawCounts[2],
                    crpSplineQ2.RawCounts[0], crpSplineQ2.RawCounts[1], crpSplineQ2.RawCounts[2]);

                File.WriteAllText(calFile, header + data);
            }
            catch { }
        }

        private void ResetGraphAxesY()
        {
            double thrustMin = CmbThrustUnit.SelectedIndex == 0 ? -0.2 : -0.2 * GRAVITY;
            double thrustMax = CmbThrustUnit.SelectedIndex == 0 ? 2.0 : 2.0 * GRAVITY;

            PlotFrontThrustSingle.Plot.Title("Front Thrust");
            PlotFrontThrustSingle.Plot.YLabel(CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)");
            PlotFrontThrustSingle.Plot.Axes.SetLimitsY(thrustMin, thrustMax);

            PlotFrontTorqueSingle.Plot.Title("Front Torque");
            PlotFrontTorqueSingle.Plot.YLabel("Torque (Nm)");
            PlotFrontTorqueSingle.Plot.Axes.SetLimitsY(-0.1, 1.0);

            PlotFrontRPMSingle.Plot.Title("Front Rotational Speed");
            PlotFrontRPMSingle.Plot.YLabel("Rotational Speed (RPM)");
            PlotFrontRPMSingle.Plot.Axes.SetLimitsY(-400, 8000);

            PlotFrontThrustCRP.Plot.Title("Front Thrust");
            PlotFrontThrustCRP.Plot.YLabel(CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)");
            PlotFrontThrustCRP.Plot.Axes.SetLimitsY(thrustMin, thrustMax);

            PlotFrontTorqueCRP.Plot.Title("Front Torque");
            PlotFrontTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotFrontTorqueCRP.Plot.Axes.SetLimitsY(-0.1, 1.0);

            PlotFrontRPMCRP.Plot.Title("Front Rotational Speed");
            PlotFrontRPMCRP.Plot.YLabel("Rotational Speed (RPM)");
            PlotFrontRPMCRP.Plot.Axes.SetLimitsY(-400, 8000);

            PlotBackThrustCRP.Plot.Title("Back Thrust");
            PlotBackThrustCRP.Plot.YLabel(CmbThrustUnit.SelectedIndex == 0 ? "Thrust (kgf)" : "Thrust (N)");
            PlotBackThrustCRP.Plot.Axes.SetLimitsY(thrustMin, thrustMax);

            PlotBackTorqueCRP.Plot.Title("Back Torque");
            PlotBackTorqueCRP.Plot.YLabel("Torque (Nm)");
            PlotBackTorqueCRP.Plot.Axes.SetLimitsY(-0.1, 1.0);

            PlotBackRPMCRP.Plot.Title("Back Rotational Speed");
            PlotBackRPMCRP.Plot.YLabel("Rotational Speed (RPM)");
            PlotBackRPMCRP.Plot.Axes.SetLimitsY(-400, 8000);

            RefreshVisiblePlots();
        }

        private void BtnResetGraphViews_Click(object sender, RoutedEventArgs e)
        {
            double minX = liveGraphTime < 15.0 ? 0 : liveGraphTime - 15.0;
            double maxX = liveGraphTime < 15.0 ? 30.0 : liveGraphTime + 15.0;

            var plots = new[]
            {
                PlotFrontThrustSingle, PlotFrontTorqueSingle, PlotFrontRPMSingle,
                PlotFrontThrustCRP, PlotFrontTorqueCRP, PlotFrontRPMCRP,
                PlotBackThrustCRP, PlotBackTorqueCRP, PlotBackRPMCRP
            };

            foreach (var p in plots)
            {
                p.Plot.Axes.SetLimitsX(minX, maxX);
            }

            ResetGraphAxesY();
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
            
            BtnStartAutoSingle.Content = "Start Automatic Test";
            BtnStartAutoCRP.Content = "Start Automatic Test";
            
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
            isCRPSelected = TabCRP.IsSelected;

            if (e.Source is TabControl && PanelLiveDataInputsRoot != null)
            {
                if (TabSingle.IsSelected)
                {
                    lastActiveTestTabIsCRP = false;
                    PanelLiveDataInputsRoot.Visibility = Visibility.Visible;
                    PanelLiveDataDerivedRoot.Visibility = Visibility.Visible;
                    PanelBackDataInputs.Visibility = Visibility.Collapsed;
                    PanelBackDataDerived.Visibility = Visibility.Collapsed;
                    UpdateHeaderCenterAlignment();

                    if (!isCalibrating && !isRecording && isSetupComplete)
                    {
                        ApplyActiveCalibrationForCurrentTab();
                    }
                }
                else if (TabCRP.IsSelected)
                {
                    lastActiveTestTabIsCRP = true;
                    PanelLiveDataInputsRoot.Visibility = Visibility.Visible;
                    PanelLiveDataDerivedRoot.Visibility = Visibility.Visible;
                    PanelBackDataInputs.Visibility = Visibility.Visible;
                    PanelBackDataDerived.Visibility = Visibility.Visible;
                    UpdateHeaderCenterAlignment();

                    if (!isCalibrating && !isRecording && isSetupComplete)
                    {
                        ApplyActiveCalibrationForCurrentTab();
                    }
                }
                else if (TabCalibration.IsSelected)
                {
                    if (!isCalibrating && CmbCalMode != null)
                    {
                        CmbCalMode.SelectedIndex = lastActiveTestTabIsCRP ? 1 : 0;
                    }

                    PanelLiveDataInputsRoot.Visibility = Visibility.Visible;
                    PanelLiveDataDerivedRoot.Visibility = Visibility.Visible;
                    bool isCrpMode = CmbCalMode != null && CmbCalMode.SelectedIndex == 1;
                    PanelBackDataInputs.Visibility = isCrpMode ? Visibility.Visible : Visibility.Collapsed;
                    PanelBackDataDerived.Visibility = isCrpMode ? Visibility.Visible : Visibility.Collapsed;
                    UpdateHeaderCenterAlignment();
                    ApplyActiveCalibrationForCurrentTab();
                }
                else if (TabLogs.IsSelected)
                {
                    PanelLiveDataInputsRoot.Visibility = Visibility.Hidden;
                    PanelLiveDataDerivedRoot.Visibility = Visibility.Hidden;
                    LoadLogFiles();
                }
            }
        }

        private void LoadLogFiles()
        {
            if (ListLogFiles == null || !Directory.Exists(logDir)) return;
            ListLogFiles.Items.Clear();
            var files = new DirectoryInfo(logDir)
                            .GetFiles("DynamometerData_*.csv")
                            .OrderByDescending(f => f.LastWriteTime)
                            .Select(f => f.Name)
                            .ToList();
            foreach (var file in files) ListLogFiles.Items.Add(file);
        }

        private void ListLogFiles_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (ListLogFiles.SelectedItem == null) return;
            string file = Path.Combine(logDir, ListLogFiles.SelectedItem.ToString() ?? "");
            if (!File.Exists(file)) return;

            int requestVersion = Interlocked.Increment(ref logLoadVersion);

            Task.Run(() =>
            {
                try
                {
                    var rows = new List<LogRow>(2048);
                    using var stream = new FileStream(file, FileMode.Open, FileAccess.Read, FileShare.ReadWrite, 65536);
                    using var reader = new StreamReader(stream);

                    // Skip header line
                    if (reader.ReadLine() == null) return;

                    string? line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        if (string.IsNullOrWhiteSpace(line)) continue;
                        string[] p = line.Split(',');
                        if (p.Length < 11) continue;

                        rows.Add(new LogRow
                        {
                            Time = p[0],
                            Thrust1 = p[1],
                            Torque1 = p[2],
                            RPM1 = p[3],
                            Voltage1 = p[4],
                            Current1 = p[5],
                            Throttle1 = p[6],
                            ElecPower1 = p[7],
                            MechPower1 = p[8],
                            MotorEff1 = p[9],
                            PropEff1 = p[10],
                            Thrust2 = p.Length > 11 ? p[11] : "0",
                            Torque2 = p.Length > 12 ? p[12] : "0",
                            RPM2 = p.Length > 13 ? p[13] : "0",
                            Voltage2 = p.Length > 14 ? p[14] : "0",
                            Current2 = p.Length > 15 ? p[15] : "0",
                            Throttle2 = p.Length > 16 ? p[16] : "0",
                            ElecPower2 = p.Length > 17 ? p[17] : "0",
                            MechPower2 = p.Length > 18 ? p[18] : "0",
                            MotorEff2 = p.Length > 19 ? p[19] : "0",
                            PropEff2 = p.Length > 20 ? p[20] : "0"
                        });
                    }

                    if (requestVersion == logLoadVersion)
                    {
                        Dispatcher.BeginInvoke(DispatcherPriority.Render, new Action(() =>
                        {
                            if (requestVersion == logLoadVersion)
                            {
                                DataGridLogs.ItemsSource = rows;
                            }
                        }));
                    }
                }
                catch { }
            });
        }

        private void BtnOpenLogsFolder_Click(object sender, RoutedEventArgs e)
        {
            Process.Start(new ProcessStartInfo { FileName = logDir, UseShellExecute = true });
        }

        private void BtnOpenSingleCalFolder_Click(object sender, RoutedEventArgs e)
        {
            Process.Start(new ProcessStartInfo { FileName = calSingleDir, UseShellExecute = true });
        }

        private void BtnOpenCRPCalFolder_Click(object sender, RoutedEventArgs e)
        {
            Process.Start(new ProcessStartInfo { FileName = calCRPDir, UseShellExecute = true });
        }

        private void SetCalibrationButtonsState(bool calibrating)
        {
            bool isSystemActive = (ChkActivateSingle.IsChecked == true || ChkActivateCRP.IsChecked == true);
            bool isCrpMode = CmbCalMode.SelectedIndex == 1;

            CmbCalMode.IsEnabled = !calibrating;
            TxtCalWeight1.IsEnabled = !calibrating;
            TxtCalWeight2.IsEnabled = !calibrating;
            TxtCalWeight3.IsEnabled = !calibrating;

            BtnTareThrustFront.IsEnabled = calibrating;
            BtnCalThrustFrontW1.IsEnabled = calibrating;
            BtnCalThrustFrontW2.IsEnabled = calibrating;
            BtnCalThrustFrontW3.IsEnabled = calibrating;

            BtnTareTorqueFront.IsEnabled = calibrating;
            BtnCalTorqueFrontW1.IsEnabled = calibrating;
            BtnCalTorqueFrontW2.IsEnabled = calibrating;
            BtnCalTorqueFrontW3.IsEnabled = calibrating;

            bool enableBack = calibrating && isCrpMode;
            BtnTareThrustBack.IsEnabled = enableBack;
            BtnCalThrustBackW1.IsEnabled = enableBack;
            BtnCalThrustBackW2.IsEnabled = enableBack;
            BtnCalThrustBackW3.IsEnabled = enableBack;

            BtnTareTorqueBack.IsEnabled = enableBack;
            BtnCalTorqueBackW1.IsEnabled = enableBack;
            BtnCalTorqueBackW2.IsEnabled = enableBack;
            BtnCalTorqueBackW3.IsEnabled = enableBack;

            BtnStartCal.IsEnabled = !calibrating && !isSystemActive && isSetupComplete;
            BtnFinishCal.IsEnabled = calibrating;
            BtnPostCalTareAll.IsEnabled = !calibrating && !isSystemActive && isSetupComplete;

            // Lock other tabs and system activation while calibrating
            TabSingle.IsEnabled = !calibrating;
            TabCRP.IsEnabled = !calibrating;
            TabLogs.IsEnabled = !calibrating;
            ChkActivateSingle.IsEnabled = !calibrating && isSetupComplete;
            ChkActivateCRP.IsEnabled = !calibrating && isSetupComplete;
        }

        private void BtnStartCal_Click(object sender, RoutedEventArgs e)
        {
            UpdateTargetWeightLabels(formatBoxes: true);
            EnsureArduinoUnityScale();
            isCalibrating = true;

            bool isCrpMode = CmbCalMode.SelectedIndex == 1;
            var t1Clone = (isCrpMode ? crpSplineT1 : singleSplineT1).Clone();
            var q1Clone = (isCrpMode ? crpSplineQ1 : singleSplineQ1).Clone();
            var t2Clone = crpSplineT2.Clone();
            var q2Clone = crpSplineQ2.Clone();

            t1Clone.ResetCaptureFlags();
            q1Clone.ResetCaptureFlags();
            t2Clone.ResetCaptureFlags();
            q2Clone.ResetCaptureFlags();

            t1Clone.SyncWeightsAndTargets(calWeightsGrams, isTorque: false);
            q1Clone.SyncWeightsAndTargets(calWeightsGrams, isTorque: true);
            t2Clone.SyncWeightsAndTargets(calWeightsGrams, isTorque: false);
            q2Clone.SyncWeightsAndTargets(calWeightsGrams, isTorque: true);

            activeSplineT1 = t1Clone;
            activeSplineQ1 = q1Clone;
            activeSplineT2 = t2Clone;
            activeSplineQ2 = q2Clone;

            SetCalibrationButtonsState(true);
        }

        private void BtnFinishCal_Click(object sender, RoutedEventArgs e)
        {
            bool isCrpMode = CmbCalMode.SelectedIndex == 1;
            isCalibrating = false;
            SetCalibrationButtonsState(false);

            if (isCrpMode)
            {
                crpSplineT1 = activeSplineT1.Clone();
                crpSplineQ1 = activeSplineQ1.Clone();
                crpSplineT2 = activeSplineT2.Clone();
                crpSplineQ2 = activeSplineQ2.Clone();
                SaveCRPCalibrationToDisk();
                ResetGraphAxesY();
                MessageBox.Show(
                    "CRP 3-Point Spline Calibration saved to DynoCalibrationsCRP.\n\n" +
                    "Remove all calibration attachments, mount your motors and propellers, and click 'Tare All Load Cells' to zero any mechanical hysteresis before testing.",
                    "CRP Calibration Saved");
            }
            else
            {
                singleSplineT1 = activeSplineT1.Clone();
                singleSplineQ1 = activeSplineQ1.Clone();
                SaveSingleCalibrationToDisk();
                ResetGraphAxesY();
                MessageBox.Show(
                    "Single Propeller 3-Point Spline Calibration saved to DynoCalibrationsSingle.\n\n" +
                    "Remove all calibration attachments, mount your motor and propeller, and click 'Tare All Load Cells' to zero any mechanical hysteresis before testing.",
                    "Single Calibration Saved");
            }
        }

        private void CalibratePoint(SplineCalibrator spline, int weightIndex, double currentRawValue, double factorPerKg, string channelName)
        {
            UpdateTargetWeightLabels(formatBoxes: true);
            if (Math.Abs(currentRawValue) < 1e-5)
            {
                MessageBox.Show($"No load detected on {channelName}. Attach the {calWeightsGrams[weightIndex]:G6} g weight before clicking this button.", "Calibration Warning");
                return;
            }

            double weightG = calWeightsGrams[weightIndex];
            double targetPhysical = (weightG / 1000.0) * factorPerKg;
            spline.SetPoint(weightIndex, weightG, currentRawValue, targetPhysical);
        }

        // Front Thrust Calibration Handlers
        public void BtnTareThrust_Click(object sender, RoutedEventArgs e) => SendCommand("A");
        private void BtnCalThrustFrontW1_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineT1, 0, rawThrust1, GRAVITY, "Front Thrust");
        private void BtnCalThrustFrontW2_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineT1, 1, rawThrust1, GRAVITY, "Front Thrust");
        private void BtnCalThrustFrontW3_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineT1, 2, rawThrust1, GRAVITY, "Front Thrust");

        // Back Thrust Calibration Handlers
        public void BtnTareThrustBack_Click(object sender, RoutedEventArgs e) => SendCommand("E");
        private void BtnCalThrustBackW1_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineT2, 0, rawThrust2, GRAVITY, "Back Thrust");
        private void BtnCalThrustBackW2_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineT2, 1, rawThrust2, GRAVITY, "Back Thrust");
        private void BtnCalThrustBackW3_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineT2, 2, rawThrust2, GRAVITY, "Back Thrust");

        // Front Torque Calibration Handlers
        public void BtnTareTorque_Click(object sender, RoutedEventArgs e) => SendCommand("C");
        private void BtnCalTorqueFrontW1_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineQ1, 0, rawTorque1, GRAVITY * TORQUE_ARM_M, "Front Torque");
        private void BtnCalTorqueFrontW2_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineQ1, 1, rawTorque1, GRAVITY * TORQUE_ARM_M, "Front Torque");
        private void BtnCalTorqueFrontW3_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineQ1, 2, rawTorque1, GRAVITY * TORQUE_ARM_M, "Front Torque");

        // Back Torque Calibration Handlers
        public void BtnTareTorqueBack_Click(object sender, RoutedEventArgs e) => SendCommand("G");
        private void BtnCalTorqueBackW1_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineQ2, 0, rawTorque2, GRAVITY * TORQUE_ARM_M, "Back Torque");
        private void BtnCalTorqueBackW2_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineQ2, 1, rawTorque2, GRAVITY * TORQUE_ARM_M, "Back Torque");
        private void BtnCalTorqueBackW3_Click(object sender, RoutedEventArgs e) => CalibratePoint(activeSplineQ2, 2, rawTorque2, GRAVITY * TORQUE_ARM_M, "Back Torque");

        // Tare All Load Cells Handlers
        private void TareAllChannelsHardware()
        {
            SendCommand("Z");
        }

        private void BtnPostCalTareAll_Click(object sender, RoutedEventArgs e)
        {
            TareAllChannelsHardware();
        }

        private void BtnTareSingleTab_Click(object sender, RoutedEventArgs e)
        {
            TareAllChannelsHardware();
        }

        private void BtnTareCRPTab_Click(object sender, RoutedEventArgs e)
        {
            TareAllChannelsHardware();
        }

        private void ChkActivateSingle_Checked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = true;
            BtnRecordSingle.IsEnabled = true;
            BtnStartAutoSingle.IsEnabled = true;
            BtnTareSingleTab.IsEnabled = false;

            TabCRP.IsEnabled = false;
            TabCalibration.IsEnabled = false;
            TabLogs.IsEnabled = false;

            BtnStartCal.IsEnabled = false;
            BtnPostCalTareAll.IsEnabled = false;

            SendCommand("UNLOCK_SINGLE");
        }

        private void ChkActivateSingle_Unchecked(object sender, RoutedEventArgs e)
        {
            SldThrottleSingle.IsEnabled = false;
            SldThrottleSingle.Value = 0; 
            BtnRecordSingle.IsEnabled = false;
            BtnStartAutoSingle.IsEnabled = false;
            BtnTareSingleTab.IsEnabled = isSetupComplete;

            TabCRP.IsEnabled = true;
            TabCalibration.IsEnabled = true;
            TabLogs.IsEnabled = true;

            if (!isCalibrating && isSetupComplete)
            {
                BtnStartCal.IsEnabled = true;
                BtnPostCalTareAll.IsEnabled = true;
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
            BtnTareCRPTab.IsEnabled = false;

            TabSingle.IsEnabled = false;
            TabCalibration.IsEnabled = false;
            TabLogs.IsEnabled = false;

            BtnStartCal.IsEnabled = false;
            BtnPostCalTareAll.IsEnabled = false;

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
            BtnTareCRPTab.IsEnabled = isSetupComplete;

            TabSingle.IsEnabled = true;
            TabCalibration.IsEnabled = true;
            TabLogs.IsEnabled = true;

            if (!isCalibrating && isSetupComplete)
            {
                BtnStartCal.IsEnabled = true;
                BtnPostCalTareAll.IsEnabled = true;
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
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string mode = isAutoRunning ? "Auto" : "Manual";
                string filePath = Path.Combine(logDir, $"DynamometerData_{timestamp}_{mode}.csv");

                Task.Run(() =>
                {
                    try
                    {
                        var writer = new StreamWriter(filePath);
                        writer.AutoFlush = true;
                        writer.WriteLine("Time,Thrust1,Torque1,RPM1,Voltage1,Current1,Throttle1,ElecPower1,MechPower1,MotorEff1,PropEff1,Thrust2,Torque2,RPM2,Voltage2,Current2,Throttle2,ElecPower2,MechPower2,MotorEff2,PropEff2");

                        lock (csvLock)
                        {
                            csvWriter = writer;
                            startTime = DateTime.Now;
                            nextLogInterval = 0.1;
                            isRecording = true;
                        }
                    }
                    catch { }
                });
                
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

        private static SolidColorBrush CreateFrozenBrush(string hex)
        {
            var brush = new SolidColorBrush((Color)ColorConverter.ConvertFromString(hex));
            brush.Freeze();
            return brush;
        }

        private void CmbTheme_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (PlotFrontThrustSingle == null) return;
            bool isDark = CmbTheme.SelectedIndex == 1;

            if (isDark)
            {
                this.Resources["MainBg"] = BrushDarkMainBg;
                this.Resources["MainFg"] = BrushDarkMainFg;
                this.Resources["BtnBg"] = BrushDarkBtnBg;
                this.Resources["HoverBtnBg"] = BrushDarkHoverBtnBg;
                this.Resources["DisabledBtnBg"] = BrushDarkDisabledBtnBg;
                this.Resources["ControlBg"] = BrushDarkControlBg;
                if (ImgLogo != null && logoDark != null) ImgLogo.Source = logoDark;
            }
            else
            {
                this.Resources["MainBg"] = BrushLightMainBg;
                this.Resources["MainFg"] = BrushLightMainFg;
                this.Resources["BtnBg"] = BrushLightBtnBg;
                this.Resources["HoverBtnBg"] = BrushLightHoverBtnBg;
                this.Resources["DisabledBtnBg"] = BrushLightDisabledBtnBg;
                this.Resources["ControlBg"] = BrushLightControlBg;
                if (ImgLogo != null && logoLight != null) ImgLogo.Source = logoLight;
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
                    p.Plot.FigureBackground.Color = SpDarkBg;
                    p.Plot.DataBackground.Color = SpDarkBg;
                    p.Plot.Axes.Color(SpDarkFg);
                    p.Plot.Grid.MajorLineColor = SpDarkGrid;
                    if (l != null) l.Color = SpDarkLine; 
                }
                else
                {
                    p.Plot.FigureBackground.Color = SpLightBg;
                    p.Plot.DataBackground.Color = ScottPlot.Colors.White;
                    p.Plot.Axes.Color(ScottPlot.Colors.Black);
                    p.Plot.Grid.MajorLineColor = SpLightGrid;
                    if (l != null) l.Color = ScottPlot.Colors.Blue;
                }
            }
            RefreshVisiblePlots();
        }

        private void CmbThrustUnit_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (PlotFrontThrustSingle == null) return;
            ResetGraphAxesY();
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
            SldAutoStepsSingle.Value = 6;
            TxtStepDurationSingle.Text = "10.0";
            TxtAutoSingle1.Text = "10"; TxtAutoSingle2.Text = "20"; TxtAutoSingle3.Text = "30"; 
            TxtAutoSingle4.Text = "40"; TxtAutoSingle5.Text = "50"; TxtAutoSingle6.Text = "60";
        }

        private void BtnResetAutoCRP_Click(object sender, RoutedEventArgs e)
        {
            SldAutoStepsCRP.Value = 6;
            TxtStepDurationCRP.Text = "10.0";
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
                    txt.Text = "10.0";
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
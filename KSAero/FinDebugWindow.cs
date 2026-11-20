using System.Globalization;
using Brutal.ImGuiApi;
using Brutal.Logging;
using KSA;

namespace KSAero;

/// <summary>
/// A debug window to drive the controlled vehicle's fins by hand until they have a real control
/// path. It shows only while the controlled vehicle has KSAero fins.
/// </summary>
internal static class FinDebugWindow
{
    private static readonly List<Part> Fins = new();
    private static bool _failureLogged;
    private static float _all;

    // Pose and aero-wait statistics, refreshed once a second.
    private static double _windowStart = double.NaN;
    private static int _posesAtWindowStart;
    private static int _posesPerSecond;
    private static AeroWaitMeter.Window _lastWait = new();

    internal static void Draw(double dt)
    {
        try
        {
            DrawWindow();
        }
        catch (Exception ex)
        {
            if (!_failureLogged)
                DefaultCategory.Log.Warning($"[KSAero] Fin window failed (logged once): {ex}");
            _failureLogged = true;
        }
    }

    private static void DrawWindow()
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null || vehicle.IsDisposed)
            return;
        FinActuators.CollectFins(vehicle, Fins);
        if (Fins.Count == 0)
            return;

        UpdateStatistics(vehicle);

        bool open = ImGui.Begin("KSAero fins", ImGuiWindowFlags.AlwaysAutoResize);
        try
        {
            if (!open)
                return;

            float max = FinActuators.MaxDeflectionDeg;
            if (ImGui.SliderFloat("All fins", ref _all, -max, max, "%.1f deg"))
            {
                foreach (Part fin in Fins)
                    FinActuators.StateOf(fin).CommandDeg = _all;
            }
            if (ImGui.Button("Zero all"))
            {
                _all = 0f;
                foreach (Part fin in Fins)
                    FinActuators.StateOf(fin).CommandDeg = 0f;
            }

            ImGui.Separator();
            for (int i = 0; i < Fins.Count; i++)
            {
                Part fin = Fins[i];
                FinActuators.FinState state = FinActuators.StateOf(fin);
                ImGui.PushID(i);
                float command = state.CommandDeg;
                if (ImGui.SliderFloat(Format("Fin {0}", i + 1), ref command, -max, max, "%.1f deg"))
                    state.CommandDeg = command;
                ImGui.SameLine();
                ImGui.TextDisabled(Format("at {0,5:F1}  {1}", state.CurrentDeg, fin.Template.DisplayName));
                ImGui.PopID();
            }

            ImGui.Separator();
            ImGui.SliderFloat("Max deflection", ref FinActuators.MaxDeflectionDeg, 1f, 60f, "%.0f deg");
            ImGui.SliderFloat("Rate", ref FinActuators.RateDegPerSecond, 1f, 360f, "%.0f deg/s");

            ImGui.Separator();
            ImGui.Text(Format("Fin poses {0}/s (panels turned in place, no re-mesh)", _posesPerSecond));
            if (vehicle.Parts?.Aerodynamics?.Mesh is { } mesh)
            {
                (int fins, int panels) = FinAeroPanels.Describe(mesh);
                ImGui.Text(Format("Aero mesh: {0} fin panels on {1} fins, {2} panels in all",
                    panels, fins, mesh.SolidPanels.Length + mesh.SheetPanels.Length));
                if (fins > 0 && panels == 0)
                    ImGui.TextDisabled("No fin panels found: the fins will not change the aero.");
            }
            ImGui.Text(Format("Physics waiting on aero rebuilds: {0:F1} ms/s, longest {1:F1} ms",
                _lastWait.TotalMs, _lastWait.MaxMs));
        }
        finally
        {
            ImGui.End();
        }
    }

    private static void UpdateStatistics(Vehicle vehicle)
    {
        double now = Environment.TickCount64 / 1000.0;
        if (double.IsNaN(_windowStart))
        {
            _windowStart = now;
            _posesAtWindowStart = FinActuators.PoseCount;
            return;
        }
        if (now - _windowStart < 1.0)
            return;
        double span = now - _windowStart;
        _posesPerSecond = (int)Math.Round((FinActuators.PoseCount - _posesAtWindowStart) / span);
        _posesAtWindowStart = FinActuators.PoseCount;
        _windowStart = now;
        if (vehicle.Parts is { } tree)
            _lastWait = AeroWaitMeter.Take(tree);
    }

    private static string Format(string format, params object[] args) => string.Format(CultureInfo.InvariantCulture, format, args);
}

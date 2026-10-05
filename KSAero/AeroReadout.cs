using System.Globalization;
using Brutal.ImGuiApi;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;

namespace KSAero;

/// <summary>
/// A small window with the controlled vehicle's aero state while it is in an atmosphere. It runs
/// the same <see cref="AeroModel.Evaluate"/> the physics does, on the same inputs.
/// </summary>
internal static class AeroReadout
{
    private static bool _failureLogged;

    internal static void Draw()
    {
        try
        {
            DrawWindow();
        }
        catch (Exception ex)
        {
            if (!_failureLogged)
                DefaultCategory.Log.Warning($"[KSAero] Readout failed (logged once): {ex}");
            _failureLogged = true;
        }
    }

    private static void DrawWindow()
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        if (vehicle == null || vehicle.IsDisposed)
            return;

        ref readonly PhysicsEnvironment environment = ref vehicle.PhysicsEnvironment;
        if (!environment.InPhysicsRadius || environment.AtmosphericDensity <= 0f)
            return;

        ref readonly VehicleProperties props = ref vehicle.Props;
        AeroBody body = AirForcePatch.BodyFor(in props);
        if (!body.IsValid)
            return;

        float3 air = PhysicsStates.ComputeAirVelocityBody(in vehicle.BubbleOrigin, in vehicle.KinematicStates, in environment);
        double speed = air.Length();
        double density = environment.AtmosphericDensity;
        double q = 0.5 * density * speed * speed;
        double soundSpeed = AeroModel.SpeedOfSound(environment.AtmosphericPressure, density);
        double mach = soundSpeed > 0.0 ? speed / soundSpeed : 0.0;
        AeroCoefficients c = speed > 1e-3
            ? AeroModel.Evaluate(in body, air.X, air.Y, air.Z, mach)
            : default;
        double qa = q * body.ReferenceArea;

        bool open = ImGui.Begin("KSAero", ImGuiWindowFlags.AlwaysAutoResize);
        try
        {
            if (!open)
                return;

            ImGui.Text(Format("Mach {0:F2}   q {1:F1} kPa", mach, q * 1e-3));
            ImGui.Text(Format("AoA {0:F1} deg {1}", c.AngleOfAttack * 180.0 / Math.PI, c.TailFirst ? "(tail-first)" : ""));
            ImGui.Text(Format("CL {0:F3}   CD {1:F3}   L/D {2:F2}", c.Lift, c.Drag, c.Drag > 0.0 ? c.Lift / c.Drag : 0.0));
            ImGui.Text(Format("Lift {0:F1} kN   Drag {1:F1} kN", c.Lift * qa * 1e-3, c.Drag * qa * 1e-3));
            if (!c.TailFirst && c.LeadingAngle * 180.0 / Math.PI > AeroParameters.StallAngleDeg)
                ImGui.Text("STALL (attached lift lost)");
            ImGui.Separator();
            ImGui.TextDisabled(Format("Cylinder L {0:F1} m  D {1:F2} m  L/D {2:F1}", body.Length, body.Diameter, body.Fineness));
        }
        finally
        {
            ImGui.End();
        }
    }

    private static string Format(string format, params object[] args) => string.Format(CultureInfo.InvariantCulture, format, args);
}

using System.Numerics;
using System.Reflection;
using Brutal.Logging;
using Brutal.Numerics;
using KSA;
using KSA.Aero;

namespace KSAero;

/// <summary>
/// Hands the fins to AdvancedFlightComputer's control-surface registry
/// (AdvancedFlightComputer.Features.AeroControl.ControlSurfaces.Register), so its aero controller
/// can sweep them and fly them. Found by name and called by reflection, with KSA and .NET types
/// only: KSAero needs no reference to AFC's assembly, and loads and flies without it.
///
/// What AFC is given, per its contract:
///  - collect: the vehicle's fins, each its part, a name, its range and its rate;
///  - deflection and command: the fin's reached angle, and its commanded one (<see cref="FinActuators"/>);
///  - poser: for AFC's aero sweep, which flies a private copy of the vehicle's aero mesh off the
///    main thread. Read on the main thread, the fin's blade boxes at rest; on the sweep's thread,
///    its panels picked out of that mesh (<see cref="FinPanelSet.Select"/>) and turned in place
///    (<see cref="FinPanelSet.Pose"/>), as <see cref="FinAeroPanels"/> turns them in the live mesh.
///    The private mesh is gathered with the blades at rest, as every mesh is.
/// </summary>
internal static class AfcSurfaces
{
    private const string RegistryType = "AdvancedFlightComputer.Features.AeroControl.ControlSurfaces";
    private const string ProviderName = "KSAero";

    /// <summary>Registers the fins with AFC if it is loaded. Called once all mods have loaded.</summary>
    internal static void TryRegister()
    {
        MethodInfo? register = FindRegister();
        if (register == null)
        {
            DefaultCategory.Log.Info("[KSAero] AdvancedFlightComputer is not loaded: the fins are driven from the KSAero fins window only.");
            return;
        }
        try
        {
            Func<Vehicle, IReadOnlyList<(Part Part, string Name, double MinDeg, double MaxDeg, double RateDegS)>> collect = Collect;
            Func<Part, double> deflection = fin => FinActuators.StateOf(fin).CurrentDeg;
            Action<Part, double> command = (fin, deg) => FinActuators.StateOf(fin).CommandDeg = (float)deg;
            Func<Vehicle, Part, Func<Aerodynamics, Action<double>?>?> poser = Poser;
            register.Invoke(null, [ProviderName, collect, deflection, command, poser]);
            DefaultCategory.Log.Info("[KSAero] Fins registered with AdvancedFlightComputer's aero controller.");
        }
        catch (Exception e)
        {
            DefaultCategory.Log.Warning($"[KSAero] Could not register the fins with AdvancedFlightComputer: {e.InnerException?.Message ?? e.Message}");
        }
    }

    private static MethodInfo? FindRegister()
    {
        foreach (Assembly assembly in AppDomain.CurrentDomain.GetAssemblies())
        {
            Type? registry = assembly.GetType(RegistryType, throwOnError: false);
            MethodInfo? method = registry?.GetMethod("Register", BindingFlags.Public | BindingFlags.Static);
            if (method != null)
                return method;
        }
        return null;
    }

    private static IReadOnlyList<(Part Part, string Name, double MinDeg, double MaxDeg, double RateDegS)> Collect(Vehicle vehicle)
    {
        var fins = new List<Part>();
        FinActuators.CollectFins(vehicle, fins);
        var surfaces = new List<(Part, string, double, double, double)>(fins.Count);
        double max = FinActuators.MaxDeflectionDeg;
        for (int i = 0; i < fins.Count; i++)
            surfaces.Add((fins[i], $"{fins[i].DisplayName} #{i + 1}", -max, max, FinActuators.RateDegPerSecond));
        return surfaces;
    }

    /// <summary>Main thread: the fin's blade boxes now. The binder and the pose action then touch only the mesh they are given.</summary>
    private static Func<Aerodynamics, Action<double>?>? Poser(Vehicle vehicle, Part fin)
    {
        List<FinAeroPanels.BladeBox> boxes = FinAeroPanels.BladeBoxes(fin);
        if (boxes.Count == 0)
            return null;
        return aerodynamics =>
        {
            AeroMesh? mesh = aerodynamics?.Mesh;
            if (mesh == null)
                return null;
            var sets = new List<(FinPanelSet Panels, FinAeroPanels.BladeBox Box)>(boxes.Count);
            foreach (FinAeroPanels.BladeBox box in boxes)
            {
                FinPanelSet panels = FinPanelSet.Select(mesh.Surface, box.Center, box.Rotation, box.HalfExtents, mesh.PanelWidth);
                if (panels.PanelCount > 0)
                    sets.Add((panels, box));
            }
            if (sets.Count == 0)
                return null;
            return deg =>
            {
                AeroSurfaceFlow? flow = aerodynamics!.GetSurfaceFlow();
                foreach ((FinPanelSet panels, FinAeroPanels.BladeBox box) in sets)
                    panels.Pose(mesh.Surface, flow, box.Hinge, FinAeroPanels.VehicleTurn(box.Fin2Vehicle, deg));
            };
        };
    }
}

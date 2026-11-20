using System.Diagnostics;
using System.Runtime.CompilerServices;
using KSA;

namespace KSAero;

/// <summary>
/// Times how long physics steps sit in <see cref="PartTree.AwaitAerodynamics"/>, waiting for an
/// aero mesh rebuild, so the debug window can show what moving a fin costs. Waits are recorded per
/// part tree; one under <see cref="RecordThresholdMs"/> means no rebuild was in flight.
/// </summary>
internal static class AeroWaitMeter
{
    private const double RecordThresholdMs = 0.05;

    internal sealed class Window
    {
        public double TotalMs;
        public double MaxMs;
        public int Count;
    }

    private sealed class Meter
    {
        public readonly object Gate = new();
        public Window Current = new();
    }

    private static readonly ConditionalWeakTable<PartTree, Meter> Meters = new();

    [ThreadStatic] private static long _start;

    internal static void Prefix() => _start = Stopwatch.GetTimestamp();

    /// <summary>Also hands the mesh to <see cref="FinAeroPanels.Apply"/>, after the wait is timed.</summary>
    internal static void Postfix(PartTree __instance, KSA.Aero.Aerodynamics? __result)
    {
        double ms = Stopwatch.GetElapsedTime(_start).TotalMilliseconds;
        FinAeroPanels.Apply(__instance, __result);
        if (ms < RecordThresholdMs)
            return;
        Meter meter = Meters.GetValue(__instance, static _ => new Meter());
        lock (meter.Gate)
        {
            meter.Current.TotalMs += ms;
            meter.Current.MaxMs = Math.Max(meter.Current.MaxMs, ms);
            meter.Current.Count++;
        }
    }

    /// <summary>The waits recorded on this tree since the last call, and starts a new window.</summary>
    internal static Window Take(PartTree tree)
    {
        if (!Meters.TryGetValue(tree, out Meter? meter))
            return new Window();
        lock (meter.Gate)
        {
            Window taken = meter.Current;
            meter.Current = new Window();
            return taken;
        }
    }
}

using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

namespace KSAero;

/// <summary>
/// Drives the all-moving fins in KSAeroParts.xml. A fin is one part whose blade is a subpart; the
/// blade turns about the part's X axis (the span axis, through the part origin), the way a
/// <see cref="KeyframeAnimationModule"/> poses a subpart with its supplemental rotation.
///
/// It runs on the main thread at the start of <see cref="PartTree.FlushDirtyDerived"/>, once a
/// frame: after the frame has waited for the vehicle solver and before it starts the next solve, so
/// it never moves a blade while a physics step reads it. It moves the blade's model and collider
/// only; the aero mesh is not rebuilt; <see cref="FinAeroPanels"/> turns the blade's panels in the
/// existing mesh instead.
/// </summary>
internal static class FinActuators
{
    /// <summary>
    /// The blade subpart templates: only subparts with one of these ids are turned, and a part with
    /// one is a fin. All are KSAero's own, so no stock part is ever taken for a fin. The 3 m blade's
    /// look is a subpart of its own (its scale would scale a collider on it).
    /// </summary>
    private static readonly HashSet<string> BladeTemplateIds = new(StringComparer.Ordinal)
    {
        "KSAero_Subpart_FinBlade1m",
        "KSAero_Subpart_FinBlade2m",
        "KSAero_Subpart_FinBlade3m",
        "KSAero_Subpart_FinBladeLook3m",
    };

    /// <summary>Applied angles are rounded to this, so a fin holding still is not re-posed.</summary>
    private const double QuantumDeg = 0.1;

    internal static float MaxDeflectionDeg = 30f;
    internal static float RateDegPerSecond = 30f;

    /// <summary>Blade poses applied, for the debug window.</summary>
    internal static int PoseCount;

    private static readonly ConditionalWeakTable<Part, FinState> States = new();
    private static double _lastSimSeconds = double.NaN;

    internal sealed class FinState
    {
        /// <summary>Commanded deflection, degrees.</summary>
        public float CommandDeg;

        /// <summary>Deflection the actuator has reached, degrees.</summary>
        public double CurrentDeg;

        /// <summary>Deflection the blade is posed at, a multiple of <see cref="QuantumDeg"/>.</summary>
        public double AppliedDeg;
    }

    internal static FinState StateOf(Part fin) => States.GetValue(fin, static _ => new FinState());

    internal static bool IsBlade(Part part) => part.IsSubPart && BladeTemplateIds.Contains(part.Template.Id);

    internal static bool IsFin(Part part)
    {
        if (part.IsSubPart)
            return false;
        ReadOnlySpan<Part> subParts = part.SubParts;
        for (int i = 0; i < subParts.Length; i++)
        {
            if (BladeTemplateIds.Contains(subParts[i].Template.Id))
                return true;
        }
        return false;
    }

    /// <summary>The fins on a vehicle, in part order.</summary>
    internal static void CollectFins(Vehicle vehicle, List<Part> into)
    {
        into.Clear();
        PartTree? tree = vehicle.Parts;
        if (tree == null || tree.IsDisposed)
            return;
        ReadOnlySpan<Part> parts = tree.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            if (IsFin(parts[i]))
                into.Add(parts[i]);
        }
    }

    private static readonly List<Part> Scratch = new();

    /// <summary>Harmony prefix on <see cref="PartTree.FlushDirtyDerived"/>.</summary>
    internal static void Update()
    {
        double now = Universe.GetElapsedSeconds();
        double dt = double.IsNaN(_lastSimSeconds) ? 0.0 : Math.Clamp(now - _lastSimSeconds, 0.0, 1.0);
        _lastSimSeconds = now;

        if (Universe.CurrentSystem == null)
            return;

        foreach (Vehicle vehicle in Universe.CurrentSystem.All.OfType<Vehicle>())
        {
            if (vehicle.IsDisposed)
                continue;
            CollectFins(vehicle, Scratch);
            for (int i = 0; i < Scratch.Count; i++)
                Step(Scratch[i], dt);
        }
        Scratch.Clear();
    }

    private static void Step(Part fin, double dt)
    {
        FinState state = StateOf(fin);
        double max = MaxDeflectionDeg;
        double target = Math.Clamp(state.CommandDeg, -max, max);
        double maxStep = RateDegPerSecond * dt;
        state.CurrentDeg += Math.Clamp(target - state.CurrentDeg, -maxStep, maxStep);

        double applied = Math.Round(state.CurrentDeg / QuantumDeg) * QuantumDeg;
        if (applied == state.AppliedDeg)
            return;
        state.AppliedDeg = applied;
        Pose(fin, applied);
    }

    private static void Pose(Part fin, double deflectionDeg)
    {
        PoseBlades(fin, deflectionDeg);
        fin.UpdateBounds();
        Span<ColliderModule> colliders = fin.SubtreeModules.Get<ColliderModule>();
        for (int i = 0; i < colliders.Length; i++)
            colliders[i].NeedsColliderUpdate = true;
        PoseCount++;
    }

    /// <summary>
    /// Sets the blade subparts' transforms only, as <see cref="KeyframeAnimationModule"/> does with
    /// its supplemental rotation. Also used to hold the blades at rest while an aero mesh is gathered.
    /// </summary>
    internal static void PoseBlades(Part fin, double deflectionDeg)
    {
        doubleQuat turn = doubleQuat.CreateFromAxisAngle(double3.UnitX, deflectionDeg * Math.PI / 180.0);
        ReadOnlySpan<Part> subParts = fin.SubParts;
        for (int i = 0; i < subParts.Length; i++)
        {
            Part blade = subParts[i];
            if (!BladeTemplateIds.Contains(blade.Template.Id))
                continue;
            blade.PositionParentAsmb = blade.PositionParentAsmbSafe.Transform(turn);
            blade.Asmb2ParentAsmb = doubleQuat.Concatenate(blade.Asmb2ParentAsmbSafe, turn);
        }
    }
}

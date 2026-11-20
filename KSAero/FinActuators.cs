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
/// frame: after the frame has waited for the vehicle solver and before it rebuilds derived data and
/// starts the next solve. So it never moves a blade while a physics step reads it, and the aero
/// mesh it asks for is rebuilt in the same flush. Moving a subpart does not dirty the aero mesh by
/// itself (only full-part moves do), so it is marked dirty here.
/// </summary>
internal static class FinActuators
{
    /// <summary>The blade subpart templates. Only subparts with one of these ids are turned.</summary>
    private static readonly HashSet<string> BladeTemplateIds = new(StringComparer.Ordinal)
    {
        "KSAero_Subpart_FinBlade1m",
        "KSAero_Subpart_FinBlade2m",
    };

    /// <summary>Applied angles are rounded to this, so a blade at rest gives the same aero mesh hash.</summary>
    private const double QuantumDeg = 0.1;

    internal static float MaxDeflectionDeg = 30f;
    internal static float RateDegPerSecond = 30f;

    /// <summary>Aero mesh rebuilds this module asked for, for the debug window.</summary>
    internal static int RemeshRequests;

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

        fin.UpdateBounds();
        Span<ColliderModule> colliders = fin.SubtreeModules.Get<ColliderModule>();
        for (int i = 0; i < colliders.Length; i++)
            colliders[i].NeedsColliderUpdate = true;

        fin.Tree?.MarkDerivedDirty(DerivedData.AeroMesh);
        RemeshRequests++;
    }
}

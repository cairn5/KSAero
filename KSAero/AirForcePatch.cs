using System.Reflection;
using System.Reflection.Emit;
using BepuPhysics.Collidables;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace KSAero;

/// <summary>
/// Swaps the atmospheric call to <c>PhysicsStates.ComputeDrag</c> inside
/// <c>PhysicsStates.ComputeDerivatives</c> for <see cref="ComputeAirForces"/>. The ocean call is
/// left on stock.
///
/// The call site is rewritten rather than ComputeDrag itself being patched, because ComputeDrag is
/// AggressiveInlining: a patch on it would be skipped wherever the JIT had already inlined the
/// original body into ComputeDerivatives.
/// </summary>
internal static class AirForcePatch
{
    // Stock's own constant in ComputeDrag, kept for the rotational damping it still owns.
    private const double StockSkinDragCoefficient = 0.1;

    // The density argument's load sits five instructions before the call. The window only has to
    // reach it without running into the previous call's arguments.
    private const int ArgumentWindow = 12;

    private delegate void ComputeDragFn(in BubbleOrigin origin, in KinematicStates kinematic, in VehicleProperties props,
        in PhysicsEnvironment environment, ref Disturbances disturbances, double3 airVelocityBub, double density,
        double dragCdA, double surfaceArea, double dt);

    private static readonly MethodInfo? ComputeDerivatives = AccessTools.Method(typeof(PhysicsStates), "ComputeDerivatives", new[]
    {
        typeof(BubbleOrigin).MakeByRefType(), typeof(KinematicStates).MakeByRefType(),
        typeof(VehicleProperties).MakeByRefType(), typeof(PhysicsEnvironment).MakeByRefType(),
        typeof(double), typeof(double), typeof(double3), typeof(double3),
        typeof(ReadOnlySpan<ActiveNozzle>), typeof(ReadOnlySpan<ActiveChute>),
    });

    private static readonly MethodInfo? ComputeDrag = AccessTools.Method(typeof(PhysicsStates), "ComputeDrag", new[]
    {
        typeof(BubbleOrigin).MakeByRefType(), typeof(KinematicStates).MakeByRefType(),
        typeof(VehicleProperties).MakeByRefType(), typeof(PhysicsEnvironment).MakeByRefType(),
        typeof(Disturbances).MakeByRefType(), typeof(double3),
        typeof(double), typeof(double), typeof(double), typeof(double),
    });

    private static readonly FieldInfo? AtmosphericDensity = AccessTools.Field(typeof(PhysicsEnvironment), nameof(PhysicsEnvironment.AtmosphericDensity));

    private static ComputeDragFn? _stockDrag;

    internal static void Apply(Harmony harmony)
    {
        if (ComputeDerivatives == null || ComputeDrag == null || AtmosphericDensity == null)
            throw new MissingMemberException("PhysicsStates.ComputeDerivatives, PhysicsStates.ComputeDrag or PhysicsEnvironment.AtmosphericDensity not found.");

        _stockDrag = AccessTools.MethodDelegate<ComputeDragFn>(ComputeDrag);
        harmony.Patch(ComputeDerivatives, transpiler: new HarmonyMethod(typeof(AirForcePatch), nameof(Transpiler)));
    }

    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        List<CodeInstruction> code = instructions.ToList();
        MethodInfo replacement = AccessTools.Method(typeof(AirForcePatch), nameof(ComputeAirForces));
        int replaced = 0;

        for (int i = 0; i < code.Count; i++)
        {
            if (code[i].Calls(ComputeDrag!) && PassesAtmosphericDensity(code, i))
            {
                code[i].operand = replacement;
                replaced++;
            }
        }

        // Anything else means the method has changed shape, and stock aero is the safe answer.
        if (replaced != 1)
            throw new InvalidOperationException($"Expected one atmospheric ComputeDrag call in ComputeDerivatives, found {replaced}.");

        return code;
    }

    private static bool PassesAtmosphericDensity(List<CodeInstruction> code, int callIndex)
    {
        for (int j = callIndex - 1; j >= Math.Max(0, callIndex - ArgumentWindow); j--)
        {
            if (code[j].LoadsField(AtmosphericDensity!))
                return true;
            if (code[j].opcode == OpCodes.Call || code[j].opcode == OpCodes.Callvirt)
                return false;
        }
        return false;
    }

    /// <summary>
    /// Same signature as stock ComputeDrag, so it drops into the call site as is. Lift and drag act
    /// through the centre of mass, and the rotational damping is stock's, unchanged.
    /// </summary>
    internal static void ComputeAirForces(in BubbleOrigin origin, in KinematicStates kinematic, in VehicleProperties props,
        in PhysicsEnvironment environment, ref Disturbances disturbances, double3 airVelocityBub, double density,
        double dragCdA, double surfaceArea, double dt)
    {
        AeroBody body = BodyFor(in props);
        if (!body.IsValid)
        {
            _stockDrag!(in origin, in kinematic, in props, in environment, ref disturbances, airVelocityBub, density, dragCdA, surfaceArea, dt);
            return;
        }

        double speedSquared = airVelocityBub.LengthSquared();
        double q = 0.5 * density * speedSquared;
        if (q.IsNearlyZero())
            return;

        double speed = Math.Sqrt(speedSquared);
        double3 dirBody = (airVelocityBub / speed).Transform(kinematic.Body2Phys.Inverse());
        double mach = speed / Math.Max(AeroModel.SpeedOfSound(environment.AtmosphericPressure, density), 1e-9);
        AeroCoefficients c = AeroModel.Evaluate(in body, dirBody.X, dirBody.Y, dirBody.Z, mach);

        // Stock passes the surface area still out of the water, so a half-submerged vehicle feels half the air.
        double exposed = props.TotalSurfaceArea > 0f ? Math.Clamp(surfaceArea / props.TotalSurfaceArea, 0.0, 1.0) : 1.0;
        double3 force = q * body.ReferenceArea * exposed * new double3(c.ForceX, c.ForceY, c.ForceZ);

        // Stock's semi-implicit limiter, applied to the whole force: one substep can never reverse the airspeed.
        double mass = props.TotalMass;
        force *= 1.0 / (1.0 + force.Length() * dt / (mass * speed));
        disturbances.AddForceBody(double3.Zero, force);

        double3 bodyRates = PhysicsStates.GetBodyRates(in origin, in kinematic, in environment);
        double rimSpeed = bodyRates.Length() * props.ComputeBoundingSphereRadiusAsmb();
        double rotationalQ = 0.5 * density * rimSpeed * rimSpeed;
        disturbances.AddPureTorqueBody(StockSkinDragCoefficient * surfaceArea * rotationalQ * -bodyRates.NormalizeOrZero());
    }

    internal static AeroBody BodyFor(in VehicleProperties props)
    {
        Box box = props.BoundingBoxAsmb;
        return AeroBody.FromBoxExtents(2.0 * box.HalfWidth, 2.0 * box.HalfHeight, 2.0 * box.HalfLength);
    }
}

namespace KSAero;

/// <summary>
/// The body of revolution KSAero flies in place of KSA's bounding box: a cylinder along the
/// assembly X axis (KSA's long and thrust axis, nose at +X).
/// </summary>
public readonly struct AeroBody
{
    public AeroBody(double length, double diameter)
    {
        Length = length;
        Diameter = diameter;
        ReferenceArea = Math.PI * 0.25 * diameter * diameter;
        PlanformArea = length * diameter;
        WettedArea = Math.PI * diameter * length;
    }

    /// <summary>L, along body X, m.</summary>
    public double Length { get; }

    /// <summary>D, m.</summary>
    public double Diameter { get; }

    /// <summary>Frontal area pi D^2 / 4. Every coefficient is referenced to it.</summary>
    public double ReferenceArea { get; }

    /// <summary>Side-on projected area L D, which the crossflow acts on.</summary>
    public double PlanformArea { get; }

    /// <summary>Side wetted area pi D L, which the skin friction acts on.</summary>
    public double WettedArea { get; }

    public double Fineness => Length / Diameter;

    public bool IsValid => Length > 0.0 && Diameter > 0.0 && double.IsFinite(Length) && double.IsFinite(Diameter);

    /// <summary>
    /// The cylinder for a bounding box. The diameter is the one whose circle has the same area as
    /// stock's elliptical X face (pi/4 dy dz), so the frontal area, and with it the nose-on drag,
    /// carries over from stock unchanged.
    /// </summary>
    public static AeroBody FromBoxExtents(double dx, double dy, double dz) => new(dx, Math.Sqrt(dy * dz));
}

/// <summary>
/// Coefficients for one flight condition. Force coefficients are referenced to
/// <see cref="AeroBody.ReferenceArea"/> and dynamic pressure.
/// </summary>
public struct AeroCoefficients
{
    /// <summary>Total angle of attack between the nose (+X) and the airspeed: 0 nose-first, pi tail-first.</summary>
    public double AngleOfAttack;

    /// <summary>Angle from whichever end leads, 0..pi/2. Equal to <see cref="AngleOfAttack"/> nose-first.</summary>
    public double LeadingAngle;

    public bool TailFirst;
    public double Mach;

    /// <summary>Across the airspeed, towards the side the leading end is pointing.</summary>
    public double Lift;

    /// <summary>Along the airspeed, opposing it.</summary>
    public double Drag;

    /// <summary>The force per unit (q A_ref) in body axes. Its length is hypot(C_L, C_D).</summary>
    public double ForceX, ForceY, ForceZ;
}

/// <summary>
/// The tunable numbers. Everything is a plain constant so the whole model reads off one page.
/// </summary>
public static class AeroParameters
{
    /// <summary>Ratio of specific heats, for the speed of sound sqrt(gamma p / rho).</summary>
    public const double Gamma = 1.4;

    // Pressure plus base drag at zero angle of attack, on the frontal area. These are stock's own
    // nose and tail face values: they were sensible already.
    public const double CdNoseFirst = 0.30;
    public const double CdTailFirst = 1.00;

    // Turbulent skin friction, on the side wetted area pi D L. Stock adds 0.1 x the whole
    // bounding-box surface instead, which is 40x a real Cf at flight Reynolds numbers and buries
    // every other term.
    public const double SkinFrictionCf = 0.0025;

    // The attached-flow (slender-body) part of the normal force holds up to the stall angle and
    // fades out over the width after it. The separated crossflow part does not stall.
    //
    // Leading with the flat base, it needs body length to build: slender-body lift depends on the
    // cross-section, not on the shape of the leading end, so a booster flying engine-first has it
    // in full, but a body barely longer than it is wide is all face and has none. It fades in
    // across this range of fineness L/D. Without it, crossflow and the tilted axial force nearly
    // cancel on a booster-shaped body and its lift flips sign with Mach.
    public const double TailFirstAttachedStartFineness = 1.0;
    public const double TailFirstAttachedFullFineness = 2.0;
    public const double StallAngleDeg = 20.0;
    public const double StallWidthDeg = 10.0;
    public const double PostStallAttachedFraction = 0.0;

    // Transonic rise in the pressure and base drag: flat to the drag-divergence Mach, peaking just
    // past Mach 1, then easing towards a supersonic level. A pointed nose sheds most of the wave
    // drag again; a blunt base leading keeps it.
    public const double DragRiseStartMach = 0.8;
    public const double DragPeakMach = 1.05;
    public const double DragDecayWidthMach = 0.6;
    public const double NoseFirstPeakFactor = 2.0;
    public const double NoseFirstSupersonicFactor = 1.3;
    public const double TailFirstPeakFactor = 1.7;
    public const double TailFirstSupersonicFactor = 1.6;

    // Crossflow drag coefficient of an infinite circular cylinder against crossflow Mach
    // M sin(alpha). 1.2 subsonic, a transonic peak, and the Newtonian 4/3 hypersonic.
    public static readonly double[] CrossflowMach = { 0.0, 0.4, 0.7, 1.0, 1.3, 1.6, 2.0, 3.0, 5.0 };
    public static readonly double[] CrossflowCd = { 1.2, 1.2, 1.55, 2.0, 1.85, 1.7, 1.55, 1.42, 1.36 };

    // Allen and Perkins' finite-length factor eta against fineness L/D: the flow escapes round the
    // ends of a short cylinder. It applies to subsonic crossflow only, and blends to 1 across the
    // crossflow Mach range below, where the ends can no longer be felt.
    public static readonly double[] FinenessGrid = { 1.0, 2.0, 4.0, 6.0, 10.0, 20.0, 40.0 };
    public static readonly double[] FinenessEta = { 0.55, 0.57, 0.60, 0.62, 0.66, 0.74, 0.82 };
    public const double EtaBlendStartMach = 0.8;
    public const double EtaBlendEndMach = 1.2;
}

/// <summary>
/// Forces on a body of revolution at any angle of attack, after Allen and Perkins' viscous
/// crossflow method as extended to 0-180 degrees by Jorgensen:
///
///   N_att = sin(2a) cos(a/2)                       attached (slender-body) normal force
///   N_x   = eta c_dc (A_p / A_ref) sin^2 a         separated crossflow normal force
///   C_A   = (C_D0 f(M) + Cf S_wet / A_ref) cos^2 a
///
///   C_L = (s(a) N_att + N_x) cos a - C_A sin a
///   C_D = (N_att + N_x) sin a + C_A cos a
///
/// The stall s(a) takes away only the lift share of the attached normal force: a stalled body
/// keeps the drag, as a separated one would. The crossflow gives drag rising as sin^3 a and lift
/// that keeps growing to about 55 degrees, so on a slender rocket the stall is a knee rather than
/// a cliff; on a short body the attached term dominates and it is a cliff. a is measured from
/// whichever end leads, so tail-first flight uses the same law with the base's drag in place of
/// the nose's. Its attached term fades out for bodies shorter than about two diameters, so a
/// slender booster flying engine-first lifts towards the side its engine end is tilted from the
/// first degree, while a capsule flying heat shield first lifts the way a real one does, from its
/// tilted axial force, at an L/D of about -0.2 to -0.3.
/// </summary>
public static class AeroModel
{
    private const double DegToRad = Math.PI / 180.0;

    public static double SpeedOfSound(double pressure, double density)
        => pressure > 0.0 && density > 0.0 ? Math.Sqrt(AeroParameters.Gamma * pressure / density) : 0.0;

    /// <param name="ux">Unit direction of the vehicle's velocity through the air, body axes.</param>
    public static AeroCoefficients Evaluate(in AeroBody body, double ux, double uy, double uz, double mach)
    {
        double cross = Math.Sqrt(uy * uy + uz * uz);
        double norm = Math.Sqrt(ux * ux + cross * cross);
        if (norm > 0.0)
        {
            ux /= norm;
            uy /= norm;
            uz /= norm;
            cross /= norm;
        }

        bool tailFirst = ux < 0.0;
        double cosLead = Math.Abs(ux);
        double sinLead = cross;
        double lead = Math.Atan2(sinLead, cosLead);

        // sin(2a) cos(a/2), written out so it needs no trig past the atan2 above.
        double attachedShare = tailFirst
            ? SmoothStep(AeroParameters.TailFirstAttachedStartFineness, AeroParameters.TailFirstAttachedFullFineness, body.Fineness)
            : 1.0;
        double attached = attachedShare * 2.0 * sinLead * cosLead * Math.Sqrt(0.5 * (1.0 + cosLead));

        double crossMach = mach * sinLead;
        double eta = Lerp(FinenessEta(body.Fineness), 1.0,
            SmoothStep(AeroParameters.EtaBlendStartMach, AeroParameters.EtaBlendEndMach, crossMach));
        double crossflow = eta * CrossflowCd(crossMach) * (body.PlanformArea / body.ReferenceArea) * sinLead * sinLead;

        double pressureDrag = tailFirst
            ? AeroParameters.CdTailFirst * TransonicFactor(mach, AeroParameters.TailFirstPeakFactor, AeroParameters.TailFirstSupersonicFactor)
            : AeroParameters.CdNoseFirst * TransonicFactor(mach, AeroParameters.NoseFirstPeakFactor, AeroParameters.NoseFirstSupersonicFactor);
        double skin = AeroParameters.SkinFrictionCf * body.WettedArea / body.ReferenceArea;
        double axial = (pressureDrag + skin) * cosLead * cosLead;

        double lift = (AttachedFraction(lead) * attached + crossflow) * cosLead - axial * sinLead;
        double drag = (attached + crossflow) * sinLead + axial * cosLead;

        // Force = -drag along the airspeed u, + lift along e_L = sign(ux) sin(a) x - cos(a) n, where
        // n is the crossflow direction. With no stall this is exactly -C_A x - C_N n.
        double sign = tailFirst ? -1.0 : 1.0;
        var c = new AeroCoefficients
        {
            AngleOfAttack = Math.Atan2(cross, ux),
            LeadingAngle = lead,
            TailFirst = tailFirst,
            Mach = mach,
            Lift = lift,
            Drag = drag,
            ForceX = -drag * ux + lift * sign * sinLead,
        };

        if (cross > 1e-12)
        {
            double across = -drag * cross - lift * cosLead;
            c.ForceY = across * uy / cross;
            c.ForceZ = across * uz / cross;
        }

        return c;
    }

    /// <summary>s(a): how much of the attached-flow lift survives at leading-end angle a.</summary>
    public static double AttachedFraction(double leadingAngle)
    {
        double start = AeroParameters.StallAngleDeg * DegToRad;
        double end = (AeroParameters.StallAngleDeg + AeroParameters.StallWidthDeg) * DegToRad;
        return Lerp(1.0, AeroParameters.PostStallAttachedFraction, SmoothStep(start, end, leadingAngle));
    }

    /// <summary>Multiplier on the zero-alpha pressure and base drag at Mach M.</summary>
    public static double TransonicFactor(double mach, double peak, double supersonic)
    {
        if (mach <= AeroParameters.DragPeakMach)
            return Lerp(1.0, peak, SmoothStep(AeroParameters.DragRiseStartMach, AeroParameters.DragPeakMach, mach));

        // Zero slope at the peak, then a slow 1/x^2 settle rather than an exponential cliff.
        double x = (mach - AeroParameters.DragPeakMach) / AeroParameters.DragDecayWidthMach;
        return supersonic + (peak - supersonic) / (1.0 + x * x);
    }

    public static double CrossflowCd(double crossMach)
        => Interpolate(AeroParameters.CrossflowMach, AeroParameters.CrossflowCd, crossMach);

    public static double FinenessEta(double fineness)
        => Interpolate(AeroParameters.FinenessGrid, AeroParameters.FinenessEta, fineness);

    private static double Interpolate(double[] xs, double[] ys, double x)
    {
        if (!(x > xs[0]))
            return ys[0];
        for (int i = 1; i < xs.Length; i++)
        {
            if (x <= xs[i])
                return Lerp(ys[i - 1], ys[i], (x - xs[i - 1]) / (xs[i] - xs[i - 1]));
        }
        return ys[^1];
    }

    private static double SmoothStep(double edge0, double edge1, double x)
    {
        double t = Math.Clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }

    private static double Lerp(double a, double b, double t) => a + (b - a) * t;
}

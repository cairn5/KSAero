using System.Globalization;
using KSAero;

CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

int failures = 0;
void Check(bool ok, string what)
{
    if (!ok)
    {
        failures++;
        Console.WriteLine($"FAIL  {what}");
    }
}

const double Deg = Math.PI / 180.0;

// Nose pitched up by alpha above the airspeed, rolled by phi about the body axis.
AeroCoefficients At(AeroBody body, double alphaDeg, double mach, double rollDeg = 0.0)
{
    double a = alphaDeg * Deg, p = rollDeg * Deg;
    return AeroModel.Evaluate(in body, Math.Cos(a), Math.Sin(a) * Math.Sin(p), -Math.Sin(a) * Math.Cos(p), mach);
}

// Stock: a box with Cd 0.3 / 1.0 on the X faces, 1.2 on the sides, plus 0.1 x box surface, as C_D on pi/4 D^2.
double StockCd(AeroBody body, double alphaDeg, double rollDeg = 0.0)
{
    double a = alphaDeg * Deg, p = rollDeg * Deg;
    double dx = body.Length, d = body.Diameter;
    double ax = Math.PI / 4.0 * d * d, side = dx * d;
    double ux = Math.Cos(a), uy = Math.Sin(a) * Math.Sin(p), uz = Math.Sin(a) * Math.Cos(p);
    double cda = (ux >= 0 ? 0.3 : 1.0) * Math.Abs(ux) * ax + 1.2 * (Math.Abs(uy) + Math.Abs(uz)) * side;
    double skin = 0.1 * 2.0 * (ax + 2.0 * side);
    return (cda + skin) / ax;
}

var slender = new AeroBody(50.0, 3.7);   // a two-stage launcher, L/D 13.5
var stubby = new AeroBody(6.0, 3.0);     // a short lander or upper stage, L/D 2
var capsule = new AeroBody(3.0, 4.0);    // squat, L/D 0.75
var bodies = new[] { ("slender", slender), ("stubby", stubby), ("capsule", capsule), ("needle", new AeroBody(80.0, 2.0)) };

foreach (var (name, body) in bodies)
{
    // Zero alpha: no lift, drag is nose drag plus skin friction, and it acts straight back.
    var zero = At(body, 0.0, 0.3);
    double cd0 = AeroParameters.CdNoseFirst + AeroParameters.SkinFrictionCf * 4.0 * body.Fineness;
    Check(Math.Abs(zero.Lift) < 1e-12, $"{name}: lift at alpha 0");
    Check(Math.Abs(zero.Drag - cd0) < 1e-12, $"{name}: drag at alpha 0 {zero.Drag} vs {cd0}");
    Check(Math.Abs(zero.ForceX + cd0) < 1e-12 && zero.ForceY == 0 && zero.ForceZ == 0, $"{name}: force at alpha 0");

    var tail = At(body, 180.0, 0.3);
    double cdTail = AeroParameters.CdTailFirst + AeroParameters.SkinFrictionCf * 4.0 * body.Fineness;
    Check(Math.Abs(tail.Drag - cdTail) < 1e-12 && tail.TailFirst && Math.Abs(tail.ForceX - cdTail) < 1e-12, $"{name}: tail-first drag");

    double prevDrag = 0.0, prevCl = 0.0;
    for (double alpha = 0.0; alpha <= 180.0; alpha += 0.25)
    {
        foreach (double mach in new[] { 0.3, 0.95, 1.05, 2.0, 6.0 })
        {
            var c = At(body, alpha, mach);
            double a = alpha * Deg;

            // Axisymmetric: rolling the vehicle changes nothing but the direction.
            for (double roll = 15.0; roll < 360.0; roll += 37.0)
            {
                var r = At(body, alpha, mach, roll);
                Check(Math.Abs(r.Lift - c.Lift) < 1e-12 && Math.Abs(r.Drag - c.Drag) < 1e-12, $"{name}: roll changes coefficients at a={alpha} r={roll}");
            }

            // The force vector carries exactly C_L and C_D. Velocity is u, lift axis is e_L.
            double ux = Math.Cos(a), uz = -Math.Sin(a);
            double sign = ux < 0 ? -1.0 : 1.0;
            double cosLead = Math.Abs(ux), sinLead = Math.Abs(uz);
            double ex = sign * sinLead, ez = cosLead;
            double alongU = c.ForceX * ux + c.ForceZ * uz;
            double alongL = c.ForceX * ex + c.ForceZ * ez;
            Check(Math.Abs(alongU + c.Drag) < 1e-9, $"{name}: force along airspeed != -C_D at a={alpha} M={mach}");
            Check(Math.Abs(alongL - c.Lift) < 1e-9, $"{name}: force along lift axis != C_L at a={alpha} M={mach}");
            Check(Math.Abs(c.ForceY) < 1e-12, $"{name}: side force in the pitch plane at a={alpha}");
            Check(c.Drag >= 0.0, $"{name}: negative drag at a={alpha} M={mach}");
        }

        // Subsonic drag never falls as the nose comes round, stall included. Short bodies are
        // exempt past 50 degrees: their end face presents more area at 60-70 than broadside. The
        // 0.5% allows the slender-body term's fade over the last few degrees to 90.
        var sub = At(body, alpha, 0.3);
        if (alpha <= (body.Fineness >= 5.0 ? 90.0 : 50.0))
        {
            Check(sub.Drag >= 0.995 * prevDrag, $"{name}: drag falls at a={alpha} ({prevDrag:F4} -> {sub.Drag:F4})");
            prevDrag = Math.Max(prevDrag, sub.Drag);
        }

        // Continuous in alpha: no step bigger than the steepest smooth slope could make.
        if (alpha > 0.0)
            Check(Math.Abs(sub.Lift - prevCl) < 0.02 * body.PlanformArea / body.ReferenceArea + 0.05, $"{name}: lift jumps at a={alpha}");
        prevCl = sub.Lift;
    }

    // Continuous in Mach, at a range of attitudes.
    foreach (double alpha in new[] { 0.0, 10.0, 45.0, 120.0, 180.0 })
    {
        double prev = At(body, alpha, 0.0).Drag;
        for (double mach = 0.001; mach <= 8.0; mach += 0.001)
        {
            double cd = At(body, alpha, mach).Drag;
            Check(Math.Abs(cd - prev) < 0.01 * Math.Max(1.0, prev), $"{name}: drag jumps at M={mach} a={alpha}");
            prev = cd;
        }
    }
}

// The transonic peak sits at the peak Mach, above both sides.
{
    double sub = At(slender, 0.0, 0.5).Drag, peak = At(slender, 0.0, AeroParameters.DragPeakMach).Drag, sup = At(slender, 0.0, 3.0).Drag;
    Check(peak > sub && peak > sup && sup > sub, $"transonic shape {sub:F3} / {peak:F3} / {sup:F3}");
}

// Stall: a short body loses lift past the stall, a slender one only bends.
{
    double before = At(stubby, AeroParameters.StallAngleDeg, 0.3).Lift;
    double after = At(stubby, AeroParameters.StallAngleDeg + AeroParameters.StallWidthDeg, 0.3).Lift;
    Check(after < 0.7 * before, $"stubby stall {before:F3} -> {after:F3}");
}

// Nose up gives lift up.
Check(At(slender, 5.0, 0.5).Lift > 0.0 && At(slender, 5.0, 0.5).ForceZ > 0.0, "lift sign");

// A body two or more diameters long, flying engine-first, lifts towards the side its engine end is
// tilted, from the first degree and more with every degree, at every Mach. Guidance steering by lift
// needs the sign to hold still; without the attached term it flipped with Mach on booster shapes.
foreach (double fineness in new[] { 2.0, 4.0, 6.0, 8.0, 11.0, 20.0 })
{
    var body = new AeroBody(fineness * 3.7, 3.7);
    foreach (double mach in new[] { 0.3, 0.8, 0.95, 1.05, 1.2, 1.5, 2.0, 3.0, 4.0, 6.0 })
    {
        double prev = 0.0;
        for (double lead = 0.5; lead <= 20.0; lead += 0.5)
        {
            double cl = At(body, 180.0 - lead, mach).Lift;
            Check(cl > prev, $"engine-first lift not rising at L/D {fineness}, M {mach}, {lead} deg: {prev:F4} -> {cl:F4}");
            prev = cl;
        }
    }
}

// A capsule heat shield first lifts like Apollo, against the slender-body sense, at every trim angle.
foreach (double lead in new[] { 10.0, 15.0, 20.0, 25.0, 30.0 })
{
    var c = At(capsule, 180.0 - lead, 3.0);
    double ld = c.Lift / c.Drag;
    Check(ld < -0.1 && ld > -0.45, $"capsule L/D at {lead} deg off heat-shield-first: {ld:F3}");
}

Console.WriteLine($"{(failures == 0 ? "PASS" : $"{failures} FAILURES")}");
Console.WriteLine();

void Table(string name, AeroBody body, double mach)
{
    Console.WriteLine($"{name}: L {body.Length} m, D {body.Diameter} m, L/D {body.Fineness:F1}, A_ref {body.ReferenceArea:F2} m^2, Mach {mach}");
    Console.WriteLine("  alpha    C_L     C_D     L/D   | stock C_D (roll 0 / 45)");
    foreach (double alpha in new[] { 0.0, 2, 5, 10, 15, 20, 25, 30, 40, 50, 60, 75, 90, 120, 150, 165, 180 })
    {
        var c = At(body, alpha, mach);
        Console.WriteLine($"  {alpha,5:F0}  {c.Lift,7:F3} {c.Drag,7:F3} {(c.Drag > 0 ? c.Lift / c.Drag : 0),6:F2}   | {StockCd(body, alpha),7:F2} / {StockCd(body, alpha, 45),7:F2}");
    }
    Console.WriteLine();
}

Table("slender launcher", slender, 0.5);
Table("stubby lander", stubby, 0.5);
Table("capsule", capsule, 0.5);

Console.WriteLine("Mach sweep, slender launcher: C_D at alpha 0 / 180, C_L at alpha 5 / 30 / 60");
foreach (double mach in new[] { 0.0, 0.5, 0.8, 0.9, 1.0, 1.05, 1.1, 1.2, 1.5, 2.0, 3.0, 5.0, 10.0 })
{
    Console.WriteLine($"  M {mach,5:F2}   C_D0 {At(slender, 0, mach).Drag,6:F3}  C_D180 {At(slender, 180, mach).Drag,6:F3}   " +
                      $"C_L5 {At(slender, 5, mach).Lift,6:F3}  C_L30 {At(slender, 30, mach).Lift,6:F3}  C_L60 {At(slender, 60, mach).Lift,6:F3}");
}

return failures == 0 ? 0 : 1;

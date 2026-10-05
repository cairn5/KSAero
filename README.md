# KSAero

A deliberately simple aerodynamics mod for Kitten Space Agency, to stand in until the game has
real aero. It replaces stock's atmospheric drag with a body-of-revolution model that has
angle-of-attack drag, lift with a stall, and a transonic drag rise. Rotational dynamics are left
exactly as stock.

Written against the [StarMap](https://github.com/StarMapLoader/StarMap) loader and tested against
KSA `v2026.10.7.5541`.

## What stock does

`PhysicsStates.ComputeDrag`, fed by `VehicleProperties.RecomputeAerodynamicProperties`:

```
F = (CdA_box(v_hat) + 0.1 * S_box) * q          along -v, through the centre of mass
CdA_box = sum over axes |v_i| * Cd_i * A_i      Cd: nose 0.3, tail 1.0, each side 1.2
```

* A box: the side term is `|v_y| A_y + |v_z| A_z`, so rolling a round rocket 45 degrees changes its
  drag by up to sqrt(2).
* Drag rises linearly with |sin(alpha)| from the first degree. Real crossflow drag grows as
  sin^3(alpha).
* The `0.1 * S_box` skin term is about 40x a real turbulent skin-friction coefficient (0.002-0.003)
  and is applied to the whole bounding-box surface. On a 50 m x 3.7 m launcher it is 95% of the
  nose-on drag, which comes out at Cd 7.4 on the frontal area. A real launcher is 0.3-0.5.
* No lift, no Mach dependence.

## What KSAero does instead

The vehicle is a cylinder along the assembly X axis (nose at +X), length L from the bounding box
and diameter `D = sqrt(dy dz)`, which keeps stock's frontal area `pi/4 dy dz`. Forces follow Allen
and Perkins' viscous-crossflow method for bodies of revolution, as extended to 0-180 degrees by
Jorgensen. With `a` the angle from whichever end leads and coefficients on `A_ref = pi D^2 / 4`:

```
N_att = sin(2a) cos(a/2)                          attached (slender-body) normal force, nose-first only
N_x   = eta(L/D, Mn) c_dc(Mn) (L D / A_ref) sin^2 a   separated crossflow, Mn = M sin a
C_A   = (Cd0 f(M) + Cf pi D L / A_ref) cos^2 a     Cd0 = 0.3 nose-first, 1.0 tail-first, Cf = 0.0025

C_L = (s(a) N_att + N_x) cos a - C_A sin a
C_D = (N_att + N_x) sin a + C_A cos a
```

* **Axisymmetric.** Roll changes the direction of the lift, never its size.
* **Drag vs alpha.** Stays near the nose-on value for the first few degrees, then rises as the
  crossflow builds. Broadside it is about half of stock's (finite-length factor eta, and no skin
  term on top).
* **Lift and stall.** The attached-flow lift stalls between 20 and 30 degrees: `s(a)` takes away its
  lift share and keeps its drag, as a separated body would. The crossflow lift does not stall and
  peaks around 55-60 degrees, as it does on real bodies. On a slender rocket the stall is a knee in
  the lift curve; on a short body (a lander, a capsule nose-first) it is a cliff.
* **Tail-first.** A flat base leading separates the flow at its edge, so there is no attached lift,
  only crossflow and the tilted axial force. A capsule flying heat shield first gets an L/D of about
  -0.2 to -0.3, like Apollo.
* **Mach 1.** The zero-alpha pressure drag is flat to M 0.8, peaks at M 1.05 (2.0x nose-first, 1.7x
  blunt base first) and settles towards 1.3x / 1.6x supersonic.
* **Lift at Mach 1?** Not as a spike. Slender-body lift slope is Mach-independent, and the
  crossflow peaks when the *crossflow* Mach `M sin a` passes 1, not when M does: at 60 degrees that
  is around M 1.5, at 5 degrees it is hypersonic. Through M 1 the small-alpha lift actually dips
  slightly, because the axial drag rise tilts against it. Wings have a Prandtl-Glauert lift rise
  towards M 1; a cylinder does not.

Forces act through the centre of mass, so there are still no aero torques. Stock's rotational
damping term is kept as is. Ocean drag is untouched.

### Numbers

Slender launcher, L 50 m, D 3.7 m, Mach 0.5. `C_L`, `C_D` on the frontal area; stock has no lift.

| alpha | C_L   | C_D   | L/D  | stock C_D (roll 0 / 45) |
|------:|------:|------:|-----:|------------------------:|
| 0     | 0.000 | 0.435 | 0.00 | 7.38 / 7.38             |
| 5     | 0.243 | 0.455 | 0.53 | 9.18 / 9.93             |
| 10    | 0.684 | 0.549 | 1.25 | 10.96 / 12.45           |
| 20    | 2.025 | 1.146 | 1.77 | 14.43 / 17.35           |
| 30    | 2.913 | 2.477 | 1.18 | 17.67 / 21.94           |
| 60    | 5.405 | 10.23 | 0.53 | 25.11 / 32.52           |
| 90    | 0.000 | 15.59 | 0.00 | 27.73 / 36.28           |
| 180   | 0.000 | 1.135 | 0.00 | 8.08 / 8.08             |

Expect much less drag on ascent than stock (about 17x less nose-on), so higher speeds low in the
atmosphere and a higher max-Q. Stock destroys a vehicle at 200 kPa dynamic pressure.

`dotnet run --project KSAero.Tests` checks the model's invariants and prints these tables for a
launcher, a short lander and a capsule, plus a Mach sweep.

## Tuning

Every number is a constant in `AeroParameters` in [KSAero/AeroModel.cs](KSAero/AeroModel.cs).

## In game

While the controlled vehicle is in an atmosphere a small **KSAero** window shows Mach, q, angle of
attack, C_L, C_D, lift and drag in kN, and the equivalent cylinder. It runs the same function the
physics does.

## How it hooks in

A Harmony transpiler on `PhysicsStates.ComputeDerivatives` swaps the one `ComputeDrag` call that is
passed `PhysicsEnvironment.AtmosphericDensity` for `AirForcePatch.ComputeAirForces`, which has the
same signature. The call site is rewritten rather than `ComputeDrag` patched, because `ComputeDrag`
is `AggressiveInlining`. If the method no longer has exactly one such call the patch refuses, logs a
warning, and stock aero stays.

Mach uses `a = sqrt(1.4 p / rho)` from the game's own pressure and density. KSA atmospheres are
isothermal exponentials, so the speed of sound is constant per body (340 m/s on an Earth-like one).

## Build

.NET 10. Game assemblies come from `KsaDir` (default `C:\Program Files\Kitten Space Agency`).

```
dotnet build KSAero
```

builds and copies `KSAero.dll` and `mod.toml` into
`Documents\My Games\Kitten Space Agency\mods\KSAero\`. Pass `-p:DeployToMods=false` to skip that.
Enable the mod in the game's mod list (or `manifest.toml`).

## Caveats

* The bounding box is the only geometry. Side boosters, fins and wings widen it, and the model
  treats the result as a fat cylinder.
* No aero torques, so nothing weathervanes, and lift does not trim the vehicle.
* AdvancedFlightComputer measures its drag table through `PhysicsStates.ComputeDerivatives`, so it
  picks up KSAero's drag and transonic rise without knowing KSAero exists. Its solvers model drag
  only; the lift is reported on its aero gauge but left out of the predictions.

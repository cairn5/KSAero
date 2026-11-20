# KSAero

Control surfaces for Kitten Space Agency, to stand in until the game has its own. KSA 5901's stock
aerodynamics builds the vehicle's aero mesh from part colliders and flies it with a supersonic
panel method, so a part that moves its collider is flown where it is. KSAero adds parts that do.

Written against the [StarMap](https://github.com/StarMapLoader/StarMap) loader and tested against
KSA `v2026.11.16.5901`. Earlier versions replaced stock's drag with a body-of-revolution model;
stock aero is better than that now, so it is gone.

## Parts

All are in the **Aero** category of the part picker and attach to a hull like a landing leg.

| Part | Blade (span x chord) | Mass |
|---|---|---|
| All-Moving Fin 1 m | 1 m x 1 m, 35 mm | 40 kg |
| All-Moving Fin 2 m | 2 m x 2 m, 35 mm | 160 kg |
| All-Moving Fin 3 m | 3 m x 3 m, 52 mm | 360 kg |

The whole blade turns about its span axis, through the middle of the chord. Its look is the stock
flat panel of the same size turned on edge (the 3 m blade is the 2 m panel scaled up); KSAero has no
meshes of its own yet.

## In game

While the controlled vehicle has fins, the **KSAero fins** window commands them: one slider for all
fins and one per fin, a maximum deflection (default 45 deg) and an actuator rate (default 30 deg/s).
It also shows how often the aero mesh is rebuilt and how long physics waits for it.

Deflections are not saved: a loaded vehicle's fins start at zero. Nothing moves them except the
window yet; there is no pitch/yaw/roll mixing.

## How it works

* A fin is one part. The blade is a subpart with its own box collider, defined in
  [KSAero/KSAeroParts.xml](KSAero/KSAeroParts.xml). The collider is 10 mm thick, so stock's aero
  mesh always makes the blade a zero-depth sheet.
* Once a frame, at the start of `PartTree.FlushDirtyDerived` (a Harmony prefix), `FinActuators`
  steps each fin towards its command at the actuator rate and re-poses the blade subpart about the
  part's X axis, as `KeyframeAnimationModule` poses a subpart with a supplemental rotation. That is
  on the main thread, after the frame has waited for the vehicle solver and before the next solve
  starts, so no physics step sees a half-moved blade. This moves the model and the collider.
* The aero mesh is not rebuilt. `AeroMeshSource.Gather`, which every mesh build starts from, is
  wrapped so the blades sit at rest while it runs: every mesh has the fins at rest, and deflection
  never changes the mesh hash. When `PartTree.AwaitAerodynamics` hands a physics step its mesh,
  `FinAeroPanels` turns each fin's sheet panels in that mesh to the fin's angle, a rigid rotation
  about the hinge computed from the rest geometry (`FinPanelSet`), and makes the surface flow
  recompute.

### Cost and accuracy

Offline, against meshes built by KSA's own `AeroAssembly`:

| Case | Turn the panels + flow + loads | Full re-mesh |
|---|---|---|
| 1 m fin on a 3.7 m hull | 0.26 ms | 7.5 ms |
| 3 m fin on a 1.2 m hull (91 fin panels) | 0.64 ms | 13.4 ms |

The flow and loads are computed every step anyway, so the pose itself adds little; and nothing
blocks physics. Where the mesh resolves the fin (3 m fin, 0.21 m panels) the loads match a full
re-mesh at the same angle within about 1-4%. Where it does not, re-meshing at each angle is noisy
and not even monotonic in the angle, while turning the panels stays linear.

### Fin size

Stock aero sizes its panels from the vehicle (about an eighth of the main body's section) and
loses about half a panel around a sheet's edges. On a 3.7 m launcher (0.65 m panels) a 1 m fin
keeps under half its area in the aero mesh, a 2 m fin a little under half, a 3 m fin about two
thirds; a 1 m fin standing clear of the hull disappears. Size fins to several panel widths.

## Build

.NET 10. Game assemblies come from `KsaDir` (default `C:\Program Files\Ahwoo\Games\Kitten Space Agency`, the Ahwoo launcher install).

```
dotnet build KSAero
```

builds and copies `KSAero.dll`, `mod.toml` and `KSAeroParts.xml` into
`Documents\My Games\Kitten Space Agency\mods\KSAero\`. Pass `-p:DeployToMods=false` to skip that.
Enable the mod in the game's mod list (or `manifest.toml`).

The assembly is AnyCPU, so the same `KSAero.dll` loads on Windows, Linux and Apple silicon Macs.
Do not build with `--runtime` or a `PlatformTarget`: that marks it for one architecture, and .NET on
another refuses to load it.

### macOS

Set `USERPROFILE`, which sets where the mod deploys, and `KsaDir`, which locates the game.

```sh
USERPROFILE="$HOME" dotnet build KSAero -p:GenerateLaunchProfile=false -p:KsaDir="$HOME/Applications/Kitten Space Agency.app/Contents/Resources"
```

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
fins and one per fin, a maximum deflection (default 30 deg) and an actuator rate (default 30 deg/s).
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
  never changes the mesh hash.
* When `PartTree.AwaitAerodynamics` hands a physics step its mesh, `FinAeroPanels` claims each
  fin's sheet panel slots in that mesh, writes the blade's true flat plate into them (equal strips
  of the real rectangle), turns it to the fin's angle about the hinge (`FinPanelSet`), and makes
  the surface flow recompute. Only panels inside a KSAero blade's box are claimed; nothing else in
  the mesh is touched.

### Why the plate is rewritten

Stock dual-contours a sheet on a lattice one panel width across (panel width is about an eighth of
the main body's cross-section, 0.65 m on a 3.7 m launcher). Each sheet quad joins the mean crossing
points of four lattice cells, and lattice edges that end inside a body are dropped. So the sheet
stops about half a cell short of the tip and both chord edges, starts about a cell off the hull,
and its root panels warp, leaning up to 15 degrees. Both faces of a sheet are flown; nothing is
one-sided. Areas stock makes of a square fin:

| Fin | 3.7 m hull (0.65 m panels) | 1.2 m hull (0.21 m panels) |
|---|---|---|
| 1 m | 0.46 m^2 of 1 | 0.63 |
| 2 m | 1.83 of 4 | 3.36 |
| 3 m | 5.95 of 9 | 7.78 |

A free-standing 1 m fin half a metre off a 3.7 m hull is dropped altogether. Rewriting the
claimed slots with the real rectangle gives the fin its full area, its true centre and a flat
normal; the load law needs nothing else (area, normal, centroid, and the corners for shielding).
A fin the mesher dropped has no slots, so no aero; the fins window says so.

Stock's own aero overlay draws the mesh's shared vertex list, so it still shows stock's sheets at
rest, not the plates flown.

### Cost and accuracy

Offline, against meshes built by KSA's own `AeroAssembly`, with a second fixed fin and a flat
panel next to the KSAero fin (neither is ever claimed or moved):

* Side force for 5 deg of deflection, against Ackeret's flat-plate value: within 1-4% for 1, 2 and
  3 m fins on both hulls at Mach 2 and 5, linear in the angle and antisymmetric; 10% low for the
  2 m fin on the 3.7 m hull, behind the flat panel. Re-meshing at each angle instead gives a
  quarter to a half of it and is not monotonic in the angle.
* Pose, flow update and loads together: 0.3-0.7 ms, against 7-14 ms for a re-mesh. The flow and
  loads run every step anyway, and nothing blocks physics.

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

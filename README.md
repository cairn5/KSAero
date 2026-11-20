# KSAero

Control surfaces for Kitten Space Agency, to stand in until the game has its own. KSA 5901's stock
aerodynamics builds the vehicle's aero mesh from part colliders and flies it with a supersonic
panel method, so a part that moves its collider is flown where it is. KSAero adds parts that do.

Written against the [StarMap](https://github.com/StarMapLoader/StarMap) loader and tested against
KSA `v2026.11.16.5901`. Earlier versions replaced stock's drag with a body-of-revolution model;
stock aero is better than that now, so it is gone.

## Parts

Both are in the **Aero** category of the part picker and attach to a hull like a landing leg.

| Part | Blade (span x chord) | Mass |
|---|---|---|
| All-Moving Fin 1 m | 1 m x 1 m, 35 mm | 40 kg |
| All-Moving Fin 2 m | 2 m x 2 m, 35 mm | 160 kg |

The whole blade turns about its span axis, through the middle of the chord. Its look is the stock
flat panel of the same size turned on edge; KSAero has no meshes of its own yet.

## In game

While the controlled vehicle has fins, the **KSAero fins** window commands them: one slider for all
fins and one per fin, a maximum deflection (default 30 deg) and an actuator rate (default 30 deg/s).
It also shows how often the aero mesh is rebuilt and how long physics waits for it.

Deflections are not saved: a loaded vehicle's fins start at zero. Nothing moves them except the
window yet; there is no pitch/yaw/roll mixing.

## How it works

* A fin is one part. The blade is a subpart with its own box collider, defined in
  [KSAero/KSAeroParts.xml](KSAero/KSAeroParts.xml).
* Once a frame, at the start of `PartTree.FlushDirtyDerived` (a Harmony prefix), `FinActuators`
  steps each fin towards its command at the actuator rate and re-poses the blade subpart about the
  part's X axis, as `KeyframeAnimationModule` poses a subpart with a supplemental rotation. That is
  on the main thread, after the frame has waited for the vehicle solver and before the next solve
  starts, so no physics step sees a half-moved blade.
* Moving a subpart does not dirty stock's aero mesh by itself, so `FinActuators` marks
  `DerivedData.AeroMesh` dirty. The same flush gathers the colliders and starts the rebuild. The
  angle is rounded to 0.1 deg, so a fin at rest never causes one.

### Cost

Each rebuild re-meshes the whole vehicle, and the next physics step waits for it
(`PartTree.AwaitAerodynamics`). Offline, a 40 m body with four fins took 5-17 ms per rebuild, so a
fin that is moving costs about that much per frame. A fin at rest costs nothing.

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

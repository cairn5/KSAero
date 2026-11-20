using System.Numerics;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;
using KSA.Aero;

namespace KSAero;

/// <summary>
/// Keeps stock aero in step with the fins without re-meshing the vehicle.
///
/// Every aero mesh is built with the blades at rest: <see cref="AeroMeshSource.Gather"/>, which
/// reads the collider poses for every build (flight and editor), is wrapped so the blades sit at
/// rest while it runs. The source hash therefore never changes with deflection, and a fin moving
/// never triggers a rebuild.
///
/// After <see cref="PartTree.AwaitAerodynamics"/> hands a physics step its mesh, each fin's sheet
/// panels in that mesh are turned to the fin's current angle (<see cref="FinPanelSet"/>), if they
/// are not there already. That is on the vehicle's own solver thread, before the step computes its
/// loads; the main thread only changes the angles between solves.
/// </summary>
internal static class FinAeroPanels
{
    private sealed class FinEntry(Part fin, FinPanelSet panels, Vector3 hinge, doubleQuat fin2Vehicle)
    {
        public readonly Part Fin = fin;
        public readonly FinPanelSet Panels = panels;
        public readonly Vector3 Hinge = hinge;
        public readonly doubleQuat Fin2Vehicle = fin2Vehicle;
        public double PosedDeg;
    }

    private sealed class MeshFins(FinEntry[] fins)
    {
        public readonly FinEntry[] Fins = fins;
    }

    private static readonly ConditionalWeakTable<AeroMesh, MeshFins> Maps = new();

    [ThreadStatic] private static List<Part>? _scratch;

    /// <summary>Fins and fin panels found in a mesh, for the debug window.</summary>
    internal static (int Fins, int Panels) Describe(AeroMesh mesh)
    {
        if (!Maps.TryGetValue(mesh, out MeshFins? map))
            return (0, 0);
        int panels = 0;
        foreach (FinEntry entry in map.Fins)
            panels += entry.Panels.PanelCount;
        return (map.Fins.Length, panels);
    }

    // ---- Gather at rest ----

    internal static void GatherPrefix(PartTree tree, out List<Part>? __state)
    {
        __state = null;
        ReadOnlySpan<Part> parts = tree.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            Part part = parts[i];
            if (!FinActuators.IsFin(part) || FinActuators.StateOf(part).AppliedDeg == 0.0)
                continue;
            FinActuators.PoseBlades(part, 0.0);
            (__state ??= new List<Part>()).Add(part);
        }
    }

    internal static void GatherPostfix(List<Part>? __state)
    {
        if (__state == null)
            return;
        foreach (Part fin in __state)
            FinActuators.PoseBlades(fin, FinActuators.StateOf(fin).AppliedDeg);
    }

    // ---- Pose the mesh ----

    internal static void Apply(PartTree tree, Aerodynamics? aerodynamics)
    {
        AeroMesh? mesh = aerodynamics?.Mesh;
        if (mesh == null || mesh.Surface.Sheets.Count == 0)
            return;

        if (!Maps.TryGetValue(mesh, out MeshFins? map))
        {
            // Built once per mesh, empty for a vehicle without fins, so the usual cost is one lookup.
            MeshFins built = Build(tree, mesh);
            map = Maps.GetValue(mesh, _ => built);
        }
        if (map.Fins.Length == 0)
            return;

        lock (map)
        {
            AeroSurfaceFlow? flow = null;
            bool fetched = false;
            foreach (FinEntry entry in map.Fins)
            {
                double deg = FinActuators.StateOf(entry.Fin).AppliedDeg;
                if (deg == entry.PosedDeg || entry.Panels.PanelCount == 0)
                    continue;
                if (!fetched)
                {
                    flow = aerodynamics!.GetSurfaceFlow();
                    fetched = true;
                }
                entry.Panels.Pose(mesh.Surface, flow, entry.Hinge, VehicleTurn(entry.Fin2Vehicle, deg));
                entry.PosedDeg = deg;
            }
        }
    }

    /// <summary>
    /// The fin's turn about its part X axis, as a rotation of vehicle-frame vectors: into the fin's
    /// frame, turn, back out.
    /// </summary>
    internal static Quaternion VehicleTurn(doubleQuat fin2Vehicle, double deg)
    {
        doubleQuat turn = doubleQuat.CreateFromAxisAngle(double3.UnitX, deg * Math.PI / 180.0);
        doubleQuat inVehicle = doubleQuat.Concatenate(doubleQuat.Concatenate(doubleQuat.Inverse(fin2Vehicle), turn), fin2Vehicle);
        return Quaternion.Normalize(inVehicle.ToBepu());
    }

    private static MeshFins Build(PartTree tree, AeroMesh mesh)
    {
        List<Part> fins = _scratch ??= new List<Part>();
        fins.Clear();
        ReadOnlySpan<Part> parts = tree.Parts;
        for (int i = 0; i < parts.Length; i++)
        {
            if (FinActuators.IsFin(parts[i]))
                fins.Add(parts[i]);
        }
        if (fins.Count == 0)
            return new MeshFins(Array.Empty<FinEntry>());

        var entries = new List<FinEntry>(fins.Count);
        foreach (Part fin in fins)
        {
            ReadOnlySpan<Part> subParts = fin.SubParts;
            for (int i = 0; i < subParts.Length; i++)
            {
                Part blade = subParts[i];
                if (!FinActuators.IsBlade(blade))
                    continue;
                Span<ColliderModule> colliders = blade.Modules.Get<ColliderModule>();
                for (int j = 0; j < colliders.Length; j++)
                {
                    if (colliders[j].ShapeTemplate is not BoxColliderTemplate box)
                        continue;
                    ColliderModule collider = colliders[j];

                    // The blade's rest pose in the vehicle frame, as Gather would see it at rest.
                    doubleQuat fin2Vehicle = fin.Asmb2VehicleAsmb;
                    doubleQuat bladeRot = doubleQuat.Concatenate(blade.Asmb2ParentAsmbSafe, fin2Vehicle);
                    double3 bladePos = fin.PositionVehicleAsmb + blade.PositionParentAsmbSafe.Transform(fin2Vehicle);
                    double3 boxCenter = bladePos + collider.PositionPartAsmb.Transform(bladeRot);
                    doubleQuat boxRot = doubleQuat.Concatenate(collider.Collider2PartAsmb, bladeRot);
                    double scale = collider.Scale;
                    var half = new Vector3(
                        (float)(0.5 * (double)box.LengthX * scale),
                        (float)(0.5 * (double)box.LengthY * scale),
                        (float)(0.5 * (double)box.LengthZ * scale));

                    FinPanelSet panels = FinPanelSet.Select(mesh.Surface, boxCenter.ToBepu(), Quaternion.Normalize(boxRot.ToBepu()), half, mesh.PanelWidth);
                    entries.Add(new FinEntry(fin, panels, fin.PositionVehicleAsmb.ToBepu(), fin2Vehicle));
                }
            }
        }
        fins.Clear();
        return new MeshFins(entries.ToArray());
    }
}

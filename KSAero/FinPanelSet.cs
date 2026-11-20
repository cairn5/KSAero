using System.Numerics;
using BepuUtilities;
using HarmonyLib;
using KSA.Aero;

namespace KSAero;

/// <summary>
/// The sheet panels of one fin blade in a stock aero mesh, and their geometry at rest. Turning the
/// fin rewrites those panels in place, a rigid rotation about the hinge, instead of re-meshing the
/// vehicle. The mesh is always built with the blade at rest (see <see cref="FinAeroPanels"/>), so
/// the geometry captured here is the rest geometry and every pose is computed from it, with no drift.
///
/// A thin blade becomes a zero-depth sheet in the mesh (thinner than a quarter panel width; the
/// blade colliders are 10 mm), with a front and a back face per panel. Panel corners, normals,
/// centroids and edge normals live in <see cref="AeroPanelGeometry"/>'s buffers; the surface flow
/// keeps its own copy of the face centroids, and skips its recompute while the flow direction moves
/// less than a degree, so both are refreshed after a pose.
///
/// Public so an offline check can run it against meshes built from <see cref="AeroAssembly"/>.
/// </summary>
public sealed class FinPanelSet
{
    private static readonly AccessTools.FieldRef<AeroSurfaceFlow, BepuUtilities.Memory.Buffer<Vector3>> FlowCentroids =
        AccessTools.FieldRefAccess<AeroSurfaceFlow, BepuUtilities.Memory.Buffer<Vector3>>("_centroids");

    private static readonly AccessTools.FieldRef<AeroSurfaceFlow, Vector3?> FlowLastDirection =
        AccessTools.FieldRefAccess<AeroSurfaceFlow, Vector3?>("_lastFlowDirection");

    private readonly int[] _panels;
    private readonly Vector3[] _corners;
    private readonly Vector3[] _edgeNormals;
    private readonly Vector3[] _normals;
    private readonly Vector3[] _centroids;

    private FinPanelSet(int[] panels, Vector3[] corners, Vector3[] edgeNormals, Vector3[] normals, Vector3[] centroids)
    {
        _panels = panels;
        _corners = corners;
        _edgeNormals = edgeNormals;
        _normals = normals;
        _centroids = centroids;
    }

    public int PanelCount => _panels.Length;

    /// <summary>The claimed sheet panel indices.</summary>
    public ReadOnlySpan<int> Panels => _panels;

    /// <summary>
    /// The sheet panels lying in a blade's box, in the mesh's frame: centroid inside the box grown by
    /// a quarter panel width (stock's warped panels at a fin root sit up to a seventh of a panel off
    /// the plane, and must turn with the rest), and normal within about 18 degrees of the box's
    /// thinnest axis.
    /// </summary>
    public static FinPanelSet Select(AeroSurface surface, Vector3 center, Quaternion orientation, Vector3 halfExtents, float panelWidth)
    {
        Vector3 ax = Vector3.Transform(Vector3.UnitX, orientation);
        Vector3 ay = Vector3.Transform(Vector3.UnitY, orientation);
        Vector3 az = Vector3.Transform(Vector3.UnitZ, orientation);
        Span<Vector3> axes = stackalloc Vector3[] { ax, ay, az };
        Span<float> half = stackalloc float[] { halfExtents.X, halfExtents.Y, halfExtents.Z };
        int thin = half[0] <= half[1] && half[0] <= half[2] ? 0 : half[1] <= half[2] ? 1 : 2;

        ref readonly AeroPanelGeometry sheets = ref surface.Sheets;
        int lanes = Vector<float>.Count;
        var picked = new List<int>();
        for (int p = 0; p < sheets.Count; p++)
        {
            Vector3Wide.ReadSlot(ref sheets.Centroids[p / lanes], p % lanes, out Vector3 c);
            Vector3Wide.ReadSlot(ref sheets.Normals[p / lanes], p % lanes, out Vector3 n);
            Vector3 d = c - center;
            bool inside = true;
            for (int a = 0; a < 3 && inside; a++)
            {
                float grow = 0.25f * panelWidth + (a == thin ? 0.01f : 0f);
                inside = MathF.Abs(Vector3.Dot(d, axes[a])) <= half[a] + grow;
            }
            if (inside && MathF.Abs(Vector3.Dot(n, axes[thin])) >= 0.95f)
                picked.Add(p);
        }

        int count = picked.Count;
        var corners = new Vector3[4 * count];
        var edgeNormals = new Vector3[4 * count];
        var normals = new Vector3[count];
        var centroids = new Vector3[count];
        for (int i = 0; i < count; i++)
        {
            int p = picked[i];
            int bundle = p / lanes, slot = p % lanes;
            for (int k = 0; k < 4; k++)
            {
                Vector3Wide.ReadSlot(ref sheets.Corners[4 * bundle + k], slot, out corners[4 * i + k]);
                Vector3Wide.ReadSlot(ref sheets.EdgeNormals[4 * bundle + k], slot, out edgeNormals[4 * i + k]);
            }
            Vector3Wide.ReadSlot(ref sheets.Normals[bundle], slot, out normals[i]);
            Vector3Wide.ReadSlot(ref sheets.Centroids[bundle], slot, out centroids[i]);
        }
        return new FinPanelSet(picked.ToArray(), corners, edgeNormals, normals, centroids);
    }

    /// <summary>
    /// Writes the panels turned from rest by <paramref name="turn"/> about <paramref name="hinge"/>,
    /// and brings the surface flow up to date with them if there is one.
    /// </summary>
    public void Pose(AeroSurface surface, AeroSurfaceFlow? flow, Vector3 hinge, Quaternion turn)
    {
        ref readonly AeroPanelGeometry sheets = ref surface.Sheets;
        int lanes = Vector<float>.Count;
        for (int i = 0; i < _panels.Length; i++)
        {
            int p = _panels[i];
            int bundle = p / lanes, slot = p % lanes;
            for (int k = 0; k < 4; k++)
            {
                Vector3Wide.WriteSlot(hinge + Vector3.Transform(_corners[4 * i + k] - hinge, turn), slot, ref sheets.Corners[4 * bundle + k]);
                Vector3Wide.WriteSlot(Vector3.Transform(_edgeNormals[4 * i + k], turn), slot, ref sheets.EdgeNormals[4 * bundle + k]);
            }
            Vector3Wide.WriteSlot(Vector3.Transform(_normals[i], turn), slot, ref sheets.Normals[bundle]);
            Vector3Wide.WriteSlot(hinge + Vector3.Transform(_centroids[i] - hinge, turn), slot, ref sheets.Centroids[bundle]);
        }

        if (flow == null)
            return;
        // The flow's face list is solids, then sheet fronts, then sheet backs.
        BepuUtilities.Memory.Buffer<Vector3> faceCentroids = FlowCentroids(flow);
        int solids = surface.Solids.Count, sheetCount = sheets.Count;
        for (int i = 0; i < _panels.Length; i++)
        {
            Vector3 c = hinge + Vector3.Transform(_centroids[i] - hinge, turn);
            faceCentroids[solids + _panels[i]] = c;
            faceCentroids[solids + sheetCount + _panels[i]] = c;
        }
        FlowLastDirection(flow) = null;
    }
}

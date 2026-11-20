using System.Numerics;
using BepuUtilities;
using HarmonyLib;
using KSA.Aero;

namespace KSAero;

/// <summary>
/// One fin blade's panels in a stock aero mesh. It claims the sheet panel slots the stock mesher
/// made for the blade and writes the blade's true flat plate into them, then turns that plate about
/// the hinge in place instead of re-meshing the vehicle. The mesh is always built with the blade at
/// rest (see <see cref="FinAeroPanels"/>), and every pose is computed from the rest plate, with no
/// drift.
///
/// Why not keep stock's own sheet panels: stock dual-contours a sheet on a lattice one panel width
/// across, with a vertex at the mean crossing point in each cell, and drops lattice edges that end
/// inside a body. So the sheet stops about half a cell short of every free edge and starts about a
/// cell off the hull. On a 3.7 m launcher (0.65 m panels) a 2 m fin keeps 1.8 of its 4 m^2, as a
/// few warped quads whose normals lean up to 14 degrees. The load law only needs each panel's area,
/// normal and centroid (and its corners, for shielding), so the claimed slots are filled with equal
/// strips of the real rectangle: full area, true centre, flat normal. A blade the mesher dropped
/// entirely has no slots and so no aero.
///
/// A thin blade becomes a zero-depth sheet (thinner than a quarter panel width; the blade colliders
/// are 10 mm), with a front and a back face per panel. Panel geometry lives in
/// <see cref="AeroPanelGeometry"/>'s buffers; the surface flow keeps its own copy of the face
/// centroids and fresh running lengths, and skips its recompute while the flow direction moves less
/// than a degree, so those are refreshed after a pose too.
///
/// Public so an offline check can run it against meshes built by <see cref="AeroAssembly"/>.
/// </summary>
public sealed class FinPanelSet
{
    private static readonly AccessTools.FieldRef<AeroSurfaceFlow, BepuUtilities.Memory.Buffer<Vector3>> FlowCentroids =
        AccessTools.FieldRefAccess<AeroSurfaceFlow, BepuUtilities.Memory.Buffer<Vector3>>("_centroids");

    private static readonly AccessTools.FieldRef<AeroSurfaceFlow, BepuUtilities.Memory.Buffer<float>> FlowFreshLengths =
        AccessTools.FieldRefAccess<AeroSurfaceFlow, BepuUtilities.Memory.Buffer<float>>("_freshLengths");

    private static readonly AccessTools.FieldRef<AeroSurfaceFlow, Vector3?> FlowLastDirection =
        AccessTools.FieldRefAccess<AeroSurfaceFlow, Vector3?>("_lastFlowDirection");

    private readonly int[] _panels;
    private readonly Vector3[] _corners;
    private readonly Vector3[] _edgeNormals;
    private readonly Vector3[] _normals;
    private readonly Vector3[] _centroids;
    private readonly float[] _areas;

    private FinPanelSet(int[] panels, Vector3[] corners, Vector3[] edgeNormals, Vector3[] normals, Vector3[] centroids, float[] areas)
    {
        _panels = panels;
        _corners = corners;
        _edgeNormals = edgeNormals;
        _normals = normals;
        _centroids = centroids;
        _areas = areas;
    }

    public int PanelCount => _panels.Length;

    /// <summary>The claimed sheet panel indices.</summary>
    public ReadOnlySpan<int> Panels => _panels;

    /// <summary>Sheet area stock gave the blade before the slots were claimed, m^2.</summary>
    public float StockArea { get; private init; }

    /// <summary>The blade's true one-sided area, written into the slots, m^2 (0 with no slots).</summary>
    public float PlateArea { get; private init; }

    /// <summary>
    /// Claims the sheet panels lying in a blade's box, in the mesh's frame (centroid inside the box
    /// grown by a quarter panel width, normal within about 18 degrees of the box's thinnest axis;
    /// stock's warped panels at a fin root sit up to a seventh of a panel off the plane), and lays the blade's rest plate out over them: the box's mid-plane,
    /// cut into equal strips along its local X axis (the blade's span). Nothing is written yet; the
    /// first <see cref="Pose"/> does that.
    /// </summary>
    public static FinPanelSet Select(AeroSurface surface, Vector3 center, Quaternion orientation, Vector3 halfExtents, float panelWidth)
    {
        Span<Vector3> axes = stackalloc Vector3[]
        {
            Vector3.Transform(Vector3.UnitX, orientation),
            Vector3.Transform(Vector3.UnitY, orientation),
            Vector3.Transform(Vector3.UnitZ, orientation),
        };
        Span<float> half = stackalloc float[] { halfExtents.X, halfExtents.Y, halfExtents.Z };
        int thin = half[0] <= half[1] && half[0] <= half[2] ? 0 : half[1] <= half[2] ? 1 : 2;

        ref readonly AeroPanelGeometry sheets = ref surface.Sheets;
        int lanes = Vector<float>.Count;
        var picked = new List<int>();
        float stockArea = 0f;
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
            {
                picked.Add(p);
                stockArea += sheets.Areas[p / lanes][p % lanes];
            }
        }

        // The plate: span along the first in-plane axis, chord along the second, normal span x chord.
        int spanAxis = thin == 0 ? 1 : 0;
        int chordAxis = 3 - thin - spanAxis;
        Vector3 s = axes[spanAxis], cAxis = axes[chordAxis];
        float hs = half[spanAxis], hc = half[chordAxis];
        Vector3 normal = Vector3.Normalize(Vector3.Cross(s, cAxis));

        int count = picked.Count;
        var corners = new Vector3[4 * count];
        var edgeNormals = new Vector3[4 * count];
        var normals = new Vector3[count];
        var centroids = new Vector3[count];
        var areas = new float[count];
        for (int i = 0; i < count; i++)
        {
            float s0 = -hs + 2f * hs * i / count, s1 = -hs + 2f * hs * (i + 1) / count;
            corners[4 * i + 0] = center + s0 * s - hc * cAxis;
            corners[4 * i + 1] = center + s1 * s - hc * cAxis;
            corners[4 * i + 2] = center + s1 * s + hc * cAxis;
            corners[4 * i + 3] = center + s0 * s + hc * cAxis;
            for (int k = 0; k < 4; k++)
            {
                // As AeroPanelGeometry.Compute: the edge from corner k to k+1, crossed with the normal.
                Vector3 edge = corners[4 * i + ((k + 1) & 3)] - corners[4 * i + k];
                edgeNormals[4 * i + k] = Vector3.Normalize(Vector3.Cross(edge, normal));
            }
            normals[i] = normal;
            centroids[i] = center + 0.5f * (s0 + s1) * s;
            areas[i] = (s1 - s0) * 2f * hc;
        }
        return new FinPanelSet(picked.ToArray(), corners, edgeNormals, normals, centroids, areas)
        {
            StockArea = stockArea,
            PlateArea = count > 0 ? 4f * hs * hc : 0f,
        };
    }

    /// <summary>
    /// Writes the plate turned from rest by <paramref name="turn"/> about <paramref name="hinge"/>
    /// into the claimed slots, and brings the surface flow up to date with it if there is one.
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
            sheets.Areas[bundle] = WithLane(sheets.Areas[bundle], slot, _areas[i]);
        }

        if (flow == null)
            return;
        // The flow's face list is solids, then sheet fronts, then sheet backs.
        BepuUtilities.Memory.Buffer<Vector3> faceCentroids = FlowCentroids(flow);
        BepuUtilities.Memory.Buffer<float> freshLengths = FlowFreshLengths(flow);
        int solids = surface.Solids.Count, sheetCount = sheets.Count;
        for (int i = 0; i < _panels.Length; i++)
        {
            Vector3 c = hinge + Vector3.Transform(_centroids[i] - hinge, turn);
            float fresh = 0.5f * MathF.Sqrt(_areas[i]);
            int front = solids + _panels[i], back = solids + sheetCount + _panels[i];
            faceCentroids[front] = c;
            faceCentroids[back] = c;
            freshLengths[front] = fresh;
            freshLengths[back] = fresh;
        }
        FlowLastDirection(flow) = null;
    }

    private static Vector<float> WithLane(Vector<float> bundle, int lane, float value)
    {
        Span<float> lanes = stackalloc float[Vector<float>.Count];
        bundle.CopyTo(lanes);
        lanes[lane] = value;
        return new Vector<float>(lanes);
    }
}

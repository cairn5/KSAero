using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace KSAero;

/// <summary>
/// KSAero adds control surfaces for KSA's stock aerodynamics: parts in KSAeroParts.xml, moved by
/// <see cref="FinActuators"/>. Stock aero builds its mesh from part colliders, so a moved blade is
/// flown as it is posed.
/// </summary>
[StarMapMod]
public sealed class Mod
{
    private const string TestedGameVersion = "v2026.11.16.5901";
    private const string HarmonyId = "ksaero";

    private static Harmony? _harmony;

    [StarMapAllModsLoaded]
    public void OnFullyLoaded()
    {
        string gameVersion = VersionInfo.Current.VersionString;
        if (gameVersion != TestedGameVersion)
            DefaultCategory.Log.Warning($"[KSAero] Tested against {TestedGameVersion}, current is {gameVersion}.");

        var harmony = new Harmony(HarmonyId);
        try
        {
            Patch(harmony);
            _harmony = harmony;
            DefaultCategory.Log.Info("[KSAero] Loaded. All-moving fins are driven from the KSAero fins window.");
            // Every mod is loaded by now, so AFC's control-surface registry is there to find if AFC is installed.
            AfcSurfaces.TryRegister();
        }
        catch (Exception ex)
        {
            harmony.UnpatchAll(HarmonyId);
            DefaultCategory.Log.Warning($"[KSAero] Patching failed, fins will not move: {ex}");
        }
    }

    internal static void Patch(Harmony harmony)
    {
        harmony.Patch(
            AccessTools.Method(typeof(PartTree), nameof(PartTree.FlushDirtyDerived)),
            prefix: new HarmonyMethod(typeof(FinActuators), nameof(FinActuators.Update)));
        harmony.Patch(
            AccessTools.Method(typeof(KSA.Aero.AeroMeshSource), nameof(KSA.Aero.AeroMeshSource.Gather)),
            prefix: new HarmonyMethod(typeof(FinAeroPanels), nameof(FinAeroPanels.GatherPrefix)),
            postfix: new HarmonyMethod(typeof(FinAeroPanels), nameof(FinAeroPanels.GatherPostfix)));
        harmony.Patch(
            AccessTools.Method(typeof(PartTree), nameof(PartTree.AwaitAerodynamics)),
            prefix: new HarmonyMethod(typeof(AeroWaitMeter), nameof(AeroWaitMeter.Prefix)),
            postfix: new HarmonyMethod(typeof(AeroWaitMeter), nameof(AeroWaitMeter.Postfix)));
    }

    [StarMapAfterGui]
    public void DrawGui(double dt)
    {
        if (_harmony != null)
            FinDebugWindow.Draw(dt);
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(HarmonyId);
        _harmony = null;
        DefaultCategory.Log.Info("[KSAero] Unloaded.");
    }
}

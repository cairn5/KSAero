using Brutal.Logging;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace KSAero;

[StarMapMod]
public sealed class Mod
{
    private const string TestedGameVersion = "v2026.10.7.5541";
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
            AirForcePatch.Apply(harmony);
            _harmony = harmony;
            DefaultCategory.Log.Info("[KSAero] Loaded. Atmospheric forces now use the body-of-revolution model.");
        }
        catch (Exception ex)
        {
            // The transpiler refuses rather than guesses, so a failure here leaves stock aero running.
            harmony.UnpatchAll(HarmonyId);
            DefaultCategory.Log.Warning($"[KSAero] Patching failed, stock aerodynamics stay in place: {ex}");
        }
    }

    [StarMapAfterGui]
    public void DrawGui(double dt)
    {
        if (_harmony != null)
            AeroReadout.Draw();
    }

    [StarMapUnload]
    public void Unload()
    {
        _harmony?.UnpatchAll(HarmonyId);
        _harmony = null;
        DefaultCategory.Log.Info("[KSAero] Unloaded.");
    }
}

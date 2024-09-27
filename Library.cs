using System;
using NordicSpaceLink.BladeRF.Imports;

namespace NordicSpaceLink.BladeRF;

public static class Library
{
    private static LogLevel logLevel = LogLevel.Info;

    public static bool IsLoaded => NativeMethods.Loaded;

    public static Version Version
    {
        get
        {
            if (!IsLoaded)
                throw new Exception("libbladeRF library was not loaded");

            NativeMethods.version(out var version);
            return new(version.major, version.minor, version.patch, version.describe);
        }
    }

    public static LogLevel LogLevel
    {
        get => logLevel;
        set
        {
            if (!IsLoaded)
                throw new Exception("libbladeRF library was not loaded");

            NativeMethods.log_set_verbosity(value);
            logLevel = value;
        }
    }
}

public record struct Version(int Major, int Minor, int Patch, string Description)
{
    public override readonly string ToString() => Description;
}
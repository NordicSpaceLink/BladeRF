/*
 * @file libbladeRF.h
 *
 * @brief bladeRF library
 *
 * Copyright (C) 2013-2017 Nuand LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */

using System;
using System.Reflection;
using System.Runtime.InteropServices;

namespace NordicSpaceLink.BladeRF.Imports;

[StructLayout(LayoutKind.Sequential)]
internal readonly struct Device
{
    private readonly IntPtr value;
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct Stream
{
    private readonly IntPtr value;
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct Image
{
    private readonly IntPtr value;
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct StrPtr
{
    private readonly IntPtr value;

    public static implicit operator string(StrPtr value)
    {
        return value.ToString();
    }

    public override readonly string ToString()
    {
        return Marshal.PtrToStringAnsi(value) ?? "";
    }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct StructArray<T> where T : struct
{
    private readonly IntPtr ptr;

    public readonly T this[int index] => Marshal.PtrToStructure<T>(ptr + index * Marshal.SizeOf<T>());
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct StructPtr<T> where T : struct
{
    private readonly IntPtr ptr;

    public readonly bool IsNull => ptr == 0;

    public readonly T Value => Marshal.PtrToStructure<T>(ptr);
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct Channel
{
    private readonly int channel;

    private Channel(int type, int idx)
    {
        channel = (idx << 1) | type;
    }

    public int Index => channel >> 1;

    public static Channel RX(int index) => new(0, index);
    public static Channel TX(int index) => new(1, index);
}

internal enum Direction
{
    RX = 0,
    TX = 1,
}

[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
internal struct Serial
{
    private const int SerialLength = 33;

    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SerialLength)]
    public string serial;
};

[StructLayout(LayoutKind.Sequential)]
internal struct Version
{
    public ushort major;
    public ushort minor;
    public ushort patch;
    public StrPtr describe;
};

[StructLayout(LayoutKind.Sequential)]
internal struct DevInfo
{
    private const int SerialLength = 33;
    private const int DescriptionLength = 33;

    public Backend backend;
    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SerialLength)]
    public string serial;
    public byte usb_bus;
    public byte usb_addr;
    public uint instance;

    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SerialLength)]
    public string manufacturer;
    [MarshalAs(UnmanagedType.ByValTStr, SizeConst = SerialLength)]
    public string product;
}

[StructLayout(LayoutKind.Sequential)]
internal struct BackendInfo
{
    public int handle_count;
    public IntPtr handle;
    public int lock_count;
    public IntPtr lock_;
};

[StructLayout(LayoutKind.Sequential)]
internal struct GainModes
{
    public StrPtr name;
    public GainMode mode;
};

[StructLayout(LayoutKind.Sequential)]
internal struct Range
{
    public long min;
    public long max;
    public long step;
    public float scale;
};

[StructLayout(LayoutKind.Sequential)]
internal struct RationalRate
{
    public ulong integer;
    public ulong num;
    public ulong den;
};

internal enum TriggerRole
{

    INVALID = -1,


    DISABLED,


    MASTER,


    SLAVE,
}

internal enum TriggerSignal
{
    INVALID = -1,
    J71_4,
    J51_1,
    MINI_EXP_1,

    USER_0 = 128,
    USER_1,
    USER_2,
    USER_3,
    USER_4,
    USER_5,
    USER_6,
    USER_7,
}

[StructLayout(LayoutKind.Sequential)]
internal struct Trigger
{
    public Channel channel;
    public TriggerRole role;
    public TriggerSignal signal;
    public ulong options;
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct LoopbackModes
{
    public StrPtr name;
    public Loopback mode;
};

internal enum ChannelLayout
{
    RX_X1 = 0,
    TX_X1 = 1,
    RX_X2 = 2,
    TX_X2 = 3,
}

[StructLayout(LayoutKind.Explicit)]
internal struct QuickTune
{
    [FieldOffset(0)]
    public byte freqsel;
    [FieldOffset(1)]
    public byte vcocap;
    [FieldOffset(2)]
    public ushort nint;
    [FieldOffset(4)]
    public uint nfrac;
    [FieldOffset(8)]
    public byte flags;
    [FieldOffset(9)]
    public byte xb_gpio;

    [FieldOffset(0)]
    public ushort nios_profile;
    [FieldOffset(2)]
    public byte rffe_profile;
    [FieldOffset(3)]
    public byte port;
    [FieldOffset(4)]
    public byte spdt;
}

internal enum Correction
{
    DCOFF_I,
    DCOFF_Q,
    PHASE,
    GAIN
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct Metadata
{
    public ulong timestamp;
    public uint flags;
    public uint status;
    public uint actual_count;
    public fixed byte reserved[32];
};

internal enum ImageType
{
    INVALID = -1,
    RAW,
    FIRMWARE,
    FPGA_40KLE,
    FPGA_115KLE,
    FPGA_A4,
    FPGA_A9,
    CALIBRATION,
    RX_DC_CAL,
    TX_DC_CAL,
    RX_IQ_CAL,
    TX_IQ_CAL,
    FPGA_A5,
    GAIN_CAL,
}

internal enum VctcxoTamerMode
{
    TAMER_INVALID = -1,
    TAMER_DISABLED = 0,
    TAMER_1_PPS = 1,
    TAMER_10_MHZ = 2
}

internal enum TuningMode
{
    INVALID = -1,
    HOST,
    FPGA,
}

internal enum Feature
{
    DEFAULT = 0,
    OVERSAMPLE
}

internal enum CalState
{
    UNINITIALIZED,
    LOADED,
    UNLOADED
}

[StructLayout(LayoutKind.Sequential)]
internal struct GainCalEntry
{
    public ulong freq;
    public double gain_corr;
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct GainCalTable
{
    public Version version;
    public Channel ch;
    public bool enabled;
    public uint n_entries;
    public ulong start_freq;
    public ulong stop_freq;
    public GainCalEntry* entries;
    public int gain_target;
    public nint file_path_len;
    public StrPtr file_path;
    public CalState state;
};

internal enum XB
{
    XB_NONE = 0,
    XB_100,
    XB_200,
    XB_300
}

internal delegate IntPtr StreamCallback(Device dev, Stream stream, StructPtr<Metadata> meta, IntPtr samples, nuint num_samples, IntPtr user_data);

internal unsafe static class NativeMethods
{
    const string DllName = "bladeRF";

    public static bool Loaded { get; }

    static NativeMethods()
    {
        NativeLibrary.SetDllImportResolver(typeof(NativeMethods).Assembly, Resolve);

        try
        {
            strerror(0);
            Loaded = true;
        }
        catch (DllNotFoundException)
        {
        }
    }

    private static nint Resolve(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
    {
        if (libraryName == DllName)
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
            {
                if (NativeLibrary.TryLoad($"lib{libraryName}.so.2", out var handle))
                    return handle;
                if (NativeLibrary.TryLoad($"lib{libraryName}.so", out var handle2))
                    return handle2;
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
            {
                if (NativeLibrary.TryLoad($"lib{libraryName}.dylib", out var handle))
                    return handle;
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                if (NativeLibrary.TryLoad($"{libraryName}.dll", out var handle))
                    return handle;
                if (NativeLibrary.TryLoad($"{libraryName}", out var handle2))
                    return handle2;
            }
        }

        return 0;
    }

    // libbladeRF
    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_open")] public static extern int open(out Device device, string device_identifier);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_close")] public static extern void close(Device device);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_open_with_devinfo")] public static extern int open_with_devinfo(out Device device, ref DevInfo devinfo);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_device_list")] public static extern int get_device_list(out StructArray<DevInfo> devices);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_free_device_list")] public static extern void free_device_list(StructArray<DevInfo> devices);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_init_devinfo")] public static extern void init_devinfo(ref DevInfo info);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_devinfo")] public static extern int get_devinfo(Device dev, out DevInfo info);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_backendinfo")] public static extern int get_backendinfo(Device dev, out BackendInfo info);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_devinfo_from_str")] public static extern int get_devinfo_from_str(string devstr, ref DevInfo info);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_devinfo_matches")] public static extern bool devinfo_matches(ref DevInfo a, ref DevInfo b);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_devstr_matches")] public static extern bool devstr_matches([MarshalAs(UnmanagedType.LPStr)] string dev_str, ref DevInfo info);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_backend_str")] public static extern StrPtr backend_str(Backend backend);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_usb_reset_on_open")] public static extern void set_usb_reset_on_open(bool enabled);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_serial_struct")] public static extern int get_serial_struct(Device dev, out Serial serial);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_fpga_size")] public static extern int get_fpga_size(Device dev, out FPGASize size);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_fpga_bytes")] public static extern int get_fpga_bytes(Device dev, out nuint size);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_flash_size")] public static extern int get_flash_size(Device dev, out uint size, out bool is_guess);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_fw_version")] public static extern int fw_version(Device dev, out Version version);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_is_fpga_configured")] public static extern int is_fpga_configured(Device dev);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_fpga_version")] public static extern int fpga_version(Device dev, out Version version);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_fpga_source")] public static extern int get_fpga_source(Device dev, out FPGASource source);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_device_speed")] public static extern DevSpeed device_speed(Device dev);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_board_name")] public static extern StrPtr get_board_name(Device dev);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_channel_count")] public static extern nuint get_channel_count(Device dev, Direction dir);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_gain")] public static extern int set_gain(Device dev, Channel ch, int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain")] public static extern int get_gain(Device dev, Channel ch, out int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_gain_mode")] public static extern int set_gain_mode(Device dev, Channel ch, GainMode mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_mode")] public static extern int get_gain_mode(Device dev, Channel ch, out GainMode mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_modes")] public static extern int get_gain_modes(Device dev, Channel ch, out StructArray<GainModes> modes);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_range")] public static extern int get_gain_range(Device dev, Channel ch, out StructPtr<Range> range);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_gain_stage")] public static extern int set_gain_stage(Device dev, Channel ch, string stage, int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_stage")] public static extern int get_gain_stage(Device dev, Channel ch, string stage, out int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_stage_range")] public static extern int get_gain_stage_range(Device dev, Channel ch, string stage, out StructPtr<Range> range);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_stages")] public static extern int get_gain_stages(Device dev, Channel ch, [Out] StrPtr[]? stages, nuint count);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_sample_rate")] public static extern int set_sample_rate(Device dev, Channel ch, uint rate, out uint actual);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rational_sample_rate")] public static extern int set_rational_sample_rate(Device dev, Channel ch, ref RationalRate rate, out RationalRate actual);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_sample_rate")] public static extern int get_sample_rate(Device dev, Channel ch, out uint rate);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_sample_rate_range")] public static extern int get_sample_rate_range(Device dev, Channel ch, out StructPtr<Range> range);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rational_sample_rate")] public static extern int get_rational_sample_rate(Device dev, Channel ch, out RationalRate rate);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_bandwidth")] public static extern int set_bandwidth(Device dev, Channel ch, uint bandwidth, out uint actual);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_bandwidth")] public static extern int get_bandwidth(Device dev, Channel ch, out uint bandwidth);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_bandwidth_range")] public static extern int get_bandwidth_range(Device dev, Channel ch, out StructPtr<Range> range);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_select_band")] public static extern int select_band(Device dev, Channel ch, ulong frequency);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_frequency")] public static extern int set_frequency(Device dev, Channel ch, ulong frequency);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_frequency")] public static extern int get_frequency(Device dev, Channel ch, out ulong frequency);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_frequency_range")] public static extern int get_frequency_range(Device dev, Channel ch, out StructPtr<Range> range);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_loopback_modes")] public static extern int get_loopback_modes(Device dev, out StructArray<LoopbackModes> modes);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_is_loopback_mode_supported")] public static extern bool is_loopback_mode_supported(Device dev, Loopback mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_loopback")] public static extern int set_loopback(Device dev, Loopback lb);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_loopback")] public static extern int get_loopback(Device dev, out Loopback lb);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_trigger_init")] public static extern int trigger_init(Device dev, Channel ch, TriggerSignal signal, out Trigger trigger);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_trigger_arm")] public static extern int trigger_arm(Device dev, ref Trigger trigger, bool arm, ulong resv1, ulong resv2);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_trigger_fire")] public static extern int trigger_fire(Device dev, ref Trigger trigger);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_trigger_state")] public static extern int trigger_state(Device dev, ref Trigger trigger, out bool is_armed, out bool has_fired, out bool fire_requested, out ulong resv1, out ulong resv2);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rx_mux")] public static extern int set_rx_mux(Device dev, RxMux mux);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rx_mux")] public static extern int get_rx_mux(Device dev, out RxMux mode);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_schedule_retune")] public static extern int schedule_retune(Device dev, Channel ch, ulong timestamp, ulong frequency, ref QuickTune quick_tune);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_cancel_scheduled_retunes")] public static extern int cancel_scheduled_retunes(Device dev, Channel ch);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_quick_tune")] public static extern int get_quick_tune(Device dev, Channel ch, out QuickTune quick_tune);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_print_quick_tune")] public static extern int print_quick_tune(Device dev, ref QuickTune qt);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_correction")] public static extern int set_correction(Device dev, Channel ch, Correction corr, short value);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_correction")] public static extern int get_correction(Device dev, Channel ch, Correction corr, out short value);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_interleave_stream_buffer")] public static extern int interleave_stream_buffer(ChannelLayout layout, Format format, uint buffer_size, IntPtr samples);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_deinterleave_stream_buffer")] public static extern int deinterleave_stream_buffer(ChannelLayout layout, Format format, uint buffer_size, IntPtr samples);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_enable_module")] public static extern int enable_module(Device dev, Channel ch, bool enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_timestamp")] public static extern int get_timestamp(Device dev, Direction dir, out ulong timestamp);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_sync_config")] public static extern int sync_config(Device dev, ChannelLayout layout, Format format, uint num_buffers, uint buffer_size, uint num_transfers, uint stream_timeout);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_sync_tx")] public static extern int sync_tx(Device dev, IntPtr samples, uint num_samples, ref Metadata metadata, uint timeout_ms);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_sync_rx")] public static extern int sync_rx(Device dev, IntPtr samples, uint num_samples, ref Metadata metadata, uint timeout_ms);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_init_stream")] public static extern int init_stream(out Stream stream, Device dev, StreamCallback callback, out StructArray<IntPtr> buffers, nuint num_buffers, Format format, nuint samples_per_buffer, nuint num_transfers, IntPtr user_data);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_stream")] public static extern int stream(Stream stream, ChannelLayout layout);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_submit_stream_buffer")] public static extern int submit_stream_buffer(Stream stream, IntPtr buffer, uint timeout_ms);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_submit_stream_buffer_nb")] public static extern int submit_stream_buffer_nb(Stream stream, IntPtr buffer);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_deinit_stream")] public static extern void deinit_stream(Stream stream);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_stream_timeout")] public static extern int set_stream_timeout(Device dev, Direction dir, uint timeout);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_stream_timeout")] public static extern int get_stream_timeout(Device dev, Direction dir, out uint timeout);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_flash_firmware")] public static extern int flash_firmware(Device dev, string firmware);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_load_fpga")] public static extern int load_fpga(Device dev, string fpga);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_flash_fpga")] public static extern int flash_fpga(Device dev, string fpga_image);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_erase_stored_fpga")] public static extern int erase_stored_fpga(Device dev);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_device_reset")] public static extern int device_reset(Device dev);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_fw_log")] public static extern int get_fw_log(Device dev, string filename);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_jump_to_bootloader")] public static extern int jump_to_bootloader(Device dev);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_bootloader_list")] public static extern int get_bootloader_list(out StructArray<DevInfo> list);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_load_fw_from_bootloader")] public static extern int load_fw_from_bootloader(string device_identifier, Backend backend, byte bus, byte addr, string file);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_alloc_image")] public static extern Image alloc_image(Device dev, ImageType type, uint address, uint length);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_alloc_cal_image")] public static extern Image alloc_cal_image(Device dev, FPGASize fpga_size, ushort vctcxo_trim);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_free_image")] public static extern void free_image(Image image);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_image_print_metadata")] public static extern int image_print_metadata(Image image);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_image_write")] public static extern int image_write(Device dev, Image image, string file);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_image_read")] public static extern int image_read(Image image, string file);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_vctcxo_tamer_mode")] public static extern int set_vctcxo_tamer_mode(Device dev, VctcxoTamerMode mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_vctcxo_tamer_mode")] public static extern int get_vctcxo_tamer_mode(Device dev, out VctcxoTamerMode mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_vctcxo_trim")] public static extern int get_vctcxo_trim(Device dev, out ushort trim);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_trim_dac_write")] public static extern int trim_dac_write(Device dev, ushort val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_trim_dac_read")] public static extern int trim_dac_read(Device dev, out ushort val);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_tuning_mode")] public static extern int set_tuning_mode(Device dev, TuningMode mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_tuning_mode")] public static extern int get_tuning_mode(Device dev, out TuningMode mode);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_read_trigger")] public static extern int read_trigger(Device dev, Channel ch, TriggerSignal signal, out byte val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_write_trigger")] public static extern int write_trigger(Device dev, Channel ch, TriggerSignal signal, byte val);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_wishbone_master_read")] public static extern int wishbone_master_read(Device dev, uint addr, out uint data);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_wishbone_master_write")] public static extern int wishbone_master_write(Device dev, uint addr, uint val);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_config_gpio_read")] public static extern int config_gpio_read(Device dev, out uint val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_config_gpio_write")] public static extern int config_gpio_write(Device dev, uint val);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_erase_flash_bytes")] public static extern int erase_flash_bytes(Device dev, uint address, uint length);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_read_flash_bytes")] public static extern int read_flash_bytes(Device dev, byte[] buf, uint address, uint bytes);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_write_flash_bytes")] public static extern int write_flash_bytes(Device dev, byte[] buf, uint address, uint length);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_lock_otp")] public static extern int lock_otp(Device dev);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_read_otp")] public static extern int read_otp(Device dev, byte[] buf);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_write_otp")] public static extern int write_otp(Device dev, byte[] buf);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rf_port")] public static extern int set_rf_port(Device dev, Channel ch, [MarshalAs(UnmanagedType.LPStr)] string port);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rf_port")] public static extern int get_rf_port(Device dev, Channel ch, out StrPtr port);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rf_ports")] public static extern int get_rf_ports(Device dev, Channel ch, [Out] StrPtr[]? ports, nuint count);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_enable_feature")] public static extern int enable_feature(Device dev, Feature feature, bool enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_feature")] public static extern int get_feature(Device dev, out Feature feature);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_load_gain_calibration")] public static extern int load_gain_calibration(Device dev, Channel ch, string cal_file_loc);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_print_gain_calibration")] public static extern int print_gain_calibration(Device dev, Channel ch, bool with_entries);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_enable_gain_calibration")] public static extern int enable_gain_calibration(Device dev, Channel ch, bool en);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_calibration")] public static extern int get_gain_calibration(Device dev, Channel ch, out StructPtr<GainCalTable> tbl);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_gain_target")] public static extern int get_gain_target(Device dev, Channel ch, out int gain_target);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_attach")] public static extern int expansion_attach(Device dev, XB xb);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_get_attached")] public static extern int expansion_get_attached(Device dev, out XB xb);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_log_set_verbosity")] public static extern void log_set_verbosity(LogLevel level);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_version")] public static extern void version(out Version version);



    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_strerror")] public static extern StrPtr strerror(int error);

    public static int CheckError(int error)
    {
        if (error < 0)
            throw new Exception(strerror(error));
        return error;
    }

    /*
    // bladeRF1
    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_txvga2")] public static extern int set_txvga2(BladeRFHandle dev, int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_txvga2")] public static extern int get_txvga2(BladeRFHandle dev, out int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_txvga1")] public static extern int set_txvga1(BladeRFHandle dev, int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_txvga1")] public static extern int get_txvga1(BladeRFHandle dev, out int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_lna_gain")] public static extern int set_lna_gain(BladeRFHandle dev, bladerf_lna_gain gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_lna_gain")] public static extern int get_lna_gain(BladeRFHandle dev, bladerf_lna_gain* gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rxvga1")] public static extern int set_rxvga1(BladeRFHandle dev, int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rxvga1")] public static extern int get_rxvga1(BladeRFHandle dev, out int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rxvga2")] public static extern int set_rxvga2(BladeRFHandle dev, int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rxvga2")] public static extern int get_rxvga2(BladeRFHandle dev, out int gain);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_sampling")] public static extern int set_sampling(BladeRFHandle dev, bladerf_sampling sampling);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_sampling")] public static extern int get_sampling(BladeRFHandle dev, bladerf_sampling* sampling);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_lpf_mode")] public static extern int set_lpf_mode(BladeRFHandle dev, bladerf_channel ch, bladerf_lpf_mode mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_lpf_mode")] public static extern int get_lpf_mode(BladeRFHandle dev, bladerf_channel ch, bladerf_lpf_mode* mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_smb_mode")] public static extern int set_smb_mode(BladeRFHandle dev, bladerf_smb_mode mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_smb_mode")] public static extern int get_smb_mode(BladeRFHandle dev, bladerf_smb_mode* mode);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rational_smb_frequency")] public static extern int set_rational_smb_frequency(BladeRFHandle dev, bladerf_rational_rate* rate, bladerf_rational_rate* actual);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_smb_frequency")] public static extern int set_smb_frequency(BladeRFHandle dev, uint rate, uint* actual);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rational_smb_frequency")] public static extern int get_rational_smb_frequency(BladeRFHandle dev, bladerf_rational_rate* rate);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_smb_frequency")] public static extern int get_smb_frequency(BladeRFHandle dev, out uint rate);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_gpio_read")] public static extern int expansion_gpio_read(BladeRFHandle dev, uint* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_gpio_write")] public static extern int expansion_gpio_write(BladeRFHandle dev, uint val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_gpio_masked_write")] public static extern int expansion_gpio_masked_write(BladeRFHandle dev, uint mask, uint value);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_gpio_dir_read")] public static extern int expansion_gpio_dir_read(BladeRFHandle dev, uint* outputs);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_gpio_dir_write")] public static extern int expansion_gpio_dir_write(BladeRFHandle dev, uint outputs);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_expansion_gpio_dir_masked_write")] public static extern int expansion_gpio_dir_masked_write(BladeRFHandle dev, uint mask, uint outputs);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb200_set_filterbank")] public static extern int xb200_set_filterbank(BladeRFHandle dev, bladerf_channel ch, bladerf_xb200_filter filter);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb200_get_filterbank")] public static extern int xb200_get_filterbank(BladeRFHandle dev, bladerf_channel ch, bladerf_xb200_filter* filter);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb200_set_path")] public static extern int xb200_set_path(BladeRFHandle dev, bladerf_channel ch, bladerf_xb200_path path);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb200_get_path")] public static extern int xb200_get_path(BladeRFHandle dev, bladerf_channel ch, bladerf_xb200_path* path);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb300_set_trx")] public static extern int xb300_set_trx(BladeRFHandle dev, bladerf_xb300_trx trx);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb300_get_trx")] public static extern int xb300_get_trx(BladeRFHandle dev, bladerf_xb300_trx* trx);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb300_set_amplifier_enable")] public static extern int xb300_set_amplifier_enable(BladeRFHandle dev, bladerf_xb300_amplifier amp, bool enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb300_get_amplifier_enable")] public static extern int xb300_get_amplifier_enable(BladeRFHandle dev, bladerf_xb300_amplifier amp, bool* enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb300_get_output_power")] public static extern int xb300_get_output_power(BladeRFHandle dev, float* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_calibrate_dc")] public static extern int calibrate_dc(BladeRFHandle dev, bladerf_cal_module module);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_dac_write")] public static extern int dac_write(BladeRFHandle dev, ushort val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_dac_read")] public static extern int dac_read(BladeRFHandle dev, ushort* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_si5338_read")] public static extern int si5338_read(BladeRFHandle dev, byte address, byte* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_si5338_write")] public static extern int si5338_write(BladeRFHandle dev, byte address, byte val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_lms_read")] public static extern int lms_read(BladeRFHandle dev, byte address, byte* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_lms_write")] public static extern int lms_write(BladeRFHandle dev, byte address, byte val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_lms_set_dc_cals")] public static extern int lms_set_dc_cals(BladeRFHandle dev, const bladerf_lms_dc_cals* dc_cals);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_lms_get_dc_cals")] public static extern int lms_get_dc_cals(BladeRFHandle dev, bladerf_lms_dc_cals* dc_cals);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_xb_spi_write")] public static extern int xb_spi_write(BladeRFHandle dev, uint val);

    // bladeRF2

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_bias_tee")] public static extern int get_bias_tee(BladeRFHandle dev, bladerf_channel ch, bool* enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_bias_tee")] public static extern int set_bias_tee(BladeRFHandle dev, bladerf_channel ch, bool enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rfic_register")] public static extern int get_rfic_register(BladeRFHandle dev, ushort address, byte* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rfic_register")] public static extern int set_rfic_register(BladeRFHandle dev, ushort address, byte val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rfic_temperature")] public static extern int get_rfic_temperature(BladeRFHandle dev, float* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rfic_rssi")] public static extern int get_rfic_rssi(BladeRFHandle dev, bladerf_channel ch, int* pre_rssi, int* sym_rssi);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rfic_ctrl_out")] public static extern int get_rfic_ctrl_out(BladeRFHandle dev, byte* ctrl_out);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rfic_rx_fir")] public static extern int get_rfic_rx_fir(BladeRFHandle dev, bladerf_rfic_rxfir* rxfir);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rfic_rx_fir")] public static extern int set_rfic_rx_fir(BladeRFHandle dev, bladerf_rfic_rxfir rxfir);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rfic_tx_fir")] public static extern int get_rfic_tx_fir(BladeRFHandle dev, bladerf_rfic_txfir* txfir);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_rfic_tx_fir")] public static extern int set_rfic_tx_fir(BladeRFHandle dev, bladerf_rfic_txfir txfir);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_pll_lock_state")] public static extern int get_pll_lock_state(BladeRFHandle dev, bool* locked);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_pll_enable")] public static extern int get_pll_enable(BladeRFHandle dev, bool* enabled);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_pll_enable")] public static extern int set_pll_enable(BladeRFHandle dev, bool enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_pll_refclk_range")] public static extern int get_pll_refclk_range(BladeRFHandle dev, out bladerf_range* range);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_pll_refclk")] public static extern int get_pll_refclk(BladeRFHandle dev, ulong* frequency);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_pll_refclk")] public static extern int set_pll_refclk(BladeRFHandle dev, ulong frequency);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_pll_register")] public static extern int get_pll_register(BladeRFHandle dev, byte address, uint* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_pll_register")] public static extern int set_pll_register(BladeRFHandle dev, byte address, uint val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_power_source")] public static extern int get_power_source(BladeRFHandle dev, bladerf_power_sources* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_clock_select")] public static extern int get_clock_select(BladeRFHandle dev, bladerf_clock_select* sel);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_clock_select")] public static extern int set_clock_select(BladeRFHandle dev, bladerf_clock_select sel);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_clock_output")] public static extern int get_clock_output(BladeRFHandle dev, bool* state);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_set_clock_output")] public static extern int set_clock_output(BladeRFHandle dev, bool enable);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_pmic_register")] public static extern int get_pmic_register(BladeRFHandle dev, bladerf_pmic_register reg, void* val);

    [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "bladerf_get_rf_switch_config")] public static extern int get_rf_switch_config(BladeRFHandle dev, bladerf_rf_switch_config* config);*/
}
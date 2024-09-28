using System.Collections.Generic;
using System.Linq;
using NordicSpaceLink.BladeRF.Imports;

namespace NordicSpaceLink.BladeRF;

public enum Direction
{
    RX,
    TX
}

public abstract class Channel
{
    private readonly Imports.Device dev;
    private readonly Imports.Channel ch;

    private bool enabled = false;
    private uint requestedSampleRate;
    private uint actualSampleRate;
    private uint actualBandwidth;
    private uint requestedBandwidth;
    private ulong frequency;

    public Direction Direction { get; }
    public int Index { get; }

    public IReadOnlyList<string> RFPorts { get; }
    public IReadOnlyList<string> GainStages { get; }

    public bool Enable
    {
        get => enabled;
        set
        {
            NativeMethods.CheckError(NativeMethods.enable_module(dev, ch, value));
            enabled = value;
        }
    }

    public int Gain
    {
        get
        {
            NativeMethods.CheckError(NativeMethods.get_gain(dev, ch, out var gain));
            return gain;
        }
        set => NativeMethods.CheckError(NativeMethods.set_gain(dev, ch, value));
    }

    public Range GainRange
    {
        get
        {
            NativeMethods.CheckError(NativeMethods.get_gain_range(dev, ch, out var range));
            var val = range.Value;
            return new(val.min * val.scale, val.max * val.scale, val.step * val.scale);
        }
    }

    public string RFPort
    {
        get
        {
            NativeMethods.CheckError(NativeMethods.get_rf_port(dev, ch, out var port));
            return port;
        }
        set => NativeMethods.CheckError(NativeMethods.set_rf_port(dev, ch, value));
    }

    public uint SampleRate
    {
        get => requestedSampleRate;
        set
        {
            NativeMethods.CheckError(NativeMethods.set_sample_rate(dev, ch, value, out actualSampleRate));
            requestedSampleRate = value;
        }
    }

    public uint ActualSampleRate => actualSampleRate;

    public Range SampleRateRange
    {
        get
        {
            NativeMethods.CheckError(NativeMethods.get_sample_rate_range(dev, ch, out var range));
            var val = range.Value;
            return new(val.min * val.scale, val.max * val.scale, val.step * val.scale);
        }
    }

    public uint Bandwidth
    {
        get => requestedBandwidth;
        set
        {
            NativeMethods.CheckError(NativeMethods.set_bandwidth(dev, ch, value, out actualBandwidth));
            requestedBandwidth = value;
        }
    }

    public uint ActualBandwidth => actualBandwidth;

    public Range BandwidthRange
    {
        get
        {
            NativeMethods.CheckError(NativeMethods.get_bandwidth_range(dev, ch, out var range));
            var val = range.Value;
            return new(val.min * val.scale, val.max * val.scale, val.step * val.scale);
        }
    }

    public ulong Frequency
    {
        get => frequency;
        set
        {
            NativeMethods.CheckError(NativeMethods.set_frequency(dev, ch, value));
            frequency = value;
        }
    }

    public Range FrequencyRange
    {
        get
        {
            NativeMethods.CheckError(NativeMethods.get_frequency_range(dev, ch, out var range));
            var val = range.Value;
            return new(val.min * val.scale, val.max * val.scale, val.step * val.scale);
        }
    }

    internal Channel(Imports.Device device, Imports.Channel channel, Direction direction)
    {
        dev = device;
        ch = channel;

        Direction = direction;
        Index = channel.Index;

        NativeMethods.CheckError(NativeMethods.get_frequency(dev, ch, out frequency));

        NativeMethods.CheckError(NativeMethods.get_sample_rate(dev, ch, out requestedSampleRate));
        actualSampleRate = requestedSampleRate;

        NativeMethods.CheckError(NativeMethods.get_bandwidth(dev, ch, out requestedBandwidth));
        actualBandwidth = requestedBandwidth;

        {
            var ret = NativeMethods.CheckError(NativeMethods.get_rf_ports(dev, ch, null, 0));
            if (ret > 0)
            {
                var buf = new StrPtr[ret];
                ret = NativeMethods.CheckError(NativeMethods.get_rf_ports(dev, ch, buf, (nuint)ret));

                RFPorts = buf.Select(x => (string)x).ToList();
            }
            else
                RFPorts = [];
        }

        {
            var ret = NativeMethods.CheckError(NativeMethods.get_gain_stages(dev, ch, null, 0));
            if (ret > 0)
            {
                var buf = new StrPtr[ret];
                ret = NativeMethods.CheckError(NativeMethods.get_gain_stages(dev, ch, buf, (nuint)ret));

                GainStages = buf.Select(x => (string)x).ToList();
            }
            else
                GainStages = [];
        }
    }
}

public sealed class TXChannel : Channel
{
    internal TXChannel(Imports.Device device, Imports.Channel channel) : base(device, channel, Direction.TX)
    {
    }
}

public sealed class RXChannel : Channel
{
    private readonly Imports.Device dev;
    private readonly Imports.Channel ch;

    public IReadOnlyList<GainMode> GainModes { get; }

    public GainMode GainMode
    {
        get
        {
            NativeMethods.CheckError(NativeMethods.get_gain_mode(dev, ch, out var mode));
            return mode;
        }
        set => NativeMethods.CheckError(NativeMethods.set_gain_mode(dev, ch, value));
    }

    internal RXChannel(Imports.Device device, Imports.Channel channel) : base(device, channel, Direction.RX)
    {
        dev = device;
        ch = channel;

        {
            var ret = NativeMethods.CheckError(NativeMethods.get_gain_modes(dev, ch, out var modes));
            if (ret > 0)
                GainModes = Enumerable.Range(0, ret).Select(idx => modes[idx].mode).ToList();
            else
                GainModes = [];
        }
    }
}
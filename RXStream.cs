using System;
using System.Collections.Concurrent;
using System.Threading;
using NordicSpaceLink.BladeRF.Imports;

namespace NordicSpaceLink.BladeRF;

public class RXStream : IDisposable
{
    const IntPtr BufferShutdown = 0;
    const IntPtr BufferNoData = -1;

    private readonly Stream stream;
    private readonly ChannelLayout layout;
    private readonly int sampleSize;
    private readonly int headerSize;

    private bool doQuit = false;

    private readonly StructArray<IntPtr> buffers;
    private bool disposedValue;

    private readonly ConcurrentQueue<IntPtr> fullBuffers = new();
    private readonly ConcurrentQueue<IntPtr> rxQueue = new();

    private readonly AutoResetEvent fullEvent;

    private readonly nint zeroBuffer;

    public int BufferSize { get; }
    public int Overflows { get; set; }

    internal RXStream(Imports.Device dev, int channelCount, int numBuffers, Format format, int samplesPerBuffer, int numTransfers)
    {
        layout = channelCount switch
        {
            1 => ChannelLayout.RX_X1,
            2 => ChannelLayout.RX_X2,
            _ => throw new ArgumentException("Invalid channel count combination")
        };

        (sampleSize, headerSize) = format switch
        {
            Format.SC16_Q11 => (4, 0),
            Format.SC16_Q11_META => (4, 16),
            Format.PACKET_META => throw new NotSupportedException("Packet format"),
            Format.SC8_Q7 => (2, 0),
            Format.SC8_Q7_META => (2, 16),
            _ => throw new ArgumentException("Unknown format", nameof(format))
        };

        BufferSize = headerSize + sampleSize * samplesPerBuffer * channelCount;

        fullEvent = new(true);
        numBuffers++;

        NativeMethods.CheckError(NativeMethods.init_stream(out stream, dev, Callback, out buffers, (nuint)numBuffers, format, (nuint)samplesPerBuffer, (nuint)numTransfers, 0));

        zeroBuffer = buffers[0];

        for (int i = 1; i < numBuffers; i++)
            rxQueue.Enqueue(buffers[i]);
    }

    public void RunBlocking()
    {
        NativeMethods.CheckError(NativeMethods.stream(stream, layout));
    }

    public unsafe void PopSamples(Span<byte> data, CancellationToken cancellationToken)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(data.Length, BufferSize);

        while (!cancellationToken.IsCancellationRequested)
        {
            if (fullBuffers.TryDequeue(out var buf))
            {
                var src = new ReadOnlySpan<byte>(buf.ToPointer(), BufferSize);
                src.CopyTo(data);

                rxQueue.Enqueue(buf);
                return;
            }
            fullEvent.WaitOne();
        }
    }

    public void Finish()
    {
        doQuit = true;
    }

    private IntPtr Callback(Imports.Device dev, Stream stream, StructPtr<Metadata> meta, IntPtr samples, nuint num_samples, IntPtr user_data)
    {
        if ((samples != 0) && (zeroBuffer != samples))
        {
            fullBuffers.Enqueue(samples);
            fullEvent.Set();
        }

        if (doQuit)
            return BufferShutdown;

        if (rxQueue.TryDequeue(out var buf))
            return buf;


        Overflows++;
        return zeroBuffer;
    }

    protected virtual void Dispose(bool disposing)
    {
        if (!disposedValue)
        {
            if (disposing)
            {
                fullEvent.Dispose();
            }

            NativeMethods.deinit_stream(stream);
            disposedValue = true;
        }
    }

    ~RXStream()
    {
        // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
        Dispose(disposing: false);
    }

    public void Dispose()
    {
        // Do not change this code. Put cleanup code in 'Dispose(bool disposing)' method
        Dispose(disposing: true);
        GC.SuppressFinalize(this);
    }
}
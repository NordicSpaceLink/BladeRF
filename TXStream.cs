using System;
using System.Collections.Concurrent;
using System.Threading;
using NordicSpaceLink.BladeRF.Imports;

namespace NordicSpaceLink.BladeRF;

public class TXStream : IDisposable
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

    private readonly ConcurrentQueue<IntPtr> freeBuffers = new();
    private readonly ConcurrentQueue<IntPtr> txQueue = new();

    private readonly AutoResetEvent freeEvent;

    private readonly nint zeroBuffer;

    public int BufferSize { get; }
    public int Underflows { get; set; }

    internal TXStream(Imports.Device dev, int channelCount, int numBuffers, Format format, int samplesPerBuffer, int numTransfers)
    {
        layout = channelCount switch
        {
            1 => ChannelLayout.TX_X1,
            2 => ChannelLayout.TX_X2,
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

        freeEvent = new(true);
        numBuffers++;

        NativeMethods.CheckError(NativeMethods.init_stream(out stream, dev, Callback, out buffers, (nuint)numBuffers, format, (nuint)samplesPerBuffer, (nuint)numTransfers, 0));

        zeroBuffer = buffers[0];
        Clear(zeroBuffer);

        for (int i = 1; i < numBuffers; i++)
            freeBuffers.Enqueue(buffers[i]);
    }

    private unsafe void Clear(IntPtr zeroBuffer)
    {
        new Span<byte>(zeroBuffer.ToPointer(), BufferSize).Clear();
    }

    public void RunBlocking()
    {
        NativeMethods.CheckError(NativeMethods.stream(stream, layout));
    }

    public unsafe bool TryPushSamples(ReadOnlySpan<byte> data)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(data.Length, BufferSize);

        if (freeBuffers.TryDequeue(out var buf))
        {
            var dest = new Span<byte>(buf.ToPointer(), BufferSize);
            data.CopyTo(dest);

            txQueue.Enqueue(buf);
            return true;
        }

        return false;
    }

    public unsafe void PushSamples(ReadOnlySpan<byte> data, CancellationToken cancellationToken)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(data.Length, BufferSize);

        while (!cancellationToken.IsCancellationRequested)
        {
            if (freeBuffers.TryDequeue(out var buf))
            {
                var dest = new Span<byte>(buf.ToPointer(), BufferSize);
                data.CopyTo(dest);

                txQueue.Enqueue(buf);
                return;
            }
            freeEvent.WaitOne();
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
            freeBuffers.Enqueue(samples);
            freeEvent.Set();
        }

        if (doQuit)
            return new(BufferShutdown);

        if (txQueue.TryDequeue(out var buf))
            return buf;

        Underflows++;
        
        return zeroBuffer;
    }

    protected virtual void Dispose(bool disposing)
    {
        if (!disposedValue)
        {
            if (disposing)
            {
                freeEvent.Dispose();
            }

            NativeMethods.deinit_stream(stream);
            disposedValue = true;
        }
    }

    ~TXStream()
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
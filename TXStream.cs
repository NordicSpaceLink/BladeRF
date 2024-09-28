using System;
using System.Collections.Concurrent;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using NordicSpaceLink.BladeRF.Imports;

namespace NordicSpaceLink.BladeRF;

public class TXStream : IDisposable
{
    public readonly ref struct TXBuffer<T>
    {
        private readonly TXStream stream;
        private readonly IntPtr buffer;
        private readonly int bufferSize;

        internal TXBuffer(TXStream stream, nint buffer, int bufferSize)
        {
            this.stream = stream;
            this.buffer = buffer;
            this.bufferSize = bufferSize;
        }

        public unsafe Span<T> Buffer => new(buffer.ToPointer(), bufferSize / Unsafe.SizeOf<T>());

        public void Dispose()
        {
            stream.FreeBuffer(buffer);
        }
    }

    const IntPtr BufferShutdown = 0;
    const IntPtr BufferNoData = -1;

    private readonly Stream stream;
    private readonly ChannelLayout layout;
    private readonly int sampleSize;
    private readonly int headerSize;

    private bool doQuit = false;

    private bool disposedValue;
    private readonly StreamCallback myDelegate;
    private readonly IntPtr callbackMethod;
    private readonly ConcurrentQueue<IntPtr> freeBuffers = new();
    private readonly ConcurrentQueue<IntPtr> txQueue = new();

    private readonly AutoResetEvent freeEvent;

    public int BufferSize { get; }
    public int Underflows { get; private set; }

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

        myDelegate = new StreamCallback(Callback);
        callbackMethod = Marshal.GetFunctionPointerForDelegate(myDelegate);
        NativeMethods.CheckError(NativeMethods.init_stream(out stream, dev, callbackMethod, out var buffers, (nuint)numBuffers, format, (nuint)samplesPerBuffer, (nuint)numTransfers, 0));

        for (int i = 0; i < numBuffers; i++)
            freeBuffers.Enqueue(buffers[i]);
    }

    public void RunBlocking()
    {
        NativeMethods.CheckError(NativeMethods.stream(stream, layout));
    }

    public bool TryPrepareTXBuffer<T>([MaybeNullWhen(false)] out TXBuffer<T> buffer)
    {
        if (freeBuffers.TryDequeue(out var buf))
        {
            buffer = new(this, buf, BufferSize);
            return true;
        }

        buffer = default;
        return false;
    }

    public bool PrepareTXBuffer<T>([MaybeNullWhen(false)] out TXBuffer<T> buffer, CancellationToken cancellationToken)
    {
        while (!doQuit && !cancellationToken.IsCancellationRequested)
        {
            if (TryPrepareTXBuffer(out buffer))
                return true;
            freeEvent.WaitOne();
        }

        buffer = default;
        return false;
    }

    private void FreeBuffer(IntPtr buffer)
    {
        txQueue.Enqueue(buffer);
    }

    public bool TryPushSamples<T>(ReadOnlySpan<T> data)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(data.Length, BufferSize);

        if (TryPrepareTXBuffer<T>(out var buffer))
            using (buffer)
            {
                data.CopyTo(buffer.Buffer);
                return true;
            }

        return false;
    }

    public bool PushSamples<T>(ReadOnlySpan<T> data, CancellationToken cancellationToken)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(data.Length, BufferSize);

        if (PrepareTXBuffer<T>(out var buffer, cancellationToken))
            using (buffer)
            {
                data.CopyTo(buffer.Buffer);
                return true;
            }

        return false;
    }

    public void Finish()
    {
        doQuit = true;
        freeEvent.Set();
    }

    private IntPtr Callback(Imports.Device dev, Stream stream, StructPtr<Metadata> meta, IntPtr samples, nuint num_samples, IntPtr user_data)
    {
        if (samples != 0)
        {
            freeBuffers.Enqueue(samples);
            freeEvent.Set();
        }

        if (doQuit)
            return new(BufferShutdown);

        if (txQueue.TryDequeue(out var buf))
            return buf;

        Underflows++;

        if (freeBuffers.TryDequeue(out var buf2))
            return buf2;

        return BufferShutdown;
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
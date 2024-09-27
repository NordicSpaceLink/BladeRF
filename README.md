# NordicSpaceLink.BladeRF

This library provides C# bindings for `libbladeRF`.

It exposes the simplest way of working with the asynchronous API.

## Example

```c#
if (!Library.IsLoaded)
{
    Console.WriteLine("Library not loaded");
    return;
}

// The default log level is quite noisy
Library.LogLevel = LogLevel.Warning;

Console.WriteLine(Library.Version);

using var dev = new Device();

if (!dev.IsFPGAConfigured)
    throw new Exception("FPGA not configured");

Console.WriteLine("Speed: {0}", dev.Info.DeviceSpeed);
Console.WriteLine("FPGA Size: {0}", dev.Info.FPGASize);
Console.WriteLine("FPGA Source: {0}", dev.Info.FPGASource);
Console.WriteLine("FPGA Version: {0}", dev.Info.FPGAVersion);
Console.WriteLine("Firmware Version: {0}", dev.Info.FirmwareVersion);

var rx = dev.RXChannels[0];
var tx = dev.TXChannels[0];

tx.Frequency = (long)2.4e9;
tx.SampleRate = (int)61.44e6;
tx.Gain = -21;
tx.Enable = true;

using (var stream = dev.CreateTXStream(1, 256, Format.SC8_Q7, 65536, 32))
{
    Console.WriteLine("Buffer size: {0}", stream.BufferSize);

    var signal = new ManualResetEvent(false);

    void DoRun()
    {
        try
        {
            signal.Set();
            stream.RunBlocking();
            Console.WriteLine("Done");
        }
        catch (Exception ex)
        {
            Console.WriteLine("EX: {0}", ex.Message);
        }
    }

    var data = new byte[stream.BufferSize];
    Random.Shared.NextBytes(data);

    long pushed = 0;
    while (stream.TryPushSamples(data))
        pushed += data.Length;

    var thread = new Thread(DoRun);
    thread.Start();
    signal.WaitOne();

    Console.WriteLine("Started");

    var sw = Stopwatch.StartNew();
    long of = 0;

    while (sw.Elapsed.TotalSeconds < 10)
    {
        stream.PushSamples(data, CancellationToken.None);
        pushed += data.Length;

        if (stream.Underflows != of)
        {
            of = stream.Underflows;
            Console.WriteLine("UF {0}", of);
        }
    }

    stream.Finish();
    thread.Join();
    sw.Stop();

    Console.WriteLine("Finished Msps: {0}", (pushed - data.Length * (256 - 32)) / sw.Elapsed.TotalSeconds / 2 / 1e6);
    Console.WriteLine("Underflows: {0}", stream.Underflows);
}
tx.Enable = false;

rx.Frequency = (long)2.4e9;
rx.SampleRate = (int)61.44e6;
rx.Enable = true;

using (var stream = dev.CreateRXStream(1, 256, Format.SC8_Q7, 65536, 32))
{
    Console.WriteLine("Buffer size: {0}", stream.BufferSize);

    var signal = new ManualResetEvent(false);

    void DoRun()
    {
        try
        {
            signal.Set();
            stream.RunBlocking();
            Console.WriteLine("Done");
        }
        catch (Exception ex)
        {
            Console.WriteLine("EX: {0}", ex.Message);
        }
    }

    var thread = new Thread(DoRun);
    thread.Start();

    signal.WaitOne();
    Console.WriteLine("Started");

    var data = new byte[stream.BufferSize];

    for (int i = 0; i < 64; i++)
        stream.PopSamples(data, CancellationToken.None);

    var sw = Stopwatch.StartNew();
    long popped = 0;

    while (sw.Elapsed.TotalSeconds < 10)
    {
        stream.PopSamples(data, CancellationToken.None);
        popped += data.Length;
    }
    sw.Stop();

    stream.Finish();
    thread.Join();

    Console.WriteLine("Finished Msps: {0}", popped / sw.Elapsed.TotalSeconds / 2 / 1e6);
    Console.WriteLine("Overflows: {0}", stream.Overflows);
}
```
using System;
using System.Collections.Generic;
using System.Linq;
using NordicSpaceLink.BladeRF.Imports;

namespace NordicSpaceLink.BladeRF
{
    public sealed class Device : IDisposable
    {
        public sealed class Information
        {
            private Imports.Device dev;

            internal Information(Imports.Device dev)
            {
                this.dev = dev;
                NativeMethods.CheckError(NativeMethods.get_devinfo(dev, out var info));

                DeviceSpeed = NativeMethods.device_speed(dev);

                Backend = info.backend;
                Serial = info.serial;
                UsbBus = info.usb_bus;
                UsbAddr = info.usb_addr;
                Instance = info.instance;
                Manufacturer = info.manufacturer;
                Product = info.product;

                BoardName = NativeMethods.get_board_name(dev);
            }

            public Backend Backend { get; }
            public string Serial { get; }
            public byte UsbBus { get; }
            public byte UsbAddr { get; }
            public uint Instance { get; }

            public string Manufacturer { get; }
            public string Product { get; }

            public string BoardName { get; }

            public DevSpeed DeviceSpeed { get; }

            public FPGASource FPGASource
            {
                get
                {
                    NativeMethods.CheckError(NativeMethods.get_fpga_source(dev, out var source));
                    return source;
                }
            }
            public FPGASize FPGASize
            {
                get
                {
                    NativeMethods.CheckError(NativeMethods.get_fpga_size(dev, out var size));
                    return size;
                }
            }

            public Version FPGAVersion
            {
                get
                {
                    NativeMethods.CheckError(NativeMethods.fpga_version(dev, out var version));
                    return new(version.major, version.minor, version.patch, version.describe);
                }
            }

            public Version FirmwareVersion
            {
                get
                {
                    NativeMethods.CheckError(NativeMethods.fw_version(dev, out var version));
                    return new(version.major, version.minor, version.patch, version.describe);
                }
            }
        }

        private readonly Imports.Device dev;
        private bool disposedValue;

        public bool IsFPGAConfigured => NativeMethods.CheckError(NativeMethods.is_fpga_configured(dev)) == 1;

        public Information Info { get; }

        public IReadOnlyList<RXChannel> RXChannels { get; }
        public IReadOnlyList<TXChannel> TXChannels { get; }

        public IEnumerable<Channel> AllChannels => RXChannels.Cast<Channel>().Concat(TXChannels);

        public ulong TXTimestamp
        {
            get
            {
                NativeMethods.CheckError(NativeMethods.get_timestamp(dev, Imports.Direction.TX, out var ts));
                return ts;
            }
        }

        public ulong RXTimestamp
        {
            get
            {
                NativeMethods.CheckError(NativeMethods.get_timestamp(dev, Imports.Direction.RX, out var ts));
                return ts;
            }
        }

        public RxMux RXMux
        {
            get
            {
                NativeMethods.CheckError(NativeMethods.get_rx_mux(dev, out var mode));
                return mode;
            }
            set => NativeMethods.CheckError(NativeMethods.set_rx_mux(dev, value));
        }

        public Loopback Loopback
        {
            get
            {
                NativeMethods.CheckError(NativeMethods.get_loopback(dev, out var mode));
                return mode;
            }
            set => NativeMethods.CheckError(NativeMethods.set_loopback(dev, value));
        }

        public bool IsLoopbackModeSupported(Loopback mode)
        {
            return NativeMethods.is_loopback_mode_supported(dev, mode);
        }

        public Device(string deviceIdentifier = "")
        {
            if (!Library.IsLoaded)
                throw new Exception("libbladeRF library was not loaded");

            NativeMethods.CheckError(NativeMethods.open(out dev, deviceIdentifier));

            Info = new(dev);

            var txChCount = NativeMethods.get_channel_count(dev, Imports.Direction.TX);
            var rxChCount = NativeMethods.get_channel_count(dev, Imports.Direction.RX);

            RXChannels = Enumerable.Range(0, (int)rxChCount).Select(idx => new RXChannel(dev, Imports.Channel.RX(idx))).ToList();
            TXChannels = Enumerable.Range(0, (int)txChCount).Select(idx => new TXChannel(dev, Imports.Channel.TX(idx))).ToList();
        }

        public TXStream CreateTXStream(int channelCount, int numBuffers, Format format, int samplesPerBuffer, int numTransfers)
        {
            return new(dev, channelCount, numBuffers, format, samplesPerBuffer, numTransfers);
        }

        public RXStream CreateRXStream(int channelCount, int numBuffers, Format format, int samplesPerBuffer, int numTransfers)
        {
            return new(dev, channelCount, numBuffers, format, samplesPerBuffer, numTransfers);
        }

        public void Reset()
        {
            NativeMethods.CheckError(NativeMethods.device_reset(dev));
        }

        private void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    // TODO: dispose managed state (managed objects)
                }

                NativeMethods.close(dev);
                disposedValue = true;
            }
        }

        ~Device()
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
}
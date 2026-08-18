# CartoTCP

This a 100% vibe coded avendish plugin that was only tested to create a TouchDesigner POP operator that gets point cloud data from carto from a TCP connection.

## Testing without the sensor

`tools/fake_cloud_sender.cpp` streams a synthetic point cloud over the same
protocol, and can inject the faults (reconnects, truncated frames, stalls,
bogus length prefixes) that make the POP misbehave. See
[tools/README.md](tools/README.md).

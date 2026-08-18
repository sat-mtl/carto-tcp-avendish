# fake_cloud_sender

Synthetic point-cloud source speaking the same wire protocol as the real carto
sender, for testing the Carto TCP POP without the sensor.

## Protocol

The POP is the TCP **server**; the sender is the client. Per frame:

| offset | type      | meaning                                  |
|--------|-----------|------------------------------------------|
| 0      | uint32 LE | payload length in bytes, multiple of 12  |
| 4      | float32[] | tightly packed `x, y, z` per point (LE)  |

Frames follow each other on the same connection with no other framing.

## Build

Standalone, no Avendish needed:

```bat
cl /std:c++20 /EHsc /O2 /I src /Fe:bin\fake_cloud_sender.exe tools\fake_cloud_sender.cpp ws2_32.lib
```

```sh
c++ -std=c++20 -O2 -I src -o fake_cloud_sender tools/fake_cloud_sender.cpp
```

Or through the project: `cmake -DCARTOTCP_BUILD_TOOLS=ON ...` (target
`fake_cloud_sender`; off by default, it is a debugging aid and is not shipped).

## Use

Set **Port** and turn **Listen** on in the POP first, then:

```bat
bin\fake_cloud_sender.exe --port 9898 --points 100000 --fps 60
```

It reconnects on its own if the POP is not listening yet, and prints
frames/s and MB/s once a second.

```
--host <addr>      POP host (default 127.0.0.1)
--port <n>         POP port (default 9898)
--points <n>       points per frame (default 65536)
--vary <min:max>   random point count per frame instead of --points
--fps <f>          frames per second, 0 = flat out (default 60)
--duration <s>     stop after s seconds (default: until Ctrl-C)
--pattern <p>      sphere | wave | cube (default sphere)
--seed <n>         RNG seed (default 1234)
```

## Fault injection

All off unless asked for. These reproduce what a real sensor or a flaky network
does, which is what makes TouchDesigner hang or die.

```
--reconnect <s>    drop and reopen the connection every s seconds
--stall <s>[:<n>]  every n frames, send the length prefix then wait s seconds
                   before the payload
--truncate <n>     every n frames, send half the payload and hang up
--badlen <n>       every n frames, send a length that is not a multiple of 12
--huge <n>         every n frames, announce a ~2 GB payload and send nothing
--split-header     write the length prefix and payload in separate sends
```

### Reproduction recipes

Memory blow-up — one bogus header makes the receiver allocate the announced
size before a single byte of payload arrives, and it never gives it back
(observed: 4.1 GB RSS from two such frames):

```bat
bin\fake_cloud_sender.exe --port 9898 --huge 90 --duration 30
```

Wedged receive — the receiver blocks in `recv` waiting for a payload that never
comes; the POP keeps showing the previous cloud and looks alive:

```bat
bin\fake_cloud_sender.exe --port 9898 --stall 30:120
```

Churn — connection lifecycle under repeated drops, half-frames and varying
point counts:

```bat
bin\fake_cloud_sender.exe --port 9898 --vary 1000:200000 --reconnect 3 --truncate 200
```

Throughput — saturate the link to see whether cooking or the network is the
bottleneck:

```bat
bin\fake_cloud_sender.exe --port 9898 --points 500000 --fps 0
```

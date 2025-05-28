# Base Station + Paired Rocket Devices

At URC, we need a way to talk to the Rover from long distances. For that, we use two things:

1. Base station antenna tower: a giant metal pole with a black box of electronics.
  - Usually powered with an electric generator when testing at FEARS.
  - When competing at URC, we instead use the power supply they provide.
  - Has its own router with SSID: `REMI`

2. A pair of "Rocket" devices: these two modules talk and link the two networks over long distances.
  - Computers can connect to the `REMI` Wi-Fi or the (long) Ethernet cable that plugs into the Base Station Ebox.
  - The Rocket on the base station connects to that on the Rover.
  - The Rover's Rocket redirects traffic to it onto other nodes.

> [!CAUTION]
> The Rover's router needs to be turned on **before** the Base Station's router. Otherwise, the statically assigned IPs won't work, and the Jetson won't be reachable [on its assigned IP address](../network.md).

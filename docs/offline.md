# How to use the Rover offline

At competition, we must have the Rover offline - there's no Wi-Fi at the MDRS. Most hotels lack a wired internet connection, and their wireless will be both spotty and registration-only.

In other words, you need to be prepared to use the Rover without an internet connection. This file explains how your workflow needs to change for that to happen.

## Offline Rust

With Rust, you can often just build something online, then everything will build offline forever after. However, if you make big changes with changes to dependencies, and those dependencies can't be downloaded to the Rover, you'll need to adopt some different tooling.

Let's install a tool called [`cargo-local-registry`](https://crates.io/crates/cargo-local-registry). It does what it sounds like - it'll take all your dependencies, then download them into a ["local" package registry](https://doc.rust-lang.org/cargo/reference/source-replacement.html#local-registry-sources). After running it, you'll be able to build offline - even if the build machine _literally_ has no internet connection!

### Installation Steps

- Get on a Linux computer - can be native, Docker, or virtualized.
- Grab `cargo-binstall` by running `curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash`.
  - If you have any trouble, see the [full installation instructions here](https://github.com/cargo-bins/cargo-binstall#installation).
- Run `cargo binstall cargo-local-registry`

You'll only need to do this once per computer.

### Usage

Using this tool is very simple - there are only three steps:

```console
# cd to the `auto_ros2` directory on your computer.
#
# in this example, we'll have it at `/home/barrett/auto_ros2`
~ $ cd auto_ros2

# great! now, we can run the `cargo-local-registry` command to grab a copy of
# all dependencies:
~/auto_ros2 $ cargo local-registry --sync Cargo.lock ../auto_ros2_registry
Updating crates.io index
  Downloaded futures-sink v0.3.31
  Downloaded byte-tools v0.3.1
  Downloaded futures-io v0.3.31
# (SNIP...)
Downloaded 164 crates (42.9 MB) in 4.06s (largest was `windows` at 9.3 MB)
add this to your .cargo/config somewhere:

  [source.crates-io]
  registry = 'sparse+https://index.crates.io/'
  replace-with = 'local-registry'

  [source.local-registry]
  local-registry = '/home/barrett/auto_ros2/../auto_ros2_registry'

# great, we've made the local registry!
#
# copy those onto a flash drive, or use FileZilla/`scp` to copy them
# onto the NVIDIA Jetson...
~/auto-ros2 $ cp -r /home/barrett/auto_ros2_registry /media/barrett/sandisk_usb_drive/auto_ros2_registry

# (get it on the rover somehow)
~/auto_ros2 $ ssh remi@192.168.1.68
/home/remi $ cp -r /media/remi/sandisk_usb_drive/auto_ros2_registry /home/remi/auto_ros2_registry
```

All done! Now, open up `/home/remi/auto_ros2/.cargo/config.toml` on the Jetson and ask it to use the local registry:

```console
/home/remi $ cd auto_ros2

~/auto_ros2 $ nano .cargo/config.toml
```

Add the following:

```toml
[source.crates-io]
registry = 'sparse+https://index.crates.io/'
replace-with = 'local-registry'

[source.local-registry]
local-registry = '/home/remi/auto_ros2_registry'
```

## Offline Python (`uv`)

Unfortunately, `uv` doesn't have any similar concept. Instead, you'll need to use [`uv sync`](https://docs.astral.sh/uv/reference/cli/#uv-sync) before you go offline.

### What if I deleted my `.venv`?

No worries! Just do `uv sync --offline`. You'll still need to have had ran `uv sync` before, when you were online, though!

### Can't we just copy the `.venv` folder from another computer?

Nope. `.venv` is specific to your user, and it uses hard-coded paths.

### Can we just copy `install/` from another user?

Yes, but tread carefully. It'll use hard-coded paths in source scripts, so you can use a one-liner to change them. Let's imagine you built it on a computer with user home dir: `/home/pat`, but you want it on `/home/remi`.

After you copy it over, use:

`find /home/remi/auto_ros2/install/ -type f -exec sed -i 's|/home/pat|/home/remi|g' {} +`

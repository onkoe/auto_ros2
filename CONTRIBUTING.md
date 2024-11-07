# Contributing

Notes on how to work on the Sooner Rover Team and this project go here. (TODO)

## Some basics

Keeping this here until I get some real docs about contributing. These are pretty typical and work well in almost every project!

- When writing your Git commit messages, please use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/). This helps folks know what you're changing.
  - Examples:
    - `fix(msg/CamId): typo`
    - `feat(node/nav): improve path planning`
    - `docs(launch): link to tutorial in README`
- Always use a project manager for your code.
  - In Rust, this is [`cargo`](https://doc.rust-lang.org/cargo/getting-started/installation.html).
  - Python has many options, but here, we use [`uv`](https://docs.astral.sh/uv/).
- Document anything that reaches the `main` branch.
  - This includes functions and types.
  - Individual subprojects (like nodes) should each include their own [`README.md` file](https://en.wikipedia.org/wiki/README).
- When possible, please test your work!

## Setup

To work on the project, you need a few things. First, you need to be using a Windows or Linux computer. macOS 11 and above currently does not support building ROS 2, so if you must use a Mac, consider [installing Asahi Linux](https://asahilinux.org/) or using a Linux virtual machine.

You also need to install Rust and `uv` (Python). Let's start with Rust. To get it, you can follow the short [instructions on their website](https://rustup.rs/). Next, you'll want to get Python. For this, we use `uv` to manage our Python installations, virtualenvs, and dependencies. Getting it is just one step: run the command shown [on their website](https://docs.astral.sh/uv/).

To continue, you'll also need Git.

- For Linux, you can install it from your package manager:
  - `sudo apt install -y git` for Ubuntu, and
  - `sudo dnf install -y git` on Fedora.
- Windows users should instead run the [Git for Windows](https://gitforwindows.org/) installer.

Afterwards, you need to sign in with GitHub. You can do that using the [GitHub Desktop](https://github.com/desktop/desktop?tab=readme-ov-file#where-can-i-get-it) tool. Alternatively, you can install [`gh-cli`](https://github.com/cli/cli/blob/trunk/docs/install_linux.md#fedora-centos-red-hat-enterprise-linux-dnf5) and run `gh auth login --git-protocol=https --web && gh auth setup-git`. This will log in to GitHub, allowing you to upload your changes.

On **Windows only**: you need to install `wsl2`. Open a PowerShell with administrator privileges, and run this command: `wsl --install`. **You must install ROS 2 inside of WSL, not on normal Windows!**

Finally, you can install ROS 2 Rolling. [Follow the steps here](https://docs.ros.org/en/rolling/Installation/Alternatives/Ubuntu-Development-Setup.html) to get it working.

So, to summarize, you just need to meet these requirements:

- Computer is running either Windows with WSL or Linux, but not macOS.
- You have Rust and Python installed.
- Git is installed and logged into GitHub.
- ROS 2 is installed and set up.

After you've done all that, you can `git clone https://github.com/Sooner-Rover-Team/auto_ros2` and start working on the implementation. If you need any assistance, please reach out.

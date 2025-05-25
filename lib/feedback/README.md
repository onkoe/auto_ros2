# feedback

This is a cross-platform library intended to deserialize the Rover's various message types into their raw bytes for external usage.

## Usage

`feedback` is compatible with native Rust, alongside Python using PyO3.

### Rust

To use this library in Rust, simply import it with `use feedback;`.

### Python

To get it into a Python project, there are a few steps:

1. Install `maturin` using the instructions [in their README](https://github.com/PyO3/maturin).
1. Build the crate into a Python wheel with `maturin build`. It should go to `feedback/target/wheels/wheels-(a bunch of stuff).whl`.
1. You can either [put that onto PyPI](https://pypi.org/project/pyo3-pack/) and install it from Rye, or use the built artifact directly. Let's assume you're using the artifact!
1. Run this command. Make sure to replace the path with a correct one: `rye add feedback --path "../target/wheels/feedback-0.1.0-cp311-cp311-macosx_11_0_arm64.whl"`.
1. Type `rye sync` into your terminal and use it in your project with: `import feedback`.
1. All done!

## Development

When making changes on the Rust side, you can easily write doctests and unit tests for your testing needs. However, Python is generally used with the `maturin develop` command. Give it a try!

Here are some resources for Rust and PyO3 in general:

- [The PyO3 Book](https://pyo3.rs/) - a good source of info for writing Python... with Rust!
- [The Rust Programming Language (book)](https://doc.rust-lang.org/book/foreword.html) - A Rust user's main reference. It explains all significant parts of the language in detail.
- [`std` Documentation](https://doc.rust-lang.org/std/index.html) - The Rust standard library documentation. All parts of the language are fully documented, so it's easy to find some examples and help.

If you have any questions, please feel free to let me know!

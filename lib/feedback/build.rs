#[cfg(not(feature = "python"))]
fn main() {}

#[cfg(feature = "python")]
fn main() {
    // this fixes the weird macOS linking
    pyo3_build_config::add_extension_module_link_args();
}

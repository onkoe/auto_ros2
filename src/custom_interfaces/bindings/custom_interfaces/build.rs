fn main() {
    println!("cargo:rustc-link-lib=custom_interfaces__rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=custom_interfaces__rosidl_generator_c");

    if let Some(e) = std::env::var_os("AMENT_PREFIX_PATH") {
        let env = e.to_str().unwrap();
        for path in env.split(':') {
            println!("cargo:rustc-link-search={path}/lib");
        }
    }

    // we'll manually add `custom_interfaces to the list of `rustc` linker
    // search locations.
    //
    // otherwise, build fails with a giant linker err lol
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={manifest_dir}/../../../../install/custom_interfaces/lib");
}

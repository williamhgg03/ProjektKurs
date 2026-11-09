fn main() {
    const REQUIRED_KEYS: [&str; 3] = ["WIFI_SSID", "WIFI_PASS", "SERVER_URL"];
    const OPTIONAL_KEYS: [&str; 2] = ["WIFI_EAP_USERNAME", "WIFI_EAP_IDENTITY"];
    const SECRETS_FILE: &str = "wifi.env";

    println!("cargo:rerun-if-changed={SECRETS_FILE}");
    let file = std::fs::read_to_string(SECRETS_FILE)
        .map(|contents| parse_env(&contents))
        .unwrap_or_default();

    for key in REQUIRED_KEYS {
        println!("cargo:rerun-if-env-changed={key}");
        let value = std::env::var(key)
            .ok()
            .or_else(|| file.get(key).cloned())
            .unwrap_or_else(|| {
                panic!("{key} is not set; copy wifi.env.example to wifi.env and fill it in")
            });
        let value = if key == "SERVER_URL" {
            value.trim_end_matches('/').to_owned()
        } else {
            value
        };
        println!("cargo:rustc-env={key}={value}");
    }

    for key in OPTIONAL_KEYS {
        println!("cargo:rerun-if-env-changed={key}");
        if let Some(value) = std::env::var(key).ok().or_else(|| file.get(key).cloned()) {
            if !value.is_empty() {
                println!("cargo:rustc-env={key}={value}");
            }
        }
    }

    linker_be_nice();
    println!("cargo:rustc-link-arg=-Tdefmt.x");
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

fn parse_env(contents: &str) -> std::collections::HashMap<String, String> {
    contents
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .filter_map(|line| line.split_once('='))
        .map(|(key, value)| {
            (
                key.trim().to_owned(),
                value.trim().trim_matches('"').to_owned(),
            )
        })
        .collect()
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                what if what.starts_with("_defmt_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`"
                    );
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                what if what.starts_with("esp_rtos_") => {
                    eprintln!();
                    eprintln!(
                        "💡 `esp-radio` has no scheduler enabled. Make sure you have initialized `esp-rtos` or provided an external scheduler."
                    );
                    eprintln!();
                }
                "embedded_test_linker_file_not_added_to_rustflags" => {
                    eprintln!();
                    eprintln!(
                        "💡 `embedded-test` not found - make sure `embedded-test.x` is added as a linker script for tests"
                    );
                    eprintln!();
                }
                "free"
                | "malloc"
                | "calloc"
                | "get_free_internal_heap_size"
                | "malloc_internal"
                | "realloc_internal"
                | "calloc_internal"
                | "free_internal" => {
                    eprintln!();
                    eprintln!(
                        "💡 Did you forget the `esp-alloc` dependency or didn't enable the `compat` feature on it?"
                    );
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=-Wl,--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}

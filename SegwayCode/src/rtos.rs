// Small FreeRTOS / ESP-IDF helpers: time since boot and spawning threads pinned to a core.

use std::ffi::CStr;
use std::thread::{self, JoinHandle};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;

/// Microseconds since boot. Same clock on both cores.
pub fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Milliseconds since boot, wraps after 49 days. u32 because the ESP32-S3 only has 32-bit
/// atomics, so this is what can be shared between cores without a lock.
pub fn now_ms() -> u32 {
    (now_us() / 1000) as u32
}

/// Spawn a std thread whose FreeRTOS task is pinned to `core` with the given priority.
///
/// `ThreadSpawnConfiguration` is per calling thread and applies to every thread it spawns
/// afterwards, so it is restored to the default before returning.
pub fn spawn_pinned<F>(
    name: &'static CStr,
    core: Core,
    priority: u8,
    stack_size: usize,
    f: F,
) -> anyhow::Result<JoinHandle<()>>
where
    F: FnOnce() + Send + 'static,
{
    ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        priority,
        pin_to_core: Some(core),
        ..Default::default()
    }
    .set()?;

    let spawned = thread::Builder::new()
        .name(name.to_string_lossy().into_owned())
        .stack_size(stack_size)
        .spawn(f);

    ThreadSpawnConfiguration::default().set()?;
    Ok(spawned?)
}

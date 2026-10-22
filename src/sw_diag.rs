//! This module provides the ESP-IDF implementation of the Matter `SwDiag` trait.

#[cfg(not(esp_idf_spiram_use_malloc))]
use esp_idf_svc::sys::MALLOC_CAP_INTERNAL;
use esp_idf_svc::sys::{
    esp, heap_caps_get_free_size, heap_caps_get_minimum_free_size, heap_caps_get_total_size,
    heap_caps_monitor_local_minimum_free_size_start,
    heap_caps_monitor_local_minimum_free_size_stop, MALLOC_CAP_DEFAULT,
};

use rs_matter_stack::matter::dm::clusters::sw_diag::{SwDiag, ThreadMetric};
use rs_matter_stack::matter::error::{Error, ErrorCode};

/// The memory `malloc` allocates from: internal RAM, plus PSRAM with `CONFIG_SPIRAM_USE_MALLOC=y`.
///
/// PSRAM is a `MALLOC_CAP_DEFAULT` region with `CONFIG_SPIRAM_USE_CAPS_ALLOC=y` too, but `malloc`
/// does not use it then.
#[cfg(esp_idf_spiram_use_malloc)]
const MALLOC_CAPS: u32 = MALLOC_CAP_DEFAULT;
#[cfg(not(esp_idf_spiram_use_malloc))]
const MALLOC_CAPS: u32 = MALLOC_CAP_DEFAULT | MALLOC_CAP_INTERNAL;

/// The ESP-IDF implementation of the Matter `SwDiag` trait.
///
/// Reports the usage of the heap `malloc` allocates from - combined internal RAM and PSRAM with
/// `CONFIG_SPIRAM_USE_MALLOC=y` - and, with `CONFIG_FREERTOS_USE_TRACE_FACILITY=y`, the FreeRTOS tasks.
///
/// The high watermark sums the low watermarks of the individual heap regions, so it can be
/// higher than the actual peak.
///
/// Advertise these with `sw_diag(heap, watermarks, thread)` on the root endpoint;
/// leave out `thread` without `CONFIG_FREERTOS_USE_TRACE_FACILITY=y`.
#[derive(Debug, Default, Copy, Clone)]
pub struct EspSwDiag;

impl EspSwDiag {
    /// Create a new instance of the `EspSwDiag` type.
    pub const fn new() -> Self {
        Self
    }
}

impl SwDiag for EspSwDiag {
    fn current_heap_free(&self) -> Result<u64, Error> {
        Ok(heap_free() as _)
    }

    fn current_heap_used(&self) -> Result<u64, Error> {
        Ok(heap_total().saturating_sub(heap_free()) as _)
    }

    fn current_heap_high_watermark(&self) -> Result<u64, Error> {
        let min_free = unsafe { heap_caps_get_minimum_free_size(MALLOC_CAPS) };

        Ok(heap_total().saturating_sub(min_free) as _)
    }

    fn thread_metrics(
        &self,
        visit: &mut dyn FnMut(&ThreadMetric<'_>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        #[cfg(esp_idf_freertos_use_trace_facility)]
        tasks::visit(visit)?;

        #[cfg(not(esp_idf_freertos_use_trace_facility))]
        let _ = visit;

        Ok(())
    }

    fn reset_watermarks(&self) -> Result<(), Error> {
        // Fails if the monitoring is not running yet, which is fine
        unsafe { heap_caps_monitor_local_minimum_free_size_stop() };

        esp!(unsafe { heap_caps_monitor_local_minimum_free_size_start() })
            .map_err(|_| ErrorCode::Failure.into())
    }
}

fn heap_free() -> usize {
    unsafe { heap_caps_get_free_size(MALLOC_CAPS) }
}

fn heap_total() -> usize {
    unsafe { heap_caps_get_total_size(MALLOC_CAPS) }
}

#[cfg(esp_idf_freertos_use_trace_facility)]
mod tasks {
    use core::ffi::CStr;

    use alloc::vec::Vec;

    use esp_idf_svc::sys::{
        uxTaskGetNumberOfTasks, uxTaskGetSystemState, vTaskSuspendAll, xTaskResumeAll, TaskStatus_t,
    };

    use rs_matter_stack::matter::dm::clusters::sw_diag::ThreadMetric;
    use rs_matter_stack::matter::error::Error;

    /// The longest thread name Matter allows
    const MAX_NAME_LEN: usize = 8;

    /// Room for tasks created between sizing the buffers and taking the snapshot
    const EXTRA_TASKS: usize = 4;

    pub fn visit(
        visit: &mut dyn FnMut(&ThreadMetric<'_>) -> Result<(), Error>,
    ) -> Result<(), Error> {
        let capacity = unsafe { uxTaskGetNumberOfTasks() } as usize + EXTRA_TASKS;

        let mut tasks = Vec::<TaskStatus_t>::with_capacity(capacity);
        let mut names = Vec::<heapless::String<MAX_NAME_LEN>>::with_capacity(capacity);

        unsafe {
            vTaskSuspendAll();

            let count =
                uxTaskGetSystemState(tasks.as_mut_ptr(), capacity as _, core::ptr::null_mut());
            tasks.set_len(count as _);

            // The names point into the task control blocks, so copy them before a task can be deleted.
            // `push` cannot reallocate, as `count <= capacity`
            for task in &tasks {
                names.push(name_prefix(CStr::from_ptr(task.pcTaskName)));
            }

            xTaskResumeAll();
        }

        for (task, name) in tasks.iter().zip(&names) {
            visit(&ThreadMetric {
                id: task.xTaskNumber as _,
                name: Some(name.as_str()),
                // FreeRTOS reports neither for stacks growing downwards
                stack_free_current: None,
                stack_size: None,
                stack_free_minimum: Some(task.usStackHighWaterMark as _),
            })?;
        }

        Ok(())
    }

    fn name_prefix(name: &CStr) -> heapless::String<MAX_NAME_LEN> {
        let name = name.to_str().unwrap_or("");

        let mut prefix = heapless::String::new();
        for ch in name.chars() {
            if prefix.push(ch).is_err() {
                break;
            }
        }

        prefix
    }
}

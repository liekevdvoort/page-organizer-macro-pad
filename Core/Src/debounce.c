#include "debounce.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

debounce_t debounce_init(debounce_get_time_ms_fptr get_time_ms,
                         bool initial_state, uint32_t debounce_time_ms,
                         debounce_callback_fptr callback) {
  debounce_t output = {.state = initial_state,
                       .get_time_ms = get_time_ms,
                       .last_time = get_time_ms(),
                       .debounce_time_ms = debounce_time_ms,
                       .callback = callback};
  return output;
}
bool debounce_update(debounce_t *debounce_handle, bool state) {
  if (state != debounce_handle->state) {
    // change occured
    if (debounce_handle->get_time_ms() - debounce_handle->last_time >
        debounce_handle->debounce_time_ms) {
      debounce_handle->state = state;
      if (debounce_handle->callback != NULL) {
        debounce_handle->callback(debounce_handle->state);
      }
    }
    debounce_handle->last_time = debounce_handle->get_time_ms();
  }
  return debounce_handle->state;
}


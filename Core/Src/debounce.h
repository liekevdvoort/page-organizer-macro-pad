#ifdef __cplusplus
extern "C" {
#endif
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t (*debounce_get_time_ms_fptr)(void);
typedef void (*debounce_callback_fptr)(bool);

typedef struct debounce_t {
  bool state;
  uint32_t last_time;
  uint32_t debounce_time_ms;
  debounce_get_time_ms_fptr get_time_ms;
  debounce_callback_fptr callback;
} debounce_t;

/**
 * @brief  initialisation function for debouncing
 * @param get_time_ms: a function to a function returning elapsed milli seconds
 * in uint32_t format
 * @param initial_state: the initial state of the signal to be debounced.
 * @param debounce_time_ms: The time that the signal should be debounced for
 * e.g. if the signal changes within 20 ms it wil not trigger a state change.
 * @param callback: the callback to be called when a state change has occurred.
 * Can be NULL
 * @retval an initialised instance of a debounce_t struct
 */
debounce_t debounce_init(debounce_get_time_ms_fptr get_time_ms,
                         bool initial_state, uint32_t debounce_time_ms,
                         debounce_callback_fptr callback);
/**
 * @brief update function for debouncing. Needs to be called repeatedly or on
 * every state change of the signal to be debounced.
 * @param debounce_handle: pointer to the debounce struct which needs to be
 * debounced.
 * @param state: the current state of the  signal to be debounced
 * retval a boolean indicating the current state of the debounced signal
 */
bool debounce_update(debounce_t *debounce_handle, bool state);

#ifdef __cplusplus
}
#endif

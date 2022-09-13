#include "pico/util/queue.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/critical_section.h"
#include "pico/mutex.h"
#include "pico/types.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "pico/bootrom.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/resets.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/divider.h"
#include "pico/platform.h"


/*! \brief Initialise a queue, allocating a (possibly shared) spinlock
 *  \ingroup queue
 *
 * \param q Pointer to a queue_t structure, used as a handle
 * \param element_size Size of each value in the queue
 * \param element_count Maximum number of entries in the queue
 */
void nostatic_queue_init(queue_t * q, uint element_size, uint element_count) {
    return queue_init(q, element_size, element_count);
}


/*! \brief Unsafe check of level of the specified queue.
 *  \ingroup queue
 *
 * \param q Pointer to a queue_t structure, used as a handle
 * \return Number of entries in the queue
 *
 * This does not use the spinlock, so may return incorrect results if the
 * spin lock is not externally locked
 */
uint nostatic_queue_get_level_unsafe(queue_t * q) {
    return queue_get_level_unsafe(q);
}


/*! \brief Check of level of the specified queue.
 *  \ingroup queue
 *
 * \param q Pointer to a queue_t structure, used as a handle
 * \return Number of entries in the queue
 */
uint nostatic_queue_get_level(queue_t * q) {
    return queue_get_level(q);
}


/*! \brief Returns the highest level reached by the specified queue since it was created
 *         or since the max level was reset
 *  \ingroup queue
 *
 * \param q Pointer to a queue_t structure, used as a handle
 * \return Maximum level of the queue
 */
uint nostatic_queue_get_max_level(queue_t * q) {
    return queue_get_max_level(q);
}


/*! \brief Reset the highest level reached of the specified queue.
 *  \ingroup queue
 *
 * \param q Pointer to a queue_t structure, used as a handle
 */
void nostatic_queue_reset_max_level(queue_t * q) {
    return queue_reset_max_level(q);
}


/*! \brief Check if queue is empty
 *  \ingroup queue
 *
 * \param q Pointer to a queue_t structure, used as a handle
 * \return true if queue is empty, false otherwise
 *
 * This function is interrupt and multicore safe.
 */
bool nostatic_queue_is_empty(queue_t * q) {
    return queue_is_empty(q);
}


/*! \brief Check if queue is full
 *  \ingroup queue
 *
 * \param q Pointer to a queue_t structure, used as a handle
 * \return true if queue is full, false otherwise
 *
 * This function is interrupt and multicore safe.
 */
bool nostatic_queue_is_full(queue_t * q) {
    return queue_is_full(q);
}


/*! \brief Attempt to set a system clock frequency in khz
 *  \ingroup pico_stdlib
 *
 * Note that not all clock frequencies are possible; it is preferred that you
 * use src/rp2_common/hardware_clocks/scripts/vcocalc.py to calculate the parameters
 * for use with set_sys_clock_pll
 *
 * \param freq_khz Requested frequency
 * \param required if true then this function will assert if the frequency is not attainable.
 * \return true if the clock was configured
 */
bool nostatic_set_sys_clock_khz(uint32_t freq_khz, bool required) {
    return set_sys_clock_khz(freq_khz, required);
}


/*! \brief Return a representation of the current time.
 * \ingroup timestamp
 *
 * Returns an opaque high fidelity representation of the current time sampled during the call.
 *
 * \return the absolute time (now) of the hardware timer
 *
 * \sa absolute_time_t
 * \sa sleep_until()
 * \sa time_us_64()
 */
absolute_time_t nostatic_get_absolute_time() {
    return get_absolute_time();
}


/*! fn to_ms_since_boot
 * \ingroup timestamp
 * \brief Convert a timestamp into a number of milliseconds since boot.
 * \param t an absolute_time_t value to convert
 * \return the number of milliseconds since boot represented by t
 * \sa to_us_since_boot()
 */
uint32_t nostatic_to_ms_since_boot(absolute_time_t t) {
    return to_ms_since_boot(t);
}


/*! \brief Return a timestamp value obtained by adding a number of microseconds to another timestamp
 * \ingroup timestamp
 *
 * \param t the base timestamp
 * \param us the number of microseconds to add
 * \return the timestamp representing the resulting time
 */
absolute_time_t nostatic_delayed_by_us(const absolute_time_t t, uint64_t us) {
    return delayed_by_us(t, us);
}


/*! \brief Return a timestamp value obtained by adding a number of milliseconds to another timestamp
 * \ingroup timestamp
 *
 * \param t the base timestamp
 * \param ms the number of milliseconds to add
 * \return the timestamp representing the resulting time
 */
absolute_time_t nostatic_delayed_by_ms(const absolute_time_t t, uint32_t ms) {
    return delayed_by_ms(t, ms);
}


/*! \brief Convenience method to get the timestamp a number of microseconds from the current time
 * \ingroup timestamp
 *
 * \param us the number of microseconds to add to the current timestamp
 * \return the future timestamp
 */
absolute_time_t nostatic_make_timeout_time_us(uint64_t us) {
    return make_timeout_time_us(us);
}


/*! \brief Convenience method to get the timestamp a number of milliseconds from the current time
 * \ingroup timestamp
 *
 * \param ms the number of milliseconds to add to the current timestamp
 * \return the future timestamp
 */
absolute_time_t nostatic_make_timeout_time_ms(uint32_t ms) {
    return make_timeout_time_ms(ms);
}


/*! \brief Return the difference in microseconds between two timestamps
 * \ingroup timestamp
 *
 * \note be careful when diffing against large timestamps (e.g. \ref at_the_end_of_time)
 * as the signed integer may overflow.
 *
 * \param from the first timestamp
 * \param to the second timestamp
 * \return the number of microseconds between the two timestamps (positive if `to` is after `from` except
 * in case of overflow)
 */
int64_t nostatic_absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return absolute_time_diff_us(from, to);
}


/*! \brief Determine if the given timestamp is "at_the_end_of_time"
 * \ingroup timestamp
 *  \param t the timestamp
 *  \return true if the timestamp is at_the_end_of_time
 *  \sa at_the_end_of_time
 */
bool nostatic_is_at_the_end_of_time(absolute_time_t t) {
    return is_at_the_end_of_time(t);
}


/*! \brief Determine if the given timestamp is nil
 * \ingroup timestamp
 *  \param t the timestamp
 *  \return true if the timestamp is nil
 *  \sa nil_time
 */
bool nostatic_is_nil_time(absolute_time_t t) {
    return is_nil_time(t);
}


/*!
 * \brief Add an alarm callback to be called after a delay specified in microseconds
 * \ingroup alarm
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core the alarm pool was created on. If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param pool the alarm pool to use for scheduling the callback (this determines which hardware alarm is used, and which core calls the callback)
 * @param us the delay (from now) in microseconds when (after which) the callback should fire
 * @param callback the callback function
 * @param user_data user data to pass to the callback function
 * @param fire_if_past if true, and the alarm time falls during this call before the alarm can be set,
 *                     then the callback should be called during (by) this function instead 
 * @return >0 the alarm id
 * @return 0 if the alarm time passed before or during the call AND there is no active alarm to return the id of.
 *           The latter can either happen because fire_if_past was false (i.e. no timer was ever created),
 *           or if the callback <i>was</i> called during this method but the callback cancelled itself by returning 0
 * @return -1 if there were no alarm slots available
 */
alarm_id_t nostatic_alarm_pool_add_alarm_in_us(alarm_pool_t * pool, uint64_t us, alarm_callback_t callback, void * user_data, bool fire_if_past) {
    return alarm_pool_add_alarm_in_us(pool, us, callback, user_data, fire_if_past);
}


/*!
 * \brief Add an alarm callback to be called after a delay specified in milliseconds
 * \ingroup alarm
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core the alarm pool was created on. If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param pool the alarm pool to use for scheduling the callback (this determines which hardware alarm is used, and which core calls the callback)
 * @param ms the delay (from now) in milliseconds when (after which) the callback should fire
 * @param callback the callback function
 * @param user_data user data to pass to the callback function
 * @param fire_if_past if true, and the alarm time falls before or during this call before the alarm can be set,
 *                     then the callback should be called during (by) this function instead 
 * @return >0 the alarm id
 * @return 0 if the alarm time passed before or during the call AND there is no active alarm to return the id of.
 *           The latter can either happen because fire_if_past was false (i.e. no timer was ever created),
 *           or if the callback <i>was</i> called during this method but the callback cancelled itself by returning 0
 * @return -1 if there were no alarm slots available
 */
alarm_id_t nostatic_alarm_pool_add_alarm_in_ms(alarm_pool_t * pool, uint32_t ms, alarm_callback_t callback, void * user_data, bool fire_if_past) {
    return alarm_pool_add_alarm_in_ms(pool, ms, callback, user_data, fire_if_past);
}


/*!
 * \brief Add an alarm callback to be called at a specific time
 * \ingroup alarm
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core of the default alarm pool (generally core 0). If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param time the timestamp when (after which) the callback should fire
 * @param callback the callback function
 * @param user_data user data to pass to the callback function
 * @param fire_if_past if true, and the alarm time falls before or during this call before the alarm can be set,
 *                     then the callback should be called during (by) this function instead 
 * @return >0 the alarm id
 * @return 0 if the alarm time passed before or during the call AND there is no active alarm to return the id of.
 *           The latter can either happen because fire_if_past was false (i.e. no timer was ever created),
 *           or if the callback <i>was</i> called during this method but the callback cancelled itself by returning 0
 * @return -1 if there were no alarm slots available
 */
alarm_id_t nostatic_add_alarm_at(absolute_time_t time, alarm_callback_t callback, void * user_data, bool fire_if_past) {
    return add_alarm_at(time, callback, user_data, fire_if_past);
}


/*!
 * \brief Add an alarm callback to be called after a delay specified in microseconds
 * \ingroup alarm
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core of the default alarm pool (generally core 0). If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param us the delay (from now) in microseconds when (after which) the callback should fire
 * @param callback the callback function
 * @param user_data user data to pass to the callback function
 * @param fire_if_past if true, and the alarm time falls during this call before the alarm can be set,
 *                     then the callback should be called during (by) this function instead 
 * @return >0 the alarm id
 * @return 0 if the alarm time passed before or during the call AND there is no active alarm to return the id of.
 *           The latter can either happen because fire_if_past was false (i.e. no timer was ever created),
 *           or if the callback <i>was</i> called during this method but the callback cancelled itself by returning 0
 * @return -1 if there were no alarm slots available
 */
alarm_id_t nostatic_add_alarm_in_us(uint64_t us, alarm_callback_t callback, void * user_data, bool fire_if_past) {
    return add_alarm_in_us(us, callback, user_data, fire_if_past);
}


/*!
 * \brief Add an alarm callback to be called after a delay specified in milliseconds
 * \ingroup alarm
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core of the default alarm pool (generally core 0). If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param ms the delay (from now) in milliseconds when (after which) the callback should fire
 * @param callback the callback function
 * @param user_data user data to pass to the callback function
 * @param fire_if_past if true, and the alarm time falls during this call before the alarm can be set,
 *                     then the callback should be called during (by) this function instead 
 * @return >0 the alarm id
 * @return 0 if the alarm time passed before or during the call AND there is no active alarm to return the id of.
 *           The latter can either happen because fire_if_past was false (i.e. no timer was ever created),
 *           or if the callback <i>was</i> called during this method but the callback cancelled itself by returning 0
 * @return -1 if there were no alarm slots available
 */
alarm_id_t nostatic_add_alarm_in_ms(uint32_t ms, alarm_callback_t callback, void * user_data, bool fire_if_past) {
    return add_alarm_in_ms(ms, callback, user_data, fire_if_past);
}


/*!
 * \brief Cancel an alarm from the default alarm pool
 * \ingroup alarm
 * \param alarm_id the alarm
 * \return true if the alarm was cancelled, false if it didn't exist
 * \sa alarm_id_t for a note on reuse of IDs
 */
bool nostatic_cancel_alarm(alarm_id_t alarm_id) {
    return cancel_alarm(alarm_id);
}


/*!
 * \brief Add a repeating timer that is called repeatedly at the specified interval in milliseconds
 * \ingroup repeating_timer
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core the alarm pool was created on. If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param pool the alarm pool to use for scheduling the repeating timer (this determines which hardware alarm is used, and which core calls the callback)
 * @param delay_ms the repeat delay in milliseconds; if >0 then this is the delay between one callback ending and the next starting; if <0 then this is the negative of the time between the starts of the callbacks. The value of 0 is treated as 1 microsecond
 * @param callback the repeating timer callback function
 * @param user_data user data to pass to store in the repeating_timer structure for use by the callback.
 * @param out the pointer to the user owned structure to store the repeating timer info in. BEWARE this storage location must outlive the repeating timer, so be careful of using stack space
 * @return false if there were no alarm slots available to create the timer, true otherwise.
 */
bool nostatic_alarm_pool_add_repeating_timer_ms(alarm_pool_t * pool, int32_t delay_ms, repeating_timer_callback_t callback, void * user_data, repeating_timer_t * out) {
    return alarm_pool_add_repeating_timer_ms(pool, delay_ms, callback, user_data, out);
}


/*!
 * \brief Add a repeating timer that is called repeatedly at the specified interval in microseconds
 * \ingroup repeating_timer
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core of the default alarm pool (generally core 0). If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param delay_us the repeat delay in microseconds; if >0 then this is the delay between one callback ending and the next starting; if <0 then this is the negative of the time between the starts of the callbacks. The value of 0 is treated as 1
 * @param callback the repeating timer callback function
 * @param user_data user data to pass to store in the repeating_timer structure for use by the callback.
 * @param out the pointer to the user owned structure to store the repeating timer info in. BEWARE this storage location must outlive the repeating timer, so be careful of using stack space
 * @return false if there were no alarm slots available to create the timer, true otherwise.
 */
bool nostatic_add_repeating_timer_us(int64_t delay_us, repeating_timer_callback_t callback, void * user_data, repeating_timer_t * out) {
    return add_repeating_timer_us(delay_us, callback, user_data, out);
}


/*!
 * \brief Add a repeating timer that is called repeatedly at the specified interval in milliseconds
 * \ingroup repeating_timer
 *
 * Generally the callback is called as soon as possible after the time specified from an IRQ handler
 * on the core of the default alarm pool (generally core 0). If the callback is in the past or happens before
 * the alarm setup could be completed, then this method will optionally call the callback itself
 * and then return a return code to indicate that the target time has passed.
 *
 * \note It is safe to call this method from an IRQ handler (including alarm callbacks), and from either core.
 *
 * @param delay_ms the repeat delay in milliseconds; if >0 then this is the delay between one callback ending and the next starting; if <0 then this is the negative of the time between the starts of the callbacks. The value of 0 is treated as 1 microsecond
 * @param callback the repeating timer callback function
 * @param user_data user data to pass to store in the repeating_timer structure for use by the callback.
 * @param out the pointer to the user owned structure to store the repeating timer info in. BEWARE this storage location must outlive the repeating timer, so be careful of using stack space
 * @return false if there were no alarm slots available to create the timer, true otherwise.
 */
bool nostatic_add_repeating_timer_ms(int32_t delay_ms, repeating_timer_callback_t callback, void * user_data, repeating_timer_t * out) {
    return add_repeating_timer_ms(delay_ms, callback, user_data, out);
}


/*! \brief  Enter a critical_section
 *  \ingroup critical_section
 *
 * If the spin lock associated with this critical section is in use, then this
 * method will block until it is released.
 *
 * \param crit_sec Pointer to critical_section structure
 */
void nostatic_critical_section_enter_blocking(critical_section_t * crit_sec) {
    return critical_section_enter_blocking(crit_sec);
}


/*! \brief  Release a critical_section
 *  \ingroup critical_section
 *
 * \param crit_sec Pointer to critical_section structure
 */
void nostatic_critical_section_exit(critical_section_t * crit_sec) {
    return critical_section_exit(crit_sec);
}


/*! \brief Test for mutex initialized state
 *  \ingroup mutex
 *
 * \param mtx Pointer to mutex structure
 * \return true if the mutex is initialized, false otherwise
 */
bool nostatic_mutex_is_initialized(mutex_t * mtx) {
    return mutex_is_initialized(mtx);
}


/*! \brief Test for recursive mutex initialized state
 *  \ingroup mutex
 *
 * \param mtx Pointer to recursive mutex structure
 * \return true if the recursive mutex is initialized, false otherwise
 */
bool nostatic_recursive_mutex_is_initialized(recursive_mutex_t * mtx) {
    return recursive_mutex_is_initialized(mtx);
}


/*! fn to_us_since_boot
 * \brief convert an absolute_time_t into a number of microseconds since boot.
 * \param t the absolute time to convert
 * \return a number of microseconds since boot, equivalent to t
 * \ingroup timestamp
 */
uint64_t nostatic_to_us_since_boot(absolute_time_t t) {
    return to_us_since_boot(t);
}


/*! fn update_us_since_boot
 * \brief update an absolute_time_t value to represent a given number of microseconds since boot
 * \param t the absolute time value to update
 * \param us_since_boot the number of microseconds since boot to represent. Note this should be representable
 *                      as a signed 64 bit integer
 * \ingroup timestamp
 */
void nostatic_update_us_since_boot(absolute_time_t * t, uint64_t us_since_boot) {
    return update_us_since_boot(t, us_since_boot);
}


/*! \brief Check the read FIFO to see if there is data available (sent by the other core)
 *  \ingroup multicore_fifo
 *
 * See the note in the \ref multicore_fifo section for considerations regarding use of the inter-core FIFOs
 *
 * \return true if the FIFO has data in it, false otherwise
 */
bool nostatic_multicore_fifo_rvalid() {
    return multicore_fifo_rvalid();
}


/*! \brief Check the write FIFO to see if it has space for more data
 *  \ingroup multicore_fifo
 *
 * See the note in the \ref multicore_fifo section for considerations regarding use of the inter-core FIFOs
 *
 *  @return true if the FIFO has room for more data, false otherwise
 */
bool nostatic_multicore_fifo_wready() {
    return multicore_fifo_wready();
}


/*! \brief Discard any data in the read FIFO
 *  \ingroup multicore_fifo
 *
 * See the note in the \ref multicore_fifo section for considerations regarding use of the inter-core FIFOs
 */
void nostatic_multicore_fifo_drain() {
    return multicore_fifo_drain();
}


/*! \brief Convert I2C instance to hardware instance number
 *  \ingroup hardware_i2c
 *
 * \param i2c I2C instance
 * \return Number of I2C, 0 or 1.
 */
uint nostatic_i2c_hw_index(i2c_inst_t * i2c) {
    return i2c_hw_index(i2c);
}


/*! \brief Attempt to write specified number of bytes to address, with timeout
 *  \ingroup hardware_i2c
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \param addr 7-bit address of device to write to
 * \param src Pointer to data to send
 * \param len Length of data in bytes to send
 * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
 *           and the next transfer will begin with a Restart rather than a Start.
 * \param timeout_us The time that the function will wait for the entire transaction to complete. Note, an individual timeout of
 *           this value divided by the length of data is applied for each byte transfer, so if the first or subsequent
 *           bytes fails to transfer within that sub timeout, the function will return with an error.
 *
 * \return Number of bytes written, or PICO_ERROR_GENERIC if address not acknowledged, no device present, or PICO_ERROR_TIMEOUT if a timeout occurred.
 */
int nostatic_i2c_write_timeout_us(i2c_inst_t * i2c, uint8_t addr, const uint8_t * src, size_t len, bool nostop, uint timeout_us) {
    return i2c_write_timeout_us(i2c, addr, src, len, nostop, timeout_us);
}


/*! \brief  Attempt to read specified number of bytes from address, with timeout
 *  \ingroup hardware_i2c
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \param addr 7-bit address of device to read from
 * \param dst Pointer to buffer to receive data
 * \param len Length of data in bytes to receive
 * \param nostop  If true, master retains control of the bus at the end of the transfer (no Stop is issued),
 *           and the next transfer will begin with a Restart rather than a Start.
 * \param timeout_us The time that the function will wait for the entire transaction to complete
 * \return Number of bytes read, or PICO_ERROR_GENERIC if address not acknowledged, no device present, or PICO_ERROR_TIMEOUT if a timeout occurred.
 */
int nostatic_i2c_read_timeout_us(i2c_inst_t * i2c, uint8_t addr, uint8_t * dst, size_t len, bool nostop, uint timeout_us) {
    return i2c_read_timeout_us(i2c, addr, dst, len, nostop, timeout_us);
}


/*! \brief Determine non-blocking write space available
 *  \ingroup hardware_i2c
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \return 0 if no space is available in the I2C to write more data. If return is nonzero, at
 * least that many bytes can be written without blocking.
 */
size_t nostatic_i2c_get_write_available(i2c_inst_t * i2c) {
    return i2c_get_write_available(i2c);
}


/*! \brief Determine number of bytes received
 *  \ingroup hardware_i2c
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \return 0 if no data available, if return is nonzero at
 * least that many bytes can be read without blocking.
 */
size_t nostatic_i2c_get_read_available(i2c_inst_t * i2c) {
    return i2c_get_read_available(i2c);
}


/*! \brief Write direct to TX FIFO
 *  \ingroup hardware_i2c
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \param src Data to send
 * \param len Number of bytes to send
 *
 * Writes directly to the I2C TX FIFO which is mainly useful for
 * slave-mode operation.
 */
void nostatic_i2c_write_raw_blocking(i2c_inst_t * i2c, const uint8_t * src, size_t len) {
    return i2c_write_raw_blocking(i2c, src, len);
}


/*! \brief Read direct from RX FIFO
 *  \ingroup hardware_i2c
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \param dst Buffer to accept data
 * \param len Number of bytes to read
 *
 * Reads directly from the I2C RX FIFO which is mainly useful for
 * slave-mode operation.
 */
void nostatic_i2c_read_raw_blocking(i2c_inst_t * i2c, uint8_t * dst, size_t len) {
    return i2c_read_raw_blocking(i2c, dst, len);
}


/*! \brief Return the DREQ to use for pacing transfers to/from a particular I2C instance
 *  \ingroup hardware_i2c
 *
 * \param i2c Either \ref i2c0 or \ref i2c1
 * \param is_tx true for sending data to the I2C instance, false for receiving data from the I2C instance
 */
uint nostatic_i2c_get_dreq(i2c_inst_t * i2c, bool is_tx) {
    return i2c_get_dreq(i2c, is_tx);
}


/*! \brief Clear a specific interrupt on the executing core
 *  \ingroup hardware_irq
 *
 * This method is only useful for "software" IRQs that are not connected to hardware (i.e. IRQs 26-31)
 * as the the NVIC always reflects the current state of the IRQ state of the hardware for hardware IRQs, and clearing
 * of the IRQ state of the hardware is performed via the hardware's registers instead.
 *
 * \param int_num Interrupt number \ref interrupt_nums
 */
void nostatic_irq_clear(uint int_num) {
    return irq_clear(int_num);
}


/*! \brief Set specified GPIO to be pulled up.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
void nostatic_gpio_pull_up(uint gpio) {
    return gpio_pull_up(gpio);
}


/*! \brief Determine if the specified GPIO is pulled up.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the GPIO is pulled up
 */
bool nostatic_gpio_is_pulled_up(uint gpio) {
    return gpio_is_pulled_up(gpio);
}


/*! \brief Set specified GPIO to be pulled down.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
void nostatic_gpio_pull_down(uint gpio) {
    return gpio_pull_down(gpio);
}


/*! \brief Determine if the specified GPIO is pulled down.
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the GPIO is pulled down
 */
bool nostatic_gpio_is_pulled_down(uint gpio) {
    return gpio_is_pulled_down(gpio);
}


/*! \brief Disable pulls on specified GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 */
void nostatic_gpio_disable_pulls(uint gpio) {
    return gpio_disable_pulls(gpio);
}


/*! \brief Return the current interrupt status (pending events) for the given GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return Bitmask of events that are currently pending for the GPIO. See \ref gpio_irq_level for details.
 * \sa gpio_acknowledge_irq
 */
uint32_t nostatic_gpio_get_irq_event_mask(uint gpio) {
    return gpio_get_irq_event_mask(gpio);
}


/*! \brief Adds a raw GPIO IRQ handler for a specific GPIO on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default callback. The order
 * relative to the default callback can be controlled via the order_priority parameter(the default callback has the priority
 * \ref GPIO_IRQ_CALLBACK_ORDER_PRIORITY which defaults to the lowest priority with the intention of it running last).
 *
 * This method adds such a callback, and disables the "default" callback for the specified GPIO.
 *
 * \note Multiple raw handlers should not be added for the same GPIO, and this method will assert if you attempt to.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio the GPIO number that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 * @param order_priority the priority order to determine the relative position of the handler in the list of GPIO IRQ handlers for this core.
 */
void nostatic_gpio_add_raw_irq_handler_with_order_priority(uint gpio, irq_handler_t handler, uint8_t order_priority) {
    return gpio_add_raw_irq_handler_with_order_priority(gpio, handler, order_priority);
}


/*! \brief Adds a raw GPIO IRQ handler for a specific GPIO on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method adds such a callback, and disables the "default" callback for the specified GPIO.
 *
 * \note Multiple raw handlers should not be added for the same GPIO, and this method will assert if you attempt to.
 *
 * A raw handler should check for whichever GPIOs and events it handles, and acknowledge them itself; it might look something like:
 *
 * \code{.c}
 * void my_irq_handler(void) {
 *     if (gpio_get_irq_event_mask(my_gpio_num) & my_gpio_event_mask) {
 *        gpio_acknowledge_irq(my_gpio_num, my_gpio_event_mask);
 *       // handle the IRQ
 *     }
 * }
 * \endcode
 *
 * @param gpio the GPIO number that will no longer be passed to the default callback for this core
 * @param handler the handler to add to the list of GPIO IRQ handlers for this core
 */
void nostatic_gpio_add_raw_irq_handler(uint gpio, irq_handler_t handler) {
    return gpio_add_raw_irq_handler(gpio, handler);
}


/*! \brief Removes a raw GPIO IRQ handler for the specified GPIO on the current core
 *  \ingroup hardware_gpio
 *
 * In addition to the default mechanism of a single GPIO IRQ event callback per core (see \ref gpio_set_irq_callback),
 * it is possible to add explicit GPIO IRQ handlers which are called independent of the default event callback.
 *
 * This method removes such a callback, and enables the "default" callback for the specified GPIO.
 *
 * @param gpio the GPIO number that will now be passed to the default callback for this core
 * @param handler the handler to remove from the list of GPIO IRQ handlers for this core
 */
void nostatic_gpio_remove_raw_irq_handler(uint gpio, irq_handler_t handler) {
    return gpio_remove_raw_irq_handler(gpio, handler);
}


/*! \brief Get state of a single specified GPIO
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return Current state of the GPIO. 0 for low, non-zero for high
 */
bool nostatic_gpio_get(uint gpio) {
    return gpio_get(gpio);
}


/*! \brief Get raw value of all GPIOs
 *  \ingroup hardware_gpio
 *
 * \return Bitmask of raw GPIO values, as bits 0-29
 */
uint32_t nostatic_gpio_get_all() {
    return gpio_get_all();
}


/*! \brief Drive high every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to set, as bits 0-29
 */
void nostatic_gpio_set_mask(uint32_t mask) {
    return gpio_set_mask(mask);
}


/*! \brief Drive low every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to clear, as bits 0-29
 */
void nostatic_gpio_clr_mask(uint32_t mask) {
    return gpio_clr_mask(mask);
}


/*! \brief Toggle every GPIO appearing in mask
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to toggle, as bits 0-29
 */
void nostatic_gpio_xor_mask(uint32_t mask) {
    return gpio_xor_mask(mask);
}


/*! \brief Drive GPIO high/low depending on parameters
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO values to change, as bits 0-29
 * \param value Value to set
 *
 * For each 1 bit in \p mask, drive that pin to the value given by
 * corresponding bit in \p value, leaving other pins unchanged.
 * Since this uses the TOGL alias, it is concurrency-safe with e.g. an IRQ
 * bashing different pins from the same core.
 */
void nostatic_gpio_put_masked(uint32_t mask, uint32_t value) {
    return gpio_put_masked(mask, value);
}


/*! \brief Drive all pins simultaneously
 *  \ingroup hardware_gpio
 *
 * \param value Bitmask of GPIO values to change, as bits 0-29
 */
void nostatic_gpio_put_all(uint32_t value) {
    return gpio_put_all(value);
}


/*! \brief Drive a single GPIO high/low
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param value If false clear the GPIO, otherwise set it.
 */
void nostatic_gpio_put(uint gpio, bool value) {
    return gpio_put(gpio, value);
}


/*! \brief Determine whether a GPIO is currently driven high or low
 *  \ingroup hardware_gpio
 *
 * This function returns the high/low output level most recently assigned to a
 * GPIO via gpio_put() or similar. This is the value that is presented outward
 * to the IO muxing, *not* the input level back from the pad (which can be
 * read using gpio_get()).
 *
 * To avoid races, this function must not be used for read-modify-write
 * sequences when driving GPIOs -- instead functions like gpio_put() should be
 * used to atomically update GPIOs. This accessor is intended for debug use
 * only.
 *
 * \param gpio GPIO number
 * \return true if the GPIO output level is high, false if low.
 */
bool nostatic_gpio_get_out_level(uint gpio) {
    return gpio_get_out_level(gpio);
}


/*! \brief Set a number of GPIOs to output
 *  \ingroup hardware_gpio
 *
 * Switch all GPIOs in "mask" to output
 *
 * \param mask Bitmask of GPIO to set to output, as bits 0-29
 */
void nostatic_gpio_set_dir_out_masked(uint32_t mask) {
    return gpio_set_dir_out_masked(mask);
}


/*! \brief Set a number of GPIOs to input
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input, as bits 0-29
 */
void nostatic_gpio_set_dir_in_masked(uint32_t mask) {
    return gpio_set_dir_in_masked(mask);
}


/*! \brief Set multiple GPIO directions
 *  \ingroup hardware_gpio
 *
 * \param mask Bitmask of GPIO to set to input, as bits 0-29
 * \param value Values to set
 *
 * For each 1 bit in "mask", switch that pin to the direction given by
 * corresponding bit in "value", leaving other pins unchanged.
 * E.g. gpio_set_dir_masked(0x3, 0x2); -> set pin 0 to input, pin 1 to output,
 * simultaneously.
 */
void nostatic_gpio_set_dir_masked(uint32_t mask, uint32_t value) {
    return gpio_set_dir_masked(mask, value);
}


/*! \brief Set direction of all pins simultaneously.
 *  \ingroup hardware_gpio
 *
 * \param values individual settings for each gpio; for GPIO N, bit N is 1 for out, 0 for in
 */
void nostatic_gpio_set_dir_all_bits(uint32_t values) {
    return gpio_set_dir_all_bits(values);
}


/*! \brief Set a single GPIO direction
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \param out true for out, false for in
 */
void nostatic_gpio_set_dir(uint gpio, bool out) {
    return gpio_set_dir(gpio, out);
}


/*! \brief Check if a specific GPIO direction is OUT
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return true if the direction for the pin is OUT
 */
bool nostatic_gpio_is_dir_out(uint gpio) {
    return gpio_is_dir_out(gpio);
}


/*! \brief Get a specific GPIO direction
 *  \ingroup hardware_gpio
 *
 * \param gpio GPIO number
 * \return 1 for out, 0 for in
 */
uint nostatic_gpio_get_dir(uint gpio) {
    return gpio_get_dir(gpio);
}


/*! \brief Set the interpolator shift value
 *  \ingroup interp_config
 *
 * Sets the number of bits the accumulator is shifted before masking, on each iteration.
 *
 * \param c Pointer to an interpolator config
 * \param shift Number of bits
 */
void nostatic_interp_config_set_shift(interp_config * c, uint shift) {
    return interp_config_set_shift(c, shift);
}


/*! \brief Set the interpolator mask range
 *  \ingroup interp_config
 *
 * Sets the range of bits (least to most) that are allowed to pass through the interpolator
 *
 * \param c Pointer to interpolation config
 * \param mask_lsb The least significant bit allowed to pass
 * \param mask_msb The most significant bit allowed to pass
 */
void nostatic_interp_config_set_mask(interp_config * c, uint mask_lsb, uint mask_msb) {
    return interp_config_set_mask(c, mask_lsb, mask_msb);
}


/*! \brief Enable cross input
 *  \ingroup interp_config
 *
 *  Allows feeding of the accumulator content from the other lane back in to this lanes shift+mask hardware.
 *  This will take effect even if the interp_config_set_add_raw option is set as the cross input mux is before the
 *  shift+mask bypass
 *
 * \param c Pointer to interpolation config
 * \param cross_input If true, enable the cross input.
 */
void nostatic_interp_config_set_cross_input(interp_config * c, bool cross_input) {
    return interp_config_set_cross_input(c, cross_input);
}


/*! \brief Enable cross results
 *  \ingroup interp_config
 *
 *  Allows feeding of the other lane’s result into this lane’s accumulator on a POP operation.
 *
 * \param c Pointer to interpolation config
 * \param cross_result If true, enables the cross result
 */
void nostatic_interp_config_set_cross_result(interp_config * c, bool cross_result) {
    return interp_config_set_cross_result(c, cross_result);
}


/*! \brief Set sign extension
 *  \ingroup interp_config
 *
 * Enables signed mode, where the shifted and masked accumulator value is sign-extended to 32 bits
 * before adding to BASE1, and LANE1 PEEK/POP results appear extended to 32 bits when read by processor.
 *
 * \param c Pointer to interpolation config
 * \param  _signed If true, enables sign extension
 */
void nostatic_interp_config_set_signed(interp_config * c, bool _signed) {
    return interp_config_set_signed(c, _signed);
}


/*! \brief Set raw add option
 *  \ingroup interp_config
 *
 * When enabled, mask + shift is bypassed for LANE0 result. This does not affect the FULL result.
 *
 * \param c Pointer to interpolation config
 * \param add_raw If true, enable raw add option.
 */
void nostatic_interp_config_set_add_raw(interp_config * c, bool add_raw) {
    return interp_config_set_add_raw(c, add_raw);
}


/*! \brief Set interpolator clamp mode (Interpolator 1 only)
 *  \ingroup interp_config
 *
 * Only present on INTERP1 on each core. If CLAMP mode is enabled:
 * - LANE0 result is a shifted and masked ACCUM0, clamped by a lower bound of BASE0 and an upper bound of BASE1.
 * - Signedness of these comparisons is determined by LANE0_CTRL_SIGNED
 *
 * \param c Pointer to interpolation config
 * \param clamp Set true to enable clamp mode
 */
void nostatic_interp_config_set_clamp(interp_config * c, bool clamp) {
    return interp_config_set_clamp(c, clamp);
}


/*! \brief Set interpolator Force bits
 *  \ingroup interp_config
 *
 * ORed into bits 29:28 of the lane result presented to the processor on the bus.
 *
 * No effect on the internal 32-bit datapath. Handy for using a lane to generate sequence
 * of pointers into flash or SRAM
 *
 * \param c Pointer to interpolation config
 * \param bits Sets the force bits to that specified. Range 0-3 (two bits)
 */
void nostatic_interp_config_set_force_bits(interp_config * c, uint bits) {
    return interp_config_set_force_bits(c, bits);
}


/*! \brief Get a default configuration
 *  \ingroup interp_config
 *
 * \return A default interpolation configuration
 */
interp_config nostatic_interp_default_config() {
    return interp_default_config();
}


/*! \brief Send configuration to a lane
 *  \ingroup interp_config
 *
 * If an invalid configuration is specified (ie a lane specific item is set on wrong lane),
 * depending on setup this function can panic.
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane to set
 * \param config Pointer to interpolation config
 */
void nostatic_interp_set_config(interp_hw_t * interp, uint lane, interp_config * config) {
    return interp_set_config(interp, lane, config);
}


/*! \brief Directly set the force bits on a specified lane
 *  \ingroup hardware_interp
 *
 * These bits are ORed into bits 29:28 of the lane result presented to the processor on the bus.
 * There is no effect on the internal 32-bit datapath.
 *
 * Useful for using a lane to generate sequence of pointers into flash or SRAM, saving a subsequent
 * OR or add operation.
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane to set
 * \param bits The bits to set (bits 0 and 1, value range 0-3)
 */
void nostatic_interp_set_force_bits(interp_hw_t * interp, uint lane, uint bits) {
    return interp_set_force_bits(interp, lane, bits);
}


/*! \brief Sets the interpolator base register by lane
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1 or 2
 * \param val The value to apply to the register
 */
void nostatic_interp_set_base(interp_hw_t * interp, uint lane, uint32_t val) {
    return interp_set_base(interp, lane, val);
}


/*! \brief Gets the content of interpolator base register by lane
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1 or 2
 * \return  The current content of the lane base register
 */
uint32_t nostatic_interp_get_base(interp_hw_t * interp, uint lane) {
    return interp_get_base(interp, lane);
}


/*! \brief Sets the interpolator base registers simultaneously
 *  \ingroup hardware_interp
 *
 *  The lower 16 bits go to BASE0, upper bits to BASE1 simultaneously.
 *  Each half is sign-extended to 32 bits if that lane’s SIGNED flag is set.
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param val The value to apply to the register
 */
void nostatic_interp_set_base_both(interp_hw_t * interp, uint32_t val) {
    return interp_set_base_both(interp, val);
}


/*! \brief Sets the interpolator accumulator register by lane
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1
 * \param val The value to apply to the register
 */
void nostatic_interp_set_accumulator(interp_hw_t * interp, uint lane, uint32_t val) {
    return interp_set_accumulator(interp, lane, val);
}


/*! \brief Gets the content of the interpolator accumulator register by lane
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1
 * \return The current content of the register
 */
uint32_t nostatic_interp_get_accumulator(interp_hw_t * interp, uint lane) {
    return interp_get_accumulator(interp, lane);
}


/*! \brief Read lane result, and write lane results to both accumulators to update the interpolator
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1
 * \return The content of the lane result register
 */
uint32_t nostatic_interp_pop_lane_result(interp_hw_t * interp, uint lane) {
    return interp_pop_lane_result(interp, lane);
}


/*! \brief Read lane result
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1
 * \return The content of the lane result register
 */
uint32_t nostatic_interp_peek_lane_result(interp_hw_t * interp, uint lane) {
    return interp_peek_lane_result(interp, lane);
}


/*! \brief Read lane result, and write lane results to both accumulators to update the interpolator
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \return The content of the FULL register
 */
uint32_t nostatic_interp_pop_full_result(interp_hw_t * interp) {
    return interp_pop_full_result(interp);
}


/*! \brief Read lane result
 *  \ingroup hardware_interp
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \return The content of the FULL register
 */
uint32_t nostatic_interp_peek_full_result(interp_hw_t * interp) {
    return interp_peek_full_result(interp);
}


/*! \brief Add to accumulator
 *  \ingroup hardware_interp
 *
 * Atomically add the specified value to the accumulator on the specified lane
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1
 * \param val Value to add
 * \return The content of the FULL register
 */
void nostatic_interp_add_accumulater(interp_hw_t * interp, uint lane, uint32_t val) {
    return interp_add_accumulater(interp, lane, val);
}


/*! \brief Get raw lane value
 *  \ingroup hardware_interp
 *
 * Returns the raw shift and mask value from the specified lane, BASE0 is NOT added
 *
 * \param interp Interpolator instance, interp0 or interp1.
 * \param lane The lane number, 0 or 1
 * \return The raw shift/mask value
 */
uint32_t nostatic_interp_get_raw(interp_hw_t * interp, uint lane) {
    return interp_get_raw(interp, lane);
}


/*! \brief Return a bootrom lookup code based on two ASCII characters
 * \ingroup pico_bootrom
 *
 * These codes are uses to lookup data or function addresses in the bootrom
 *
 * \param c1 the first character
 * \param c2 the second character
 * \return the 'code' to use in rom_func_lookup() or rom_data_lookup()
 */
uint32_t nostatic_rom_table_code(uint8_t c1, uint8_t c2) {
    return rom_table_code(c1, c2);
}


/*!
 * \brief Reboot the device into BOOTSEL mode
 * \ingroup pico_bootrom
 *
 * This function reboots the device into the BOOTSEL mode ('usb boot").
 *
 * Facilities are provided to enable an "activity light" via GPIO attached LED for the USB Mass Storage Device,
 * and to limit the USB interfaces exposed.
 *
 * \param usb_activity_gpio_pin_mask 0 No pins are used as per a cold boot. Otherwise a single bit set indicating which
 *                               GPIO pin should be set to output and raised whenever there is mass storage activity
 *                               from the host.
 * \param disable_interface_mask value to control exposed interfaces
 *  - 0 To enable both interfaces (as per a cold boot)
 *  - 1 To disable the USB Mass Storage Interface
 *  - 2 To disable the USB PICOBOOT Interface
 */
void nostatic_reset_usb_boot(uint32_t usb_activity_gpio_pin_mask, uint32_t disable_interface_mask) {
    return reset_usb_boot(usb_activity_gpio_pin_mask, disable_interface_mask);
}


/*! \brief Convert UART instance to hardware instance number
 *  \ingroup hardware_uart
 *
 * \param uart UART instance
 * \return Number of UART, 0 or 1.
 */
uint nostatic_uart_get_index(uart_inst_t * uart) {
    return uart_get_index(uart);
}


/*! \brief Set UART flow control CTS/RTS
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param cts If true enable flow control of TX  by clear-to-send input
 * \param rts If true enable assertion of request-to-send output by RX flow control
 */
void nostatic_uart_set_hw_flow(uart_inst_t * uart, bool cts, bool rts) {
    return uart_set_hw_flow(uart, cts, rts);
}


/*! \brief Set UART data format
 *  \ingroup hardware_uart
 *
 * Configure the data format (bits etc() for the UART
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param data_bits Number of bits of data. 5..8
 * \param stop_bits Number of stop bits 1..2
 * \param parity Parity option.
 */
void nostatic_uart_set_format(uart_inst_t * uart, uint data_bits, uint stop_bits, uart_parity_t parity) {
    return uart_set_format(uart, data_bits, stop_bits, parity);
}


/*! \brief Setup UART interrupts
 *  \ingroup hardware_uart
 *
 * Enable the UART's interrupt output. An interrupt handler will need to be installed prior to calling
 * this function.
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param rx_has_data If true an interrupt will be fired when the RX FIFO contains data.
 * \param tx_needs_data If true an interrupt will be fired when the TX FIFO needs data.
 */
void nostatic_uart_set_irq_enables(uart_inst_t * uart, bool rx_has_data, bool tx_needs_data) {
    return uart_set_irq_enables(uart, rx_has_data, tx_needs_data);
}


/*! \brief Test if specific UART is enabled
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \return true if the UART is enabled
 */
bool nostatic_uart_is_enabled(uart_inst_t * uart) {
    return uart_is_enabled(uart);
}


/*! \brief Enable/Disable the FIFOs on specified UART
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param enabled true to enable FIFO (default), false to disable
 */
void nostatic_uart_set_fifo_enabled(uart_inst_t * uart, bool enabled) {
    return uart_set_fifo_enabled(uart, enabled);
}


/*! \brief Determine if space is available in the TX FIFO
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \return false if no space available, true otherwise
 */
bool nostatic_uart_is_writable(uart_inst_t * uart) {
    return uart_is_writable(uart);
}


/*! \brief Wait for the UART TX fifo to be drained
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 */
void nostatic_uart_tx_wait_blocking(uart_inst_t * uart) {
    return uart_tx_wait_blocking(uart);
}


/*! \brief Determine whether data is waiting in the RX FIFO
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \return 0 if no data available, otherwise the number of bytes, at least, that can be read
 *
 * \note HW limitations mean this function will return either 0 or 1.
 */
bool nostatic_uart_is_readable(uart_inst_t * uart) {
    return uart_is_readable(uart);
}


/*! \brief  Write to the UART for transmission.
 *  \ingroup hardware_uart
 *
 * This function will block until all the data has been sent to the UART
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param src The bytes to send
 * \param len The number of bytes to send
 */
void nostatic_uart_write_blocking(uart_inst_t * uart, const uint8_t * src, size_t len) {
    return uart_write_blocking(uart, src, len);
}


/*! \brief  Read from the UART
 *  \ingroup hardware_uart
 *
 * This function will block until all the data has been received from the UART
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param dst Buffer to accept received bytes
 * \param len The number of bytes to receive.
 */
void nostatic_uart_read_blocking(uart_inst_t * uart, uint8_t * dst, size_t len) {
    return uart_read_blocking(uart, dst, len);
}


/*! \brief  Write single character to UART for transmission.
 *  \ingroup hardware_uart
 *
 * This function will block until the entire character has been sent
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param c The character  to send
 */
void nostatic_uart_putc_raw(uart_inst_t * uart, char c) {
    return uart_putc_raw(uart, c);
}


/*! \brief  Write single character to UART for transmission, with optional CR/LF conversions
 *  \ingroup hardware_uart
 *
 * This function will block until the character has been sent
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param c The character  to send
 */
void nostatic_uart_putc(uart_inst_t * uart, char c) {
    return uart_putc(uart, c);
}


/*! \brief  Write string to UART for transmission, doing any CR/LF conversions
 *  \ingroup hardware_uart
 *
 * This function will block until the entire string has been sent
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param s The null terminated string to send
 */
void nostatic_uart_puts(uart_inst_t * uart, const char * s) {
    return uart_puts(uart, s);
}


/*! \brief  Read a single character to UART
 *  \ingroup hardware_uart
 *
 * This function will block until the character has been read
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \return The character read.
 */
char nostatic_uart_getc(uart_inst_t * uart) {
    return uart_getc(uart);
}


/*! \brief Assert a break condition on the UART transmission.
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param en Assert break condition (TX held low) if true. Clear break condition if false.
 */
void nostatic_uart_set_break(uart_inst_t * uart, bool en) {
    return uart_set_break(uart, en);
}


/*! \brief Wait for the default UART's TX FIFO to be drained
 *  \ingroup hardware_uart
 */
void nostatic_uart_default_tx_wait_blocking() {
    return uart_default_tx_wait_blocking();
}


/*! \brief Return the DREQ to use for pacing transfers to/from a particular UART instance
 *  \ingroup hardware_uart
 *
 * \param uart UART instance. \ref uart0 or \ref uart1
 * \param is_tx true for sending data to the UART instance, false for receiving data from the UART instance
 */
uint nostatic_uart_get_dreq(uart_inst_t * uart, bool is_tx) {
    return uart_get_dreq(uart, is_tx);
}


/*! \brief Check if the specified timestamp has been reached
 *  \ingroup hardware_timer
 *
 * \param t Absolute time to compare against current time
 * \return true if it is now after the specified timestamp
 */
bool nostatic_time_reached(absolute_time_t t) {
    return time_reached(t);
}


/*! \brief Set the 'out' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * Can overlap with the 'in', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param out_base 0-31 First pin to set as output
 * \param out_count 0-32 Number of pins to set.
 */
void nostatic_sm_config_set_out_pins(pio_sm_config * c, uint out_base, uint out_count) {
    return sm_config_set_out_pins(c, out_base, out_count);
}


/*! \brief Set the 'set' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * Can overlap with the 'in', 'out' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param set_base 0-31 First pin to set as
 * \param set_count 0-5 Number of pins to set.
 */
void nostatic_sm_config_set_set_pins(pio_sm_config * c, uint set_base, uint set_count) {
    return sm_config_set_set_pins(c, set_base, set_count);
}


/*! \brief Set the 'in' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * Can overlap with the 'out', 'set' and 'sideset' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param in_base 0-31 First pin to use as input
 */
void nostatic_sm_config_set_in_pins(pio_sm_config * c, uint in_base) {
    return sm_config_set_in_pins(c, in_base);
}


/*! \brief Set the 'sideset' pins in a state machine configuration
 *  \ingroup sm_config
 *
 * Can overlap with the 'in', 'out' and 'set' pins
 *
 * \param c Pointer to the configuration structure to modify
 * \param sideset_base 0-31 base pin for 'side set'
 */
void nostatic_sm_config_set_sideset_pins(pio_sm_config * c, uint sideset_base) {
    return sm_config_set_sideset_pins(c, sideset_base);
}


/*! \brief Set the 'sideset' options in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param bit_count Number of bits to steal from delay field in the instruction for use of side set (max 5)
 * \param optional True if the topmost side set bit is used as a flag for whether to apply side set on that instruction
 * \param pindirs True if the side set affects pin directions rather than values
 */
void nostatic_sm_config_set_sideset(pio_sm_config * c, uint bit_count, bool optional, bool pindirs) {
    return sm_config_set_sideset(c, bit_count, optional, pindirs);
}


/*! \brief Set the state machine clock divider (from integer and fractional parts - 16:8) in a state machine configuration
 *  \ingroup sm_config
 *
 * The clock divider can slow the state machine's execution to some rate below
 * the system clock frequency, by enabling the state machine on some cycles
 * but not on others, in a regular pattern. This can be used to generate e.g.
 * a given UART baud rate. See the datasheet for further detail.
 *
 * \param c Pointer to the configuration structure to modify
 * \param div_int Integer part of the divisor
 * \param div_frac Fractional part in 1/256ths
 * \sa sm_config_set_clkdiv()
 */
void nostatic_sm_config_set_clkdiv_int_frac(pio_sm_config * c, uint16_t div_int, uint8_t div_frac) {
    return sm_config_set_clkdiv_int_frac(c, div_int, div_frac);
}


/*! \brief Set the state machine clock divider (from a floating point value) in a state machine configuration
 *  \ingroup sm_config
 *
 * The clock divider slows the state machine's execution by masking the
 * system clock on some cycles, in a repeating pattern, so that the state
 * machine does not advance. Effectively this produces a slower clock for the
 * state machine to run from, which can be used to generate e.g. a particular
 * UART baud rate. See the datasheet for further detail.
 *
 * \param c Pointer to the configuration structure to modify
 * \param div The fractional divisor to be set. 1 for full speed. An integer clock divisor of n
 *  will cause the state machine to run 1 cycle in every n.
 *  Note that for small n, the jitter introduced by a fractional divider (e.g. 2.5) may be unacceptable
 *  although it will depend on the use case.
 */
void nostatic_sm_config_set_clkdiv(pio_sm_config * c, float div) {
    return sm_config_set_clkdiv(c, div);
}


/*! \brief Set the wrap addresses in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param wrap_target the instruction memory address to wrap to
 * \param wrap        the instruction memory address after which to set the program counter to wrap_target
 *                    if the instruction does not itself update the program_counter
 */
void nostatic_sm_config_set_wrap(pio_sm_config * c, uint wrap_target, uint wrap) {
    return sm_config_set_wrap(c, wrap_target, wrap);
}


/*! \brief Set the 'jmp' pin in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param pin The raw GPIO pin number to use as the source for a `jmp pin` instruction
 */
void nostatic_sm_config_set_jmp_pin(pio_sm_config * c, uint pin) {
    return sm_config_set_jmp_pin(c, pin);
}


/*! \brief Setup 'in' shifting parameters in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param shift_right true to shift ISR to right, false to shift ISR to left
 * \param autopush whether autopush is enabled
 * \param push_threshold threshold in bits to shift in before auto/conditional re-pushing of the ISR
 */
void nostatic_sm_config_set_in_shift(pio_sm_config * c, bool shift_right, bool autopush, uint push_threshold) {
    return sm_config_set_in_shift(c, shift_right, autopush, push_threshold);
}


/*! \brief Setup 'out' shifting parameters in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param shift_right true to shift OSR to right, false to shift OSR to left
 * \param autopull whether autopull is enabled
 * \param pull_threshold threshold in bits to shift out before auto/conditional re-pulling of the OSR
 */
void nostatic_sm_config_set_out_shift(pio_sm_config * c, bool shift_right, bool autopull, uint pull_threshold) {
    return sm_config_set_out_shift(c, shift_right, autopull, pull_threshold);
}


/*! \brief Setup the FIFO joining in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param join Specifies the join type. \see enum pio_fifo_join
 */
void nostatic_sm_config_set_fifo_join(pio_sm_config * c, enum pio_fifo_join join) {
    return sm_config_set_fifo_join(c, join);
}


/*! \brief Set special 'out' operations in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param sticky to enable 'sticky' output (i.e. re-asserting most recent OUT/SET pin values on subsequent cycles)
 * \param has_enable_pin true to enable auxiliary OUT enable pin
 * \param enable_pin_index pin index for auxiliary OUT enable
 */
void nostatic_sm_config_set_out_special(pio_sm_config * c, bool sticky, bool has_enable_pin, uint enable_pin_index) {
    return sm_config_set_out_special(c, sticky, has_enable_pin, enable_pin_index);
}


/*! \brief Set source for 'mov status' in a state machine configuration
 *  \ingroup sm_config
 *
 * \param c Pointer to the configuration structure to modify
 * \param status_sel the status operation selector. \see enum pio_mov_status_type
 * \param status_n parameter for the mov status operation (currently a bit count)
 */
void nostatic_sm_config_set_mov_status(pio_sm_config * c, enum pio_mov_status_type status_sel, uint status_n) {
    return sm_config_set_mov_status(c, status_sel, status_n);
}


/*! \brief  Get the default state machine configuration
 *  \ingroup sm_config
 *
 * Setting | Default
 * --------|--------
 * Out Pins | 32 starting at 0
 * Set Pins | 0 starting at 0
 * In Pins (base) | 0
 * Side Set Pins (base) | 0
 * Side Set | disabled
 * Wrap | wrap=31, wrap_to=0
 * In Shift | shift_direction=right, autopush=false, push_threshold=32
 * Out Shift | shift_direction=right, autopull=false, pull_threshold=32
 * Jmp Pin | 0
 * Out Special | sticky=false, has_enable_pin=false, enable_pin_index=0
 * Mov Status | status_sel=STATUS_TX_LESSTHAN, n=0
 *
 * \return the default state machine configuration which can then be modified.
 */
pio_sm_config nostatic_pio_get_default_sm_config() {
    return pio_get_default_sm_config();
}


/*! \brief Return the instance number of a PIO instance
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \return the PIO instance number (either 0 or 1)
 */
uint nostatic_pio_get_index(PIO pio) {
    return pio_get_index(pio);
}


/*! \brief Setup the function select for a GPIO to use output from the given PIO instance
 *  \ingroup hardware_pio
 *
 * PIO appears as an alternate function in the GPIO muxing, just like an SPI
 * or UART. This function configures that multiplexing to connect a given PIO
 * instance to a GPIO. Note that this is not necessary for a state machine to
 * be able to read the *input* value from a GPIO, but only for it to set the
 * output value or output enable.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param pin the GPIO pin whose function select to set
 */
void nostatic_pio_gpio_init(PIO pio, uint pin) {
    return pio_gpio_init(pio, pin);
}


/*! \brief Return the DREQ to use for pacing transfers to/from a particular state machine FIFO
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param is_tx true for sending data to the state machine, false for receiving data from the state machine
 */
uint nostatic_pio_get_dreq(PIO pio, uint sm, bool is_tx) {
    return pio_get_dreq(pio, sm, is_tx);
}


/*! \brief Enable or disable a PIO state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param enabled true to enable the state machine; false to disable
 */
void nostatic_pio_sm_set_enabled(PIO pio, uint sm, bool enabled) {
    return pio_sm_set_enabled(pio, sm, enabled);
}


/*! \brief Enable or disable multiple PIO state machines
 *  \ingroup hardware_pio
 *
 * Note that this method just sets the enabled state of the state machine;
 * if now enabled they continue exactly from where they left off.
 *
 * \see pio_enable_sm_mask_in_sync() if you wish to enable multiple state machines
 * and ensure their clock dividers are in sync.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 * \param enabled true to enable the state machines; false to disable
 */
void nostatic_pio_set_sm_mask_enabled(PIO pio, uint32_t mask, bool enabled) {
    return pio_set_sm_mask_enabled(pio, mask, enabled);
}


/*! \brief Restart a state machine with a known state
 *  \ingroup hardware_pio
 *
 * This method clears the ISR, shift counters, clock divider counter
 * pin write flags, delay counter, latched EXEC instruction, and IRQ wait condition.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
void nostatic_pio_sm_restart(PIO pio, uint sm) {
    return pio_sm_restart(pio, sm);
}


/*! \brief Restart multiple state machine with a known state
 *  \ingroup hardware_pio
 *
 * This method clears the ISR, shift counters, clock divider counter
 * pin write flags, delay counter, latched EXEC instruction, and IRQ wait condition.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 */
void nostatic_pio_restart_sm_mask(PIO pio, uint32_t mask) {
    return pio_restart_sm_mask(pio, mask);
}


/*! \brief Restart a state machine's clock divider from a phase of 0
 *  \ingroup hardware_pio
 *
 * Each state machine's clock divider is a free-running piece of hardware,
 * that generates a pattern of clock enable pulses for the state machine,
 * based *only* on the configured integer/fractional divisor. The pattern of
 * running/halted cycles slows the state machine's execution to some
 * controlled rate.
 *
 * This function clears the divider's integer and fractional phase
 * accumulators so that it restarts this pattern from the beginning. It is
 * called automatically by pio_sm_init() but can also be called at a later
 * time, when you enable the state machine, to ensure precisely consistent
 * timing each time you load and run a given PIO program.
 *
 * More commonly this hardware mechanism is used to synchronise the execution
 * clocks of multiple state machines -- see pio_clkdiv_restart_sm_mask().
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
void nostatic_pio_sm_clkdiv_restart(PIO pio, uint sm) {
    return pio_sm_clkdiv_restart(pio, sm);
}


/*! \brief Restart multiple state machines' clock dividers from a phase of 0.
 *  \ingroup hardware_pio
 *
 * Each state machine's clock divider is a free-running piece of hardware,
 * that generates a pattern of clock enable pulses for the state machine,
 * based *only* on the configured integer/fractional divisor. The pattern of
 * running/halted cycles slows the state machine's execution to some
 * controlled rate.
 *
 * This function simultaneously clears the integer and fractional phase
 * accumulators of multiple state machines' clock dividers. If these state
 * machines all have the same integer and fractional divisors configured,
 * their clock dividers will run in precise deterministic lockstep from this
 * point.
 *
 * With their execution clocks synchronised in this way, it is then safe to
 * e.g. have multiple state machines performing a 'wait irq' on the same flag,
 * and all clear it on the same cycle.
 *
 * Also note that this function can be called whilst state machines are
 * running (e.g. if you have just changed the clock divisors of some state
 * machines and wish to resynchronise them), and that disabling a state
 * machine does not halt its clock divider: that is, if multiple state
 * machines have their clocks synchronised, you can safely disable and
 * reenable one of the state machines without losing synchronisation.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 */
void nostatic_pio_clkdiv_restart_sm_mask(PIO pio, uint32_t mask) {
    return pio_clkdiv_restart_sm_mask(pio, mask);
}


/*! \brief Enable multiple PIO state machines synchronizing their clock dividers
 *  \ingroup hardware_pio
 *
 * This is equivalent to calling both pio_set_sm_mask_enabled() and
 * pio_clkdiv_restart_sm_mask() on the *same* clock cycle. All state machines
 * specified by 'mask' are started simultaneously and, assuming they have the
 * same clock divisors, their divided clocks will stay precisely synchronised.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param mask bit mask of state machine indexes to modify the enabled state of
 */
void nostatic_pio_enable_sm_mask_in_sync(PIO pio, uint32_t mask) {
    return pio_enable_sm_mask_in_sync(pio, mask);
}


/*! \brief  Enable/Disable a single source on a PIO's IRQ 0
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param source the source number (see \ref pio_interrupt_source)
 * \param enabled true to enable IRQ 0 for the source, false to disable.
 */
void nostatic_pio_set_irq0_source_enabled(PIO pio, enum pio_interrupt_source source, bool enabled) {
    return pio_set_irq0_source_enabled(pio, source, enabled);
}


/*! \brief  Enable/Disable a single source on a PIO's IRQ 1
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param source the source number (see \ref pio_interrupt_source)
 * \param enabled true to enable IRQ 0 for the source, false to disable.
 */
void nostatic_pio_set_irq1_source_enabled(PIO pio, enum pio_interrupt_source source, bool enabled) {
    return pio_set_irq1_source_enabled(pio, source, enabled);
}


/*! \brief  Enable/Disable multiple sources on a PIO's IRQ 0
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param source_mask Mask of bits, one for each source number (see \ref pio_interrupt_source) to affect
 * \param enabled true to enable all the sources specified in the mask on IRQ 0, false to disable all the sources specified in the mask on IRQ 0
 */
void nostatic_pio_set_irq0_source_mask_enabled(PIO pio, uint32_t source_mask, bool enabled) {
    return pio_set_irq0_source_mask_enabled(pio, source_mask, enabled);
}


/*! \brief  Enable/Disable multiple sources on a PIO's IRQ 1
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param source_mask Mask of bits, one for each source number (see \ref pio_interrupt_source) to affect
 * \param enabled true to enable all the sources specified in the mask on IRQ 1, false to disable all the source specified in the mask on IRQ 1
 */
void nostatic_pio_set_irq1_source_mask_enabled(PIO pio, uint32_t source_mask, bool enabled) {
    return pio_set_irq1_source_mask_enabled(pio, source_mask, enabled);
}


/*! \brief  Enable/Disable a single source on a PIO's specified (0/1) IRQ index
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param irq_index the IRQ index; either 0 or 1
 * \param source the source number (see \ref pio_interrupt_source)
 * \param enabled true to enable the source on the specified IRQ, false to disable.
 */
void nostatic_pio_set_irqn_source_enabled(PIO pio, uint irq_index, enum pio_interrupt_source source, bool enabled) {
    return pio_set_irqn_source_enabled(pio, irq_index, source, enabled);
}


/*! \brief  Enable/Disable multiple sources on a PIO's specified (0/1) IRQ index
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param irq_index the IRQ index; either 0 or 1
 * \param source_mask Mask of bits, one for each source number (see \ref pio_interrupt_source) to affect
 * \param enabled true to enable all the sources specified in the mask on the specified IRQ, false to disable all the sources specified in the mask on the specified IRQ
 */
void nostatic_pio_set_irqn_source_mask_enabled(PIO pio, uint irq_index, uint32_t source_mask, bool enabled) {
    return pio_set_irqn_source_mask_enabled(pio, irq_index, source_mask, enabled);
}


/*! \brief  Determine if a particular PIO interrupt is set
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param pio_interrupt_num the PIO interrupt number 0-7
 * \return true if corresponding PIO interrupt is currently set
 */
bool nostatic_pio_interrupt_get(PIO pio, uint pio_interrupt_num) {
    return pio_interrupt_get(pio, pio_interrupt_num);
}


/*! \brief  Clear a particular PIO interrupt
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param pio_interrupt_num the PIO interrupt number 0-7
 */
void nostatic_pio_interrupt_clear(PIO pio, uint pio_interrupt_num) {
    return pio_interrupt_clear(pio, pio_interrupt_num);
}


/*! \brief Return the current program counter for a state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return the program counter
 */
uint8_t nostatic_pio_sm_get_pc(PIO pio, uint sm) {
    return pio_sm_get_pc(pio, sm);
}


/*! \brief Determine if an instruction set by pio_sm_exec() is stalled executing
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the executed instruction is still running (stalled)
 */
bool nostatic_pio_sm_is_exec_stalled(PIO pio, uint sm) {
    return pio_sm_is_exec_stalled(pio, sm);
}


/*! \brief Immediately execute an instruction on a state machine and wait for it to complete
 *  \ingroup hardware_pio
 *
 * This instruction is executed instead of the next instruction in the normal control flow on the state machine.
 * Subsequent calls to this method replace the previous executed
 * instruction if it is still running. \see pio_sm_is_exec_stalled() to see if an executed instruction
 * is still running (i.e. it is stalled on some condition)
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param instr the encoded PIO instruction
 */
void nostatic_pio_sm_exec_wait_blocking(PIO pio, uint sm, uint instr) {
    return pio_sm_exec_wait_blocking(pio, sm, instr);
}


/*! \brief Set the current wrap configuration for a state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param wrap_target the instruction memory address to wrap to
 * \param wrap        the instruction memory address after which to set the program counter to wrap_target
 *                    if the instruction does not itself update the program_counter
 */
void nostatic_pio_sm_set_wrap(PIO pio, uint sm, uint wrap_target, uint wrap) {
    return pio_sm_set_wrap(pio, sm, wrap_target, wrap);
}


/*! \brief Set the current 'out' pins for a state machine
 *  \ingroup hardware_pio
 *
 * Can overlap with the 'in', 'set' and 'sideset' pins
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param out_base 0-31 First pin to set as output
 * \param out_count 0-32 Number of pins to set.
 */
void nostatic_pio_sm_set_out_pins(PIO pio, uint sm, uint out_base, uint out_count) {
    return pio_sm_set_out_pins(pio, sm, out_base, out_count);
}


/*! \brief Set the current 'set' pins for a state machine
 *  \ingroup hardware_pio
 *
 * Can overlap with the 'in', 'out' and 'sideset' pins
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param set_base 0-31 First pin to set as
 * \param set_count 0-5 Number of pins to set.
 */
void nostatic_pio_sm_set_set_pins(PIO pio, uint sm, uint set_base, uint set_count) {
    return pio_sm_set_set_pins(pio, sm, set_base, set_count);
}


/*! \brief Set the current 'in' pins for a state machine
 *  \ingroup hardware_pio
 *
 * Can overlap with the 'out', 'set' and 'sideset' pins
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param in_base 0-31 First pin to use as input
 */
void nostatic_pio_sm_set_in_pins(PIO pio, uint sm, uint in_base) {
    return pio_sm_set_in_pins(pio, sm, in_base);
}


/*! \brief Set the current 'sideset' pins for a state machine
 *  \ingroup hardware_pio
 *
 * Can overlap with the 'in', 'out' and 'set' pins
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param sideset_base 0-31 base pin for 'side set'
 */
void nostatic_pio_sm_set_sideset_pins(PIO pio, uint sm, uint sideset_base) {
    return pio_sm_set_sideset_pins(pio, sm, sideset_base);
}


/*! \brief Write a word of data to a state machine's TX FIFO
 *  \ingroup hardware_pio
 *
 * This is a raw FIFO access that does not check for fullness. If the FIFO is
 * full, the FIFO contents and state are not affected by the write attempt.
 * Hardware sets the TXOVER sticky flag for this FIFO in FDEBUG, to indicate
 * that the system attempted to write to a full FIFO.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param data the 32 bit data value
 *
 * \sa pio_sm_put_blocking()
 */
void nostatic_pio_sm_put(PIO pio, uint sm, uint32_t data) {
    return pio_sm_put(pio, sm, data);
}


/*! \brief Read a word of data from a state machine's RX FIFO
 *  \ingroup hardware_pio
 *
 * This is a raw FIFO access that does not check for emptiness. If the FIFO is
 * empty, the hardware ignores the attempt to read from the FIFO (the FIFO
 * remains in an empty state following the read) and the sticky RXUNDER flag
 * for this FIFO is set in FDEBUG to indicate that the system tried to read
 * from this FIFO when empty. The data returned by this function is undefined
 * when the FIFO is empty.
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 *
 * \sa pio_sm_get_blocking()
 */
uint32_t nostatic_pio_sm_get(PIO pio, uint sm) {
    return pio_sm_get(pio, sm);
}


/*! \brief Determine if a state machine's RX FIFO is full
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the RX FIFO is full
 */
bool nostatic_pio_sm_is_rx_fifo_full(PIO pio, uint sm) {
    return pio_sm_is_rx_fifo_full(pio, sm);
}


/*! \brief Determine if a state machine's RX FIFO is empty
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the RX FIFO is empty
 */
bool nostatic_pio_sm_is_rx_fifo_empty(PIO pio, uint sm) {
    return pio_sm_is_rx_fifo_empty(pio, sm);
}


/*! \brief Return the number of elements currently in a state machine's RX FIFO
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return the number of elements in the RX FIFO
 */
uint nostatic_pio_sm_get_rx_fifo_level(PIO pio, uint sm) {
    return pio_sm_get_rx_fifo_level(pio, sm);
}


/*! \brief Determine if a state machine's TX FIFO is full
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the TX FIFO is full
 */
bool nostatic_pio_sm_is_tx_fifo_full(PIO pio, uint sm) {
    return pio_sm_is_tx_fifo_full(pio, sm);
}


/*! \brief Determine if a state machine's TX FIFO is empty
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return true if the TX FIFO is empty
 */
bool nostatic_pio_sm_is_tx_fifo_empty(PIO pio, uint sm) {
    return pio_sm_is_tx_fifo_empty(pio, sm);
}


/*! \brief Return the number of elements currently in a state machine's TX FIFO
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \return the number of elements in the TX FIFO
 */
uint nostatic_pio_sm_get_tx_fifo_level(PIO pio, uint sm) {
    return pio_sm_get_tx_fifo_level(pio, sm);
}


/*! \brief Write a word of data to a state machine's TX FIFO, blocking if the FIFO is full
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param data the 32 bit data value
 */
void nostatic_pio_sm_put_blocking(PIO pio, uint sm, uint32_t data) {
    return pio_sm_put_blocking(pio, sm, data);
}


/*! \brief Read a word of data from a state machine's RX FIFO, blocking if the FIFO is empty
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
uint32_t nostatic_pio_sm_get_blocking(PIO pio, uint sm) {
    return pio_sm_get_blocking(pio, sm);
}


/*! \brief set the current clock divider for a state machine using a 16:8 fraction
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param div_int the integer part of the clock divider
 * \param div_frac the fractional part of the clock divider in 1/256s
 */
void nostatic_pio_sm_set_clkdiv_int_frac(PIO pio, uint sm, uint16_t div_int, uint8_t div_frac) {
    return pio_sm_set_clkdiv_int_frac(pio, sm, div_int, div_frac);
}


/*! \brief set the current clock divider for a state machine
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 * \param div the floating point clock divider
 */
void nostatic_pio_sm_set_clkdiv(PIO pio, uint sm, float div) {
    return pio_sm_set_clkdiv(pio, sm, div);
}


/*! \brief Clear a state machine's TX and RX FIFOs
 *  \ingroup hardware_pio
 *
 * \param pio The PIO instance; either \ref pio0 or \ref pio1
 * \param sm State machine index (0..3)
 */
void nostatic_pio_sm_clear_fifos(PIO pio, uint sm) {
    return pio_sm_clear_fifos(pio, sm);
}


/*! \brief Encode just the delay slot bits of an instruction
 *  \ingroup pio_instructions
 *
 * \note This function does not return a valid instruction encoding; instead it returns an encoding of the delay
 * slot suitable for `OR`ing with the result of an encoding function for an actual instruction. Care should be taken when
 * combining the results of this function with the results of \ref pio_encode_sideset and \ref pio_encode_sideset_opt
 * as they share the same bits within the instruction encoding.
 *
 * \param cycles the number of cycles 0-31 (or less if side set is being used)
 * \return the delay slot bits to be ORed with an instruction encoding
 */
uint nostatic_pio_encode_delay(uint cycles) {
    return pio_encode_delay(cycles);
}


/*! \brief Encode just the side set bits of an instruction (in non optional side set mode)
 *  \ingroup pio_instructions
 *
 * \note This function does not return a valid instruction encoding; instead it returns an encoding of the side set bits
 * suitable for `OR`ing with the result of an encoding function for an actual instruction. Care should be taken when
 * combining the results of this function with the results of \ref pio_encode_delay as they share the same bits
 * within the instruction encoding.
 *
 * \param sideset_bit_count number of side set bits as would be specified via `.sideset` in pioasm
 * \param value the value to sideset on the pins
 * \return the side set bits to be ORed with an instruction encoding
 */
uint nostatic_pio_encode_sideset(uint sideset_bit_count, uint value) {
    return pio_encode_sideset(sideset_bit_count, value);
}


/*! \brief Encode just the side set bits of an instruction (in optional -`opt` side set mode)
 *  \ingroup pio_instructions
 *
 * \note This function does not return a valid instruction encoding; instead it returns an encoding of the side set bits
 * suitable for `OR`ing with the result of an encoding function for an actual instruction. Care should be taken when
 * combining the results of this function with the results of \ref pio_encode_delay as they share the same bits
 * within the instruction encoding.
 *
 * \param sideset_bit_count number of side set bits as would be specified via `.sideset <n> opt` in pioasm
 * \param value the value to sideset on the pins
 * \return the side set bits to be ORed with an instruction encoding
 */
uint nostatic_pio_encode_sideset_opt(uint sideset_bit_count, uint value) {
    return pio_encode_sideset_opt(sideset_bit_count, value);
}


/*! \brief Encode an unconditional JMP instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp(uint addr) {
    return pio_encode_jmp(addr);
}


/*! \brief Encode a conditional JMP if scratch X zero instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP !X <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp_not_x(uint addr) {
    return pio_encode_jmp_not_x(addr);
}


/*! \brief Encode a conditional JMP if scratch X non-zero (and post-decrement X) instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP X-- <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp_x_dec(uint addr) {
    return pio_encode_jmp_x_dec(addr);
}


/*! \brief Encode a conditional JMP if scratch Y zero instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP !Y <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp_not_y(uint addr) {
    return pio_encode_jmp_not_y(addr);
}


/*! \brief Encode a conditional JMP if scratch Y non-zero (and post-decrement Y) instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP Y-- <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp_y_dec(uint addr) {
    return pio_encode_jmp_y_dec(addr);
}


/*! \brief Encode a conditional JMP if scratch X not equal scratch Y instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP X!=Y <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp_x_ne_y(uint addr) {
    return pio_encode_jmp_x_ne_y(addr);
}


/*! \brief Encode a conditional JMP if input pin high instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP PIN <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp_pin(uint addr) {
    return pio_encode_jmp_pin(addr);
}


/*! \brief Encode a conditional JMP if output shift register not empty instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `JMP !OSRE <addr>`
 *
 * \param addr The target address 0-31 (an absolute address within the PIO instruction memory)
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_jmp_not_osre(uint addr) {
    return pio_encode_jmp_not_osre(addr);
}


/*! \brief Encode a WAIT for GPIO pin instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `WAIT <polarity> GPIO <gpio>`
 *
 * \param polarity true for `WAIT 1`, false for `WAIT 0`
 * \param gpio The real GPIO number 0-31
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_wait_gpio(bool polarity, uint gpio) {
    return pio_encode_wait_gpio(polarity, gpio);
}


/*! \brief Encode a WAIT for pin instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `WAIT <polarity> PIN <pin>`
 *
 * \param polarity true for `WAIT 1`, false for `WAIT 0`
 * \param pin The pin number 0-31 relative to the executing SM's input pin mapping
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_wait_pin(bool polarity, uint pin) {
    return pio_encode_wait_pin(polarity, pin);
}


/*! \brief Encode a WAIT for IRQ instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `WAIT <polarity> IRQ <irq> <relative>`
 *
 * \param polarity true for `WAIT 1`, false for `WAIT 0`
 * \param relative true for a `WAIT IRQ <irq> REL`, false for regular `WAIT IRQ <irq>`
 * \param irq the irq number 0-7
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_wait_irq(bool polarity, bool relative, uint irq) {
    return pio_encode_wait_irq(polarity, relative, irq);
}


/*! \brief Encode an IN instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `IN <src>, <count>`
 *
 * \param src The source to take data from
 * \param count The number of bits 1-32
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_in(enum pio_src_dest src, uint count) {
    return pio_encode_in(src, count);
}


/*! \brief Encode an OUT instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `OUT <src>, <count>`
 *
 * \param dest The destination to write data to
 * \param count The number of bits 1-32
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_out(enum pio_src_dest dest, uint count) {
    return pio_encode_out(dest, count);
}


/*! \brief Encode a PUSH instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `PUSH <if_full>, <block>`
 *
 * \param if_full true for `PUSH IF_FULL ...`, false for `PUSH ...`
 * \param block true for `PUSH ... BLOCK`, false for `PUSH ...`
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_push(bool if_full, bool block) {
    return pio_encode_push(if_full, block);
}


/*! \brief Encode a PULL instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `PULL <if_empty>, <block>`
 *
 * \param if_empty true for `PULL IF_EMPTY ...`, false for `PULL ...`
 * \param block true for `PULL ... BLOCK`, false for `PULL ...`
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_pull(bool if_empty, bool block) {
    return pio_encode_pull(if_empty, block);
}


/*! \brief Encode a MOV instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `MOV <dest>, <src>`
 *
 * \param dest The destination to write data to
 * \param src The source to take data from
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_mov(enum pio_src_dest dest, enum pio_src_dest src) {
    return pio_encode_mov(dest, src);
}


/*! \brief Encode a MOV instruction with bit invert
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `MOV <dest>, ~<src>`
 *
 * \param dest The destination to write inverted data to
 * \param src The source to take data from
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_mov_not(enum pio_src_dest dest, enum pio_src_dest src) {
    return pio_encode_mov_not(dest, src);
}


/*! \brief Encode a MOV instruction with bit reverse
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `MOV <dest>, ::<src>`
 *
 * \param dest The destination to write bit reversed data to
 * \param src The source to take data from
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_mov_reverse(enum pio_src_dest dest, enum pio_src_dest src) {
    return pio_encode_mov_reverse(dest, src);
}


/*! \brief Encode a IRQ SET instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `IRQ SET <irq> <relative>`
 *
 * \param relative true for a `IRQ SET <irq> REL`, false for regular `IRQ SET <irq>`
 * \param irq the irq number 0-7
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_irq_set(bool relative, uint irq) {
    return pio_encode_irq_set(relative, irq);
}


/*! \brief Encode a IRQ WAIT instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `IRQ WAIT <irq> <relative>`
 *
 * \param relative true for a `IRQ WAIT <irq> REL`, false for regular `IRQ WAIT <irq>`
 * \param irq the irq number 0-7
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_irq_wait(bool relative, uint irq) {
    return pio_encode_irq_wait(relative, irq);
}


/*! \brief Encode a IRQ CLEAR instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `IRQ CLEAR <irq> <relative>`
 *
 * \param relative true for a `IRQ CLEAR <irq> REL`, false for regular `IRQ CLEAR <irq>`
 * \param irq the irq number 0-7
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_irq_clear(bool relative, uint irq) {
    return pio_encode_irq_clear(relative, irq);
}


/*! \brief Encode a SET instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `SET <dest>, <value>`
 *
 * \param dest The destination to apply the value to
 * \param value The value 0-31
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_set(enum pio_src_dest dest, uint value) {
    return pio_encode_set(dest, value);
}


/*! \brief Encode a NOP instruction
 *  \ingroup pio_instructions
 *
 * This is the equivalent of `NOP` which is itself encoded as `MOV y, y`
 *
 * \return The instruction encoding with 0 delay and no side set value
 * \see pio_encode_delay, pio_encode_sideset, pio_encode_sideset_opt
 */
uint nostatic_pio_encode_nop() {
    return pio_encode_nop();
}


/*! \brief  Enable PWM instance interrupt
 *  \ingroup hardware_pwm
 *
 * Used to enable a single PWM instance interrupt.
 *
 * \param slice_num PWM block to enable/disable
 * \param enabled true to enable, false to disable
 */
void nostatic_pwm_set_irq_enabled(uint slice_num, bool enabled) {
    return pwm_set_irq_enabled(slice_num, enabled);
}


/*! \brief  Enable multiple PWM instance interrupts
 *  \ingroup hardware_pwm
 *
 * Use this to enable multiple PWM interrupts at once.
 *
 * \param slice_mask Bitmask of all the blocks to enable/disable. Channel 0 = bit 0, channel 1 = bit 1 etc.
 * \param enabled true to enable, false to disable
 */
void nostatic_pwm_set_irq_mask_enabled(uint32_t slice_mask, bool enabled) {
    return pwm_set_irq_mask_enabled(slice_mask, enabled);
}


/*! \brief  Clear a single PWM channel interrupt
 *  \ingroup hardware_pwm
 *
 * \param slice_num PWM slice number
 */
void nostatic_pwm_clear_irq(uint slice_num) {
    return pwm_clear_irq(slice_num);
}


/*! \brief  Get PWM interrupt status, raw
 *  \ingroup hardware_pwm
 *
 * \return Bitmask of all PWM interrupts currently set
 */
uint32_t nostatic_pwm_get_irq_status_mask() {
    return pwm_get_irq_status_mask();
}


/*! \brief  Force PWM interrupt
 *  \ingroup hardware_pwm
 *
 * \param slice_num PWM slice number
 */
void nostatic_pwm_force_irq(uint slice_num) {
    return pwm_force_irq(slice_num);
}


/*! \brief Return the DREQ to use for pacing transfers to a particular PWM slice
 *  \ingroup hardware_pwm
 *
 * \param slice_num PWM slice number
 */
uint nostatic_pwm_get_dreq(uint slice_num) {
    return pwm_get_dreq(slice_num);
}


/*! \brief  Set DMA channel read increment in a channel configuration object
 *  \ingroup channel_config
 *
 * \param c Pointer to channel configuration object
 * \param incr True to enable read address increments, if false, each read will be from the same address
 *             Usually disabled for peripheral to memory transfers
 */
void nostatic_channel_config_set_read_increment(dma_channel_config * c, bool incr) {
    return channel_config_set_read_increment(c, incr);
}


/*! \brief  Set DMA channel write increment in a channel configuration object
 *  \ingroup channel_config
 *
 * \param c Pointer to channel configuration object
 * \param incr True to enable write address increments, if false, each write will be to the same address
 *             Usually disabled for memory to peripheral transfers
 * Usually disabled for memory to peripheral transfers
 */
void nostatic_channel_config_set_write_increment(dma_channel_config * c, bool incr) {
    return channel_config_set_write_increment(c, incr);
}


/*! \brief  Select a transfer request signal in a channel configuration object
 *  \ingroup channel_config
 *
 * The channel uses the transfer request signal to pace its data transfer rate.
 * Sources for TREQ signals are internal (TIMERS) or external (DREQ, a Data Request from the system).
 * 0x0 to 0x3a -> select DREQ n as TREQ
 * 0x3b -> Select Timer 0 as TREQ
 * 0x3c -> Select Timer 1 as TREQ
 * 0x3d -> Select Timer 2 as TREQ (Optional)
 * 0x3e -> Select Timer 3 as TREQ (Optional)
 * 0x3f -> Permanent request, for unpaced transfers.
 *
 * \param c Pointer to channel configuration data
 * \param dreq Source (see description)
 */
void nostatic_channel_config_set_dreq(dma_channel_config * c, uint dreq) {
    return channel_config_set_dreq(c, dreq);
}


/*! \brief  Set DMA channel chain_to channel in a channel configuration object
 *  \ingroup channel_config
 *
 * When this channel completes, it will trigger the channel indicated by chain_to. Disable by
 * setting chain_to to itself (the same channel)
 *
 * \param c Pointer to channel configuration object
 * \param chain_to Channel to trigger when this channel completes.
 */
void nostatic_channel_config_set_chain_to(dma_channel_config * c, uint chain_to) {
    return channel_config_set_chain_to(c, chain_to);
}


/*! \brief Set the size of each DMA bus transfer in a channel configuration object
 *  \ingroup channel_config
 *
 * Set the size of each bus transfer (byte/halfword/word). The read and write addresses
 * advance by the specific amount (1/2/4 bytes) with each transfer.
 *
 * \param c Pointer to channel configuration object
 * \param size See enum for possible values.
 */
void nostatic_channel_config_set_transfer_data_size(dma_channel_config * c, enum dma_channel_transfer_size size) {
    return channel_config_set_transfer_data_size(c, size);
}


/*! \brief  Set address wrapping parameters in a channel configuration object
 *  \ingroup channel_config
 *
 * Size of address wrap region. If 0, don’t wrap. For values n > 0, only the lower n bits of the address
 * will change. This wraps the address on a (1 << n) byte boundary, facilitating access to naturally-aligned
 * ring buffers.
 * Ring sizes between 2 and 32768 bytes are possible (size_bits from 1 - 15)
 *
 * 0x0 -> No wrapping.
 *
 * \param c Pointer to channel configuration object
 * \param write True to apply to write addresses, false to apply to read addresses
 * \param size_bits 0 to disable wrapping. Otherwise the size in bits of the changing part of the address.
 *        Effectively wraps the address on a (1 << size_bits) byte boundary.
 */
void nostatic_channel_config_set_ring(dma_channel_config * c, bool write, uint size_bits) {
    return channel_config_set_ring(c, write, size_bits);
}


/*! \brief  Set DMA byte swapping config in a channel configuration object
 *  \ingroup channel_config
 *
 * No effect for byte data, for halfword data, the two bytes of each halfword are
 * swapped. For word data, the four bytes of each word are swapped to reverse their order.
 *
 * \param c Pointer to channel configuration object
 * \param bswap True to enable byte swapping
 */
void nostatic_channel_config_set_bswap(dma_channel_config * c, bool bswap) {
    return channel_config_set_bswap(c, bswap);
}


/*! \brief  Set IRQ quiet mode in a channel configuration object
 *  \ingroup channel_config
 *
 * In QUIET mode, the channel does not generate IRQs at the end of every transfer block. Instead,
 * an IRQ is raised when NULL is written to a trigger register, indicating the end of a control
 * block chain.
 *
 * \param c Pointer to channel configuration object
 * \param irq_quiet True to enable quiet mode, false to disable.
 */
void nostatic_channel_config_set_irq_quiet(dma_channel_config * c, bool irq_quiet) {
    return channel_config_set_irq_quiet(c, irq_quiet);
}


/*!
 *  \brief Set the channel priority in a channel configuration object
 *  \ingroup channel_config
 *
 * When true, gives a channel preferential treatment in issue scheduling: in each scheduling round,
 * all high priority channels are considered first, and then only a single low
 * priority channel, before returning to the high priority channels.
 *
 * This only affects the order in which the DMA schedules channels. The DMA's bus priority is not changed.
 * If the DMA is not saturated then a low priority channel will see no loss of throughput.
 *
 * \param c Pointer to channel configuration object
 * \param high_priority True to enable high priority
 */
void nostatic_channel_config_set_high_priority(dma_channel_config * c, bool high_priority) {
    return channel_config_set_high_priority(c, high_priority);
}


/*!
 *  \brief Enable/Disable the DMA channel in a channel configuration object
 *  \ingroup channel_config
 *
 * When false, the channel will ignore triggers, stop issuing transfers, and pause the current transfer sequence (i.e. BUSY will
 * remain high if already high)
 *
 * \param c Pointer to channel configuration object
 * \param enable True to enable the DMA channel. When enabled, the channel will respond to triggering events, and start transferring data.
 *
 */
void nostatic_channel_config_set_enable(dma_channel_config * c, bool enable) {
    return channel_config_set_enable(c, enable);
}


/*! \brief  Enable access to channel by sniff hardware in a channel configuration object
 *  \ingroup channel_config
 *
 * Sniff HW must be enabled and have this channel selected.
 *
 * \param c Pointer to channel configuration object
 * \param sniff_enable True to enable the Sniff HW access to this DMA channel.
 */
void nostatic_channel_config_set_sniff_enable(dma_channel_config * c, bool sniff_enable) {
    return channel_config_set_sniff_enable(c, sniff_enable);
}


/*! \brief  Get the default channel configuration for a given channel
 *  \ingroup channel_config
 *
 * Setting | Default
 * --------|--------
 * Read Increment | true
 * Write Increment | false
 * DReq | DREQ_FORCE
 * Chain to | self
 * Data size | DMA_SIZE_32
 * Ring | write=false, size=0 (i.e. off)
 * Byte Swap | false
 * Quiet IRQs | false
 * High Priority | false
 * Channel Enable | true
 * Sniff Enable | false
 *
 * \param channel DMA channel
 * \return the default configuration which can then be modified.
 */
dma_channel_config nostatic_dma_channel_get_default_config(uint channel) {
    return dma_channel_get_default_config(channel);
}


/*! \brief  Get the current configuration for the specified channel.
 *  \ingroup channel_config
 *
 * \param channel DMA channel
 * \return The current configuration as read from the HW register (not cached)
 */
dma_channel_config nostatic_dma_get_channel_config(uint channel) {
    return dma_get_channel_config(channel);
}


/*! \brief  Get the raw configuration register from a channel configuration
 *  \ingroup channel_config
 *
 * \param config Pointer to a config structure.
 * \return Register content
 */
uint32_t nostatic_channel_config_get_ctrl_value(const dma_channel_config * config) {
    return channel_config_get_ctrl_value(config);
}


/*! \brief  Set a channel configuration
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \param config Pointer to a config structure with required configuration
 * \param trigger True to trigger the transfer immediately
 */
void nostatic_dma_channel_set_config(uint channel, const dma_channel_config * config, bool trigger) {
    return dma_channel_set_config(channel, config, trigger);
}


/*! \brief  Set the DMA initial read address.
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \param read_addr Initial read address of transfer.
 * \param trigger True to start the transfer immediately
 */
void nostatic_dma_channel_set_read_addr(uint channel, const volatile void * read_addr, bool trigger) {
    return dma_channel_set_read_addr(channel, read_addr, trigger);
}


/*! \brief  Set the DMA initial write address
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \param write_addr Initial write address of transfer.
 * \param trigger True to start the transfer immediately
 */
void nostatic_dma_channel_set_write_addr(uint channel, volatile void * write_addr, bool trigger) {
    return dma_channel_set_write_addr(channel, write_addr, trigger);
}


/*! \brief  Set the number of bus transfers the channel will do
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \param trans_count The number of transfers (not NOT bytes, see channel_config_set_transfer_data_size)
 * \param trigger True to start the transfer immediately
 */
void nostatic_dma_channel_set_trans_count(uint channel, uint32_t trans_count, bool trigger) {
    return dma_channel_set_trans_count(channel, trans_count, trigger);
}


/*! \brief  Configure all DMA parameters and optionally start transfer
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \param config Pointer to DMA config structure
 * \param write_addr Initial write address
 * \param read_addr Initial read address
 * \param transfer_count Number of transfers to perform
 * \param trigger True to start the transfer immediately
 */
void nostatic_dma_channel_configure(uint channel, const dma_channel_config * config, volatile void * write_addr, const volatile void * read_addr, uint transfer_count, bool trigger) {
    return dma_channel_configure(channel, config, write_addr, read_addr, transfer_count, trigger);
}


/*! \brief  Start one or more channels simultaneously
 *  \ingroup hardware_dma
 *
 * \param chan_mask Bitmask of all the channels requiring starting. Channel 0 = bit 0, channel 1 = bit 1 etc.
 */
void nostatic_dma_start_channel_mask(uint32_t chan_mask) {
    return dma_start_channel_mask(chan_mask);
}


/*! \brief  Start a single DMA channel
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 */
void nostatic_dma_channel_start(uint channel) {
    return dma_channel_start(channel);
}


/*! \brief  Stop a DMA transfer
 *  \ingroup hardware_dma
 *
 * Function will only return once the DMA has stopped.
 *
 * Note that due to errata RP2040-E13, aborting a channel which has transfers
 * in-flight (i.e. an individual read has taken place but the corresponding write has not), the ABORT
 * status bit will clear prematurely, and subsequently the in-flight
 * transfers will trigger a completion interrupt once they complete.
 *
 * The effect of this is that you \em may see a spurious completion interrupt
 * on the channel as a result of calling this method.
 *
 * The calling code should be sure to ignore a completion IRQ as a result of this method. This may
 * not require any additional work, as aborting a channel which may be about to complete, when you have a completion
 * IRQ handler registered, is inherently race-prone, and so code is likely needed to disambiguate the two occurrences.
 *
 * If that is not the case, but you do have a channel completion IRQ handler registered, you can simply
 * disable/re-enable the IRQ around the call to this method as shown by this code fragment (using DMA IRQ0).
 *
 * \code
 *  // disable the channel on IRQ0
 *  dma_channel_set_irq0_enabled(channel, false);
 *  // abort the channel
 *  dma_channel_abort(channel);
 *  // clear the spurious IRQ (if there was one)
 *  dma_channel_acknowledge_irq0(channel);
 *  // re-enable the channel on IRQ0
 *  dma_channel_set_irq0_enabled(channel, true);
 *\endcode
 *
 * \param channel DMA channel
 */
void nostatic_dma_channel_abort(uint channel) {
    return dma_channel_abort(channel);
}


/*! \brief  Enable single DMA channel's interrupt via DMA_IRQ_0
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \param enabled true to enable interrupt 0 on specified channel, false to disable.
 */
void nostatic_dma_channel_set_irq0_enabled(uint channel, bool enabled) {
    return dma_channel_set_irq0_enabled(channel, enabled);
}


/*! \brief  Enable multiple DMA channels' interrupts via DMA_IRQ_0
 *  \ingroup hardware_dma
 *
 * \param channel_mask Bitmask of all the channels to enable/disable. Channel 0 = bit 0, channel 1 = bit 1 etc.
 * \param enabled true to enable all the interrupts specified in the mask, false to disable all the interrupts specified in the mask.
 */
void nostatic_dma_set_irq0_channel_mask_enabled(uint32_t channel_mask, bool enabled) {
    return dma_set_irq0_channel_mask_enabled(channel_mask, enabled);
}


/*! \brief  Enable single DMA channel's interrupt via DMA_IRQ_1
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \param enabled true to enable interrupt 1 on specified channel, false to disable.
 */
void nostatic_dma_channel_set_irq1_enabled(uint channel, bool enabled) {
    return dma_channel_set_irq1_enabled(channel, enabled);
}


/*! \brief  Enable multiple DMA channels' interrupts via DMA_IRQ_1
 *  \ingroup hardware_dma
 *
 * \param channel_mask Bitmask of all the channels to enable/disable. Channel 0 = bit 0, channel 1 = bit 1 etc.
 * \param enabled true to enable all the interrupts specified in the mask, false to disable all the interrupts specified in the mask.
 */
void nostatic_dma_set_irq1_channel_mask_enabled(uint32_t channel_mask, bool enabled) {
    return dma_set_irq1_channel_mask_enabled(channel_mask, enabled);
}


/*! \brief  Enable single DMA channel interrupt on either DMA_IRQ_0 or DMA_IRQ_1
 *  \ingroup hardware_dma
 *
 * \param irq_index the IRQ index; either 0 or 1 for DMA_IRQ_0 or DMA_IRQ_1
 * \param channel DMA channel
 * \param enabled true to enable interrupt via irq_index for specified channel, false to disable.
 */
void nostatic_dma_irqn_set_channel_enabled(uint irq_index, uint channel, bool enabled) {
    return dma_irqn_set_channel_enabled(irq_index, channel, enabled);
}


/*! \brief  Enable multiple DMA channels' interrupt via either DMA_IRQ_0 or DMA_IRQ_1
 *  \ingroup hardware_dma
 *
 * \param irq_index the IRQ index; either 0 or 1 for DMA_IRQ_0 or DMA_IRQ_1
 * \param channel_mask Bitmask of all the channels to enable/disable. Channel 0 = bit 0, channel 1 = bit 1 etc.
 * \param enabled true to enable all the interrupts specified in the mask, false to disable all the interrupts specified in the mask.
 */
void nostatic_dma_irqn_set_channel_mask_enabled(uint irq_index, uint32_t channel_mask, bool enabled) {
    return dma_irqn_set_channel_mask_enabled(irq_index, channel_mask, enabled);
}


/*! \brief  Determine if a particular channel is a cause of DMA_IRQ_0
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \return true if the channel is a cause of DMA_IRQ_0, false otherwise
 */
bool nostatic_dma_channel_get_irq0_status(uint channel) {
    return dma_channel_get_irq0_status(channel);
}


/*! \brief  Determine if a particular channel is a cause of DMA_IRQ_1
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 * \return true if the channel is a cause of DMA_IRQ_1, false otherwise
 */
bool nostatic_dma_channel_get_irq1_status(uint channel) {
    return dma_channel_get_irq1_status(channel);
}


/*! \brief  Determine if a particular channel is a cause of DMA_IRQ_N
 *  \ingroup hardware_dma
 *
 * \param irq_index the IRQ index; either 0 or 1 for DMA_IRQ_0 or DMA_IRQ_1
 * \param channel DMA channel
 * \return true if the channel is a cause of the DMA_IRQ_N, false otherwise
 */
bool nostatic_dma_irqn_get_channel_status(uint irq_index, uint channel) {
    return dma_irqn_get_channel_status(irq_index, channel);
}


/*! \brief  Acknowledge a channel IRQ, resetting it as the cause of DMA_IRQ_0
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 */
void nostatic_dma_channel_acknowledge_irq0(uint channel) {
    return dma_channel_acknowledge_irq0(channel);
}


/*! \brief  Acknowledge a channel IRQ, resetting it as the cause of DMA_IRQ_1
 *  \ingroup hardware_dma
 *
 * \param channel DMA channel
 */
void nostatic_dma_channel_acknowledge_irq1(uint channel) {
    return dma_channel_acknowledge_irq1(channel);
}


/*! \brief  Acknowledge a channel IRQ, resetting it as the cause of DMA_IRQ_N
 *  \ingroup hardware_dma
 *
 * \param irq_index the IRQ index; either 0 or 1 for DMA_IRQ_0 or DMA_IRQ_1
 * \param channel DMA channel
 */
void nostatic_dma_irqn_acknowledge_channel(uint irq_index, uint channel) {
    return dma_irqn_acknowledge_channel(irq_index, channel);
}


/*! \brief Set the divider for the given DMA timer
 *  \ingroup hardware_dma
 *
 * The timer will run at the system_clock_freq * numerator / denominator, so this is the speed
 * that data elements will be transferred at via a DMA channel using this timer as a DREQ
 *
 * \param timer the dma timer
 * \param numerator the fraction's numerator
 * \param denominator the fraction's denominator
 */
void nostatic_dma_timer_set_fraction(uint timer, uint16_t numerator, uint16_t denominator) {
    return dma_timer_set_fraction(timer, numerator, denominator);
}


/*! \brief Return the DREQ number for a given DMA timer
 *  \ingroup hardware_dma
 *
 * \param timer_num DMA timer number 0-3
 */
uint nostatic_dma_get_timer_dreq(uint timer_num) {
    return dma_get_timer_dreq(timer_num);
}


/*! \brief Reset the specified HW blocks
 *  \ingroup hardware_resets
 *
 * \param bits Bit pattern indicating blocks to reset. See \ref reset_bitmask
 */
void nostatic_reset_block(uint32_t bits) {
    return reset_block(bits);
}


/*! \brief bring specified HW blocks out of reset
 *  \ingroup hardware_resets
 *
 * \param bits Bit pattern indicating blocks to unreset. See \ref reset_bitmask
 */
void nostatic_unreset_block(uint32_t bits) {
    return unreset_block(bits);
}


/*! \brief Bring specified HW blocks out of reset and wait for completion
 *  \ingroup hardware_resets
 *
 * \param bits Bit pattern indicating blocks to unreset. See \ref reset_bitmask
 */
void nostatic_unreset_block_wait(uint32_t bits) {
    return unreset_block_wait(bits);
}


/*! \brief  Initialise the gpio for use as an ADC pin
 *  \ingroup hardware_adc
 *
 * Prepare a GPIO for use with ADC by disabling all digital functions.
 *
 * \param gpio The GPIO number to use. Allowable GPIO numbers are 26 to 29 inclusive.
 */
void nostatic_adc_gpio_init(uint gpio) {
    return adc_gpio_init(gpio);
}


/*! \brief  ADC input select
 *  \ingroup hardware_adc
 *
 * Select an ADC input. 0...3 are GPIOs 26...29 respectively.
 * Input 4 is the onboard temperature sensor.
 *
 * \param input Input to select.
 */
void nostatic_adc_select_input(uint input) {
    return adc_select_input(input);
}


/*! \brief  Get the currently selected ADC input channel
 *  \ingroup hardware_adc
 *
 * \return The currently selected input channel. 0...3 are GPIOs 26...29 respectively. Input 4 is the onboard temperature sensor.
 */
uint nostatic_adc_get_selected_input() {
    return adc_get_selected_input();
}


/*! \brief  Round Robin sampling selector
 *  \ingroup hardware_adc
 *
 * This function sets which inputs are to be run through in round robin mode.
 * Value between 0 and 0x1f (bit 0 to bit 4 for GPIO 26 to 29 and temperature sensor input respectively)
 *
 * \param input_mask A bit pattern indicating which of the 5 inputs are to be sampled. Write a value of 0 to disable round robin sampling.
 */
void nostatic_adc_set_round_robin(uint input_mask) {
    return adc_set_round_robin(input_mask);
}


/*! \brief Enable the onboard temperature sensor
 *  \ingroup hardware_adc
 *
 * \param enable Set true to power on the onboard temperature sensor, false to power off.
 *
 */
void nostatic_adc_set_temp_sensor_enabled(bool enable) {
    return adc_set_temp_sensor_enabled(enable);
}


/*! \brief Perform a single conversion
 *  \ingroup hardware_adc
 *
 *  Performs an ADC conversion, waits for the result, and then returns it.
 *
 * \return Result of the conversion.
 */
uint16_t nostatic_adc_read() {
    return adc_read();
}


/*! \brief Enable or disable free-running sampling mode
 *  \ingroup hardware_adc
 *
 * \param run false to disable, true to enable free running conversion mode.
 */
void nostatic_adc_run(bool run) {
    return adc_run(run);
}


/*! \brief Set the ADC Clock divisor
 *  \ingroup hardware_adc
 *
 * Period of samples will be (1 + div) cycles on average. Note it takes 96 cycles to perform a conversion,
 * so any period less than that will be clamped to 96.
 *
 * \param clkdiv If non-zero, conversion will be started at intervals rather than back to back.
 */
void nostatic_adc_set_clkdiv(float clkdiv) {
    return adc_set_clkdiv(clkdiv);
}


/*! \brief Setup the ADC FIFO
 *  \ingroup hardware_adc
 *
 * FIFO is 4 samples long, if a conversion is completed and the FIFO is full, the result is dropped.
 *
 * \param en Enables write each conversion result to the FIFO
 * \param dreq_en Enable DMA requests when FIFO contains data
 * \param dreq_thresh Threshold for DMA requests/FIFO IRQ if enabled.
 * \param err_in_fifo If enabled, bit 15 of the FIFO contains error flag for each sample
 * \param byte_shift Shift FIFO contents to be one byte in size (for byte DMA) - enables DMA to byte buffers.
 */
void nostatic_adc_fifo_setup(bool en, bool dreq_en, uint16_t dreq_thresh, bool err_in_fifo, bool byte_shift) {
    return adc_fifo_setup(en, dreq_en, dreq_thresh, err_in_fifo, byte_shift);
}


/*! \brief Check FIFO empty state
 *  \ingroup hardware_adc
 *
 * \return Returns true if the FIFO is empty
 */
bool nostatic_adc_fifo_is_empty() {
    return adc_fifo_is_empty();
}


/*! \brief Get number of entries in the ADC FIFO
 *  \ingroup hardware_adc
 *
 * The ADC FIFO is 4 entries long. This function will return how many samples are currently present.
 */
uint8_t nostatic_adc_fifo_get_level() {
    return adc_fifo_get_level();
}


/*! \brief Get ADC result from FIFO
 *  \ingroup hardware_adc
 *
 * Pops the latest result from the ADC FIFO.
 */
uint16_t nostatic_adc_fifo_get() {
    return adc_fifo_get();
}


/*! \brief Wait for the ADC FIFO to have data.
 *  \ingroup hardware_adc
 *
 * Blocks until data is present in the FIFO
 */
uint16_t nostatic_adc_fifo_get_blocking() {
    return adc_fifo_get_blocking();
}


/*! \brief Drain the ADC FIFO
 *  \ingroup hardware_adc
 *
 * Will wait for any conversion to complete then drain the FIFO, discarding any results.
 */
void nostatic_adc_fifo_drain() {
    return adc_fifo_drain();
}


/*! \brief Enable/Disable ADC interrupts.
 *  \ingroup hardware_adc
 *
 * \param enabled Set to true to enable the ADC interrupts, false to disable
 */
void nostatic_adc_irq_set_enabled(bool enabled) {
    return adc_irq_set_enabled(enabled);
}


/*! \brief Convert SPI instance to hardware instance number
 *  \ingroup hardware_spi
 *
 * \param spi SPI instance
 * \return Number of SPI, 0 or 1.
 */
uint nostatic_spi_get_index(const spi_inst_t * spi) {
    return spi_get_index(spi);
}


/*! \brief Configure SPI
 *  \ingroup hardware_spi
 *
 * Configure how the SPI serialises and deserialises data on the wire
 *
 * \param spi SPI instance specifier, either \ref spi0 or \ref spi1
 * \param data_bits Number of data bits per transfer. Valid values 4..16.
 * \param cpol SSPCLKOUT polarity, applicable to Motorola SPI frame format only.
 * \param cpha SSPCLKOUT phase, applicable to Motorola SPI frame format only
 * \param order Must be SPI_MSB_FIRST, no other values supported on the PL022
 */
void nostatic_spi_set_format(spi_inst_t * spi, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha, __unused spi_order_t order) {
    return spi_set_format(spi, data_bits, cpol, cpha, order);
}


/*! \brief Set SPI master/slave
 *  \ingroup hardware_spi
 *
 * Configure the SPI for master- or slave-mode operation. By default,
 * spi_init() sets master-mode.
 *
 * \param spi SPI instance specifier, either \ref spi0 or \ref spi1
 * \param slave true to set SPI device as a slave device, false for master.
 */
void nostatic_spi_set_slave(spi_inst_t * spi, bool slave) {
    return spi_set_slave(spi, slave);
}


/*! \brief Check whether a write can be done on SPI device
 *  \ingroup hardware_spi
 *
 * \param spi SPI instance specifier, either \ref spi0 or \ref spi1
 * \return false if no space is available to write. True if a write is possible
 */
bool nostatic_spi_is_writable(const spi_inst_t * spi) {
    return spi_is_writable(spi);
}


/*! \brief Check whether a read can be done on SPI device
 *  \ingroup hardware_spi
 *
 * \param spi SPI instance specifier, either \ref spi0 or \ref spi1
 * \return true if a read is possible i.e. data is present
 */
bool nostatic_spi_is_readable(const spi_inst_t * spi) {
    return spi_is_readable(spi);
}


/*! \brief Check whether SPI is busy
 *  \ingroup hardware_spi
 *
 * \param spi SPI instance specifier, either \ref spi0 or \ref spi1
 * \return true if SPI is busy
 */
bool nostatic_spi_is_busy(const spi_inst_t * spi) {
    return spi_is_busy(spi);
}


/*! \brief Return the DREQ to use for pacing transfers to/from a particular SPI instance
 *  \ingroup hardware_spi
 *
 * \param spi SPI instance specifier, either \ref spi0 or \ref spi1
 * \param is_tx true for sending data to the SPI instance, false for receiving data from the SPI instance
 */
uint nostatic_spi_get_dreq(spi_inst_t * spi, bool is_tx) {
    return spi_get_dreq(spi, is_tx);
}


/*! \brief Start a signed asynchronous divide
 *  \ingroup hardware_divider
 *
 * Start a divide of the specified signed parameters. You should wait for 8 cycles (__div_pause()) or wait for the ready bit to be set
 * (hw_divider_wait_ready()) prior to reading the results.
 *
 * \param a The dividend
 * \param b The divisor
 */
void nostatic_hw_divider_divmod_s32_start(int32_t a, int32_t b) {
    return hw_divider_divmod_s32_start(a, b);
}


/*! \brief Start an unsigned asynchronous divide
 *  \ingroup hardware_divider
 *
 * Start a divide of the specified unsigned parameters. You should wait for 8 cycles (__div_pause()) or wait for the ready bit to be set
 * (hw_divider_wait_ready()) prior to reading the results.
 *
 * \param a The dividend
 * \param b The divisor
 */
void nostatic_hw_divider_divmod_u32_start(uint32_t a, uint32_t b) {
    return hw_divider_divmod_u32_start(a, b);
}


/*! \brief Wait for a divide to complete
 *  \ingroup hardware_divider
 *
 * Wait for a divide to complete
 */
void nostatic_hw_divider_wait_ready() {
    return hw_divider_wait_ready();
}


/*! \brief Return result of HW divide, nowait
 *  \ingroup hardware_divider
 *
 * \note This is UNSAFE in that the calculation may not have been completed.
 *
 * \return Current result. Most significant 32 bits are the remainder, lower 32 bits are the quotient.
 */
divmod_result_t nostatic_hw_divider_result_nowait() {
    return hw_divider_result_nowait();
}


/*! \brief Return result of last asynchronous HW divide
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current result. Most significant 32 bits are the remainder, lower 32 bits are the quotient.
 */
divmod_result_t nostatic_hw_divider_result_wait() {
    return hw_divider_result_wait();
}


/*! \brief Return result of last asynchronous HW divide, unsigned quotient only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current unsigned quotient result.
 */
uint32_t nostatic_hw_divider_u32_quotient_wait() {
    return hw_divider_u32_quotient_wait();
}


/*! \brief Return result of last asynchronous HW divide, signed quotient only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current signed quotient result.
 */
int32_t nostatic_hw_divider_s32_quotient_wait() {
    return hw_divider_s32_quotient_wait();
}


/*! \brief Return result of last asynchronous HW divide, unsigned remainder only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current unsigned remainder result.
 */
uint32_t nostatic_hw_divider_u32_remainder_wait() {
    return hw_divider_u32_remainder_wait();
}


/*! \brief Return result of last asynchronous HW divide, signed remainder only
 *  \ingroup hardware_divider
 *
 * This function waits for the result to be ready by calling hw_divider_wait_ready().
 *
 * \return Current remainder results.
 */
int32_t nostatic_hw_divider_s32_remainder_wait() {
    return hw_divider_s32_remainder_wait();
}


/*! \brief Do an unsigned HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient results of the divide
 */
uint32_t nostatic_hw_divider_u32_quotient(uint32_t a, uint32_t b) {
    return hw_divider_u32_quotient(a, b);
}


/*! \brief Do an unsigned HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder results of the divide
 */
uint32_t nostatic_hw_divider_u32_remainder(uint32_t a, uint32_t b) {
    return hw_divider_u32_remainder(a, b);
}


/*! \brief Do a signed HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient results of the divide
 */
int32_t nostatic_hw_divider_quotient_s32(int32_t a, int32_t b) {
    return hw_divider_quotient_s32(a, b);
}


/*! \brief Do a signed HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder results of the divide
 */
int32_t nostatic_hw_divider_remainder_s32(int32_t a, int32_t b) {
    return hw_divider_remainder_s32(a, b);
}


/*! \brief Pause for exact amount of time needed for a asynchronous divide to complete
 *  \ingroup hardware_divider
 */
void nostatic_hw_divider_pause() {
    return hw_divider_pause();
}


/*! \brief Do a hardware unsigned HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient result of the divide
 */
uint32_t nostatic_hw_divider_u32_quotient_inlined(uint32_t a, uint32_t b) {
    return hw_divider_u32_quotient_inlined(a, b);
}


/*! \brief Do a hardware unsigned HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder result of the divide
 */
uint32_t nostatic_hw_divider_u32_remainder_inlined(uint32_t a, uint32_t b) {
    return hw_divider_u32_remainder_inlined(a, b);
}


/*! \brief Do a hardware signed HW divide, wait for result, return quotient
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return quotient.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Quotient result of the divide
 */
int32_t nostatic_hw_divider_s32_quotient_inlined(int32_t a, int32_t b) {
    return hw_divider_s32_quotient_inlined(a, b);
}


/*! \brief Do a hardware signed HW divide, wait for result, return remainder
 *  \ingroup hardware_divider
 *
 * Divide \p a by \p b, wait for calculation to complete, return remainder.
 *
 * \param a The dividend
 * \param b The divisor
 * \return Remainder result of the divide
 */
int32_t nostatic_hw_divider_s32_remainder_inlined(int32_t a, int32_t b) {
    return hw_divider_s32_remainder_inlined(a, b);
}


/*! \brief Execute a breakpoint instruction
 *  \ingroup pico_platform
 */
void nostatic___breakpoint() {
    return __breakpoint();
}


/*! \brief Returns the RP2040 rom version number
 *  \ingroup pico_platform
 * @return the RP2040 rom version number (1 for RP2040-B0, 2 for RP2040-B1, 3 for RP2040-B2)
 */
uint8_t nostatic_rp2040_rom_version() {
    return rp2040_rom_version();
}


/*! \brief Helper method to busy-wait for at least the given number of cycles
 *  \ingroup pico_platform
 *
 * This method is useful for introducing very short delays.
 *
 * This method busy-waits in a tight loop for the given number of system clock cycles. The total wait time is only accurate to within 2 cycles,
 * and this method uses a loop counter rather than a hardware timer, so the method will always take longer than expected if an
 * interrupt is handled on the calling core during the busy-wait; you can of course disable interrupts to prevent this.
 *
 * You can use \ref clock_get_hz(clk_sys) to determine the number of clock cycles per second if you want to convert an actual
 * time duration to a number of cycles.
 *
 * \param minimum_cycles the minimum number of system clock cycles to delay for
 */
void nostatic_busy_wait_at_least_cycles(uint32_t minimum_cycles) {
    return busy_wait_at_least_cycles(minimum_cycles);
}




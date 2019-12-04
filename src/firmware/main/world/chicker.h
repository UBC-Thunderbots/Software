#pragma once

/**
 * This struct represents a robot chicker (kicker and chipper) from the perspective of
 * firmware
 */
struct Chicker;
typedef struct Chicker Chicker;

/**
 * Create a chicker with the given functions for interacting with it
 *
 * @param kick A function that we can call to kick with the chicker with a given speed,
 *             in meters per second
 * @param chip A function that we can call to chip a given distance, in meters, with the
 *             chicker. The distance parameter is the distance to the first bounce.
 * @param enable_autokick A function that we can call to enable autokick on the chicker
 *                        (ie. the chicker will kick the ball as soon as the ball is close
 *                        enough to the front of the chicker to be kicked), with a kick
 *                        speed in meters per second
 * @param enable_autochip A function that we can call to enable autochip on the chicker
 *                        (ie. the chicker will chip the ball as soon as the ball is close
 *                        enough to the front of the chicker to be chipped), with a chip
 *                        distance in meters
 * @param disable_autokick A function that we can call to disable autokick
 * @param disable_autochip A function that we can call to disable autochip
 *
 * @return A pointer to the created chicker, ownership is given to the caller
 */
Chicker* Chicker_create(void (*kick)(float speed_m_per_s), void (*chip)(float distance_m),
                        void (*enable_autokick)(float speed_m_per_s),
                        void (*enable_autochip)(float distance_m),
                        void (*disable_autokick)(void), void (*disable_autochip)(void));

/**
 * Destroy the given chicker, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed into
 *       `Firmwarechicker_create`
 *
 * @param chicker The chicker to destroy
 */
void Chicker_destroy(Chicker* chicker);

/**
 * Kick with the given chicker at the given speed
 * @param chicker The chicker to kick with
 * @param speed_m_per_s  The speed to kick at, in meters per second
 */
void Chicker_kick(Chicker* chicker, float speed_m_per_s);

/**
 * Chip with the given chicker at the given speed
 * @param chicker The chicker to chip with
 * @param distance_m  The distance to chip, in meters
 */
void Chicker_chip(Chicker* chicker, float distance_m);

/**
 * Enable autokick on the given chicker
 * @param chicker The chicker to enable autokick on
 * @param speed_m_per_s The speed of the kick, in meters per second
 */
void Chicker_enableAutokick(Chicker* chicker, float speed_m_per_s);

/**
 * Enable autochip on the given chicker
 * @param chicker The chicker to enable autochip on
 * @param distance_m The distance to chip, in meters
 */
void Chicker_enableAutochip(Chicker* chicker, float distance_m);

/**
 * Disable autokick on the given chicker
 * @param chicker The chicker to disable autokick on
 */
void Chicker_disableAutokick(Chicker* chicker);

/**
 * Disable autochip on the given chicker
 * @param chicker The chicker to disable autochip on
 */
void Chicker_disableAutochip(Chicker* chicker);

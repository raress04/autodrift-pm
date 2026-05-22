/* =========================================================
 *  AutoDrift — Motor Test (DEBUG ONLY)
 *  Target: ATmega328P Xplained Mini @ 16 MHz
 *
 *  Hardware present: LM2596 + TB6612FNG + 2 motors. Nothing else.
 *
 *  NOTE: No hardware PWM timer is used here. PWMA and PWMB are
 *  driven statically HIGH (full speed) so we can verify basic
 *  motor direction without worrying about which timer OC pin
 *  maps where on the 328P. Speed control can be added later.
 *
 *  Behaviour: FORWARD 5 s → stop 0.5 s → BACKWARD 5 s → repeat.
 *
 *  TB6612FNG wiring (ATmega328P pins):
 *    PWMA → PB3   (static HIGH = full speed Motor A)
 *    PWMB → PB4   (static HIGH = full speed Motor B)
 *    AIN1 → PD4   (Motor A direction 1)
 *    AIN2 → PD5   (Motor A direction 2)
 *    BIN1 → PD6   (Motor B direction 1)
 *    BIN2 → PD7   (Motor B direction 2)
 *    STBY → PC5   (HIGH = chip enabled)
 * ========================================================= */

#include <avr/io.h>
#include <util/delay.h>

/* ── Pin definitions ─────────────────────────────────────── */
#define PWMA_PIN   PB3
#define PWMB_PIN   PB4

#define MOTOR_AIN1 PD4
#define MOTOR_AIN2 PD5
#define MOTOR_BIN1 PD6
#define MOTOR_BIN2 PD7

/* STBY (moved from PC5 → PB0 to free I2C SCL) */
#define STBY_PIN   PB0

/* ── Hardware init ───────────────────────────────────────── */
static void hw_init(void)
{
    /* STBY HIGH → enable TB6612FNG */
    DDRB  |= (1 << STBY_PIN);
    PORTB |= (1 << STBY_PIN);

    /* PWMA and PWMB as outputs, driven HIGH = full speed enable.
     * No timer needed for a basic direction test.             */
    DDRB  |= (1 << PWMA_PIN) | (1 << PWMB_PIN);
    PORTB |= (1 << PWMA_PIN) | (1 << PWMB_PIN);   /* static HIGH */

    /* Direction pins as outputs, all LOW initially (coast)    */
    DDRD  |= (1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
           | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2);
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
                        | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));
}

/* ── Motor primitives ────────────────────────────────────── */
static void motors_stop(void)
{
    /* Coast: all direction pins LOW */
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
                        | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));
}

static void motors_forward(void)
{
    /* AIN1=H, AIN2=L → Motor A forward */
    PORTD |=  (1 << MOTOR_AIN1);
    PORTD &= (uint8_t)(~(1 << MOTOR_AIN2));

    /* BIN1=H, BIN2=L → Motor B forward */
    PORTD |=  (1 << MOTOR_BIN1);
    PORTD &= (uint8_t)(~(1 << MOTOR_BIN2));
}

static void motors_backward(void)
{
    /* AIN1=L, AIN2=H → Motor A backward */
    PORTD &= (uint8_t)(~(1 << MOTOR_AIN1));
    PORTD |=  (1 << MOTOR_AIN2);

    /* BIN1=L, BIN2=H → Motor B backward */
    PORTD &= (uint8_t)(~(1 << MOTOR_BIN1));
    PORTD |=  (1 << MOTOR_BIN2);
}

/* ── Main ────────────────────────────────────────────────── */
int main(void)
{
    hw_init();

    while (1) {
        motors_forward();
        _delay_ms(5000);

        motors_stop();
        _delay_ms(500);

        motors_backward();
        _delay_ms(5000);

        motors_stop();
        _delay_ms(500);
    }

    return 0;
}

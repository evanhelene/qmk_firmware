#include QMK_KEYBOARD_H

#include "i2c_master.h"
#include "debug.h"
#include "version.h"
#include "bno055_support.h"

#define BASE 0 // default layer
#define TIMEOUT 50

#define BADDR  0x28
 // BNO055_I2C_ADDR << 1

 uint16_t timeout = TIMEOUT;

enum
{
    _BL = 0,
    _SYMB,
    _MDIA,
};

static struct bno055_t myBNO;
static struct bno055_euler myEulerData;
uint16_t scan_timer = 0;
// unsigned char bno055_i2c_address = BNO055_I2C_ADDR << 1;

void read_bno055(void){

        BNO055_S16 temperature;
        struct bno055_gyro gry;
        struct bno055_accel acc;
        struct bno055_linear_accel lia;
        struct bno055_gravity grvt;
        struct bno055_quaternion qur;
        struct bno055_mag mag;
        int status = bno055_read_euler_hrp(&myEulerData);
        dprintf("%d %d %d %d euler\n", status,myEulerData.r,myEulerData.p,myEulerData.h);
        status = bno055_read_temperature_data(&temperature);
        dprintf("%d %d temp \n", status, temperature);
        status = bno055_read_mag_xyz(&mag);
        dprintf("%d %d %d %d mag\n", status, mag.x, mag.y, mag.z);
        status = bno055_read_gyro_xyz(&gry);
        dprintf("%d %d %d %d gyro\n", status, gry.x, gry.y, gry.z);
        status = bno055_read_linear_accel_xyz(&lia);
        dprintf("%d %d %d %d lia\n", status, lia.x, lia.y, lia.z);
        status = bno055_read_accel_xyz(&acc);
        dprintf("%d %d %d %d accel\n", status, acc.x, acc.y, acc.z);
        bno055_read_gravity_xyz(&grvt);
        dprintf("%d %d %d %d grvt\n", status, grvt.x, grvt.y, grvt.z);
        bno055_read_quaternion_wxyz(&qur);
        dprintf("%d %d %d %d %d quat\n", status, qur.w, qur.x, qur.y, qur.z);


        // int x = int(float(myEulerData.r) / 16.0) * 100;     // roll in degrees
        // int y = int(float(myEulerData.p) / 16.0) * 100;     // pitch in degrees
        // int z = int(float(myEulerData.h) / 16.0) * 100;     // heading in degrees
}


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
/* Keymap 0: Basic layer
 *
 * ,-----------------------------------------.                    ,-----------------------------------------.
 * |   =  |   1  |   2  |   3  |   4  |   5  |                    |   6  |   7  |   8  |   9  |   0  |  -   |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * | Del  |   Q  |   W  |   E  |   R  |   T  |                    |   Y  |   U  |   I  |   O  |   P  |  \   |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * | BkSp |   A  |   S  |   D  |   F  |   G  |                    |   H  |   J  |   K  |   L  |; / L2|'/Cmd |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |LShift|Z/Ctrl|   X  |   C  |   V  |   B  |                    |   N  |   M  |   ,  |   .  |//Ctrl|RShift|
 * |------+------+------+------+------+------'                    `------+------+------+------+------+------|
 * |Grv/L1|  '"  |AltShf| Left | Right|                                  |  Up  | Down |   [  |   ]  | ~L1  |
 * `----------------------------------'                                  `----------------------------------'
 *                                      ,-------------.  ,-------------.
 *                                      | App  | LGui |  | Alt  | ^/Esc|
 *                               ,------|------|------|  |------+------+------.
 *                               |      |      | Home |  | PgUp |      |      |
 *                               | Space|Backsp|------|  |------|  Tab |Enter |
 *                               |      |ace   | End  |  | PgDn |      |      |
 *                               `--------------------'  `--------------------'
 */
[_BL] = LAYOUT_pterodactyl(  // layer 0 : default
        // left hand
          KC_EQL,          KC_1,           KC_2,     KC_3,     KC_4,  KC_5,                             KC_6,   KC_7,     KC_8,     KC_9,               KC_0,         KC_MINS,
          KC_DEL,          KC_Q,           KC_W,     KC_E,     KC_R,  KC_T,                             KC_Y,   KC_U,     KC_I,     KC_O,               KC_P,         KC_BSLS,
          KC_BSPC,         KC_A,           KC_S,     KC_D,     KC_F,  KC_G,                             KC_H,   KC_J,     KC_K,     KC_L,  LT(_MDIA, KC_SCLN),  GUI_T(KC_QUOT),
          KC_LSFT,  CTL_T(KC_Z),           KC_X,     KC_C,     KC_V,  KC_B,                             KC_N,   KC_M,  KC_COMM,   KC_DOT,     CTL_T(KC_SLSH),         KC_RSFT,
  LT(_SYMB,KC_GRV),      KC_QUOT,  LALT(KC_LSFT),  KC_LEFT,  KC_RGHT,                                           KC_UP, KC_DOWN,  KC_LBRC,            KC_RBRC,       TT(_SYMB),

                                                                 ALT_T(KC_APP),  KC_LGUI,      KC_RALT,  CTL_T(KC_ESC),
                                                                                 KC_HOME,      KC_PGUP,
                                                          KC_SPC,      KC_BSPC,   KC_END,      KC_PGDN, KC_TAB, KC_ENT
),
/* Keymap 1: Symbol Layer
 *
 * ,-----------------------------------------.                    ,-----------------------------------------.
 * |Versn |  F1  |  F2  |  F3  |  F4  |  F5  |                    |  F6  |  F7  |  F8  |  F9  |  F10 |  F11 |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |   !  |   @  |   {  |   }  |   |  |                    |  Up  |   7  |   8  |   9  |   *  |  F12 |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |   #  |   $  |   (  |   )  |   `  |                    | Down |   4  |   5  |   6  |   +  |      |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |   %  |   ^  |   [  |   ]  |   ~  |                    |   &  |   1  |   2  |   3  |   \  |      |
 * |------+------+------+------+------+------'                    `------+------+------+------+------+------|
 * |RESET |      |      |      |      |                                  |      |   .  |   0  |   =  |      |
 * `----------------------------------'                                  `----------------------------------'
 *                                      ,-------------.  ,-------------.
 *                                      |      |      |  |      |      |
 *                               ,------|------|------|  |------+------+------.
 *                               |      |      |      |  |      |      |      |
 *                               |      |      |------|  |------|      |      |
 *                               |      |      |      |  |      |      |      |
 *                               `--------------------'  `--------------------'
 */
// SYMBOLS
[_SYMB] = LAYOUT_pterodactyl(
       // left hand
       KC_TRNS,    KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,                KC_F6,      KC_F7,   KC_F8, KC_F9,  KC_F10,   KC_F11,
       KC_TRNS,  KC_EXLM,    KC_AT,  KC_LCBR,  KC_RCBR,  KC_PIPE,                KC_UP,       KC_7,    KC_8,  KC_9, KC_ASTR,   KC_F12,
       KC_TRNS,  KC_HASH,   KC_DLR,  KC_LPRN,  KC_RPRN,   KC_GRV,                KC_DOWN,     KC_4,    KC_5,  KC_6, KC_PLUS,  KC_TRNS,
       KC_TRNS,  KC_PERC,  KC_CIRC,  KC_LBRC,  KC_RBRC,  KC_TILD,                KC_AMPR,     KC_1,    KC_2,  KC_3, KC_BSLS,  KC_TRNS,
         RESET,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,                                    KC_TRNS,  KC_DOT,  KC_0,  KC_EQL,  KC_TRNS,

                                                     KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,
                                                               KC_TRNS,      KC_TRNS,
                                           KC_TRNS,  KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,  KC_TRNS
),
/* Keymap 2: Media and mouse keys
 * ,-----------------------------------------.                    ,-----------------------------------------.
 * |      |      |      |      |      |      |                    |      |      |      |      |      |      |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |      |      | MsUp |      |      |                    |      |      |      |      |      |      |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |      |MsLeft|MsDown|MsRght|      |                    |      |      |      |      |      | Play |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |      |      |      |      |      |                    |      |      | Prev | Next |      |      |
 * |------+------+------+------+------+------'                    `------+------+------+------+------+------|
 * |      |      |      | Lclk | Rclk |                                  | VolUp| VolDn| Mute |      |      |
 * `----------------------------------'                                  `----------------------------------'
 *                                      ,-------------.  ,-------------.
 *                                      |      |      |  |      |      |
 *                               ,------|------|------|  |------+------+------.
 *                               |      |      |      |  |      |      |Brwser|
 *                               |      |      |------|  |------|      |Back  |
 *                               |      |      |      |  |      |      |      |
 *                               `--------------------'  `--------------------'
 *
 */
// MEDIA AND MOUSE
[_MDIA] = LAYOUT_pterodactyl(
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_MS_U,  KC_TRNS,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,
       KC_TRNS,  KC_TRNS,  KC_MS_L,  KC_MS_D,  KC_MS_R,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_MPLY,
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_MPRV,  KC_MNXT,  KC_TRNS,  KC_TRNS,
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_BTN1,  KC_BTN2,                                    KC_VOLU,  KC_VOLD,  KC_MUTE,  KC_TRNS,  KC_TRNS,
                                                    KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,
                                                              KC_TRNS,      KC_TRNS,
                                          KC_TRNS,  KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,  KC_WBAK
),
};


// TODO: remove patch
#ifdef PROTOCOL_CHIBIOS
#    pragma message("ChibiOS is currently 'best effort' and might not report accurate results")

i2c_status_t i2c_start_bodge(uint8_t address, uint16_t timeout) {
    i2c_start(address);

    // except on ChibiOS where the only way is do do "something"
    uint8_t data = 0;
    return i2c_readReg(address, 0, &data, sizeof(data), timeout);
}

#    define i2c_start i2c_start_bodge
#endif

// const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
//     LAYOUT_ortho_1x1(KC_A)
// };

void do_scan(void) {
    uint8_t nDevices = 0;

    dprintf("Scanning...\n");
    for (uint8_t address = 1; address < 127; ) {
        // The i2c_scanner uses the return value of
        // i2c_start to see if a device did acknowledge to the address.
        i2c_status_t error = i2c_start(address << 1, timeout);
        if (error == I2C_STATUS_SUCCESS) {
            i2c_stop();
            dprintf("  I2C device found at address %X\n", address);
            nDevices++;
            address++;
        // } else if(I2C_STATUS_TIMEOUT == error){
        //     timeout *=2;
            // dprintf("  Increasing timeout: (%u) at address %X\n", timeout, address);
        // } else if(I2C_STATUS_ERROR == error){
        //     address++;
        } else {
            // dprintf("  Unknown error (%u) at address %X\n", error, address);
            address++;
        }
    }

    if (nDevices == 0)
        dprintf("No I2C devices found\n");
    else
        dprintf("done\n");
}


void keyboard_post_init_user(void) {
    debug_enable = true;
    debug_matrix = true;

    i2c_init();
    unsigned char sys_status;
    i2c_status_t status = bno055_get_system_status_code(&sys_status);
    dprintf("bno055_get_system_status_code %X %x \n", status, sys_status);

    // i2c_status_t error = i2c_start(BNO055_I2C_ADDR << 1, I2C_TIMEOUT);
    // if (error == I2C_STATUS_SUCCESS) {
    //     int status = BNO_Init(&myBNO);
    //     dprintf("BNO_Init %d \n", status);
    //     status = bno055_set_operation_mode(OPERATION_MODE_CONFIG);
    //     dprintf("bno055_set_operation_mode %d OPERATION_MODE_CONFIG\n", status);
    //     status = bno055_set_powermode(POWER_MODE_NORMAL);
    //     dprintf("bno055_set_powermode %d POWER_MODE_NORMAL\n", status);
    //     status = bno055_set_operation_mode(OPERATION_MODE_NDOF);
    //     dprintf("bno055_set_operation_mode %d OPERATION_MODE_NDOF\n", status);

    //     unsigned char chip_id;
    //     BNO055_U16 sw_id;
    //     unsigned char pg_id;
    //     status = bno055_read_chip_id(&chip_id);
    //     dprintf("chip_id %d %u bno055_read_chip_id\n", status, chip_id);
    //     status = bno055_read_sw_revision_id(&sw_id);
    //     dprintf("sw_id %d %u bno055_read_sw_revision_id\n", status, sw_id);
    //     status = bno055_read_page_id(&pg_id);
    //     dprintf("pg_id %d %u bno055_read_page_id\n", status, pg_id);
    //     // i2c_stop();
    // } else {
    //     dprintf("  Unknown error (%u) at address %u\n", error, BNO055_I2C_ADDR << 1);
    // }

    // use low power mode where accelerometer is always on
    // unsigned char mode = 0;
    // int status = BNO055_I2C_bus_write(BNO055_I2C_ADDR << 1, BNO055_PWR_MODE_ADDR, &mode, I2C_TIMEOUT);
    // dprintf("%d ", status);

    scan_timer = timer_read();

//     BNO_Init(&myBNO); //Assigning the structure to hold information about the device
//     bno055_set_operation_mode(OPERATION_MODE_NDOF);
// dprintf("i2c_init %d %d",myBNO.chip_id,myBNO.sw_revision_id);
}



void matrix_scan_user(void) {
        read_bno055();

    // if (timer_elapsed(scan_timer) > 1000) {
    //     // do_scan();
    //     read_bno055();

    //     scan_timer = timer_read();
    // }
}

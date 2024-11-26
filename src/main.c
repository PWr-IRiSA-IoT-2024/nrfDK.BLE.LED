#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define LED0_PIN DT_GPIO_PIN(DT_ALIAS(led0), gpios)
#define LED1_PIN DT_GPIO_PIN(DT_ALIAS(led1), gpios)
#define LED2_PIN DT_GPIO_PIN(DT_ALIAS(led2), gpios)
#define LED3_PIN DT_GPIO_PIN(DT_ALIAS(led3), gpios)

static const uint32_t LED_PINS[] = {LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN};
static const struct device *led_device;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "LED_BLE_nrfDK", sizeof("LED_BLE_nrfDK") - 1),
};

#define LED_SERVICE_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x1234567890AB))
#define LED_CHARACTERISTIC_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xABCD1234, 0x5678, 0x5678, 0x5678, 0x1234567890AB))

static struct k_work led_work;
static uint8_t current_const_state = 0x00;
static uint8_t current_blink_state = 0x00;

void control_leds(struct k_work *work) {

    uint8_t blink_state = current_blink_state;
    uint8_t constant_state = current_const_state;

    printk("LED blinking state: 0x%02X, constant state: 0x%02X\n", blink_state, constant_state);

    uint8_t constant_mask = (constant_state & 0xF0) >> 4; // First 4 bits 0xF0 = 11110000
    for (int i = 0; i < 4; i++) {
        gpio_pin_set(led_device, LED_PINS[i], (constant_mask & (1 << i)) ? 0 : 1);
    }
    if (constant_mask != 0) {
        return;
    }

    uint8_t once_mask = (blink_state & 0xF0) >> 4; // First 4 bits 0xF0 = 11110000
    uint8_t twice_mask = blink_state & 0x0F;      // Last 4 bits 0x0F = 00001111
    while (true)
    {    
        for (int i = 0; i < 4; i++) {
            uint8_t blink_count = ((once_mask & (1 << i)) ? 1 : 0) +
                                ((twice_mask & (1 << i)) ? 2 : 0);
            // printk("LED %d: %d blinks\n", i + 1, blink_count);
            for (int j = 0; j < blink_count; j++) {
                gpio_pin_set(led_device, LED_PINS[i], 0);
                k_msleep(300); // Blink delay
                gpio_pin_set(led_device, LED_PINS[i], 1);
                k_msleep(300);
            }
        }
        k_msleep(2000); // Delay between blinks
    }
}

ssize_t write_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != 2) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    current_blink_state = ((uint8_t *)buf)[0];
    current_const_state = ((uint8_t *)buf)[1];
    printk("Received state const: 0x%02X, blink: 0x%02X\n", current_const_state, current_blink_state);


    k_work_submit(&led_work);
    return len;
}

ssize_t read_state(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                       uint16_t len, uint16_t offset)
{
    printk("Read LED state requested, 0x%04X\n", current_const_state);
    uint8_t *state_ptr = (uint8_t *)buf;
    state_ptr[0] = current_blink_state;
    state_ptr[1] = current_const_state;

    return sizeof(current_blink_state) + sizeof(current_const_state);
}

BT_GATT_SERVICE_DEFINE(led_svc,
    BT_GATT_PRIMARY_SERVICE(LED_SERVICE_UUID),
    BT_GATT_CHARACTERISTIC(LED_CHARACTERISTIC_UUID,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                            read_state, write_state, NULL),
);

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Bluetooth advertising failed (err %d)\n", err);
        return;
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
    } else {
        printk("Connected\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);

    int err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Bluetooth advertising failed (err %d)\n", err);
    } else {
        printk("Bluetooth advertising restarted\n");
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

void main(void)
{
    int err;

    // Initialize LEDs 
    led_device = DEVICE_DT_GET(DT_GPIO_CTLR(DT_ALIAS(led0), gpios));
    if (!led_device) {
        printk("LED device not found\n");
        return;
    }

    gpio_pin_configure(led_device, LED0_PIN, GPIO_OUTPUT);
    gpio_pin_configure(led_device, LED1_PIN, GPIO_OUTPUT);
    gpio_pin_configure(led_device, LED2_PIN, GPIO_OUTPUT);
    gpio_pin_configure(led_device, LED3_PIN, GPIO_OUTPUT);
    gpio_pin_set(led_device, LED0_PIN, 1);
    gpio_pin_set(led_device, LED1_PIN, 1);
    gpio_pin_set(led_device, LED2_PIN, 1);
    gpio_pin_set(led_device, LED3_PIN, 1);

    // Initialize BLE
    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    k_work_init(&led_work, control_leds);

    printk("LED control service started\n");
}

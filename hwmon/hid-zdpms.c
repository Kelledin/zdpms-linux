/*
 *  Linux hwmon driver for Enermax ZDPMS controller
 *
 *  Copyright (c) 2015 Frank Schaefer <kelledin@gmail.com>
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/hid.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <asm/atomic.h>

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Enermax ZDPMS controller");
MODULE_AUTHOR("Frank Schaefer <kelledin@gmail.com>");

/* TODO: if we integrate with the mainline kernel, we can just include
 * hid-ids.h from drivers/hid/.  This isn't in include/linux yet for some
 * reason. :-/
 */
#ifndef USB_VENDOR_ID_MICROCHIP
#  define USB_VENDOR_ID_MICROCHIP        0x04d8
#endif

#ifndef USB_DEVICE_ID_MICROCHIP_ZDPMS
#  define USB_DEVICE_ID_MICROCHIP_ZDPMS  0xf590
#endif

static const struct hid_device_id zdpms_devices[] = {
    { HID_USB_DEVICE(USB_VENDOR_ID_MICROCHIP, USB_DEVICE_ID_MICROCHIP_ZDPMS) },
    { }
};
MODULE_DEVICE_TABLE(hid, zdpms_devices);

static int zdpms_timeout = 100;
module_param(zdpms_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(zdpms_timeout,
                 "Transaction timeout in milliseconds (minimum 20)");

static int zdpms_12v_config = 0;
module_param(zdpms_12v_config, int, S_IRUGO);
MODULE_PARM_DESC(zdpms_12v_config,
                 "Set 12V rail configuration (1: single rail, 2: dual rail)");

enum {
    /* No idea what these mean, but they are part of the initialization
     * sequence used by the Enermax ZDPMS OEM application, with the
     * single byte value 0x5 passed to each one.  I suspect they set
     * something, I just don't know what.
     */
    ZDPMS_INIT1 = 0x25,
    ZDPMS_INIT2 = 0x24,
    ZDPMS_INIT3 = 0x23,

    /* 12V rail configuration setting.  Passing byte value 0x1 sets
     * single-rail mode, while passing byte value 0x2 sets dual-rail
     * mode.  Some PSU models may support more values for more rail
     * configurations, but this is what we get on our one sample.
     */
    ZDPMS_12V_CONFIG = 0x26,

    /* Sensor values, as near as we can tell. */
    ZDPMS_SENSOR_MIN = 0x27,      /* Minimum sensor value */
    ZDPMS_AC_V = 0x27,            /* AC input voltage level */
    ZDPMS_DC_12V1 = 0x28,         /* One of the 12V rail voltage levels.  Which one?  Who knows? */
    ZDPMS_DC_5V = 0x29,           /* DC 5V rail voltage level. */
    ZDPMS_DC_3V = 0x2a,           /* DC 3.3V rail voltage level */
    ZDPMS_DC_12V2 = 0x2b,         /* One of the 21V rail voltage levels.  Which one?  Who knows? */
    ZDPMS_DC_12V1_CURRENT = 0x2c, /* 12V1 current level in amperes */
    ZDPMS_DC_5V_CURRENT = 0x2d,   /* 5V current level in amperes */
    ZDPMS_DC_3V_CURRENT = 0x2e,   /* 3.3V current level in amperes */
    ZDPMS_DC_12V2_CURRENT = 0x2f, /* 12V2 current level in amperes */
    ZDPMS_TEMP = 0x30,            /* PSU temperature in degrees Celcius */
    ZDPMS_SENSOR_MAX = 0x30,      /* Maximum sensor value */

    ZDPMS_MODEL_NUMBER = 0x32,    /* PSU model number string, arbitrary length */

    /* Message types (a.k.a. HID report IDs) that we've seen. */
    ZDPMS_SET_CONFIG = 0x82,      /* Used for setting ZDPMS configuration. */
    ZDPMS_READ_SENSOR = 0x83,     /* Used for reading ZDPMS values. */
    ZDPMS_WTF_1 = 0x90,           /* No idea at all.  Doesn't get a valid response. */
    ZDPMS_WTF_2 = 0x91,           /* As above, no idea at all. */
};

static const char *zdpms_labels[] = {
    "AC input voltage",
    "DC 12V1",
    "DC 5V",
    "DC 3.3V",
    "DC 12V2",
    "DC 12V1 current",
    "DC 5V current",
    "DC 3.3V current",
    "DC 12V2 current",
    "PSU temp",
};

struct zdpms_device {
    /* The hid_device we belong to. */
    struct hid_device *hdev;

    /* The hwmon device pointer. */
    struct device *hwmon_dev;

    /* The wait queue used to track transaction replies. */
    wait_queue_head_t io_wait;

    /* The atomic condition variable to indicate that a message is received. */
    atomic_t msg_received;

    /* The last data read by the raw event handler.  This is expected to be a
     * command or query response (up to 64 bytes), and to the best of my
     * knowledge, only one pending command or query is supported at any time.
     */
    uint8_t data[64];
    unsigned int sz;

    /* The ZDPMS model number (i.e. "EDF550AWN" for the Enermax DigiFanless
     * 550W).  We don't expect this to be more than 32 bytes.
     */
    char model[32];

    /* Transaction lock mutex.  This keeps us from launching a second
     * command/query while one is still pending.
     */
    struct mutex io_lock;

};

/**
 * Perform a command/response or query/response exchange with an Enermax
 * ZDPMS controller.  The command is launched immediately through the
 * controller's interrupt endpoint; the response is copied back for the
 * caller to perform further analysis.  A minimum response timeout
 * (maintained in the zdpms_timeout variable) is implicit to avoid
 * potential deadlocks.  No validation is performed on the response
 * content.
 *
 * Note that commands, queries, and responses are generally zero padded to
 * 64 bytes.  The first byte of any message must be the report ID (typically
 * either ZDPMS_SET_CONFIG or ZDPMS_READ_SENSOR); the second byte is supposed
 * to be the length of the message payload, minus the first two bytes and any
 * zero padding.  For successful commands, the ZDPMS controller appears to
 * copy the report ID to the end of the message; for unsupported commands,
 * the ZDPMS controller appears to set the length byte to 0xFF in the response.
 *
 * @param hdev
 *     The hid_device to which we're sending the command/query
 *
 * @param msg
 *     The command/query to send to the ZDPMS controller, starting with the
 *     report ID and message-length byte
 *
 * @param sz
 *     The size of the command/query in bytes, including the report ID and
 *     message-length byte
 *
 * @param resp
 *     A user-supplied buffer to store the response.  This MUST be at least
 *     64 bytes in order to avoid potential buffer overflows.
 *
 * @return
 *     Returns the number of bytes read on success (usually 64), or a
 *     negative errno value on failure.
 */
static int zdpms_xchg(struct hid_device *hdev,
                          const uint8_t *data,
                           unsigned int  sz,
                                uint8_t *resp)
{
    uint8_t msg[64];
    struct zdpms_device *dev;
    unsigned long last_jiffy;
    int ret;

    if (hdev == NULL || (sz > 0 && msg == NULL) || resp == NULL) {
        return -EINVAL;
    }

    if (sz > sizeof(msg)) {
        return -E2BIG;
    }

    dev = hid_get_drvdata(hdev);

    if (dev == NULL) {
        return -EINVAL;
    }

    /* Copy the message and clear any padding. */
    memcpy(msg, data, sz);

    if (sz < sizeof(msg)) {
        memset(msg + sz, 0, sizeof(msg) - sz);
    }

    mutex_lock(&(dev->io_lock));

    /* Send the command/query.  We use hid_hw_output_report() specifically
     * because it will send the raw message to an interrupt endpoint.
     * NOTE: No return type (WTF really?).  If it fails, we'll supposedly
     * find out when we time out waiting for the response...
     */
    hid_hw_output_report(hdev, msg, sizeof(msg));

    /* Clamp the timeout to 20ms.  Any smaller and it's dicey to represent
     * it in jiffies...
     */
    if (zdpms_timeout < 20) {
        zdpms_timeout = 20;
    }

    /* We track the last allowable jiffy, in order to avoid our timeout getting
     * mistakenly prolonged by a signal storm or similar event.
     */
    last_jiffy = jiffies + msecs_to_jiffies(zdpms_timeout);

    do {
        ret = wait_event_interruptible_timeout(dev->io_wait,
                                               atomic_read(&(dev->msg_received)),
                                               last_jiffy - jiffies);

        if (ret == -ERESTARTSYS) {
            /* Interrupted by a signal.  Keep trying... */
            continue;
        }

        if (ret < 0) {
            /* Unspecified error.  Punt! */
            goto fail_mutex_locked;
        }

        if (ret == 0) {
            /* Timed out! */
            ret = -ETIMEDOUT;
            goto fail_mutex_locked;
        }

        if (ret > 0) {
            /* Message is received. */
            break;
        }
    } while (jiffies < last_jiffy);

    /* Now see about copying the received data to the user-supplied buffer. */
    memcpy(resp, dev->data, dev->sz);
    ret = dev->sz;
    atomic_set(&(dev->msg_received), 0);

fail_mutex_locked:
    mutex_unlock(&(dev->io_lock));
    return ret;
}

/**
 * Sets a specific ZDPMS configuration item.  Currently-known configuration
 * commands only take a one-byte configuration value.  Response validation
 * is performed.
 *
 * @param hdev
 *     Pointer to the hid_device structure to configure
 *
 * @param cmd
 *     Device setting to configure
 *
 * @param val
 *     Configuration setting to apply
 *
 * @return
 *     Returns the size of the return message on success, or a negative
 *     errno value on failure
 */
static int zdpms_set(struct hid_device *hdev, uint8_t cmd, uint8_t val)
{
    uint8_t msg[8], resp[64];
    int result;

    if (hdev == NULL) {
        return -1;
    }

    msg[0] = ZDPMS_SET_CONFIG;
    msg[1] = 0x03; /* 3-byte message */
    msg[2] = 0x40; /* no idea what this is supposed to mean */
    msg[3] = cmd;  /* Command value */
    msg[4] = val;

    result = zdpms_xchg(hdev, msg, 5, resp);

    if (result < 0) {
        return result;
    }

    if (result < 6) {
        return -EMSGSIZE;
    }

    if (resp[1] == 0xff) {
        /* Unsupported command? */
        return -ENOSYS;
    }

    if (resp[0] != msg[0] || resp[1] != 4 || resp[2] != msg[2] ||
        resp[3] != msg[3] || resp[4] != msg[4] || resp[5] != resp[0]) {
        return -EPROTO;
    }

    return result;
}

/**
 * Reads the ASCII model number string from the ZDPMS controller and stores it
 * in the caller-supplied buffer and appends a NULL terminator if possible.  If
 * the buffer is not large enough to hold the entire string, it will be
 * truncated to fit (but no NULL terminator will be added).
 *
 * @param hdev
 *     The hid_device we're attempting to query
 *
 * @param buf
 *     The caller-supplied buffer to store the model number string
 *
 * @param sz
 *     The maximum buffer size (in bytes)
 *
 * @return
 *     Returns the length of the model-number string in bytes (not counting
 *     the NULL terminator), or a negative errno value on failure.  The
 *     returned string length may be larger than the supplied buffer size.
 *
 */
static int zdpms_read_model(struct hid_device *hdev, char *buf, size_t sz)
{
    uint8_t msg[8], resp[64];
    int result;

    if (hdev == NULL || buf == NULL || sz <= 0) {
        return -EINVAL;
    }

    msg[0] = ZDPMS_READ_SENSOR;
    msg[1] = 0x2; /* 2-byte message */
    msg[2] = 0x40; /* nope, still no idea what this is for. */
    msg[3] = ZDPMS_MODEL_NUMBER;

    if ((result = zdpms_xchg(hdev, msg, 4, resp)) < 0) {
        return result;
    }

    if (result < 6) {
        return -EMSGSIZE;
    }

    if (resp[1] == 0xff) {
        return -ENOSYS;
    }

    if (resp[1] + 2 > 64 || resp[0] != msg[0] || resp[2] != msg[2] ||
        resp[3] != msg[3] || resp[resp[1]+1] != resp[0]) {
        return -EPROTO;
    }

    result = resp[1] - 3;

    if (result < sz) {
        /* Good news, we can fit the entire model name in the supplied buffer,
         * complete with a NULL terminator.
         */
        memcpy(buf, resp + 4, result);
        buf[result] = '\0';
    }
    else {
        /* Whoops, we can only fit in part of it. */
        memcpy(buf, resp + 4, sz);
    }

    return result;
}

/**
 * Query a sensor reading from a ZDPMS controller.  This reading is reported
 * in millivolts, milliamperes, or millidegrees Celcius, in order to conform
 * with the Linux hwmon subsystem expectations.
 *
 * @param hdev
 *     The hid_device we're attempting to query
 *
 * @param sensor
 *     The sensor index we're attempting to query
 *
 * @return
 *     Returns the sensor value (currently always >=0) on success, or a
 *     negative errno value on failure.
 *
 */
static int zdpms_read_sensor(struct hid_device *hdev, uint8_t sensor)
{
    uint8_t msg[8], resp[64];
    int result;
    uint32_t val;
    unsigned int exponent;

    if (hdev == NULL) {
        return -EINVAL;
    }

    msg[0] = ZDPMS_READ_SENSOR;
    msg[1] = 0x2; /* 2-byte message */
    msg[2] = 0x40; /* no idea what this is supposed to mean */
    msg[3] = sensor; /* Sensor value */

    if ((result = zdpms_xchg(hdev, msg, 4, resp)) < 4) {
        return result;
    }

    if (result < 7) {
        return -EMSGSIZE;
    }

    if (resp[1] == 0xff) {
        /* Nonexistent sensor? */
        return -ENODEV;
    }

    if (resp[0] != msg[0] || resp[1] != 5 || resp[2] != msg[2] ||
        resp[3] != msg[3] || resp[6] != resp[0]) {
        return -EPROTO;
    }

    /* The sensor value is encoded as little-endian quasi-IEEE 754, in
     * bytes 4 and 5.
     */
    val = ((resp[5] & 7) << 8) | resp[4];
    exponent = resp[5] >> 3;

    /* We are't sure yet if the mantissa really extends to 11 bits.  Bit 11
     * may belong to the exponent field instead...
     */
    if (val > 1023) {
        hid_warn(hdev, "sensor 0x%02x mantissa %u is greater than 1023?\n", sensor, val);
    }

    /* All sensor reports we've seen appear to have a 5-bit exponent field,
     * with the high bit always set.  We can calculate the shift value by
     * subtracting this exponent value from 32; however, if some reading
     * defies our empirical expectations and comes back with the high bit of
     * the exponent clear, then we're likely to generate an invalid (very
     * small or absolute zero) reading.
     */
    if (exponent < 16) {
        hid_warn(hdev,
                 "sensor 0x%02x exponent %u is less than 16?\n", sensor,
                 exponent);
    }

    /* All hwmon sensors we support are supposed to be reported in milliunits
     * (millivolts, millidegrees Celcius, or milliamperes).  Given that the
     * maximum possible value for the mantissa is 2^11 (2048), we'll remain
     * well within 32-bit integer range even after applying this multiplier.
     */
    val *= 1000;

    /* Apply the exponent shift *after* applying the units multiplier, so we
     * preserve as much accuracy as possible.
     */
    val >>= (32 - exponent);
    return val;
}

/**
 * Callback for reporting ZDPMS sensor labels via sysfs, in accordance with
 * standard hwmon conventions.
 *
 * @param dev
 *     hwmon device pointer.  The corresponding zdpms_device structure is
 *     referenced by the dev_get_drvdata() result and contains a pointer to
 *     the associated hid_device structure of the ZDPMS controller.
 *
 * @param attr
 *     sysfs device attribute pointer.  The corresponding sensor device
 *     attribute can be deduced from this and supplies the desired ZDPMS
 *     sensor ID.
 *
 * @param buf
 *     buffer for storing a string representation of the sensor reading.
 *
 * @return
 *     Returns the size of the string representation stored in the buffer,
 *     not including the NULL terminator, or a negative errno value on
 *     failure.
 *
 */
static ssize_t zdpms_show_label(struct device *dev,
                      struct device_attribute *attr,
                                         char *buf)
{
    struct sensor_device_attribute *sens_attr;
    struct zdpms_device            *zdev;
    struct hid_device              *hdev;
    unsigned int                    idx;

    if (dev == NULL || attr == NULL || buf == NULL) {
        return -EINVAL;
    }

    sens_attr = to_sensor_dev_attr(attr);
    zdev = dev_get_drvdata(dev);

    if (sens_attr == NULL || sens_attr->index > 255 ||
        zdev == NULL || zdev->hdev == NULL) {
        return -EINVAL;
    }

    hdev = zdev->hdev;
    idx = sens_attr->index;

    if (idx < ZDPMS_SENSOR_MIN || idx > ZDPMS_SENSOR_MAX) {
        return -ERANGE;
    }

    /* We use a straight sequential array as a lookup table. */
    idx -= ZDPMS_SENSOR_MIN;

    if (zdpms_labels[idx] == NULL) {
        return sprintf(buf, "\n");
    }

    return sprintf(buf, "%s\n", zdpms_labels[idx]);
}

/**
 * Callback for reporting ZDPMS sensor values via sysfs, in accordance with
 * standard hwmon conventions.
 *
 * @param dev
 *     hwmon device pointer.  The corresponding zdpms_device structure is
 *     referenced by the dev_get_drvdata() result and contains a pointer to
 *     the associated hid_device structure of the ZDPMS controller.
 *
 * @param attr
 *     sysfs device attribute pointer.  The corresponding sensor device
 *     attribute can be deduced from this and supplies the desired ZDPMS
 *     sensor ID.
 *
 * @param buf
 *     buffer for storing a string representation of the sensor reading.
 *
 * @return
 *     Returns the size of the string representation stored in the buffer,
 *     not including the NULL terminator, or a negative errno value on
 *     failure.
 *
 */
static ssize_t zdpms_show_sensor(struct device *dev,
                       struct device_attribute *attr,
                                          char *buf)
{
    struct sensor_device_attribute *sens_attr;
    struct zdpms_device            *zdev;
    struct hid_device              *hdev;
    unsigned int                    idx;
    int                             val;

    if (dev == NULL || attr == NULL || buf == NULL) {
        return -EINVAL;
    }

    sens_attr = to_sensor_dev_attr(attr);
    zdev = dev_get_drvdata(dev);

    if (sens_attr == NULL || sens_attr->index > 255 ||
        zdev == NULL || zdev->hdev == NULL) {
        return -EINVAL;
    }

    hdev = zdev->hdev;
    idx = sens_attr->index;
    val = zdpms_read_sensor(hdev, idx);

    if (val < 0) {
        hid_err(hdev, "reading sensor 0x%02x FAILED (error %d)\n", idx, val);
        return val;
    }

    return sprintf(buf, "%d\n", val);
}

/**
 * Callback for reporting raw HID events from a ZDPMS controller.  ZDPMS
 * controllers generally only send HID events in response to commands or
 * queries sent to their interrupt endpoints; generally only one such
 * command or query is expected to be pending at any time.
 *
 * @param hdev
 *     Pointer to the HID device (ZDPMS controller) responsible for this
 *     HID event
 *
 * @param report
 *     Pointer to the HID report structure for this event.  This is
 *     generally ignored, since we prefer to parse raw messages.
 *
 * @param data
 *     Pointer to the HID data payload from this event (not including any
 *     headers such as the USB Request Block (URB)
 *
 * @param sz
 *     Size of the HID data payload from this event (usually 64 bytes).
 *
 * @return
 *     In accordance with the kernel HID API, returns 0 when the event has
 *     been successfully parsed, or -1 if a parse error occurs.
 *
 */
static int zdpms_raw_event(struct hid_device *hdev,
                           struct hid_report *report,
                                          u8 *data,
                                         int  sz)
{
    struct zdpms_device *dev;

    if (hdev == NULL || report == NULL || (sz > 0 && data == NULL)) {
        return -1;
    }

    dev = hid_get_drvdata(hdev);

    if (dev == NULL) {
        return -1;
    }

    /* We shouldn't receive any messages larger than 64 bytes.  If we do, we
     * need to clip it so it fits in our buffer.
     */
    if (sz > 64) {
        hid_warn(hdev, "received oversize %d-byte event message; excess data discarded!\n", sz);
        sz = 64;
    }

    memcpy(dev->data, data, sz);
    dev->sz = sz;

    if (atomic_inc_return(&(dev->msg_received)) > 1) {
        /* This implies we already had a message pending, and we just overwrote
         * it.  D'oh!
         */
        hid_warn(hdev, "overwriting unacknowledged message\n");
    }

    /* Now wake up anyone waiting on a transaction to complete. */
    wake_up_interruptible(&(dev->io_wait));
    return 0;
}

#define ZDPMS_ATTR(name, id) \
        static SENSOR_DEVICE_ATTR(name##_input, S_IRUGO, zdpms_show_sensor, NULL, id); \
        static SENSOR_DEVICE_ATTR(name##_label, S_IRUGO, zdpms_show_label, NULL, id);

ZDPMS_ATTR(in1, ZDPMS_DC_12V1);
ZDPMS_ATTR(in2, ZDPMS_DC_5V);
ZDPMS_ATTR(in3, ZDPMS_DC_3V);
ZDPMS_ATTR(in4, ZDPMS_DC_12V2);
ZDPMS_ATTR(in5, ZDPMS_AC_V);
ZDPMS_ATTR(curr1, ZDPMS_DC_12V1_CURRENT);
ZDPMS_ATTR(curr2, ZDPMS_DC_5V_CURRENT);
ZDPMS_ATTR(curr3, ZDPMS_DC_3V_CURRENT);
ZDPMS_ATTR(curr4, ZDPMS_DC_12V2_CURRENT);
ZDPMS_ATTR(temp1, ZDPMS_TEMP);

static struct attribute *zdpms_attrs[] = {
    &sensor_dev_attr_in1_label.dev_attr.attr,
    &sensor_dev_attr_in1_input.dev_attr.attr,
    &sensor_dev_attr_in2_label.dev_attr.attr,
    &sensor_dev_attr_in2_input.dev_attr.attr,
    &sensor_dev_attr_in3_label.dev_attr.attr,
    &sensor_dev_attr_in3_input.dev_attr.attr,
    &sensor_dev_attr_in4_label.dev_attr.attr,
    &sensor_dev_attr_in4_input.dev_attr.attr,
    &sensor_dev_attr_in5_label.dev_attr.attr,
    &sensor_dev_attr_in5_input.dev_attr.attr,
    &sensor_dev_attr_curr1_label.dev_attr.attr,
    &sensor_dev_attr_curr1_input.dev_attr.attr,
    &sensor_dev_attr_curr2_label.dev_attr.attr,
    &sensor_dev_attr_curr2_input.dev_attr.attr,
    &sensor_dev_attr_curr3_label.dev_attr.attr,
    &sensor_dev_attr_curr3_input.dev_attr.attr,
    &sensor_dev_attr_curr4_label.dev_attr.attr,
    &sensor_dev_attr_curr4_input.dev_attr.attr,
    &sensor_dev_attr_temp1_label.dev_attr.attr,
    &sensor_dev_attr_temp1_input.dev_attr.attr,
    NULL,
};

ATTRIBUTE_GROUPS(zdpms);

static int zdpms_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
    struct zdpms_device *dev;
    int ret;

    if ((ret = hid_parse(hdev)) < 0) {
        hid_err(hdev, "parse failed\n");
        return ret;
    }

    if ((ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW)) < 0) {
        hid_err(hdev, "hw start failed\n");
        return ret;
    }

    if ((ret = hid_hw_open(hdev)) < 0) {
        hid_err(hdev, "hw open failed\n");
        goto fail_hid_started;
    }

    if ((ret = hid_hw_power(hdev, PM_HINT_FULLON)) < 0) {
        hid_err(hdev, "power management error: %d\n", ret);
        goto fail_hid_open;
    }

    dev = kzalloc(sizeof(struct zdpms_device), GFP_KERNEL);

    if (dev == NULL) {
        ret = -ENOMEM;
        goto fail_hid_power_on;
    }

    /* Initialize the private device data structure. */
    dev->hdev = hdev;
    init_waitqueue_head(&(dev->io_wait));
    atomic_set(&(dev->msg_received), 0);
    mutex_init(&(dev->io_lock));
    hid_set_drvdata(hdev, dev);

    /* Allow event processing during probe. */
    hid_device_io_start(hdev);

    /* Perform the initialization sequence */
    if ((ret = zdpms_set(hdev, ZDPMS_INIT1, 0x5)) < 0 ||
        (ret = zdpms_set(hdev, ZDPMS_INIT2, 0x5)) < 0 ||
        (ret = zdpms_set(hdev, ZDPMS_INIT3, 0x5)) < 0) {
        hid_err(hdev, "Primary initialization sequence FAILED (error %d)\n", ret);
        goto fail_io_started;
    }

    if (zdpms_12v_config > 0 && zdpms_12v_config < 3) {
        hid_info(hdev, "Setting 12V %s-rail configuration...",
                 zdpms_12v_config == 1 ? "single" : "dual");

        if ((ret = zdpms_set(hdev, ZDPMS_12V_CONFIG, zdpms_12v_config)) < 0) {
            hid_err(hdev, "12V rail configuration FAILED (error %d)\n", ret);
            goto fail_io_started;
        }
    }

    if ((ret = zdpms_read_model(hdev, dev->model, sizeof(dev->model))) < 0) {
        hid_err(hdev, "Model number query FAILED (error %d)\n", ret);
        goto fail_io_started;
    }

    if (ret >= sizeof(dev->model)) {
        hid_warn(hdev,
                 "model number truncated to %ld bytes (full size %d bytes)\n",
                 sizeof(dev->model) - 1, ret);
        dev->model[sizeof(dev->model) - 1] = '\0';
    }
    else {
        dev->model[ret] = '\0';
    }

    hid_info(hdev, "Detected model number: %s\n", dev->model);

    /* Undo hid_device_io_start(). */
    hid_device_io_stop(hdev);

    /* Now register this as an hwmon device. */
    dev->hwmon_dev = devm_hwmon_device_register_with_groups(&(hdev->dev),
                                                            "zdpms",
                                                            dev,
                                                            zdpms_groups);

    ret = PTR_ERR_OR_ZERO(dev->hwmon_dev);

    if (ret < 0) {
        hid_err(hdev, "hwmon device registration FAILED (error %d)\n", ret);
        goto fail_data_alloced;
    }

    return 0;

fail_io_started:
    hid_device_io_stop(hdev);
fail_data_alloced:
    hid_set_drvdata(hdev, NULL);
    kfree(dev);
fail_hid_power_on:
    hid_hw_power(hdev, PM_HINT_NORMAL);
fail_hid_open:
    hid_hw_close(hdev);
fail_hid_started:
    hid_hw_stop(hdev);
    return ret;
}

static void zdpms_remove(struct hid_device *hdev)
{
    struct zdpms_device *dev;

    if (hdev != NULL) {
        dev = hid_get_drvdata(hdev);

        if (dev != NULL) {
            hid_set_drvdata(hdev, NULL);
            kfree(dev);
        }

        hid_hw_power(hdev, PM_HINT_NORMAL);
        hid_hw_close(hdev);
        hid_hw_stop(hdev);
    }
}

static struct hid_driver zdpms_driver = {
    .name       = "zdpms",
    .id_table   = zdpms_devices,
    .probe      = zdpms_probe,
    .remove     = zdpms_remove,
    .raw_event  = zdpms_raw_event,
};

module_hid_driver(zdpms_driver);


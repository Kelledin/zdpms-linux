#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include <getopt.h>

#include "hidapi.h"

#define ZDPMS_VENDOR_ID   0x04d8
#define ZDPMS_PRODUCT_ID  0xf590
#define ZDPMS_TIMEOUT     500 /* Half second timeout */

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
    ZDPMS_MODEL_NUMBER = 0x32,    /* PSU model number string, arbitrary length */

    /* Message types (a.k.a. HID report IDs) that we've seen. */
    ZDPMS_SET = 0x82,             /* Used for setting ZDPMS configuration. */
    ZDPMS_GET = 0x83,             /* Used for reading ZDPMS values. */
    ZDPMS_WTF_1 = 0x90,           /* No idea at all.  Doesn't get a valid response. */
    ZDPMS_WTF_2 = 0x91,           /* As above, no idea at all. */
};

/* Set a specific ZDPMS configuration item.  Currently-known configuration
 * commands only take a one-byte configuration value.
 */
int zdpms_set(hid_device *dev, uint8_t cmd, uint8_t val)
{
    uint8_t msg[8], resp[8];
    int result;

    if (dev == NULL) {
        return -1;
    }

    msg[0] = ZDPMS_SET;
    msg[1] = 0x03; /* 3-byte message */
    msg[2] = 0x40; /* no idea what this is supposed to mean */
    msg[3] = cmd;  /* Command value */
    msg[4] = val;

    result = hid_write(dev, msg, 5);

    if (result < 0) {
        return result;
    }

    if (result < 5) {
        return -1;
    }

    result = hid_read_timeout(dev, resp, 6, ZDPMS_TIMEOUT);

    if (result < 0) {
        return result;
    }

    if (result < 6 || resp[0] != msg[0] || resp[1] != 0x04 ||
        resp[2] != msg[2] || resp[3] != msg[3] || resp[4] != msg[4] ||
        resp[5] != resp[0]) {
        return -1;
    }

    return result;
}

/* Read a ZDPMS sensor. */
float zdpms_read_sensor(hid_device *dev, uint8_t sensor)
{
    uint8_t msg[8], resp[8];
    int result;
    uint32_t val;
    unsigned int exponent;

    if (dev == NULL) {
        return NAN;
    }

    msg[0] = ZDPMS_GET;
    msg[1] = 0x2; /* 2-byte message */
    msg[2] = 0x40; /* no idea what this is supposed to mean */
    msg[3] = sensor; /* Sensor value */

    result = hid_write(dev, msg, 4);

    if (result < 0) {
        return NAN;
    }

    if (result < 4) {
        return NAN;
    }

    result = hid_read_timeout(dev, resp, 7, ZDPMS_TIMEOUT);

    if (result < 7 || resp[0] != msg[0] || resp[1] != 5 || resp[2] != msg[2] ||
        resp[3] != msg[3] || resp[6] != resp[0]) {
        return NAN;
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
        fprintf(stderr, "WARNING: sensor mantissa %u is greater than 1023?\n", val);
    }

    /* We can turn this into a 32-bit fixed point representation by shifting
     * it to the left based on the exponent minus 16.  However, if the
     * exponent is less than 16, I'm not sure what to do with it...
     */
    if (exponent < 16) {
        fprintf(stderr, "WARNING: sensor exponent %u is less than 16?\n", exponent);
    }
    else {
        exponent -= 16;
    }

    val <<= exponent;

    /* And divide it by 65536.0 to turn it into an equivalent floating-point
     * representation.
     */
    return (val / 65536.0);
}

char *zdpms_read_model(hid_device *dev, char *buf, size_t sz)
{
    uint8_t msg[8], resp[32];
    int result;

    if (dev == NULL || buf == NULL || sz <= 0) {
        errno = EINVAL;
        return NULL;
    }

    msg[0] = ZDPMS_GET;
    msg[1] = 0x2; /* 2-byte message */
    msg[2] = 0x40; /* nope, still no idea what this is for. */
    msg[3] = ZDPMS_MODEL_NUMBER;

    result = hid_write(dev, msg, 4);

    if (result < 4) {
        return NULL;
    }

    result = hid_read_timeout(dev, resp, 32, ZDPMS_TIMEOUT);

    if (result < 6 || resp[1] + 2 >= result || resp[0] != msg[0] ||
        resp[2] != msg[2] || resp[3] != msg[3] || resp[resp[1]+1] != resp[0]) {
        return NULL;
    }

    if (resp[1] - 3 < sz) {
        /* Good news, we can fit the entire model name in the supplied buffer,
         * complete with a NULL terminator.
         */
        memcpy(buf, resp + 4, resp[1] - 3);
        buf[resp[1] - 3] = '\0';
    }
    else {
        /* Whoops, we can only fit in part of it. */
        memcpy(buf, resp + 4, sz);
    }

    return buf;
}

int usage(const char *cmd, FILE *fd)
{
    const char *c;

    if (cmd == NULL || *cmd == '\0') {
        cmd = "zdpms-config";
    }

    if (fd == NULL) {
        fd = stdout;
    }

    for (c = cmd + strlen(cmd) - 1; c >= cmd && *c != '/'; c--) { };
    c++;

    fprintf(fd,
"USAGE: %s [-s|--single] [-d|--dual] [-p|--path <dev>] [-h|--help]\n"
"\n"
"    -s,--single     : configure single-rail mode\n"
"    -d,--dual       : configure dual-rail mode\n"
"    -p,--path <dev> : specify ZDPMS device path (usually /dev/hidrawX)\n"
"    -h,--help       : print help (this screen)\n"
"\n", c);

    return 0;
}

int main(int argc, char **argv)
{
    static const struct option long_opts[] = {
        { "single",  no_argument,        NULL,  's' },
        { "dual",    no_argument,        NULL,  'd' },
        { "help",    no_argument,        NULL,  'h' },
        { "path",    required_argument,  NULL,  'p' },  
        { NULL,      0,                  NULL,  0   },
    };

    struct hid_device_info *info_start = NULL;
    struct hid_device_info *info = NULL;
    hid_device             *dev = NULL;
    int                     ret = 0;
    int                     opt;
    unsigned int            rail_cfg = 0;
    const char             *path = NULL;
    char                   *psu_model;
    char                    buf[32];
    float                   ac_V;
    float                   dc_12V1;
    float                   dc_5V;
    float                   dc_3V;
    float                   dc_12V2;
    float                   dc_12V1_I;
    float                   dc_5V_I;
    float                   dc_3V_I;
    float                   dc_12V2_I;
    float                   dc_12V1_W;
    float                   dc_5V_W;
    float                   dc_3V_W;
    float                   dc_12V2_W;
    float                   psu_temp;

    do {
        opt = getopt_long(argc, argv, "sdp:h", long_opts, NULL);

        switch(opt) {
        case -1:
            break;

        case 's':
            rail_cfg = 1;
            break;

        case 'd':
            rail_cfg = 2;
            break;

        case 'p':
            path = optarg;
            break;

        case 'h':
            usage(*argv, NULL);
            return 0;
            break;

        case '?':
        default:
            usage(*argv, stderr);
            return 1;
            break;
        }
    } while (opt != -1);

    if (hid_init() < 0) {
        fprintf(stderr, "FATAL: HID initialization failure: %s\n", strerror(errno));
        return 1;
    }

    info_start = hid_enumerate(0, 0);

    for (info = info_start; info != NULL; info = info->next) {
        if (info->serial_number == NULL) {
            printf("HID device at %s: %04x:%04x (no serial number)\n",
                   info->path, info->vendor_id, info->product_id);
        }
        else {
            printf("HID device at %s: %04x:%04x (serial number '%ls')\n",
                   info->path, info->vendor_id, info->product_id,
                   info->serial_number);
        }

        if (info->manufacturer_string != NULL) {
            printf("    Manufacturer name: '%ls'\n", info->manufacturer_string);
        }

        if (info->product_string != NULL) {
            printf("    Product name: '%ls'\n", info->product_string);
        }

        if (info->vendor_id == ZDPMS_VENDOR_ID && info->product_id == ZDPMS_PRODUCT_ID) {
            printf("    Device appears to be an Enermax ZDPMS controller\n");

            if (path != NULL && strcmp(path, info->path) != 0) {
                printf("    Device not at specified path '%s'; ignoring...\n\n",
                       path);
                continue;
            }

            printf("\n");
            break;
        }

        printf("    Device is not supported; continuing enumeration...\n\n");
    }

    if (info == NULL) {
        printf("No supported ZDPMS device found.\n");
        ret = 1;
        goto fail_hid_enumerated;
    }

    /* The Enermax ZDPMS controller seems to be a bit slapdash; in a classic
     * rookie mistake, the device neglects to advertise a manufacturer string,
     * product string, or (most critically) a serial number.  Even worse,
     * Enermax appears to be drawing from Microchip Inc's USB ID pool, and I
     * fear Enermax may have stupidly neglected to secure any guarantee that
     * nobody else would choose the same USB vendor:product ID.
     *
     * To cope with the outside possibility of multiple ZDPMS-capable PSUs
     * being connected to a single host, we have to open the device by device
     * path rather than by vendor/product ID+serial number.  This path *should*
     * be unique per device instance on a single host.  Incidentally, if
     * Enermax and some other vendor end up clobbering each other's USB IDs,
     * we're most likely screwed sideways, and both vendors deserve to be
     * beaten soundly about the head and neck.
     */
    printf("Opening device by path '%s'\n", info->path);
    dev = hid_open_path(info->path);

    if (dev == NULL) {
        fprintf(stderr, "    FATAL: opening device: %s\n", strerror(errno));
        ret = 1;
        goto fail_hid_enumerated;
    }

    /* Attempt the initialization sequence. */
    printf("Attempting initialization sequence...");

    /* I have no idea what the first three configuration settings are supposed
     * to do.
     * TODO: The fourth initialization message is hardcoded to configure the
     * PSU for dual-rail mode.  We should probably add a command-line option
     * to change this.
     */
    if (zdpms_set(dev, ZDPMS_INIT1, 0x5) < 0 ||
        zdpms_set(dev, ZDPMS_INIT2, 0x5) < 0 ||
        zdpms_set(dev, ZDPMS_INIT3, 0x5) < 0) {
        fprintf(stderr, " FAILED\n");
        ret = 1;
        goto fail_dev_open;
    }

    printf(" done\n");

    if (rail_cfg != 0) {
        printf("Setting 12V %s-rail configuration...",
               rail_cfg == 1 ? "single" : "dual");

        if (zdpms_set(dev, ZDPMS_12V_CONFIG, rail_cfg) < 0) {
            fprintf(stderr, " FAILED\n");
            ret = 1;
            goto fail_dev_open;
        }

        printf(" done\n");
    }

    ac_V = zdpms_read_sensor(dev, ZDPMS_AC_V);
    dc_12V1 = zdpms_read_sensor(dev, ZDPMS_DC_12V1);
    dc_5V = zdpms_read_sensor(dev, ZDPMS_DC_5V);
    dc_3V = zdpms_read_sensor(dev, ZDPMS_DC_3V);
    dc_12V2 = zdpms_read_sensor(dev, ZDPMS_DC_12V2);
    dc_12V1_I = zdpms_read_sensor(dev, ZDPMS_DC_12V1_CURRENT);
    dc_5V_I = zdpms_read_sensor(dev, ZDPMS_DC_5V_CURRENT);
    dc_3V_I = zdpms_read_sensor(dev, ZDPMS_DC_3V_CURRENT);
    dc_12V2_I = zdpms_read_sensor(dev, ZDPMS_DC_12V2_CURRENT);
    psu_temp = zdpms_read_sensor(dev, ZDPMS_TEMP);
    psu_model = zdpms_read_model(dev, buf, sizeof(buf));
    dc_12V1_W = dc_12V1 * dc_12V1_I;
    dc_5V_W = dc_5V * dc_5V_I;
    dc_3V_W = dc_3V * dc_3V_I;
    dc_12V2_W = dc_12V2 * dc_12V2_I;

    printf("PSU model      : %s\n", psu_model);
    printf("AC input       : %fV\n", ac_V);
    printf("DC 12V1 rail   : %fV/%fA/%fW\n", dc_12V1, dc_12V1_I, dc_12V1_W);
    printf("DC 5V rail     : %fV/%fA/%fW\n", dc_5V, dc_5V_I, dc_5V_W);
    printf("DC 3.3V rail   : %fV/%fA/%fW\n", dc_3V, dc_3V_I, dc_3V_W);
    printf("DC 12V2 rail   : %fV/%fA/%fW\n", dc_12V2, dc_12V2_I, dc_12V2_W);
    printf("PSU temp       : %f C\n", psu_temp);
    printf("Total DC power : %fW\n", dc_12V1_W + dc_12V2_W + dc_5V_W + dc_3V_W);
    printf("Closing device...\n");

fail_dev_open:
    if (dev != NULL) {
        hid_close(dev);
    }
     
fail_hid_enumerated:
    if (info_start != NULL) {
        hid_free_enumeration(info_start);
    }

    hid_exit();    
    return ret;
}


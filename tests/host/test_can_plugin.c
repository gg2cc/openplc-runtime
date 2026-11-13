#include <assert.h>
#include <errno.h>
#include <linux/can.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/time.h>

static uint64_t now_ms = 1000;

static int test_gettimeofday(struct timeval *tv, void *tz)
{
    (void)tz;
    tv->tv_sec  = now_ms / 1000;
    tv->tv_usec = (now_ms % 1000) * 1000;
    return 0;
}

#define gettimeofday test_gettimeofday
#include "../../core/src/drivers/plugins/native/can/can_plugin.c"
#undef gettimeofday

static IEC_BOOL enabled;
static IEC_BOOL *outputs[4][8];
static IEC_BYTE value     = 42;
static IEC_BYTE *bytes[4] = {&value};
static int errors;
static int lock_depth;

static void log_ignore(const char *fmt, ...)
{
    (void)fmt;
}

static void log_error_count(const char *fmt, ...)
{
    (void)fmt;
    errors++;
}

static void image_lock(void)
{
    assert(lock_depth == 0);
    lock_depth++;
}

static void image_unlock(void)
{
    assert(lock_depth == 1);
    lock_depth--;
}

static void write_config(const char *path, const char *enable_json, const char *trigger)
{
    FILE *file = fopen(path, "w");
    assert(file);
    assert(fprintf(file,
                   "{\"interfaces\":[{\"interface\":\"testcan\","
                   "\"tx_frames\":[{\"can_id\":\"0x123\",\"dlc\":1,"
                   "\"trigger\":\"%s\",\"cycle_time_ms\":100,%s"
                   "\"mappings\":[{\"byte_offset\":0,\"data_type\":\"u8\","
                   "\"plc_address\":\"%%QB0\"}]}]}]}",
                   trigger, enable_json) > 0);
    assert(fclose(file) == 0);
}

static void expect_frame(int fd, uint8_t expected)
{
    struct can_frame frame;
    assert(recv(fd, &frame, sizeof(frame), MSG_DONTWAIT) == sizeof(frame));
    assert(frame.can_id == 0x123);
    assert(frame.can_dlc == 1);
    assert(frame.data[0] == expected);
}

static void expect_no_frame(int fd)
{
    struct can_frame frame;
    assert(recv(fd, &frame, sizeof(frame), MSG_DONTWAIT) == -1);
    assert(errno == EAGAIN || errno == EWOULDBLOCK);
    assert(lock_depth == 0);
}

static int connect_output(void)
{
    int pair[2];
    assert(socketpair(AF_UNIX, SOCK_DGRAM, 0, pair) == 0);
    g_can_fds[0]   = pair[0];
    g_can_fd_count = 1;
    return pair[1];
}

int main(void)
{
    char path[]   = "/tmp/openplc-can-enable-XXXXXX";
    int config_fd = mkstemp(path);
    assert(config_fd >= 0);
    assert(close(config_fd) == 0);
    outputs[2][7]              = &enabled;
    plugin_runtime_args_t args = {0};
    args.buffer_size           = 4;
    args.bool_output           = outputs;
    args.byte_output           = bytes;
    assert(snprintf(args.plugin_specific_config_file_path,
                    sizeof(args.plugin_specific_config_file_path), "%s", path) > 0);
    args.log_info     = log_ignore;
    args.log_error    = log_error_count;
    args.image_lock   = image_lock;
    args.image_unlock = image_unlock;

    const char *invalid[] = {
        "\"enable_plc_address\":\"%IX2.7\",", "\"enable_plc_address\":\"%QB2\",",
        "\"enable_plc_address\":\"%QX2.8\",", "\"enable_plc_address\":\"%QX2.\",",
        "\"enable_plc_address\":\"\",",       "\"enable_plc_address\":true,",
        "\"enable_plc_address\":null,",       "\"enable_plc_address\":\"%QX-1.0\",",
        "\"enable_plc_address\":\"%QX4.0\",", "\"enable_plc_address\":\"%QX2147483648.0\",",
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++)
    {
        write_config(path, invalid[i], "cyclic");
        int previous_errors = errors;
        assert(init(&args) == -1);
        assert(errors > previous_errors);
        cleanup();
    }

    write_config(path, "\"enable_plc_address\":\"%QX1.0\",", "cyclic");
    assert(init(&args) == 0);
    assert(start_loop() == -1);
    cleanup();

    write_config(path, "\"enable_plc_address\":\"%QX2.7\",", "cyclic");
    args.bool_output = NULL;
    assert(init(&args) == 0);
    assert(start_loop() == -1);
    cleanup();
    args.bool_output = outputs;

    write_config(path, "", "cyclic");
    assert(init(&args) == 0);
    assert(!g_config.interfaces[0].tx_frames[0].has_enable);
    int receiver = connect_output();
    enabled      = false;
    cycle_end();
    expect_frame(receiver, 42);
    cleanup();
    assert(close(receiver) == 0);

    write_config(path, "\"enable_plc_address\":\"%qx2.7\",", "cyclic");
    assert(init(&args) == 0);
    receiver = connect_output();
    now_ms   = 2000;
    enabled  = false;
    cycle_end();
    expect_no_frame(receiver);
    can_tx_frame_config_t *frame = &g_config.interfaces[0].tx_frames[0];
    assert(!frame->has_sent_once && frame->last_send_time_ms == 0);
    enabled = true;
    cycle_end();
    expect_frame(receiver, 42);
    enabled = false;
    now_ms  = 2020;
    value   = 43;
    cycle_end();
    expect_no_frame(receiver);
    assert(frame->last_send_time_ms == 2000 && frame->prev_payload[0] == 42);
    enabled = true;
    now_ms  = 2050;
    cycle_end();
    expect_no_frame(receiver);
    now_ms = 2100;
    cycle_end();
    expect_frame(receiver, 43);
    enabled = false;
    now_ms  = 2300;
    cycle_end();
    expect_no_frame(receiver);
    enabled = true;
    cycle_end();
    expect_frame(receiver, 43);
    cleanup();
    assert(close(receiver) == 0);

    write_config(path, "\"enable_plc_address\":\"%QX2.7\",", "on_change");
    assert(init(&args) == 0);
    receiver = connect_output();
    enabled  = false;
    cycle_end();
    expect_no_frame(receiver);
    enabled = true;
    cycle_end();
    expect_frame(receiver, 43);
    enabled = false;
    cycle_end();
    expect_no_frame(receiver);
    enabled = true;
    cycle_end();
    expect_no_frame(receiver);
    enabled = false;
    value   = 44;
    cycle_end();
    expect_no_frame(receiver);
    enabled = true;
    cycle_end();
    expect_frame(receiver, 44);
    cleanup();
    assert(close(receiver) == 0);
    assert(unlink(path) == 0);
    puts("CAN TX enable tests passed");
    return 0;
}

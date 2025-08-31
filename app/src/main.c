/*
 * SPDX-FileCopyrightText: 2024-2025 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0  // 遵循Apache-2.0开源协议
 */

// 系统与硬件基础头文件
#include "rtthread.h"                  // RT-Thread实时操作系统核心库
#include "bf0_hal.h"                   // BF0系列芯片硬件抽象层接口
#include "drv_io.h"                    // 板级IO驱动（引脚复用、方向等）
#include "stdio.h"                     // 标准输入输出（调试打印）
#include "string.h"                    // 字符串操作（如 memcpy、sprintf）
#include "xiaozhi2.h"                  // “小智2代”功能核心接口（业务逻辑）
#include "./iot/iot_c_api.h"           // IoT功能C语言API（联网、云服务）
#ifdef BSP_USING_PM
    #include "gui_app_pm.h"            // GUI电源管理（仅启用电源管理时编译）
#endif // BSP_USING_PM
#include "xiaozhi_public.h"            // “小智”设备公共功能（如显示、音频）
#include "bf0_pm.h"                    // BF0芯片电源管理（休眠、唤醒）
#include <drivers/rt_drv_encoder.h>    // 编码器驱动（音量调节）
#include "drv_flash.h"                 // Flash存储器（存储配置、MAC地址）
#include "xiaozhi_weather.h"           // 天气服务（获取、同步天气信息）
#include "lv_timer.h"                  // LittlevGL图形库定时器（UI刷新）
#include "lv_display.h"                // LittlevGL显示驱动（屏幕控制）
#include "rtdevice.h"                  // RT-Thread设备驱动基础接口


// UI与业务功能外部声明（跨模块调用）
extern void xiaozhi_ui_update_ble(char *string);          // 更新蓝牙状态显示
extern void xiaozhi_ui_update_emoji(char *string);        // 更新表情图标
extern void xiaozhi_ui_chat_status(char *string);         // 更新聊天状态提示
extern void xiaozhi_ui_chat_output(char *string);         // 主界面聊天内容输出
extern void xiaozhi_ui_standby_chat_output(char *string); // 待机界面聊天内容输出
extern void ui_swith_to_standby_screen();                 // 切换到待机屏幕
extern void ui_swith_to_xiaozhi_screen();                 // 切换到交互屏幕
extern void xiaozhi_ui_task(void *args);                   // “小智”UI任务主函数
extern void xiaozhi(int argc, char **argv);                // “小智1代”功能入口
extern void xiaozhi2(int argc, char **argv);               // “小智2代”功能入口
extern void reconnect_xiaozhi();                           // 重连“小智”服务
extern void xz_button_init(void);                          // “小智”按键初始化
extern void xz_ws_audio_init();                            // 音频WebSocket初始化
extern rt_tick_t last_listen_tick;                         // 最后一次语音监听时间戳
extern xiaozhi_ws_t g_xz_ws;                               // WebSocket全局状态
extern rt_mailbox_t g_button_event_mb;                     // 按键事件邮箱（线程通信）
extern void ui_sleep_callback(lv_timer_t *timer);          // UI睡眠回调（无操作超时）
extern lv_obj_t *standby_screen;                           // 待机屏幕对象（LittlevGL）
rt_mailbox_t g_battery_mb;                                 // 电池信息邮箱（线程通信）
extern lv_timer_t *ui_sleep_timer;                         // UI睡眠定时器
extern lv_obj_t *shutdown_screen;                          // 关机屏幕对象
extern lv_obj_t *sleep_screen;                             // 睡眠屏幕对象


/* 板级硬件初始化（RT-Thread平台通用） */
/**
 * @brief  初始化板级默认配置（引脚复用、外设使能等）
 * @note   不同板型可通过条件编译适配引脚功能
 */
void HAL_MspInit(void)
{
    //__asm("B .");        /* 调试用：强制暂停在此处 */
    BSP_IO_Init();       // 初始化板级IO（引脚方向、复用功能）
#ifdef BSP_USING_BOARD_SF32LB52_XTY_AI
    // 特定板型：配置PA38/PA40为定时器通道，使能上拉
    HAL_PIN_Set(PAD_PA38, GPTIM1_CH1, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA40, GPTIM1_CH2, PIN_PULLUP, 1);
#endif
}


/* 用户业务代码开始 */
#include "bts2_app_inc.h"        // BTS2蓝牙协议栈应用接口
#include "ble_connection_manager.h" // BLE连接管理
#include "bt_connection_manager.h"  // 蓝牙连接管理
#include "bt_env.h"              // 蓝牙环境配置（如设备地址、连接状态）
#include "ulog.h"                // 轻量级日志系统


// 蓝牙应用事件类型（用于线程间邮箱通信）
#define BT_APP_READY 0                   // 蓝牙协议栈就绪
#define BT_APP_CONNECT_PAN 1             // 发起PAN网络连接
#define BT_APP_CONNECT_PAN_SUCCESS 2     // PAN连接成功
#define WEBSOC_RECONNECT 4               // WebSocket重连“小智”
#define KEEP_FIRST_PAN_RECONNECT 5       // 首次PAN连接保活重连
#define XZ_CONFIG_UPDATE        6        // “小智”配置更新
#define BT_APP_PHONE_DISCONNECTED 7      // 手机主动断开蓝牙
#define BT_APP_ABNORMAL_DISCONNECT 8     // 蓝牙异常断开
#define BT_APP_RECONNECT_TIMEOUT 9       // 重连超时
#define BT_APP_RECONNECT 10              // 执行蓝牙重连
#define UPDATE_REAL_WEATHER_AND_TIME 11  // 更新天气与时间
#define PAN_TIMER_MS 3000                // PAN连接触发延迟（避免SDP冲突）


// 蓝牙应用全局状态
bt_app_t g_bt_app_env;               // 蓝牙应用环境（存储连接状态、设备地址等）
rt_mailbox_t g_bt_app_mb;            // 蓝牙事件邮箱（线程间通信）
BOOL g_pan_connected = FALSE;        // PAN网络连接状态（FALSE=未连接）
BOOL first_pan_connected = FALSE;    // 首次PAN连接标记（是否成功过）
int first_reconnect_attempts = 0;    // 首次PAN重连尝试次数


// 重连与睡眠定时器
static rt_timer_t s_reconnect_timer = NULL;  // 蓝牙重连定时器
static rt_timer_t s_sleep_timer = NULL;      // 睡眠定时器
static int reconnect_attempts = 0;           // 重连尝试计数器
#define MAX_RECONNECT_ATTEMPTS 30            // 最大重连次数（30次，每次1秒）
static uint8_t g_sleep_enter_flag = 0;       // 进入睡眠标志（1=需要睡眠）
uint8_t Initiate_disconnection_flag = 0;     // 蓝牙主动断开标志（1=主动操作）


// 线程栈配置（根据需求分配内存大小）
#define XIAOZHI_UI_THREAD_STACK_SIZE (6144)  // “小智”UI线程栈大小（字节）
#define BATTERY_THREAD_STACK_SIZE (2048)     // 电池检测线程栈大小（字节）


// 线程与栈定义
static struct rt_thread xiaozhi_ui_thread;    // “小智”UI线程（处理界面更新、用户交互）
static struct rt_thread battery_thread;       // 电池检测线程（周期性读取电量）


//ui线程
#if defined(__CC_ARM) || defined(__CLANG_ARM)
L2_RET_BSS_SECT_BEGIN(xiaozhi_ui_thread_stack) //6000地址
static uint32_t xiaozhi_ui_thread_stack[XIAOZHI_UI_THREAD_STACK_SIZE / sizeof(uint32_t)];
L2_RET_BSS_SECT_END
#else
static uint32_t
    xiaozhi_ui_thread_stack[XIAOZHI_UI_THREAD_STACK_SIZE / sizeof(uint32_t)] L2_RET_BSS_SECT(xiaozhi_ui_thread_stack);
#endif


// 电池线程栈（编译器适配：将栈放在指定内存段）
#if defined(__CC_ARM) || defined(__CLANG_ARM)
L2_RET_BSS_SECT_BEGIN(battery_thread_stack) //6000地址
static uint32_t battery_thread_stack[BATTERY_THREAD_STACK_SIZE / sizeof(uint32_t)];
L2_RET_BSS_SECT_END
#else
static uint32_t
    battery_thread_stack[BATTERY_THREAD_STACK_SIZE / sizeof(uint32_t)] L2_RET_BSS_SECT(battery_thread_stack);
#endif


#ifdef BSP_USING_BOARD_SF32LB52_XTY_AI
// 编码器（音量调节）相关定义
static rt_timer_t s_pulse_encoder_timer = NULL;  // 编码器检测定时器
static struct rt_device *s_encoder_device;       // 编码器设备指针


/**
 * @brief  初始化编码器（用于音量调节）
 * @retval RT_EOK=成功，其他=失败
 */
static int pulse_encoder_init(void)
{
    // 查找编码器设备（设备名“encoder1”）
    s_encoder_device = rt_device_find("encoder1");
    if (s_encoder_device == RT_NULL)
    {
        LOG_E("Failed to find encoder device\n"); // 日志：未找到编码器
        return -RT_ERROR;
    }
    // 配置编码器：使能所有通道
    struct rt_encoder_configuration config;
    config.channel = GPT_CHANNEL_ALL;

    // 使能编码器
    rt_err_t result = rt_device_control((struct rt_device *)s_encoder_device,
                          PULSE_ENCODER_CMD_ENABLE, (void *)&config);
    if (result != RT_EOK)
    {
        rt_kprintf("Failed to enable encoder\n"); // 打印：编码器使能失败
        return -RT_ERROR;
    }
    return RT_EOK;
}


/**
 * @brief  编码器定时器回调（周期性检测旋转，调整音量）
 * @param  parameter: 未使用
 * @note   编码器每“4个单位”为1次有效旋转，正转增大音量，反转减小音量
 */
static void pulse_encoder_timeout_handle(void *parameter)
{
    static int32_t last_count = 0;  // 上一次编码器计数
    rt_err_t result;
    struct rt_encoder_configuration config_count;
    config_count.get_count = 0;     // 用于获取当前计数

    // 读取编码器当前计数
    result = rt_device_control((struct rt_device *)s_encoder_device,
                          PULSE_ENCODER_CMD_GET_COUNT, (void *)&config_count);
    if (result != RT_EOK)
    {
        LOG_E("Failed to get encoder count\n"); // 日志：读取计数失败
        return;
    }

    int32_t current_count = config_count.get_count;  // 当前计数
    int32_t delta_count = current_count - last_count; // 计数变化量
    last_count = current_count;                       // 更新上次计数

    // 编码器特性：每4个单位为1次有效脉冲（过滤噪声）
    if (delta_count % 4 != 0)
    {
        LOG_W("Encoder count is not a multiple of 4, delta_count: %d\n", delta_count);
        return;
    }
    int32_t pulses = delta_count / 4;  // 转换为实际脉冲数（正=顺时针，负=逆时针）
    if (pulses != 0)
    {
        LOG_I("Pulse encoder count: %d\n", pulses); // 日志：脉冲数
        // 获取当前音量（本地音乐类型）
        int current_volume = audio_server_get_private_volume(AUDIO_TYPE_LOCAL_MUSIC);
        int new_volume = current_volume + pulses;   // 计算新音量

        // 音量范围限制（0-15）
        if (new_volume < 0)  new_volume = 0;
        else if (new_volume > 15)  new_volume = 15;

        // 设置新音量
        audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, new_volume);
    }
}
#endif


/**
 * @brief  电池电量检测任务（周期性读取电压，计算百分比并发送到邮箱）
 * @param  parameter: 未使用
 * @note   假设电池电压范围3.6V~4.2V对应0%~100%，每10秒检测一次
 */
static void battery_level_task(void *parameter)
{
    // 创建电池信息邮箱（容量1，FIFO模式）
    g_battery_mb = rt_mb_create("battery_level", 1, RT_IPC_FLAG_FIFO);
    if (g_battery_mb == NULL)
    {
        rt_kprintf("Failed to create mailbox g_battery_mb\n"); // 打印：邮箱创建失败
        return;
    }

    while (1) // 循环检测
    {
        // 查找电池ADC设备（设备名“bat1”）
        rt_device_t battery_device = rt_device_find("bat1");
        rt_adc_cmd_read_arg_t read_arg;
        read_arg.channel = 7; // 电池电量对应ADC通道7

        // 使能ADC通道
        rt_err_t result = rt_adc_enable((rt_adc_device_t)battery_device, read_arg.channel);
        if (result != RT_EOK)
        {
            LOG_E("Failed to enable ADC for battery read\n"); // 日志：ADC使能失败
            return;
        }

        // 读取电池电压（单位：mV）
        rt_uint32_t battery_level = rt_adc_read((rt_adc_device_t)battery_device, read_arg.channel);
        rt_adc_disable((rt_adc_device_t)battery_device, read_arg.channel); // 关闭ADC通道

        // 获取到的是电池电压，单位是mV
        // 假设电池电压范围是3.6V到4.2V，对应的电量范围是0%到100%
        uint32_t battery_percentage = 0;
        if (battery_level < 36000)       battery_percentage = 0;    // <3.6V → 0%
        else if (battery_level > 42000)  battery_percentage = 100; // >4.2V → 100%
        else  battery_percentage = ((battery_level - 36000) * 100) / (42000 - 36000);

        // 发送电量到邮箱
        rt_mb_send(g_battery_mb, battery_percentage);
        rt_thread_mdelay(10000); // 每10秒检测一次
    }
}


/**
 * @brief  PAN连接超时回调（触发PAN连接事件到邮箱）
 * @param  parameter: 蓝牙应用环境变量
 */
void bt_app_connect_pan_timeout_handle(void *parameter)
{
    LOG_I("bt_app_connect_pan_timeout_handle %x, %d", g_bt_app_mb, g_bt_app_env.bt_connected);
    // 若邮箱有效且蓝牙已连接，发送“PAN连接”事件
    if ((g_bt_app_mb != NULL) && (g_bt_app_env.bt_connected))
        rt_mb_send(g_bt_app_mb, BT_APP_CONNECT_PAN);
    return;
}


#if defined(BSP_USING_SPI_NAND) && defined(RT_USING_DFS)
    // SPI NAND闪存与文件系统相关头文件
    #include "dfs_file.h"
    #include "dfs_posix.h"
    #include "drv_flash.h"
    #define NAND_MTD_NAME "root"  // MTD设备名（用于文件系统挂载）


/**
 * @brief  初始化SPI NAND文件系统（挂载或格式化）
 * @retval RT_EOK=成功
 * @note   若挂载失败，自动尝试格式化后重新挂载
 */
int mnt_init(void)
{
    // 注册NAND设备（指定地址、大小）
    register_nand_device(FS_REGION_START_ADDR & (0xFC000000),
                         FS_REGION_START_ADDR - (FS_REGION_START_ADDR & (0xFC000000)),
                         FS_REGION_SIZE, NAND_MTD_NAME);
    // 尝试挂载文件系统
    if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0) // 挂载成功
    {
        rt_kprintf("mount fs on flash to root success\n");
    }
    else // 挂载失败，尝试格式化
    {
        rt_kprintf("mount fs on flash to root fail\n");
        if (dfs_mkfs("elm", NAND_MTD_NAME) == 0) // 格式化成功
        {
            rt_kprintf("make elm fs on flash sucess, mount again\n");
            if (dfs_mount(NAND_MTD_NAME, "/", "elm", 0, 0) == 0)
                rt_kprintf("mount fs on flash success\n");
            else
                rt_kprintf("mount to fs on flash fail\n");
        }
        else
            rt_kprintf("dfs_mkfs elm flash fail\n");
    }
    return RT_EOK;
}
INIT_ENV_EXPORT(mnt_init); // 导出为环境初始化函数（系统启动时自动执行）
#endif


/**
 * @brief  睡眠定时器回调（设置“进入睡眠”标志）
 * @param  parameter: 未使用
 */
static void sleep_timer_timeout_handle(void *parameter)
{
    rt_kprintf("30 seconds timeout, set sleep enter flag\n");
    g_sleep_enter_flag = 1;  // 标记需要进入睡眠模式
}


/**
 * @brief  启动睡眠定时器（30秒后触发睡眠）
 */
static void start_sleep_timer(void)
{
    if (s_sleep_timer == RT_NULL) // 定时器未创建则新建
    {
        s_sleep_timer = rt_timer_create("sleep_timer",
                                        sleep_timer_timeout_handle,
                                        RT_NULL,
                                        rt_tick_from_millisecond(30000),  // 30秒
                                        RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    } else {
        // 定时器已存在则重置时间
        rt_timer_stop(s_sleep_timer);
        rt_timer_control(s_sleep_timer, RT_TIMER_CTRL_SET_TIME, (void *)&(rt_tick_t){rt_tick_from_millisecond(30000)});
    }

    if (s_sleep_timer != RT_NULL) {
        rt_timer_start(s_sleep_timer);
        rt_kprintf("Sleep timer started, will trigger after 30 seconds\n");
    } else {
        rt_kprintf("Failed to create sleep timer\n");
    }
}


/**
 * @brief  重连定时器回调（发送“重连”事件到邮箱）
 * @param  parameter: 未使用
 */
static void reconnect_timeout_handle(void *parameter)
{
    rt_mb_send(g_bt_app_mb, BT_APP_RECONNECT);
}


/**
 * @brief  启动重连定时器（1秒间隔，周期性触发重连）
 */
static void start_reconnect_timer(void)
{
    if (s_reconnect_timer == NULL) {
        s_reconnect_timer = rt_timer_create(
            "reconnect", reconnect_timeout_handle, NULL,
            rt_tick_from_millisecond(10000), // 1秒间隔
            RT_TIMER_FLAG_PERIODIC);
    }

    if (s_reconnect_timer) {
        reconnect_attempts = 0;
        rt_timer_start(s_reconnect_timer);
        LOG_I("Start reconnect timer");
    }
}


/**
 * @brief  PAN网络重连（处理首次连接失败后的重试逻辑）
 * @note   最多重试3次，失败后提示用户检查网络
 */
void pan_reconnect()
{
    static int first_reconnect_attempts = 0;
    const int max_reconnect_attempts = 3; // 最大重试次数

    LOG_I("Attempting to reconnect PAN, attempt %d",
          first_reconnect_attempts + 1);
    // 更新UI提示（主界面+待机界面）
    xiaozhi_ui_chat_status("重新连接 PAN...");
    xiaozhi_ui_chat_output("正在重连PAN...");
    xiaozhi_ui_standby_chat_output("正在重连PAN...");

    if (first_reconnect_attempts < max_reconnect_attempts)
    {
        // 用与主流程相同的定时器触发PAN连接
        if (!g_bt_app_env.pan_connect_timer)
            g_bt_app_env.pan_connect_timer = rt_timer_create(
                "connect_pan", bt_app_connect_pan_timeout_handle,
                (void *)&g_bt_app_env,
                rt_tick_from_millisecond(PAN_TIMER_MS),
                RT_TIMER_FLAG_SOFT_TIMER);
        else
            rt_timer_stop(g_bt_app_env.pan_connect_timer);
        rt_timer_start(g_bt_app_env.pan_connect_timer);

        first_reconnect_attempts++;
    }
    else
    {
        // 重试次数用尽，更新UI并停止定时器
        LOG_W("Failed to keep_first_reconnect PAN after %d attempts",
              max_reconnect_attempts);
        xiaozhi_ui_chat_status("无法连接PAN");
        xiaozhi_ui_chat_output("请确保设备开启了共享网络,重新发起连接");
        xiaozhi_ui_update_emoji("thinking");
        xiaozhi_ui_standby_chat_output("请确保设备开启了共享网络,重新发起连接");
        first_reconnect_attempts = 0; // 重置重试计数

        if (g_bt_app_env.pan_connect_timer) {
            rt_timer_stop(g_bt_app_env.pan_connect_timer);
        }
        return;
    }
}


/**
 * @brief  蓝牙事件回调（处理蓝牙栈、PAN、HID等事件）
 * @param  type: 事件类型（通用/ PAN/ HID）
 * @param  event_id: 具体事件ID
 * @param  data: 事件数据
 * @param  data_len: 数据长度
 * @retval 0=处理成功
 */
static int bt_app_interface_event_handle(uint16_t type, uint16_t event_id,
                                         uint8_t *data, uint16_t data_len)
{
    if (type == BT_NOTIFY_COMMON) // 通用蓝牙事件
    {
        int pan_conn = 0; // PAN连接触发标记

        switch (event_id)
        {
        case BT_NOTIFY_COMMON_BT_STACK_READY: // 蓝牙栈就绪
            rt_mb_send(g_bt_app_mb, BT_APP_READY);
            break;
        case BT_NOTIFY_COMMON_ACL_DISCONNECTED: // 蓝牙连接断开
        {
            bt_notify_device_base_info_t *info =
                (bt_notify_device_base_info_t *)data;
            LOG_I("disconnected(0x%.2x:%.2x:%.2x:%.2x:%.2x:%.2x) res %d",
                  info->mac.addr[5], info->mac.addr[4], info->mac.addr[3],
                  info->mac.addr[2], info->mac.addr[1], info->mac.addr[0],
                  info->res);
            g_bt_app_env.bt_connected = FALSE; // 标记蓝牙未连接
            // 更新UI提示
            xiaozhi_ui_chat_output("蓝牙断开连接");
            xiaozhi_ui_standby_chat_output("蓝牙断开连接");
            // 切换到待机屏幕（若当前不是待机/睡眠/关机屏）
            lv_obj_t *now_screen = lv_screen_active();
            if (now_screen != standby_screen && now_screen != sleep_screen && now_screen != shutdown_screen)
            {
                ui_swith_to_standby_screen();
            }
            // 区分断开原因：主动断开→准备睡眠；异常断开→启动重连
            if (info->res == BT_NOTIFY_COMMON_SCO_DISCONNECTED)
            {
                LOG_I("Phone actively disconnected, prepare to enter sleep mode after 30 seconds");
                rt_mb_send(g_bt_app_mb, BT_APP_PHONE_DISCONNECTED);
            }
            else
            {
                LOG_I("Abnormal disconnection, start reconnect attempts");
                rt_mb_send(g_bt_app_mb, BT_APP_ABNORMAL_DISCONNECT);
            }
            // 停止PAN连接定时器
            if (g_bt_app_env.pan_connect_timer)
                rt_timer_stop(g_bt_app_env.pan_connect_timer);
            break;
        }
        case BT_NOTIFY_COMMON_ENCRYPTION: // 蓝牙加密完成
        {
            bt_notify_device_mac_t *mac = (bt_notify_device_mac_t *)data;
            LOG_I("Encryption competed");
            g_bt_app_env.bd_addr = *mac; // 保存设备地址
            pan_conn = 1; // 标记需要触发PAN连接
            break;
        }
        case BT_NOTIFY_COMMON_PAIR_IND: // 蓝牙配对完成
        {
            bt_notify_device_base_info_t *info =
                (bt_notify_device_base_info_t *)data;
            LOG_I("Pairing completed %d", info->res);
            if (info->res == BTS2_SUCC) // 配对成功
            {
                g_bt_app_env.bd_addr = info->mac; // 保存设备地址
                pan_conn = 1; // 标记需要触发PAN连接
            }
            break;
        }
        case BT_NOTIFY_COMMON_KEY_MISSING: // 蓝牙密钥丢失
        {
            bt_notify_device_base_info_t *info =
                (bt_notify_device_base_info_t *)data;
            LOG_I("Key missing %d", info->res);
            // 清空设备地址（标记为未配对）
            memset(&g_bt_app_env.bd_addr, 0xFF, sizeof(g_bt_app_env.bd_addr));
            // 删除已绑定设备
            bt_cm_delete_bonded_devs_and_linkkey(info->mac.addr);
            break;
        }
        default:
            break;
        }

        if (pan_conn) // 需要触发PAN连接
        {
            LOG_I("bd addr 0x%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n",
                  g_bt_app_env.bd_addr.addr[5], g_bt_app_env.bd_addr.addr[4],
                  g_bt_app_env.bd_addr.addr[3], g_bt_app_env.bd_addr.addr[2],
                  g_bt_app_env.bd_addr.addr[1], g_bt_app_env.bd_addr.addr[0]);
            g_bt_app_env.bt_connected = TRUE; // 标记蓝牙已连接
            // 延迟触发PAN连接（避免SDP协议冲突）
            if (!g_bt_app_env.pan_connect_timer)
                g_bt_app_env.pan_connect_timer = rt_timer_create(
                    "connect_pan", bt_app_connect_pan_timeout_handle,
                    (void *)&g_bt_app_env,
                    rt_tick_from_millisecond(PAN_TIMER_MS),
                    RT_TIMER_FLAG_SOFT_TIMER);
            else
                rt_timer_stop(g_bt_app_env.pan_connect_timer);
            rt_timer_start(g_bt_app_env.pan_connect_timer);
        }
    }
    else if (type == BT_NOTIFY_PAN) // PAN协议事件
    {
        switch (event_id)
        {
        case BT_NOTIFY_PAN_PROFILE_CONNECTED: // PAN连接成功
        {
            // 更新UI提示
            xiaozhi_ui_chat_output("PAN连接成功");
            xiaozhi_ui_standby_chat_output("PAN连接成功");
            xiaozhi_ui_update_ble("open"); // 蓝牙状态显示“已连接”
            LOG_I("pan connect successed \n");
            // 停止PAN连接定时器
            if ((g_bt_app_env.pan_connect_timer))
            {
                rt_timer_stop(g_bt_app_env.pan_connect_timer);
            }
            rt_mb_send(g_bt_app_mb, BT_APP_CONNECT_PAN_SUCCESS);
            g_pan_connected = TRUE; // 更新PAN连接状态为“已连接”
            break;
        }
        case BT_NOTIFY_PAN_PROFILE_DISCONNECTED: // PAN连接断开
        {
            // 更新UI提示
            xiaozhi_ui_chat_status("PAN断开...");
            xiaozhi_ui_chat_output("PAN断开,尝试唤醒键重新连接");
            xiaozhi_ui_standby_chat_output("PAN断开,尝试唤醒键重新连接");
            xiaozhi_ui_update_ble("close"); // 蓝牙状态显示“断开”
            last_listen_tick = 0; // 重置语音监听时间戳
            LOG_I("pan disconnect with remote device\n");
            g_pan_connected = FALSE; // 更新PAN连接状态为“未连接”

            // 若首次连接从未成功，触发保活重连
            if (first_pan_connected == FALSE)
            {
                rt_mb_send(g_bt_app_mb, KEEP_FIRST_PAN_RECONNECT);
            }
            break;
        }
        default:
            break;
        }
    }
    else if (type == BT_NOTIFY_HID) // HID协议事件
    {
        switch (event_id)
        {
        case BT_NOTIFY_HID_PROFILE_CONNECTED: // HID连接成功
        {
            LOG_I("HID connected\n");
            // 若PAN未连接，触发PAN连接
            if (!g_pan_connected)
            {
                if (g_bt_app_env.pan_connect_timer)
                {
                    rt_timer_stop(g_bt_app_env.pan_connect_timer);
                }
                bt_interface_conn_ext((char *)&g_bt_app_env.bd_addr,
                                      BT_PROFILE_PAN);
            }
            break;
        }
        case BT_NOTIFY_HID_PROFILE_DISCONNECTED: // HID连接断开
        {
            LOG_I("HID disconnected\n");
            break;
        }
        default:
            break;
        }
    }

    return 0;
}


/**
 * @brief  获取蓝牙设备类别（用于协议栈识别设备类型）
 * @retval 设备类别掩码（网络服务 + 外设 + 远程控制）
 */
uint32_t bt_get_class_of_device()
{
    return (uint32_t)BT_SRVCLS_NETWORK | BT_DEVCLS_PERIPHERAL |
           BT_PERIPHERAL_REMCONTROL;
}


// 引脚定义（电源控制、按键检测）
#define BSP_POWER_ON 8      // 电源控制引脚（PMOS使能）
#define BSP_POWER_CHECK 7   // 电源检测引脚
#define BSP_USER_KEY 38     // 用户按键引脚


/**
 * @brief  设置GPIO引脚输出电平
 * @param  pin: 引脚号
 * @param  val: 输出电平（1=高，0=低）
 */
void gpio_pin_set(int pin, int val)
{
    GPIO_TypeDef *gpio;
    GPIO_InitTypeDef GPIO_InitStruct;
    int pad = 0;
    // 区分GPIO组（GPIO1或GPIO2）
    if (pin > 96)
    {
        gpio = hwp_gpio2;
        pad =  pin - 96;
    }
    else
    {
        gpio = hwp_gpio1;
        pad =  pin;
    }

    // 配置引脚为输出模式
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pin = pad;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // 设置引脚电平
    HAL_GPIO_WritePin(gpio, pad, (GPIO_PinState)val);
}


/**
 * @brief  读取GPIO引脚输入电平
 * @param  pin: 引脚号
 * @retval GPIO_PinState: 输入电平（1=高，0=低）
 */
GPIO_PinState gpio_pin_read(int pin)
{
    GPIO_PinState state = 1;
    GPIO_TypeDef *gpio;
    GPIO_InitTypeDef GPIO_InitStruct;
    int pad = 0;
    // 区分GPIO组（GPIO1或GPIO2）
    if (pin > 96)
    {
        gpio = hwp_gpio2;
        pad =  pin - 96;
    }
    else
    {
        gpio = hwp_gpio1;
        pad =  pin;
    }

    // 配置引脚为输入模式（上拉）
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pin = pad;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(gpio, &GPIO_InitStruct);

    // 读取引脚电平
    state = HAL_GPIO_ReadPin(gpio, pad);
    return state;
}


/**
 * @brief  检查设备开机原因（正常启动/休眠唤醒/按键唤醒等）
 * @note   区分不同唤醒源，处理电源使能逻辑（如长按开机、误触关机）
 */
static void check_poweron_reason(void)
{
    // 配置电源控制和检测引脚
    rt_pin_mode(BSP_POWER_ON, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_POWER_CHECK, PIN_MODE_INPUT_PULLUP);
    // 获取系统开机模式
    switch (SystemPowerOnModeGet())
    {
    case PM_REBOOT_BOOT:       // 重启导致的开机
    case PM_COLD_BOOT:         // 冷启动（断电后重新上电）
    {
        rt_thread_mdelay(1000);// 延时1秒消抖
        gpio_pin_set(BSP_POWER_ON, 1);// 打开PMOS电源
        rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![START]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Power IO ON!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        break;
    }
    case PM_HIBERNATE_BOOT:    // 从休眠模式唤醒
    case PM_SHUTDOWN_BOOT:     // 从关机状态唤醒
    {
        // RTC定时器唤醒
        if (PMUC_WSR_RTC & pm_get_wakeup_src())
        {
            NVIC_EnableIRQ(RTC_IRQn); // 使能RTC中断（用于定时功能）
        }
#ifdef BSP_USING_CHARGER
        // 充电引脚唤醒（预留，暂未实现）
        else if ((PMUC_WSR_PIN0 << (pm_get_charger_pin_wakeup())) & pm_get_wakeup_src())
        {
        }
#endif
        // 按键引脚唤醒（检测长按/短按）
        else if (PMUC_WSR_PIN_ALL & pm_get_wakeup_src())
        {
            rt_thread_mdelay(1000); // 延时消抖
#ifdef BSP_USING_BOARD_SF32LB52_LCD_N16R8
            int val = rt_pin_read(BSP_KEY1_PIN);  // 读取KEY1电平
#else
            int val = rt_pin_read(BSP_KEY2_PIN);  // 读取KEY2电平
#endif
            rt_kprintf("Power key level after 1s: %d\n", val);
            // 短按（误触发）→ 关机
            if (val != KEY2_ACTIVE_LEVEL)
            {
                rt_kprintf("Not long press, shutdown now.\n");
                gpio_pin_set(BSP_POWER_ON, 0);// 关闭PMOS电源
                rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Power IO OFF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                PowerDownCustom(); // 自定义关机流程
                while (1) {}; // 死循环确保关机
            }
            else // 长按→ 正常开机
            {
                rt_kprintf("Long press detected, power on as normal.\n");
                gpio_pin_set(BSP_POWER_ON, 1);// 打开PMOS电源
                rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!![WKUP]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Power IO ON!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                rt_kprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            }
        }
        else if (0 == pm_get_wakeup_src()) // 无唤醒源（异常）
        {
            RT_ASSERT(0); // 断言失败（调试用）
        }
        break;
    }
    default: // 未知开机模式（异常）
    {
        RT_ASSERT(0); // 断言失败（调试用）
    }
    }
}


/**
 * @brief  命令行工具：写入MAC地址到Flash
 * @param  argc: 参数个数（需1个命令 + 6个MAC字节）
 * @param  argv: 参数列表（如 "write_mac AA BB CC DD EE FF"）
 * @retval 0=成功，非0=失败
 */
static int32_t Write_MAC(int argc, char **argv)
{
    uint8_t len;
    uint8_t mac[6] = {0};
    char *endptr;

    if (argc < 7) // 参数不足
    {
        rt_kprintf("write_mac FAIL\n");
        return 1;
    }
    // 解析6个MAC字节（十六进制）
    for (len = 0; len < 6; len++)
    {
        mac[5 - len] = (uint8_t)(strtoul(argv[1 + len], &endptr, 16) & 0xFF);
        if (endptr == argv[1 + len]) // 解析失败
        {
            rt_kprintf("incorrect MAC\n");
            return 2;
        }
    }

    // 写入Flash配置区
    len = rt_flash_config_write(FACTORY_CFG_ID_MAC, (uint8_t *)&mac[0], 6);
    if (len < 6) // 写入失败
    {
        rt_kprintf("write_mac FAIL\n");
    }
    else // 写入成功，打印MAC
    {
        rt_kprintf("MAC: %02x-%02x-%02x-%02x-%02x-%02x\n", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
        rt_kprintf("write_mac PASS\n");
    }
    return 0;
}
MSH_CMD_EXPORT(Write_MAC, write mac); // 导出为RT-Thread命令行命令


/**
 * @brief  主函数：系统初始化与主循环
 * @note   完成硬件、线程、蓝牙的初始化，进入事件处理循环
 * @retval int: 嵌入式主循环通常不返回
 */
int main(void)
{
    check_poweron_reason();// 检查设备启动原因（正常启动/休眠唤醒等）
    // 初始化按键事件邮箱
    g_button_event_mb = rt_mb_create("btn_evt", 8, RT_IPC_FLAG_FIFO);
    if (g_button_event_mb == NULL)
    {
        rt_kprintf("Failed to create mailbox g_button_event_mb\n");
        return 0;
    }
    rt_kprintf("Xiaozhi start!!!\n");
    // 设置默认音量
    audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, VOL_DEFAULE_LEVEL);
    xz_set_lcd_brightness(LCD_BRIGHTNESS_DEFAULT); // 设置LCD默认亮度
    iot_initialize(); // 初始化IoT功能（联网、云服务）
    xiaozhi_time_weather_init();// 初始化时间和天气服务
    xz_ws_audio_init(); // 初始化音频WebSocket（语音交互）

#ifdef BSP_USING_BOARD_SF32LB52_LCHSPI_ULP
    // 特定板型的寄存器配置（预留）
    unsigned int *addr2 = (unsigned int *)0x50003088;
    *addr2 = 0x00000200;
    unsigned int *addr = (unsigned int *)0x500030B0;
    *addr = 0x00000200;

    // 传感器引脚配置（下拉）
    HAL_PIN_Set(PAD_PA30, GPIO_A30, PIN_PULLDOWN, 1);
    BSP_GPIO_Set(30, 0, 1);
    HAL_PIN_Set(PAD_PA39, GPIO_A39, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA40, GPIO_A40, PIN_PULLDOWN, 1);
#endif
    // 创建“小智”UI线程（处理界面更新、用户交互）
    rt_err_t result = rt_thread_init(&xiaozhi_ui_thread,
                                     "xz_ui",
                                     xiaozhi_ui_task,
                                     NULL,
                                     &xiaozhi_ui_thread_stack[0],
                                     XIAOZHI_UI_THREAD_STACK_SIZE,
                                     30,  // 优先级
                                     10); // 时间片
    if (result == RT_EOK)
    {
        rt_thread_startup(&xiaozhi_ui_thread); // 启动UI线程
    }
    else
    {
        rt_kprintf("Failed to init xiaozhi UI thread\n");
    }

    // 初始化蓝牙PAN连接
    g_bt_app_mb = rt_mb_create("bt_app", 8, RT_IPC_FLAG_FIFO);
#ifdef BSP_BT_CONNECTION_MANAGER
    bt_cm_set_profile_target(BT_CM_HID, BT_LINK_PHONE, 1); // 设置蓝牙配置文件目标
#endif // BSP_BT_CONNECTION_MANAGER

    // 注册蓝牙事件回调
    bt_interface_register_bt_event_notify_callback(
        bt_app_interface_event_handle);

    sifli_ble_enable(); // 使能BLE功能

    // 创建电池检测线程（周期性读取电量）
    rt_err_t battery_thread_result = rt_thread_init(&battery_thread,
                                                    "battery",
                                                    battery_level_task,
                                                    NULL,
                                                    &battery_thread_stack[0],
                                                    BATTERY_THREAD_STACK_SIZE,
                                                    20,  // 优先级
                                                    10); // 时间片
    if (battery_thread_result == RT_EOK)
    {
        rt_thread_startup(&battery_thread); // 启动电池检测线程
    }
    else
    {
        rt_kprintf("Failed to init battery thread\n");
    }

#ifdef BSP_USING_BOARD_SF32LB52_XTY_AI
    // 初始化编码器（用于音量调节）
    if (pulse_encoder_init() != RT_EOK)
    {
        rt_kprintf("Pulse encoder initialization failed.\n");
        return -RT_ERROR;
    }
    // 创建编码器定时器（200ms周期），读取转动值并调整音量
    s_pulse_encoder_timer =
        rt_timer_create("pulse_encoder", pulse_encoder_timeout_handle, NULL,
                        rt_tick_from_millisecond(200), RT_TIMER_FLAG_PERIODIC);
    if (s_pulse_encoder_timer)
    {
        rt_kprintf("Pulse encoder timer created successfully.\n");
        rt_timer_start(s_pulse_encoder_timer);
    }
    else
    {
        rt_kprintf("Failed to create pulse encoder timer.\n");
        return -1;
    }
#endif
    // 主循环：处理蓝牙事件（永久阻塞，等待事件）
    while (1)
    {
        uint32_t value;

        // 等待蓝牙事件（邮箱永久阻塞）
        rt_mb_recv(g_bt_app_mb, (rt_uint32_t *)&value, RT_WAITING_FOREVER);

        if (value == BT_APP_CONNECT_PAN) // 发起PAN网络连接
        {
            if (g_bt_app_env.bt_connected)
            {
                bt_interface_conn_ext((char *)&g_bt_app_env.bd_addr,
                                      BT_PROFILE_PAN);
            }
        }
        else if (value == BT_APP_READY) // 蓝牙协议栈就绪
        {
            LOG_I("BT/BLE stack and profile ready");

#ifdef BT_NAME_MAC_ENABLE
            char local_name[32];
            bd_addr_t addr;
            ble_get_public_address(&addr);
            // 生成带MAC的设备名
            sprintf(local_name, "%s-%02x:%02x:%02x:%02x:%02x:%02x",
                    BLUETOOTH_NAME, addr.addr[0], addr.addr[1], addr.addr[2],
                    addr.addr[3], addr.addr[4], addr.addr[5]);
#else
            const char *local_name = BLUETOOTH_NAME;
#endif
            // 设置蓝牙设备名
            bt_interface_set_local_name(strlen(local_name), (void *)local_name);
        }
        else if (value == BT_APP_CONNECT_PAN_SUCCESS) // PAN连接成功
        {
            rt_kputs("BT_APP_CONNECT_PAN_SUCCESS\r\n");
            xiaozhi_ui_standby_chat_output("初始化 请稍等...");
            xiaozhi_ui_update_ble("open");
            xiaozhi_ui_chat_status("初始化...");
            xiaozhi_ui_update_emoji("neutral");
            Initiate_disconnection_flag = 0; // 清除主动断开标志
            rt_thread_mdelay(2000);
            xiaozhi_time_weather(); // 同步时间和天气
            xiaozhi_ui_standby_chat_output("请按键连接小智...");

#ifdef XIAOZHI_USING_MQTT
            xiaozhi(0, NULL); // 启动MQTT版本“小智”
            rt_kprintf("Select MQTT Version\n");
#else
            xz_button_init(); // 初始化按键（非MQTT版本）
            // xiaozhi2(0, NULL); // Start Xiaozhi
#endif
            // 创建UI睡眠定时器（40秒无操作则睡眠）
            if (!ui_sleep_timer && g_pan_connected)
            {
                rt_kprintf("create sleep timer2\n");
                ui_sleep_timer = lv_timer_create(ui_sleep_callback, 40000, NULL);
            }
        }
        else if (value == KEEP_FIRST_PAN_RECONNECT) // 首次PAN连接保活重连
        {
            pan_reconnect();
        }
#ifdef XIAOZHI_USING_MQTT
#else
        else if (value == WEBSOC_RECONNECT) // WebSocket重连“小智”
        {
            xiaozhi2(0,NULL); // 重连“小智”WebSocket
        }
        else if(value == BT_APP_PHONE_DISCONNECTED) // 手机主动断开蓝牙
        {
            rt_kprintf("Phone actively disconnected, enter sleep mode after 30 seconds\n");
            Initiate_disconnection_flag = 1;
            start_sleep_timer(); // 启动30秒后睡眠的定时器
        }
        else if(value == BT_APP_ABNORMAL_DISCONNECT) // 蓝牙异常断开
        {
            rt_kprintf("Abnormal disconnection, start reconnect attempts\n");
            rt_thread_mdelay(3000);
            reconnect_attempts = 0;
            start_reconnect_timer(); // 启动重连定时器
        }
        else if(value == BT_APP_RECONNECT_TIMEOUT) // 蓝牙重连超时
        {
            rt_kprintf("Reconnect timeout, enter sleep mode\n");
            g_sleep_enter_flag = 1;
        }
        else if(value == BT_APP_RECONNECT) // 执行蓝牙重连
        {
            if (g_bt_app_env.bt_connected)
            {
                if (s_reconnect_timer) {
                    rt_timer_stop(s_reconnect_timer);
                }
                reconnect_attempts = 0;
                LOG_I("Reconnect successful, stop reconnect timer");
            }
            else
            {
                reconnect_attempts++;
                LOG_I("Reconnect attempt %d/%d", reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
            }

            if (reconnect_attempts <= MAX_RECONNECT_ATTEMPTS)
            {
                bt_interface_conn_ext((char *)&g_bt_app_env.bd_addr, BT_PROFILE_HID);
            }
            else
            {
                LOG_I("Reconnect timeout, send timeout event");
                rt_mb_send(g_bt_app_mb, BT_APP_RECONNECT_TIMEOUT);
                reconnect_attempts = 0;
                if (s_reconnect_timer) {
                    rt_timer_stop(s_reconnect_timer);
                }
            }
        }
        else if(value == UPDATE_REAL_WEATHER_AND_TIME) // 更新天气和时间
        {
            xiaozhi_time_weather();
        }
        else // WebSocket断开（未知原因）
        {
            rt_kprintf("WEBSOCKET_DISCONNECT\r\n");
            xiaozhi_ui_chat_output("请重启");
            xiaozhi_ui_standby_chat_output("请重启");
        }
#endif
    }
    return 0;
}


/**
 * @brief  命令行工具：PAN网络相关命令（删除绑定/连接PAN）
 * @param  argc: 参数个数
 * @param  argv: 参数列表（如 "pan_cmd del_bond" 或 "pan_cmd conn_pan"）
 */
static void pan_cmd(int argc, char **argv)
{
    if (strcmp(argv[1], "del_bond") == 0) // 删除蓝牙绑定设备
    {
#ifdef BSP_BT_CONNECTION_MANAGER
        bt_cm_delete_bonded_devs();
        LOG_D("Delete bond");
#endif // BSP_BT_CONNECTION_MANAGER
    }
    else if (strcmp(argv[1], "conn_pan") == 0) // 连接PAN网络
        bt_app_connect_pan_timeout_handle(NULL);
}
MSH_CMD_EXPORT(pan_cmd, Connect PAN to last paired device); // 导出为命令行命令

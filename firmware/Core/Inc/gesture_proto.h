/**
  ******************************************************************************
  * @file    gesture_proto.h
  * @brief   统一串口协议（AA 55 + 功能字 + 长度 + 数据 + CRC16）
  *          本模块不依赖 HAL / FreeRTOS，两个工程共用同一份实现。
  ******************************************************************************
  *
  * 帧格式：
  *
  *   +------+------+------+------+-----------+---------+---------+
  *   | 0xAA | 0x55 | FUNC | LEN  |  PAYLOAD  | CRC_L   | CRC_H   |
  *   +------+------+------+------+-----------+---------+---------+
  *     [0]    [1]    [2]    [3]    [4..]      [4+LEN]   [5+LEN]
  *
  *   - LEN     : PAYLOAD 字节数（不含帧头/功能字/长度/CRC）
  *   - 总长度  : LEN + 6
  *   - CRC     : CRC-16/MODBUS，覆盖 [0]..[4+LEN-1]，低字节在前
  *   - 功能字  : bit7 = 0 表示下行(设备->主机)，bit7 = 1 表示上行(主机->设备)
  ******************************************************************************
  */

#ifndef __GESTURE_PROTO_H__
#define __GESTURE_PROTO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 帧常量 ---------------- */
#define PROTO_HEAD0             0xAAu
#define PROTO_HEAD1             0x55u
#define PROTO_MAX_PAYLOAD       64u
#define PROTO_OVERHEAD          6u          /* 头2 + 功能字1 + 长度1 + CRC2 */
#define PROTO_MAX_FRAME         (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)

/* ---------------- 功能字：下行（设备 -> 主机），bit7 = 0 ---------------- */
#define PROTO_F_ACK             0x7Fu       /* 命令应答 */
#define PROTO_F_STATUS          0x01u       /* 状态上报 */
#define PROTO_F_IMU             0x02u       /* 六轴原始数据 */
#define PROTO_F_RESULT          0x03u       /* 识别结果 */
#define PROTO_F_PARAM           0x04u       /* 参数应答 */

/* ---------------- 功能字：上行（主机 -> 设备），= 下行 | 0x80 ---------------- */
#define PROTO_F_REQ_STATUS      0x81u       /* 查询状态 */
#define PROTO_F_REQ_IMU         0x82u       /* 请求一帧 IMU 数据 */
#define PROTO_F_REQ_RESULT      0x83u       /* 查询最近识别结果 */
#define PROTO_F_REQ_PARAM       0x84u       /* 查询参数 */
#define PROTO_F_SET_PARAM       0x86u       /* 设置参数 */
#define PROTO_F_CTRL            0x87u       /* 运行控制 */

/* ---------------- 运行控制子命令（PROTO_F_CTRL 的 payload[0]） ---------------- */
#define PROTO_CTRL_CAPTURE_START 0x01u      /* 开始采集 */
#define PROTO_CTRL_CAPTURE_STOP  0x02u      /* 停止采集 */
#define PROTO_CTRL_INFER_START   0x03u      /* 开始推理 */
#define PROTO_CTRL_INFER_STOP    0x04u      /* 停止推理 */
#define PROTO_CTRL_RESET         0x05u      /* 软复位统计 */

/* ---------------- 参数 ID（PROTO_F_SET_PARAM / PROTO_F_PARAM 的 payload[0]） --- */
#define PROTO_PARAM_IMU_DIV      0x01u      /* IMU 调试行分频，payload[1] */
#define PROTO_PARAM_CONF_MIN     0x02u      /* 置信度门限 0~100，payload[1] */

/* ---------------- 应答码（PROTO_F_ACK 的 payload[0]） ---------------- */
#define PROTO_ACK_OK             0x00u
#define PROTO_ACK_BAD_FUNC       0x01u
#define PROTO_ACK_BAD_PARAM      0x02u
#define PROTO_ACK_BUSY           0x03u

/* ---------------- 解析器 ---------------- */
typedef struct
{
    uint8_t  buf[PROTO_MAX_FRAME];
    uint16_t idx;
    uint16_t need;
    uint8_t  state;
} ProtoParser;

/* ---------------- 统计 ---------------- */
typedef struct
{
    uint32_t frames_ok;      /* 校验通过的完整帧 */
    uint32_t frames_bad;     /* 校验失败 / 超长 */
    uint32_t bytes_rx;       /* 收到的总字节数 */
    uint32_t overflow;       /* 命令队列溢出次数 */
} ProtoStats;

extern ProtoStats g_proto_stats;

/* ---------------- API ---------------- */

/** CRC-16/MODBUS：poly 0x8005（反射 0xA001），init 0xFFFF */
uint16_t Proto_Crc16(const uint8_t *data, uint16_t len);

/**
 * @brief  组帧
 * @param  out     输出缓冲，至少 PROTO_MAX_FRAME 字节
 * @param  func    功能字
 * @param  payload 数据（可为 NULL，len 为 0）
 * @param  len     数据长度（<= PROTO_MAX_PAYLOAD）
 * @retval 帧总长度；参数非法返回 0
 */
uint16_t Proto_Pack(uint8_t *out, uint8_t func,
                    const uint8_t *payload, uint8_t len);

/** 初始化解析器 */
void Proto_ParserInit(ProtoParser *p);

/**
 * @brief  逐字节喂入解析器（可在中断中调用）
 * @retval 1 = 收到一个校验通过的完整帧（内容在 p->buf 中）
 *         0 = 需要继续喂
 *        -1 = 出错（校验失败 / 超长），状态已复位
 */
int Proto_ParserFeed(ProtoParser *p, uint8_t byte);

/**
 * @brief  中断侧：把解析成功的帧压入命令队列（可在 ISR 中调用）
 */
void Proto_PushIsr(const uint8_t *frame, uint16_t len);

/**
 * @brief  任务侧：取出一条命令
 * @param  func    输出功能字
 * @param  payload 输出数据指针（指向模块内部静态缓冲，下次 Pop 前有效）
 * @param  len     输出数据长度
 * @retval 1 = 取到一条命令；0 = 队列空
 */
int Proto_Pop(uint8_t *func, const uint8_t **payload, uint8_t *len);

/** 统计清零 */
void Proto_StatsReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __GESTURE_PROTO_H__ */

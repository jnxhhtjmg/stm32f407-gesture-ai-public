/**
  ******************************************************************************
  * @file    gesture_proto.c
  * @brief   统一串口协议实现（CRC16 + 组帧 + 流式解析 + ISR/任务命令队列）
  *          不依赖 HAL / FreeRTOS，可在裸机和 RTOS 下通用。
  ******************************************************************************
  */

#include "gesture_proto.h"
#include <string.h>

/* ---------------- 命令队列深度（必须是 2 的幂或能整除即可，这里用取模） ---- */
#define PROTO_QUEUE_DEPTH   4u

typedef struct
{
    uint8_t  data[PROTO_MAX_FRAME];
    uint16_t len;
} ProtoSlot;

static ProtoSlot        s_queue[PROTO_QUEUE_DEPTH];
static volatile uint8_t s_head;                     /* ISR 写 */
static volatile uint8_t s_tail;                     /* 任务读 */
static uint8_t          s_pop_buf[PROTO_MAX_FRAME]; /* Proto_Pop 返回的数据区 */

ProtoStats g_proto_stats;

/* ========================================================================== */
/*  CRC-16/MODBUS                                                             */
/*  多项式 0x8005（反射形式 0xA001），初值 0xFFFF，无输出异或                  */
/*  自检：Proto_Crc16((const uint8_t*)"123456789", 9) == 0x4B37               */
/* ========================================================================== */
uint16_t Proto_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint8_t  i;

    if (data == 0)
    {
        return 0u;
    }

    while (len-- != 0u)
    {
        crc ^= (uint16_t)(*data++);
        for (i = 0u; i < 8u; ++i)
        {
            if ((crc & 0x0001u) != 0u)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            }
            else
            {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

/* ========================================================================== */
/*  组帧                                                                      */
/* ========================================================================== */
uint16_t Proto_Pack(uint8_t *out, uint8_t func,
                    const uint8_t *payload, uint8_t len)
{
    uint16_t i = 0u;
    uint16_t crc;
    uint8_t  k;

    if (out == 0)
    {
        return 0u;
    }
    if (len > PROTO_MAX_PAYLOAD)
    {
        return 0u;
    }

    out[i++] = PROTO_HEAD0;
    out[i++] = PROTO_HEAD1;
    out[i++] = func;
    out[i++] = len;

    for (k = 0u; k < len; ++k)
    {
        out[i++] = (payload != 0) ? payload[k] : 0u;
    }

    crc = Proto_Crc16(out, i);
    out[i++] = (uint8_t)(crc & 0xFFu);          /* CRC 低字节 */
    out[i++] = (uint8_t)((crc >> 8) & 0xFFu);   /* CRC 高字节 */

    return i;                                    /* 总长 = len + 6 */
}

/* ========================================================================== */
/*  流式解析器                                                                */
/* ========================================================================== */
#define PROTO_ST_HEAD0   0u
#define PROTO_ST_HEAD1   1u
#define PROTO_ST_BODY    2u

void Proto_ParserInit(ProtoParser *p)
{
    if (p == 0)
    {
        return;
    }
    p->idx   = 0u;
    p->need  = 0u;
    p->state = PROTO_ST_HEAD0;
}

int Proto_ParserFeed(ProtoParser *p, uint8_t byte)
{
    uint16_t calc;
    uint16_t recv;

    if (p == 0)
    {
        return -1;
    }

    g_proto_stats.bytes_rx++;

    /* ---- 等帧头 0xAA ---- */
    if (p->state == PROTO_ST_HEAD0)
    {
        if (byte == PROTO_HEAD0)
        {
            p->buf[0] = byte;
            p->idx    = 1u;
            p->state  = PROTO_ST_HEAD1;
        }
        return 0;
    }

    /* ---- 等帧头 0x55 ---- */
    if (p->state == PROTO_ST_HEAD1)
    {
        if (byte == PROTO_HEAD1)
        {
            p->buf[1] = byte;
            p->idx    = 2u;
            p->need   = 0u;
            p->state  = PROTO_ST_BODY;
        }
        else if (byte == PROTO_HEAD0)
        {
            /* 连续 0xAA，保持在等 0x55 的相位 */
            p->buf[0] = byte;
            p->idx    = 1u;
        }
        else
        {
            p->state = PROTO_ST_HEAD0;
        }
        return 0;
    }

    /* ---- 收帧体 ---- */
    if (p->idx >= PROTO_MAX_FRAME)
    {
        p->state = PROTO_ST_HEAD0;
        g_proto_stats.frames_bad++;
        return -1;
    }

    p->buf[p->idx++] = byte;

    /* 收到 帧头(2) + 功能字 + 长度 = 4 字节时，算出总长 */
    if (p->idx == 4u)
    {
        if (p->buf[3] > PROTO_MAX_PAYLOAD)
        {
            p->state = PROTO_ST_HEAD0;
            g_proto_stats.frames_bad++;
            return -1;
        }
        p->need = (uint16_t)(4u + (uint16_t)p->buf[3] + 2u);
    }

    if ((p->need != 0u) && (p->idx >= p->need))
    {
        calc = Proto_Crc16(p->buf, (uint16_t)(p->need - 2u));
        recv = (uint16_t)p->buf[p->need - 2u] |
               (uint16_t)((uint16_t)p->buf[p->need - 1u] << 8);

        p->state = PROTO_ST_HEAD0;

        if (calc == recv)
        {
            g_proto_stats.frames_ok++;
            return 1;
        }
        g_proto_stats.frames_bad++;
        return -1;
    }

    return 0;
}

/* ========================================================================== */
/*  命令队列（单生产者 ISR / 单消费者 任务）                                   */
/* ========================================================================== */
void Proto_PushIsr(const uint8_t *frame, uint16_t len)
{
    uint8_t next;

    if ((frame == 0) || (len == 0u) || (len > PROTO_MAX_FRAME))
    {
        return;
    }

    next = (uint8_t)((s_head + 1u) % PROTO_QUEUE_DEPTH);
    if (next == s_tail)
    {
        /* 队列满：丢弃新帧，保证已入队的命令不丢 */
        g_proto_stats.overflow++;
        return;
    }

    (void)memcpy(s_queue[s_head].data, frame, len);
    s_queue[s_head].len = len;
    s_head = next;
}

int Proto_Pop(uint8_t *func, const uint8_t **payload, uint8_t *len)
{
    uint16_t n;

    if (s_head == s_tail)
    {
        return 0;
    }

    n = s_queue[s_tail].len;
    if (n > PROTO_MAX_FRAME)
    {
        n = PROTO_MAX_FRAME;
    }
    (void)memcpy(s_pop_buf, s_queue[s_tail].data, n);
    s_tail = (uint8_t)((s_tail + 1u) % PROTO_QUEUE_DEPTH);

    if (n < PROTO_OVERHEAD)
    {
        return 0;
    }

    if (func != 0)
    {
        *func = s_pop_buf[2];
    }
    if (len != 0)
    {
        *len = s_pop_buf[3];
    }
    if (payload != 0)
    {
        *payload = &s_pop_buf[4];
    }
    return 1;
}

void Proto_StatsReset(void)
{
    g_proto_stats.frames_ok  = 0u;
    g_proto_stats.frames_bad = 0u;
    g_proto_stats.bytes_rx   = 0u;
    g_proto_stats.overflow   = 0u;
}

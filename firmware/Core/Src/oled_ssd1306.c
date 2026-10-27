#include "oled_ssd1306.h"

#include <string.h>

#include "oled_font.h"

#define OLED_WIDTH   128U
#define OLED_PAGES   8U
#define OLED_CELL    6U
#define OLED_CHARS   (OLED_WIDTH / OLED_CELL)

static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];
static I2C_HandleTypeDef *oled_i2c = NULL;
static uint16_t oled_address = 0U;
static uint8_t oled_ready = 0U;

static uint8_t OLED_WriteCommand(uint8_t command)
{
  if (oled_ready == 0U)
  {
    return 0U;
  }
  return (HAL_I2C_Mem_Write(oled_i2c, oled_address, 0x00U,
                            I2C_MEMADD_SIZE_8BIT, &command, 1U, 100U) == HAL_OK)
             ? 1U
             : 0U;
}

static uint8_t OLED_WriteData(const uint8_t *data, uint16_t length)
{
  if (oled_ready == 0U)
  {
    return 0U;
  }
  return (HAL_I2C_Mem_Write(oled_i2c, oled_address, 0x40U,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length,
                            300U) == HAL_OK)
             ? 1U
             : 0U;
}

static uint8_t OLED_SendCommands(const uint8_t *commands, uint16_t count)
{
  uint16_t index;
  for (index = 0U; index < count; ++index)
  {
    if (OLED_WriteCommand(commands[index]) == 0U)
    {
      return 0U;
    }
  }
  return 1U;
}

uint8_t OLED_Init(I2C_HandleTypeDef *hi2c)
{
  static const uint8_t init_commands[] = {
      0xAEU,             /* display off */
      0x20U, 0x10U,      /* page addressing mode */
      0xB0U,             /* page 0 */
      0xC8U,             /* COM scan direction */
      0x00U, 0x10U,      /* column 0 */
      0x40U,             /* start line 0 */
      0x81U, 0xDFU,      /* contrast */
      0xA1U,             /* segment remap */
      0xA6U,             /* normal display */
      0xA8U, 0x3FU,      /* multiplex 64 */
      0xA4U,             /* output follows RAM */
      0xD3U, 0x00U,      /* display offset */
      0xD5U, 0xF0U,      /* clock divide */
      0xD9U, 0x22U,      /* pre-charge */
      0xDAU, 0x12U,      /* COM pins */
      0xDBU, 0x20U,      /* VCOMH */
      0x8DU, 0x14U,      /* charge pump on */
      0xAFU              /* display on */
  };
  static const uint8_t addresses[] = {0x3CU, 0x3DU};
  uint8_t index;

  oled_i2c = hi2c;
  oled_ready = 0U;
  oled_address = 0U;

  if (hi2c == NULL)
  {
    return 0U;
  }

  for (index = 0U; index < sizeof(addresses); ++index)
  {
    uint16_t candidate = (uint16_t)((uint16_t)addresses[index] << 1);
    if (HAL_I2C_IsDeviceReady(hi2c, candidate, 2U, 20U) == HAL_OK)
    {
      oled_address = candidate;
      break;
    }
  }

  if (oled_address == 0U)
  {
    return 0U;
  }

  oled_ready = 1U;
  if (OLED_SendCommands(init_commands, sizeof(init_commands)) == 0U)
  {
    oled_ready = 0U;
    return 0U;
  }

  HAL_Delay(100U);
  memset(oled_buffer, 0, sizeof(oled_buffer));
  OLED_Update();
  return 1U;
}

void OLED_Clear(void)
{
  memset(oled_buffer, 0, sizeof(oled_buffer));
}

void OLED_DrawString(uint8_t x, uint8_t page, const char *text)
{
  uint8_t column;

  if (page >= OLED_PAGES || text == NULL)
  {
    return;
  }

  while (*text != '\0' && x + OLED_CELL <= OLED_WIDTH)
  {
    uint8_t code = (uint8_t)*text;
    uint8_t font_index;

    if (code < 32U || code > 127U)
    {
      code = (uint8_t)'?';
    }
    font_index = (uint8_t)(code - 32U);

    for (column = 0U; column < 5U; ++column)
    {
      oled_buffer[(uint16_t)page * OLED_WIDTH + x + column] =
          oled_font5x7[font_index][column];
    }
    oled_buffer[(uint16_t)page * OLED_WIDTH + x + 5U] = 0x00U;

    x = (uint8_t)(x + OLED_CELL);
    ++text;
  }
}

void OLED_Update(void)
{
  uint8_t page;

  if (oled_ready == 0U)
  {
    return;
  }

  for (page = 0U; page < OLED_PAGES; ++page)
  {
    (void)OLED_WriteCommand((uint8_t)(0xB0U + page));
    (void)OLED_WriteCommand(0x00U);
    (void)OLED_WriteCommand(0x10U);
    (void)OLED_WriteData(&oled_buffer[(uint16_t)page * OLED_WIDTH],
                         OLED_WIDTH);
  }
}

static void OLED_SetPixel(uint8_t x, uint8_t y)
{
  if (x < OLED_WIDTH && y < (OLED_PAGES * 8U))
  {
    oled_buffer[(uint16_t)(y / 8U) * OLED_WIDTH + x] |=
        (uint8_t)(1U << (y % 8U));
  }
}

static void OLED_DrawStringScaled(uint8_t x, uint8_t y, const char *text,
                                  uint8_t scale)
{
  while (*text != '\0' && (uint16_t)x + 5U * scale <= OLED_WIDTH)
  {
    uint8_t code = (uint8_t)*text;
    uint8_t font_index;
    uint8_t column;
    uint8_t row;

    if (code < 32U || code > 127U)
    {
      code = (uint8_t)'?';
    }
    font_index = (uint8_t)(code - 32U);

    for (column = 0U; column < 5U; ++column)
    {
      uint8_t bits = oled_font5x7[font_index][column];
      for (row = 0U; row < 7U; ++row)
      {
        if ((bits & (uint8_t)(1U << row)) != 0U)
        {
          uint8_t dx;
          uint8_t dy;
          for (dy = 0U; dy < scale; ++dy)
          {
            for (dx = 0U; dx < scale; ++dx)
            {
              OLED_SetPixel((uint8_t)(x + column * scale + dx),
                            (uint8_t)(y + row * scale + dy));
            }
          }
        }
      }
    }

    x = (uint8_t)(x + OLED_CELL * scale);
    ++text;
  }
}

static void OLED_DrawStringCentered(uint8_t y, const char *text,
                                    uint8_t scale)
{
  uint16_t length = 0U;
  uint16_t width;

  while (text[length] != '\0')
  {
    ++length;
  }
  width = (uint16_t)(length * OLED_CELL * scale);
  if (width > scale)
  {
    width = (uint16_t)(width - scale);
  }
  if (width >= OLED_WIDTH)
  {
    OLED_DrawStringScaled(0U, y, text, scale);
  }
  else
  {
    OLED_DrawStringScaled((uint8_t)((OLED_WIDTH - width) / 2U), y, text,
                          scale);
  }
}

void OLED_ShowGesture(const char *name, uint8_t confidence)
{
  char upper[OLED_CHARS + 1U];
  char line[OLED_CHARS + 1U];
  uint8_t index = 0U;
  uint8_t digits = 0U;
  uint8_t value = confidence;
  char reversed[4];

  if (oled_ready == 0U)
  {
    return;
  }

  while (name != NULL && name[index] != '\0' && index < OLED_CHARS)
  {
    char ch = name[index];
    if (ch >= 'a' && ch <= 'z')
    {
      ch = (char)(ch - 'a' + 'A');
    }
    upper[index] = ch;
    ++index;
  }
  upper[index] = '\0';

  OLED_Clear();
  OLED_DrawStringCentered(0U, "GESTURE", 1U);
  OLED_DrawStringCentered(10U, upper, 2U);

  if (value > 100U)
  {
    value = 100U;
  }
  do
  {
    reversed[digits++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value > 0U && digits < sizeof(reversed));

  index = 0U;
  while (digits > 0U)
  {
    line[index++] = reversed[--digits];
  }
  line[index] = '\0';

  OLED_DrawStringCentered(26U, "CONF", 1U);
  OLED_DrawStringCentered(34U, line, 3U);

  {
    uint16_t width = (uint16_t)(index * OLED_CELL * 3U);
    uint8_t percent_x;
    if (width > 3U)
    {
      width = (uint16_t)(width - 3U);
    }
    percent_x = (uint8_t)((OLED_WIDTH + width) / 2U + 2U);
    if (percent_x > OLED_WIDTH - OLED_CELL)
    {
      percent_x = (uint8_t)(OLED_WIDTH - OLED_CELL);
    }
    OLED_DrawString(percent_x, 6U, "%");
  }

  OLED_DrawStringCentered(56U, "F407 TFLM", 1U);
  OLED_Update();
}

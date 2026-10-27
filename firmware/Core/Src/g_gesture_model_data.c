#include "g_gesture_model_data.h"

/*
 * Placeholder model.  The real INT8 model is intentionally not included.
 * Replace this file with the output of training/generate_model_c_array.py.
 *
 * The 16 zero bytes are enough for the TFLM schema-version check to fail
 * gracefully, so the firmware still boots and logs "gesture inference init
 * failed" until a trained model is generated.
 */
#if defined(__CC_ARM)
__align(4)
#elif defined(__ARMCC_VERSION)
__attribute__((aligned(4)))
#endif
const unsigned char g_gesture_model_data[] = {
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

const unsigned int g_gesture_model_data_len =
    (unsigned int)sizeof(g_gesture_model_data);

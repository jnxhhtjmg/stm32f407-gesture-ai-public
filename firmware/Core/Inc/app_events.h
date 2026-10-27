/**
 * @file    app_events.h
 * @brief   Small hooks between the HAL timebase ISR and the application.
 */
#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#ifdef __cplusplus
extern "C" {
#endif

/** Called from the TIM6 update interrupt at APP_SAMPLE_RATE_HZ. */
void App_OnSampleTickFromIsr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_EVENTS_H */

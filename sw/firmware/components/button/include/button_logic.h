#pragma once

// Debounced press/hold state machine for an active-low button. No ESP-IDF includes, so host tests include it directly.
//
// The driver calls btnSample() after every edge, once the contact has settled, and again when btnWaitUs() has run out.
// btnSample() reports what changed; the caller never reads the pin more than once per call.

#include <stdbool.h>
#include <stdint.h>

#define BTN_EVENT_PRESS 0x1u    // debounced press-down
#define BTN_EVENT_RELEASE 0x2u  // debounced release
#define BTN_EVENT_HOLD 0x4u     // still down after holdUs, once per press

#define BTN_WAIT_FOREVER INT64_MAX

typedef struct
{
  bool    down;         // debounced level
  bool    armed;        // false while the button was already down at init: ignored until it is released once
  bool    holdFired;    // HOLD already reported for this press
  int64_t downSinceUs;  // valid while down
  int64_t lastPressUs;  // time of the last reported press, 0 = none since init
} BtnState;

// pressedAtInit: the pin read low when the driver started. That is a held button (or a stuck one), not a press.
static inline void btnInit(BtnState* s, bool pressedAtInit)
{
  s->down        = pressedAtInit;
  s->armed       = !pressedAtInit;
  s->holdFired   = false;
  s->downSinceUs = 0;
  s->lastPressUs = 0;
}

// `pressed`: settled pin level (true = low = down). Returns a BTN_EVENT_* mask. A held button that was down at init
// reports nothing until it is released.
static inline uint32_t btnSample(BtnState* s, bool pressed, int64_t nowUs, int64_t holdUs)
{
  uint32_t ev = 0;
  if (pressed && !s->down)
  {
    s->down        = true;
    s->downSinceUs = nowUs;
    s->holdFired   = false;
    if (s->armed)
    {
      s->lastPressUs = nowUs > 0 ? nowUs : 1;  // 0 is "no press"
      ev |= BTN_EVENT_PRESS;
    }
  }
  else if (!pressed && s->down)
  {
    s->down      = false;
    s->holdFired = false;
    if (s->armed)
    {
      ev |= BTN_EVENT_RELEASE;
    }
    s->armed = true;
  }
  if (s->down && s->armed && !s->holdFired && nowUs - s->downSinceUs >= holdUs)
  {
    s->holdFired = true;
    ev |= BTN_EVENT_HOLD;
  }
  return ev;
}

// How long the driver may sleep before it must call btnSample() again without an edge: until the hold fires, else forever.
static inline int64_t btnWaitUs(const BtnState* s, int64_t nowUs, int64_t holdUs)
{
  if (!s->down || !s->armed || s->holdFired)
  {
    return BTN_WAIT_FOREVER;
  }
  const int64_t left = holdUs - (nowUs - s->downSinceUs);
  return left > 0 ? left : 0;
}

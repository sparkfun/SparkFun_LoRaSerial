
//Platform specific reset commands
void systemReset()
{
#if defined(ARDUINO_ARCH_SAMD)
  NVIC_SystemReset();
#elif defined(ARDUINO_ARCH_ESP32)
  ESP.restart();
#endif
}

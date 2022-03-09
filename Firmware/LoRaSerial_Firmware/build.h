
//Be aware turning on debug prints will mess with all sorts of radio timing
#if defined(ENABLE_DEVELOPER)
  #define LRS_DEBUG
#endif

#if !defined(LRS_DEBUG_PORT)
  #define LRS_DEBUG_PORT   Serial
#endif

#if defined(LRS_DEBUG)
    #define LRS_DEBUG_PRINT(...) { LRS_DEBUG_PORT.print(__VA_ARGS__); }
    #define LRS_DEBUG_PRINTLN(...) { LRS_DEBUG_PORT.println(__VA_ARGS__); }
#else
  #define LRS_DEBUG_PRINT(...) {}
  #define LRS_DEBUG_PRINTLN(...) {}
#endif

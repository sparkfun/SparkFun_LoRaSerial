
//Be aware turning on debug prints will mess with all sorts of radio timing
#if defined(ENABLE_DEVELOPER)
  //#define STR_DEBUG
#endif

#if !defined(STR_DEBUG_PORT)
  #define STR_DEBUG_PORT   Serial
#endif

#if defined(STR_DEBUG)
    #define STR_DEBUG_PRINT(...) { STR_DEBUG_PORT.print(__VA_ARGS__); }
    #define STR_DEBUG_PRINTLN(...) { STR_DEBUG_PORT.println(__VA_ARGS__); }
#else
  #define STR_DEBUG_PRINT(...) {}
  #define STR_DEBUG_PRINTLN(...) {}
#endif
